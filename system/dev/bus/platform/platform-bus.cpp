// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/protocol/platform-defs.h>
#include <zircon/process.h>
#include <zircon/syscalls/iommu.h>

#include <fbl/unique_ptr.h>

#include "platform-bus.h"
#include "platform-device.h"
#include "platform-proxy.h"

namespace platform_bus {

static zx_status_t platform_bus_get_bti(void* ctx, uint32_t iommu_index, uint32_t bti_id,
                                        zx_handle_t* out_handle) {
    auto bus = static_cast<PlatformBus*>(ctx);
    if (iommu_index != 0) {
        return ZX_ERR_OUT_OF_RANGE;
    }
    return zx_bti_create(bus->iommu_handle_, 0, bti_id, out_handle);
}

// default IOMMU protocol to use if the board driver does not set one via pbus_set_protocol()
static iommu_protocol_ops_t platform_bus_default_iommu_ops = {
    .get_bti = platform_bus_get_bti,
};


zx_status_t PlatformBus::GetBti(uint32_t iommu_index, uint32_t bti_id, zx_handle_t* out_handle) {
    if (iommu_index != 0) {
        return ZX_ERR_OUT_OF_RANGE;
    }
    return zx_bti_create(iommu_handle_, 0, bti_id, out_handle);
}

zx_status_t PlatformBus::SetProtocol(uint32_t proto_id, void* protocol) {
    if (!protocol) {
        return ZX_ERR_INVALID_ARGS;
    }

    fbl::AllocChecker ac;

    switch (proto_id) {
    case ZX_PROTOCOL_USB_MODE_SWITCH: {
        ums_.reset(new (&ac) ddk::UmsProtocolProxy(
                                    static_cast<usb_mode_switch_protocol_t*>(protocol)));
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        break;
    }
    case ZX_PROTOCOL_GPIO: {
        gpio_.reset(new (&ac) ddk::GpioProtocolProxy(static_cast<gpio_protocol_t*>(protocol)));
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        break;
    }
    case ZX_PROTOCOL_I2C_IMPL: {
        auto proto = static_cast<i2c_impl_protocol_t*>(protocol);
        auto status = I2cInit(proto);
        if (status != ZX_OK) {
            return status;
         }

        i2c_impl_.reset(new (&ac) ddk::I2cImplProtocolProxy(proto));
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        break;
    }
    case ZX_PROTOCOL_CLK: {
        clk_.reset(new (&ac) ddk::ClkProtocolProxy(static_cast<clk_protocol_t*>(protocol)));
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        break;
    }
    case ZX_PROTOCOL_IOMMU: {
        iommu_.reset(new (&ac) ddk::IommuProtocolProxy(static_cast<iommu_protocol_t*>(protocol)));
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        break;
    }
    case ZX_PROTOCOL_SCPI: {
        scpi_.reset(new (&ac) ddk::ScpiProtocolProxy(static_cast<scpi_protocol_t*>(protocol)));
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        break;
    }
    case ZX_PROTOCOL_CANVAS: {
        canvas_.reset(new (&ac) ddk::CanvasProtocolProxy(
                                                    static_cast<canvas_protocol_t*>(protocol)));
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        break;
    }
    default:
        // TODO(voydanoff) consider having a registry of arbitrary protocols
        return ZX_ERR_NOT_SUPPORTED;
    }

    sync_completion_signal(&proto_completion_);
    return ZX_OK;
}

zx_status_t PlatformBus::WaitProtocol(uint32_t proto_id) {
    platform_bus_protocol_t dummy;
    while (DdkGetProtocol(proto_id, &dummy) == ZX_ERR_NOT_SUPPORTED) {
        sync_completion_reset(&proto_completion_);
        zx_status_t status = sync_completion_wait(&proto_completion_, ZX_TIME_INFINITE);
        if (status != ZX_OK) {
            return status;
        }
    }
    return ZX_OK;
}

zx_status_t PlatformBus::DeviceAdd(const pbus_dev_t* pdev, uint32_t flags) {
    zx_status_t status = ZX_OK;

    if (flags & ~(PDEV_ADD_DISABLED | PDEV_ADD_PBUS_DEVHOST)) {
        return ZX_ERR_INVALID_ARGS;
    }

    // add PCI root at top level
    zx_device_t* parent = zxdev();
    if (pdev->vid == PDEV_VID_GENERIC && pdev->pid == PDEV_PID_GENERIC && pdev->did == PDEV_DID_KPCI) {
        parent = device_get_parent(parent);
    }

    fbl::AllocChecker ac;
    fbl::unique_ptr<platform_bus::PlatformDevice> dev(new (&ac)
                                                platform_bus::PlatformDevice(parent, pdev, flags));
    if (!ac.check()) {
        return ZX_ERR_NO_MEMORY;
    }

    dev->bus_ = this;
    dev->flags_ = flags;
    if (!pdev->name) {
        return ZX_ERR_INVALID_ARGS;
    }

    if (pdev->mmio_count) {
        dev->mmios_.reserve(pdev->mmio_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->mmio_count; i++) {
            dev->mmios_.push_back(pdev->mmios[i]);
        }
    }
    if (pdev->irq_count) {
        dev->irqs_.reserve(pdev->irq_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->irq_count; i++) {
            dev->irqs_.push_back(pdev->irqs[i]);
        }
    }
    if (pdev->gpio_count) {
        dev->gpios_.reserve(pdev->gpio_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->gpio_count; i++) {
            dev->gpios_.push_back(pdev->gpios[i]);
        }
    }
    if (pdev->i2c_channel_count) {
        dev->i2c_channels_.reserve(pdev->i2c_channel_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->i2c_channel_count; i++) {
            dev->i2c_channels_.push_back(pdev->i2c_channels[i]);
        }
    }
    if (pdev->clk_count) {
        dev->clks_.reserve(pdev->clk_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->clk_count; i++) {
            dev->clks_.push_back(pdev->clks[i]);
        }
    }
    if (pdev->bti_count) {
        dev->btis_.reserve(pdev->bti_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->bti_count; i++) {
            dev->btis_.push_back(pdev->btis[i]);
        }
    }
    if (pdev->metadata_count) {
        dev->metadata_.reserve(pdev->metadata_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->metadata_count; i++) {
            dev->metadata_.push_back(pdev->metadata[i]);
        }
    }

    devices_.push_back(fbl::move(dev), &ac);
    if (!ac.check()) {
        return ZX_ERR_NO_MEMORY;
    }

    if ((flags & PDEV_ADD_DISABLED) == 0) {
        status = dev->Enable(true);
    }

    return status;
}

zx_status_t PlatformBus::DeviceEnable(uint32_t vid, uint32_t pid, uint32_t did, bool enable) {
    for (auto& dev : devices_) {
        if (dev->vid_ == vid && dev->pid_ == pid && dev->did_ == did) {
            return dev->Enable(enable);
        }
    }

    return ZX_ERR_NOT_FOUND;
}

const char* PlatformBus::GetBoardName() {
    return platform_id_.board_name;
}

zx_status_t PlatformBus::DdkGetProtocol(uint32_t proto_id, void* protocol) {
    switch (proto_id) {
    case ZX_PROTOCOL_PLATFORM_BUS: {
        auto proto = static_cast<platform_bus_protocol_t*>(protocol);
        proto->ctx = this;
        proto->ops = &pbus_proto_ops_;
        return ZX_OK;
    }
    case ZX_PROTOCOL_USB_MODE_SWITCH:
        if (ums_ != nullptr) {
            ums_->GetProto(static_cast<usb_mode_switch_protocol_t*>(protocol));
            return ZX_OK;
        }
        break;
    case ZX_PROTOCOL_GPIO:
        if (gpio_ != nullptr) {
            gpio_->GetProto(static_cast<gpio_protocol_t*>(protocol));
            return ZX_OK;
        }
        break;
    case ZX_PROTOCOL_I2C_IMPL:
        if (i2c_impl_ != nullptr) {
            i2c_impl_->GetProto(static_cast<i2c_impl_protocol_t*>(protocol));
            return ZX_OK;
        }
        break;
    case ZX_PROTOCOL_CLK:
        if (clk_ != nullptr) {
            clk_->GetProto(static_cast<clk_protocol_t*>(protocol));
            return ZX_OK;
        }
        break;
    case ZX_PROTOCOL_IOMMU:
        if (iommu_ != nullptr) {
            iommu_->GetProto(static_cast<iommu_protocol_t*>(protocol));
            return ZX_OK;
        }
        break;
    case ZX_PROTOCOL_SCPI:
        if (scpi_ != nullptr) {
            scpi_->GetProto(static_cast<scpi_protocol_t*>(protocol));
            return ZX_OK;
        }
        break;
    case ZX_PROTOCOL_CANVAS:
        if (canvas_ != nullptr) {
            canvas_->GetProto(static_cast<canvas_protocol_t*>(protocol));
            return ZX_OK;
        }
        break;
    default:
        // TODO(voydanoff) consider having a registry of arbitrary protocols
        return ZX_ERR_NOT_SUPPORTED;
    }

    return ZX_ERR_NOT_SUPPORTED;
}

/*
static void platform_bus_release(void* ctx) {
    platform_bus_t* bus = static_cast<platform_bus_t*>(ctx);

    bus->devices.reset();

    zx_handle_close(bus->dummy_iommu_handle);
    free(bus->metadata);
    free(bus);
}
*/

static zx_protocol_device_t sys_device_proto = {
    .version = DEVICE_OPS_VERSION,
    .get_protocol = nullptr,
    .open = nullptr,
    .open_at = nullptr,
    .close = nullptr,
    .unbind = nullptr,
    .release = nullptr,
    .read = nullptr,
    .write = nullptr,
    .get_size = nullptr,
    .ioctl = nullptr,
    .suspend = nullptr,
    .resume = nullptr,
    .rxrpc = nullptr,
    .message = nullptr,
};

zx_status_t PlatformBus::ReadZbi(zx_handle_t vmo) {
    zbi_header_t header;

    zx_status_t status = zx_vmo_read(vmo, &header, 0, sizeof(header));
    if (status != ZX_OK) {
        return status;
    }
    if ((header.type != ZBI_TYPE_CONTAINER) || (header.extra != ZBI_CONTAINER_MAGIC)) {
        zxlogf(ERROR, "platform_bus: ZBI VMO not contain ZBI container\n");
        return ZX_ERR_INTERNAL;
    }

    size_t zbi_length = header.length;

    // compute size of ZBI records we need to save for metadata
    size_t metadata_size = 0;
    size_t len = zbi_length;
    size_t off = sizeof(header);

    while (len > sizeof(header)) {
        zx_status_t status = zx_vmo_read(vmo, &header, off, sizeof(header));
        if (status < 0) {
            zxlogf(ERROR, "zx_vmo_read failed: %d\n", status);
            return status;
        }
        size_t itemlen = ZBI_ALIGN(sizeof(zbi_header_t) + header.length);
        if (itemlen > len) {
            zxlogf(ERROR, "platform_bus: ZBI item too large (%zd > %zd)\n", itemlen, len);
            break;
        }
        if (ZBI_TYPE_DRV_METADATA(header.type)) {
            metadata_size += itemlen;
        }
        off += itemlen;
        len -= itemlen;
    }

    if (metadata_size) {
        metadata_ = static_cast<uint8_t*>(malloc(metadata_size));
        if (!metadata_) {
            return ZX_ERR_NO_MEMORY;
        }
    }

    bool got_platform_id = false;
    zx_off_t metadata_offset = 0;
    uint8_t* metadata = metadata_;
    len = zbi_length;
    off = sizeof(header);

    // find platform ID record and copy metadata records
    while (len > sizeof(header)) {
        zx_status_t status = zx_vmo_read(vmo, &header, off, sizeof(header));
        if (status < 0) {
            break;
        }
        size_t itemlen = ZBI_ALIGN(sizeof(zbi_header_t) + header.length);
        if (itemlen > len) {
            zxlogf(ERROR, "platform_bus: ZBI item too large (%zd > %zd)\n", itemlen, len);
            break;
        }
        if (header.type == ZBI_TYPE_PLATFORM_ID) {
            status = zx_vmo_read(vmo, &platform_id_, off + sizeof(zbi_header_t),
                                 sizeof(platform_id_));
            if (status != ZX_OK) {
                zxlogf(ERROR, "zx_vmo_read failed: %d\n", status);
                return status;
            }
            got_platform_id = true;
        } else if (ZBI_TYPE_DRV_METADATA(header.type)) {
            status = zx_vmo_read(vmo, metadata + metadata_offset, off, itemlen);
            if (status != ZX_OK) {
                zxlogf(ERROR, "zx_vmo_read failed: %d\n", status);
                return status;
            }
            metadata_offset += itemlen;
        }
        off += itemlen;
        len -= itemlen;
    }
    metadata_size_ = metadata_size;

    if (!got_platform_id) {
         zxlogf(ERROR, "platform_bus: ZBI_TYPE_PLATFORM_ID not found\n");
        return ZX_ERR_INTERNAL;
    }
    return ZX_OK;
}

typedef struct {
    uint32_t txid;
    zx_handle_t channel_handle;

    list_node_t node;
    size_t write_length;
    size_t read_length;
    uint16_t address;
    i2c_complete_cb complete_cb;
    void* cookie;
    uint8_t write_buffer[];
} i2c_txn_t;

static void platform_i2c_complete(i2c_txn_t* txn, zx_status_t status, const uint8_t* data,
                                     size_t data_length) {
    struct {
        pdev_resp_t resp;
        uint8_t data[PDEV_I2C_MAX_TRANSFER_SIZE] = {};
    } resp = {
        .resp = {
            .txid = txn->txid,
            .status = status,
            .i2c_txn = {
                .write_length = 0,
                .read_length = 0,
                .complete_cb = txn->complete_cb,
                .cookie = txn->cookie,
            },
        },
    };

    if (status == ZX_OK) {
        memcpy(resp.data, data, data_length);
    }

    status = zx_channel_write(txn->channel_handle, 0, &resp,
                              static_cast<uint32_t>(sizeof(resp.resp) + data_length), nullptr, 0);
    if (status != ZX_OK) {
        zxlogf(ERROR, "platform_i2c_read_complete: zx_channel_write failed %d\n", status);
    }
}

static int i2c_bus_thread(void *arg) {
    platform_i2c_bus_t* i2c_bus = static_cast<platform_i2c_bus_t*>(arg);

    uint8_t* read_buffer = static_cast<uint8_t*>(malloc(i2c_bus->max_transfer));
    if (!read_buffer) {
        return -1;
    }

    while (1) {
        sync_completion_wait(&i2c_bus->txn_signal, ZX_TIME_INFINITE);
        sync_completion_reset(&i2c_bus->txn_signal);

        i2c_txn_t* txn;

        mtx_lock(&i2c_bus->lock);
        while ((txn = list_remove_head_type(&i2c_bus->queued_txns, i2c_txn_t, node)) != nullptr) {
            mtx_unlock(&i2c_bus->lock);

            zx_status_t status = i2c_impl_transact(&i2c_bus->i2c, i2c_bus->bus_id, txn->address,
                                                        txn->write_buffer, txn->write_length,
                                                        read_buffer, txn->read_length);
            size_t actual = (status == ZX_OK ? txn->read_length : 0);
            platform_i2c_complete(txn, status, read_buffer, actual);

            mtx_lock(&i2c_bus->lock);
            list_add_tail(&i2c_bus->free_txns, &txn->node);
        }
        mtx_unlock(&i2c_bus->lock);
    }

    free(read_buffer);
    return 0;
}

zx_status_t PlatformBus::I2cInit(i2c_impl_protocol_t* i2c) {
    if (!i2c_buses_.is_empty()) {
        // already initialized
        return ZX_ERR_BAD_STATE;
    }

    uint32_t bus_count = i2c_impl_get_bus_count(i2c);
    if (!bus_count) {
        return ZX_ERR_NOT_SUPPORTED;
    }

    fbl::AllocChecker ac;
    i2c_buses_.reserve(bus_count, &ac);
    if (!ac.check()) {
        return ZX_ERR_NO_MEMORY;
    }

    for (uint32_t i = 0; i < bus_count; i++) {
        platform_i2c_bus_t dummy = {};
        fbl::AllocChecker ac;
        i2c_buses_.push_back(dummy);

        platform_i2c_bus_t* i2c_bus = &i2c_buses_[i];

        i2c_bus->bus_id = i;
        mtx_init(&i2c_bus->lock, mtx_plain);
        list_initialize(&i2c_bus->queued_txns);
        list_initialize(&i2c_bus->free_txns);
        sync_completion_reset(&i2c_bus->txn_signal);
        memcpy(&i2c_bus->i2c, i2c, sizeof(i2c_bus->i2c));

        auto status = i2c_impl_get_max_transfer_size(i2c, i, &i2c_bus->max_transfer);
        if (status != ZX_OK) {
            return status;
        }

        char name[32];
        snprintf(name, sizeof(name), "i2c_bus_thread[%u]", i);
        thrd_create_with_name(&i2c_bus->thread, i2c_bus_thread, i2c_bus, name);
    }

    return ZX_OK;
}

zx_status_t PlatformBus::I2cTransact(pdev_req_t* req, pbus_i2c_channel_t* channel,
                                  const void* write_buf, zx_handle_t channel_handle) {
    if (channel->bus_id >= i2c_buses_.size()) {
        return ZX_ERR_INVALID_ARGS;
    }
    platform_i2c_bus_t* i2c_bus = &i2c_buses_[channel->bus_id];

    const size_t write_length = req->i2c_txn.write_length;
    const size_t read_length = req->i2c_txn.read_length;
    if (write_length > i2c_bus->max_transfer || read_length > i2c_bus->max_transfer) {
        return ZX_ERR_INVALID_ARGS;
    }

    mtx_lock(&i2c_bus->lock);

    i2c_txn_t* txn = list_remove_head_type(&i2c_bus->free_txns, i2c_txn_t, node);
    if (!txn) {
        // add space for write buffer
        txn = static_cast<i2c_txn_t*>(calloc(1, sizeof(i2c_txn_t) + i2c_bus->max_transfer));
    }
    if (!txn) {
        mtx_unlock(&i2c_bus->lock);
        return ZX_ERR_NO_MEMORY;
    }

    txn->address = channel->address;
    txn->write_length = write_length;
    txn->read_length = read_length;
    memcpy(txn->write_buffer, write_buf, write_length);
    txn->complete_cb = req->i2c_txn.complete_cb;
    txn->cookie = req->i2c_txn.cookie;
    txn->txid = req->txid;
    txn->channel_handle = channel_handle;

    list_add_tail(&i2c_bus->queued_txns, &txn->node);
    mtx_unlock(&i2c_bus->lock);
    sync_completion_signal(&i2c_bus->txn_signal);

    return ZX_OK;
}

void PlatformBus::DdkRelease() {
    delete this;
}

zx_status_t PlatformBus::Create(zx_device_t* parent, const char* name, zx_handle_t zbi_vmo) {

    // This creates the "sys" device
    device_add_args_t self_args = {};
    self_args.version = DEVICE_ADD_ARGS_VERSION;
    self_args.name = name;
    self_args.ops = &sys_device_proto;
    self_args.flags = DEVICE_ADD_NON_BINDABLE;

    auto status = device_add(parent, &self_args, &parent);
    if (status != ZX_OK) {
        return status;
    }


    // set up a dummy IOMMU protocol to use in the case where our board driver does not
    // set a real one.
    zx_iommu_desc_dummy_t desc;
    zx_handle_t iommu_handle;
    status = zx_iommu_create(get_root_resource(), ZX_IOMMU_TYPE_DUMMY,  &desc, sizeof(desc),
                             &iommu_handle);
    if (status != ZX_OK) {
        zx_handle_close(zbi_vmo);
        return status;
    }

    fbl::AllocChecker ac;
    fbl::unique_ptr<platform_bus::PlatformBus> bus(new (&ac)
                                        platform_bus::PlatformBus(parent, iommu_handle));
    if (!ac.check()) {
        zx_handle_close(zbi_vmo);
        zx_handle_close(iommu_handle);
        return ZX_ERR_NO_MEMORY;
    }

    iommu_protocol_t iommu;
    iommu.ops = &platform_bus_default_iommu_ops;
    iommu.ctx = bus.get();

    bus->iommu_.reset(new (&ac) ddk::IommuProtocolProxy(&iommu));
    if (!ac.check()) {
        zx_handle_close(zbi_vmo);
        zx_handle_close(iommu_handle);
        return ZX_ERR_NO_MEMORY;
    }

    status = bus->ReadZbi(zbi_vmo);
    zx_handle_close(zbi_vmo);
    if (status != ZX_OK) {
        return status;
    }

    status = bus->DdkAdd(name);
    if (status != ZX_OK) {
        return status;
    }
/*
    // Then we attach the platform-bus device below it
    zx_device_prop_t props[] = {
        {BIND_PLATFORM_DEV_VID, 0, bus->platform_id.vid},
        {BIND_PLATFORM_DEV_PID, 0, bus->platform_id.pid},
    };

    device_add_args_t add_args = {};
    add_args.version = DEVICE_ADD_ARGS_VERSION;
    add_args.name = "platform";
    add_args.ctx = bus;
    add_args.ops = &platform_bus_proto;
    add_args.proto_id = ZX_PROTOCOL_PLATFORM_BUS;
    add_args.proto_ops = &platform_bus_proto_ops;
    add_args.props = props;
    add_args.prop_count = countof(props);

    return device_add(parent, &add_args, &bus->zxdev);
*/  
    
    // devmgr is now in charge of the device.
    __UNUSED auto* dummy = bus.release();
    return ZX_OK;
}

PlatformBus::PlatformBus(zx_device_t* parent, zx_handle_t iommu)
        : PlatformBusType(parent), iommu_handle_(iommu) {
    sync_completion_reset(&proto_completion_);
}


} // namespace platform_bus


zx_status_t platform_bus_create(void* ctx, zx_device_t* parent, const char* name,
                                const char* args, zx_handle_t zbi_vmo) {
    return platform_bus::PlatformBus::Create(parent, name, zbi_vmo);
}
