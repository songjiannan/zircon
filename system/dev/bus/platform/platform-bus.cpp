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

namespace platform_bus {

zx_status_t PbusDevice::GetBti(uint32_t iommu_index, uint32_t bti_id, zx_handle_t* out_handle) {
    if (iommu_index != 0) {
        return ZX_ERR_OUT_OF_RANGE;
    }
    return zx_bti_create(iommu_handle_, 0, bti_id, out_handle);
}

zx_status_t PbusDevice::SetProtocol(uint32_t proto_id, void* protocol) {
    if (!protocol) {
        return ZX_ERR_INVALID_ARGS;
    }

    switch (proto_id) {
    case ZX_PROTOCOL_USB_MODE_SWITCH:
        memcpy(&ums_, protocol, sizeof(ums_));
        break;
    case ZX_PROTOCOL_GPIO:
        memcpy(&gpio_, protocol, sizeof(gpio_));
        break;
/*FIXME
    case ZX_PROTOCOL_I2C_IMPL: {
        zx_status_t status = platform_i2c_init(bus, (i2c_impl_protocol_t *)protocol);
        if (status != ZX_OK) {
            return status;
         }
        memcpy(&i2c_, protocol, sizeof(i2c_));
        break;
    }
*/
    case ZX_PROTOCOL_CLK:
        memcpy(&clk_, protocol, sizeof(clk_));
        break;
    case ZX_PROTOCOL_IOMMU:
//FIXME        memcpy(&bus->iommu, protocol, sizeof(bus->iommu));
        break;
    case ZX_PROTOCOL_SCPI:
        memcpy(&scpi_, protocol, sizeof(scpi_));
        break;
    case ZX_PROTOCOL_CANVAS:
        memcpy(&canvas_, protocol, sizeof(canvas_));
        break;
    default:
        // TODO(voydanoff) consider having a registry of arbitrary protocols
        return ZX_ERR_NOT_SUPPORTED;
    }

    sync_completion_signal(&proto_completion_);
    return ZX_OK;
}

zx_status_t PbusDevice::WaitProtocol(uint32_t proto_id) {
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

zx_status_t PbusDevice::DeviceAdd(const pbus_dev_t* dev, uint32_t flags) {
//FIXME    platform_bus_t* bus = static_cast<platform_bus_t*>(ctx);
//FIXME    return platform_device_add(bus, dev, flags);
return 0;
}

zx_status_t PbusDevice::DeviceEnable(uint32_t vid, uint32_t pid, uint32_t did, bool enable) {
    for (auto dev : devices_) {
        if (dev->vid == vid && dev->pid == pid && dev->did == did) {
            return platform_device_enable(dev, enable);
        }
    }

    return ZX_ERR_NOT_FOUND;
}

const char* PbusDevice::GetBoardName() {
    return platform_id_.board_name;
}

zx_status_t PbusDevice::DdkGetProtocol(uint32_t proto_id, void* protocol) {
    switch (proto_id) {
    case ZX_PROTOCOL_PLATFORM_BUS: {
        auto proto = static_cast<platform_bus_protocol_t*>(protocol);
        proto->ctx = this;
        proto->ops = &pbus_proto_ops_;
        return ZX_OK;
    }
    case ZX_PROTOCOL_USB_MODE_SWITCH:
        if (ums_.ops) {
            memcpy(protocol, &ums_, sizeof(ums_));
            return ZX_OK;
        }
        break;
    case ZX_PROTOCOL_GPIO:
        if (gpio_.ops) {
            memcpy(protocol, &gpio_, sizeof(gpio_));
            return ZX_OK;
        }
        break;
    case ZX_PROTOCOL_I2C_IMPL:
        if (i2c_.ops) {
            memcpy(protocol, &i2c_, sizeof(i2c_));
            return ZX_OK;
        }
        break;
    case ZX_PROTOCOL_CLK:
        if (clk_.ops) {
            memcpy(protocol, &clk_, sizeof(clk_));
            return ZX_OK;
        }
        break;
    case ZX_PROTOCOL_IOMMU: {
        auto proto = static_cast<iommu_protocol_t*>(protocol);
        proto->ctx = this;
        proto->ops = &iommu_proto_ops_;
        return ZX_OK;
    }
    case ZX_PROTOCOL_SCPI:
        if (scpi_.ops) {
            memcpy(protocol, &scpi_, sizeof(scpi_));
            return ZX_OK;
        }
        break;
    case ZX_PROTOCOL_CANVAS:
        if (canvas_.ops) {
            memcpy(protocol, &canvas_, sizeof(canvas_));
            return ZX_OK;
        }
        break;
    default:
        // TODO(voydanoff) consider having a registry of arbitrary protocols
        return ZX_ERR_NOT_SUPPORTED;
    }

    return ZX_ERR_NOT_SUPPORTED;
}

static void platform_bus_release(void* ctx) {
    platform_bus_t* bus = static_cast<platform_bus_t*>(ctx);

    bus->devices.reset();

    zx_handle_close(bus->dummy_iommu_handle);
    free(bus->metadata);
    free(bus);
}

/*
static zx_protocol_device_t platform_bus_proto = {
    .version = DEVICE_OPS_VERSION,
    .get_protocol = platform_bus_get_protocol,
    .open = nullptr,
    .open_at = nullptr,
    .close = nullptr,
    .unbind = nullptr,
    .release = platform_bus_release,
    .read = nullptr,
    .write = nullptr,
    .get_size = nullptr,
    .ioctl = nullptr,
    .suspend = nullptr,
    .resume = nullptr,
    .rxrpc = nullptr,
    .message = nullptr,
};

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
    .suspend = platform_bus_suspend,
    .resume = nullptr,
    .rxrpc = nullptr,
    .message = nullptr,
};
*/

static zx_status_t platform_bus_read_zbi(platform_bus_t* bus, zx_handle_t vmo) {
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
        bus->metadata = static_cast<uint8_t*>(malloc(metadata_size));
        if (!bus->metadata) {
            return ZX_ERR_NO_MEMORY;
        }
    }

    bool got_platform_id = false;
    zx_off_t metadata_offset = 0;
    uint8_t* metadata = (uint8_t*)bus->metadata;
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
            status = zx_vmo_read(vmo, &bus->platform_id, off + sizeof(zbi_header_t),
                                 sizeof(bus->platform_id));
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
    bus->metadata_size = metadata_size;

    if (!got_platform_id) {
         zxlogf(ERROR, "platform_bus: ZBI_TYPE_PLATFORM_ID not found\n");
        return ZX_ERR_INTERNAL;
    }
    return ZX_OK;
}

void PbusDevice::DdkRelease() {
    delete this;
}

zx_status_t PbusDevice::Create(zx_device_t* parent, const char* name, zx_handle_t zbi_vmo) {

    // set up a dummy IOMMU protocol to use in the case where our board driver does not
    // set a real one.
    zx_iommu_desc_dummy_t desc;
    zx_handle_t iommu_handle;
    auto status = zx_iommu_create(get_root_resource(), ZX_IOMMU_TYPE_DUMMY,  &desc, sizeof(desc),
                             &iommu_handle);
    if (status != ZX_OK) {
        zx_handle_close(zbi_vmo);
        return status;
    }

    fbl::AllocChecker ac;
    fbl::unique_ptr<platform_bus::PbusDevice> proxy(new (&ac)
                                                platform_bus::PbusDevice(parent, zbi_vmo, iommu_handle));
    if (!ac.check()) {
        zx_handle_close(zbi_vmo);
        zx_handle_close(iommu_handle);
        return ZX_ERR_NO_MEMORY;
    }

    status = proxy->DdkAdd(name);
    if (status != ZX_OK) {
        return status;
    }
/*
    // This creates the "sys" device
    device_add_args_t self_args = {};
    self_args.version = DEVICE_ADD_ARGS_VERSION;
    self_args.name = name;
    self_args.ops = &sys_device_proto;
    self_args.flags = DEVICE_ADD_NON_BINDABLE;

    status = device_add(parent, &self_args, &parent);
    if (status != ZX_OK) {
        zx_handle_close(bus->dummy_iommu_handle);
        free(bus);
        return status;
    }

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
    __UNUSED auto* dummy = proxy.release();
    return ZX_OK;
}

PbusDevice::PbusDevice(zx_device_t* parent, zx_handle_t zbi_vmo, zx_handle_t iommu)
        : PbusDeviceType(parent), zbi_vmo_(zbi_vmo), iommu_handle_(iommu) {
    sync_completion_reset(&proto_completion_);
    resource_ = get_root_resource();

/*
    zx_status_t status = platform_bus_read_zbi(bus, zbi_vmo);
    zx_handle_close(zbi_vmo);
    if (status != ZX_OK) {
        free(bus);
        return status;
    }
*/

//FIXME    bus->iommu.ops = &platform_bus_default_iommu_ops;
//    bus->iommu.ctx = bus;
}


} // namespace platform_bus


zx_status_t platform_bus_create(void* ctx, zx_device_t* parent, const char* name,
                                const char* args, zx_handle_t zbi_vmo) {
    return platform_bus::PbusDevice::Create(parent, name, zbi_vmo);
}
