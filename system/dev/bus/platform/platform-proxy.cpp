// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>

#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/protocol/platform-bus.h>
#include <ddk/protocol/platform-device.h>
#include <ddk/protocol/clk.h>
#include <ddk/protocol/usb-mode-switch.h>
#include <fbl/unique_ptr.h>

#include "platform-proxy.h"

namespace platform_bus {

zx_status_t ProxyDevice::Rpc(pdev_req_t* req, uint32_t req_length, pdev_resp_t* resp,
                             uint32_t resp_length, zx_handle_t* in_handles,
                             uint32_t in_handle_count, zx_handle_t* out_handles,
                             uint32_t out_handle_count, uint32_t* out_data_received) {
    uint32_t resp_size, handle_count;

    zx_channel_call_args_t args = {
        .wr_bytes = req,
        .wr_handles = in_handles,
        .rd_bytes = resp,
        .rd_handles = out_handles,
        .wr_num_bytes = req_length,
        .wr_num_handles = in_handle_count,
        .rd_num_bytes = resp_length,
        .rd_num_handles = out_handle_count,
    };
    auto status = rpc_channel_.call(0, zx::time::infinite(), &args, &resp_size, &handle_count);
    if (status != ZX_OK) {
        return status;
    } else if (resp_size < sizeof(*resp)) {
        zxlogf(ERROR, "%s: platform_dev_rpc resp_size too short: %u\n", name_, resp_size);
        status = ZX_ERR_INTERNAL;
        goto fail;
    } else if (handle_count != out_handle_count) {
        zxlogf(ERROR, "%s: platform_dev_rpc handle count %u expected %u\n", name_,
               handle_count, out_handle_count);
        status = ZX_ERR_INTERNAL;
        goto fail;
    }

    status = resp->status;
    if (out_data_received) {
        *out_data_received = static_cast<uint32_t>(resp_size - sizeof(pdev_resp_t));
    }

fail:
    if (status != ZX_OK) {
        for (uint32_t i = 0; i < handle_count; i++) {
            zx_handle_close(out_handles[i]);
        }
    }
    return status;
}

zx_status_t ProxyDevice::SetUsbMode(usb_mode_t mode) {
    pdev_req_t req = {};
    req.op = PDEV_UMS_SET_MODE;
    req.usb_mode = mode;
    pdev_resp_t resp;

    return Rpc(&req, &resp);
}

zx_status_t ProxyDevice::GpioConfig(uint32_t index, uint32_t flags) {
    pdev_req_t req = {};
    req.op = PDEV_GPIO_CONFIG;
    req.index = index;
    req.gpio_flags = flags;
    pdev_resp_t resp;

    return Rpc(&req, &resp);
}

zx_status_t ProxyDevice::GpioSetAltFunction(uint32_t index, uint64_t function) {
    pdev_req_t req = {};
    req.op = PDEV_GPIO_SET_ALT_FUNCTION;
    req.index = index;
    req.gpio_alt_function = function;
    pdev_resp_t resp;

    return Rpc(&req, &resp);
}

zx_status_t ProxyDevice::GpioGetInterrupt(uint32_t index, uint32_t flags, zx_handle_t* out_handle) {
    pdev_req_t req = {};
    req.op = PDEV_GPIO_GET_INTERRUPT;
    req.index = index;
    req.flags = flags;
    pdev_resp_t resp;

    return Rpc(&req, sizeof(req), &resp, sizeof(resp), nullptr, 0, out_handle, 1, nullptr);
}

zx_status_t ProxyDevice::GpioSetPolarity(uint32_t index, uint32_t polarity) {
    pdev_req_t req = {};
    req.op = PDEV_GPIO_SET_POLARITY;
    req.index = index;
    req.flags = polarity;
    pdev_resp_t resp;

    return Rpc(&req, &resp);
}

zx_status_t ProxyDevice::GpioReleaseInterrupt(uint32_t index) {
    pdev_req_t req = {};
    req.op = PDEV_GPIO_RELEASE_INTERRUPT;
    req.index = index;
    pdev_resp_t resp;

    return Rpc(&req, &resp);
}

zx_status_t ProxyDevice::GpioRead(uint32_t index, uint8_t* out_value) {
    pdev_req_t req = {};
    req.op = PDEV_GPIO_READ;
    req.index = index;
    pdev_resp_t resp;

    auto status = Rpc(&req, &resp);

    if (status != ZX_OK) {
        return status;
    }
    *out_value = resp.gpio_value;
    return ZX_OK;
}

zx_status_t ProxyDevice::GpioWrite(uint32_t index, uint8_t value) {
    pdev_req_t req = {};
    req.op = PDEV_GPIO_WRITE;
    req.index = index;
    req.gpio_value = value;
    pdev_resp_t resp;

    return Rpc(&req, &resp);
}

zx_status_t ProxyDevice::ScpiGetSensorValue(uint32_t sensor_id, uint32_t* sensor_value) {
    pdev_req_t req = {};
    req.op = PDEV_SCPI_GET_SENSOR_VALUE;
    req.scpi_sensor_id = sensor_id;
    pdev_resp_t resp;

    auto status = Rpc(&req, &resp);
    if (status == ZX_OK) {
        *sensor_value = resp.scpi_sensor_value;
    }
    return status;
}

zx_status_t ProxyDevice::ScpiGetSensor(const char* name, uint32_t* sensor_id) {
    pdev_req_t req = {};
    req.op = PDEV_SCPI_GET_SENSOR;
    uint32_t max_len = sizeof(req.scpi_name);
    size_t len = strnlen(name, max_len);
    if (len == max_len) {
        return ZX_ERR_INVALID_ARGS;
    }
    memcpy(&req.scpi_name, name, len + 1);
    pdev_resp_t resp;

    auto status = Rpc(&req, &resp);
    if (status == ZX_OK) {
        *sensor_id = resp.scpi_sensor_id;
    }
    return status;
}

zx_status_t ProxyDevice::ScpiGetDvfsInfo(uint8_t power_domain, scpi_opp_t* opps) {
    pdev_req_t req = {};
    req.op = PDEV_SCPI_GET_DVFS_INFO;
    req.scpi_power_domain = power_domain;
    pdev_resp_t resp;

    auto status = Rpc(&req, &resp);
    if (status == ZX_OK) {
        memcpy(opps, &resp.scpi_opps, sizeof(scpi_opp_t));
    }
    return status;
}

zx_status_t ProxyDevice::ScpiGetDvfsIdx(uint8_t power_domain, uint16_t* idx) {
    pdev_req_t req = {};
    req.op = PDEV_SCPI_GET_DVFS_IDX;
    req.scpi_power_domain = power_domain;
    pdev_resp_t resp;

    auto status = Rpc(&req, &resp);
    if (status == ZX_OK) {
        *idx = resp.scpi_dvfs_idx;
    }
    return status;
}

zx_status_t ProxyDevice::ScpiSetDvfsIdx(uint8_t power_domain, uint16_t idx) {
    pdev_req_t req = {};
    req.op = PDEV_SCPI_SET_DVFS_IDX;
    req.index = idx;
    req.scpi_power_domain = power_domain;
    pdev_resp_t resp;
    return Rpc(&req, &resp);
}

zx_status_t ProxyDevice::CanvasConfig(zx_handle_t vmo, size_t offset, canvas_info_t* info,
                                      uint8_t* canvas_idx) {
    zx_status_t status = ZX_OK;
    pdev_resp_t resp;
    pdev_req_t req = {};
    req.op = PDEV_CANVAS_CONFIG;

    memcpy((void*)&req.canvas.info, info, sizeof(canvas_info_t));
    req.canvas.offset = offset;

    status = Rpc(&req, sizeof(req), &resp, sizeof(resp), &vmo, 1, nullptr, 0, nullptr);
    if (status == ZX_OK) {
        *canvas_idx = resp.canvas_idx;
    }
    return status;
}

zx_status_t ProxyDevice::CanvasFree(uint8_t canvas_idx) {
    pdev_req_t req = {};
    req.op = PDEV_CANCAS_FREE;
    req.canvas_idx = canvas_idx;
    pdev_resp_t resp;

    return Rpc(&req, &resp);
}

zx_status_t ProxyDevice::I2cGetMaxTransferSize(uint32_t index, size_t* out_size) {
    pdev_req_t req = {};
    req.op = PDEV_I2C_GET_MAX_TRANSFER;
    req.index = index;
    pdev_resp_t resp;

    auto status = Rpc(&req, &resp);
    if (status == ZX_OK) {
        *out_size = resp.i2c_max_transfer;
    }
    return status;
}

zx_status_t ProxyDevice::I2cTransact(uint32_t index, const void* write_buf, size_t write_length,
                                     size_t read_length, i2c_complete_cb complete_cb, void* cookie) {
    if (!read_length && !write_length) {
        return ZX_ERR_INVALID_ARGS;
    }
    if (write_length > PDEV_I2C_MAX_TRANSFER_SIZE ||
        read_length > PDEV_I2C_MAX_TRANSFER_SIZE) {
        return ZX_ERR_OUT_OF_RANGE;
    }
    struct {
        pdev_req_t req;
        uint8_t data[PDEV_I2C_MAX_TRANSFER_SIZE];
    } req = {
        .req = {
            .txid = 0,
            .op = PDEV_I2C_TRANSACT,
            .index = index,
            .i2c_txn = {
                .write_length = write_length,
                .read_length = read_length,
                .complete_cb = complete_cb,
                .cookie = cookie,
            },
        },
        .data = {},
    };
    struct {
        pdev_resp_t resp;
        uint8_t data[PDEV_I2C_MAX_TRANSFER_SIZE];
    } resp;

    if (write_length) {
        memcpy(req.data, write_buf, write_length);
    }
    uint32_t data_received;
    auto status = Rpc(&req.req, static_cast<uint32_t>(sizeof(req.req) + write_length),
                      &resp.resp, sizeof(resp), nullptr, 0, nullptr, 0, &data_received);
    if (status != ZX_OK) {
        return status;
    }

    // TODO(voydanoff) This proxying code actually implements i2c_transact synchronously
    // due to the fact that it is unsafe to respond asynchronously on the devmgr rxrpc channel.
    // In the future we may want to redo the plumbing to allow this to be truly asynchronous.
    if (data_received != read_length) {
        status = ZX_ERR_INTERNAL;
    } else {
        status = resp.resp.status;
    }
    if (complete_cb) {
        complete_cb(status, resp.data, resp.resp.i2c_txn.cookie);
    }

    return ZX_OK;
}

zx_status_t ProxyDevice::ClkEnable(uint32_t index) {
    pdev_req_t req = {};
    req.op = PDEV_CLK_ENABLE;
    req.index = index;
    pdev_resp_t resp;

    return Rpc(&req, &resp);
}

zx_status_t ProxyDevice::ClkDisable(uint32_t index) {
    pdev_req_t req = {};
    req.op = PDEV_CLK_DISABLE;
    req.index = index;
    pdev_resp_t resp;

    return Rpc(&req, &resp);
}

zx_status_t ProxyDevice::MapMmio(uint32_t index, uint32_t cache_policy, void** out_vaddr,
                                 size_t* out_size, zx_paddr_t* out_paddr, zx_handle_t* out_handle) {
    if (index > mmio_count_) {
        return ZX_ERR_INVALID_ARGS;
    }

    pbus_mmio_t* mmio = &mmios_[index];
    zx_paddr_t vmo_base = ROUNDDOWN(mmio->base, PAGE_SIZE);
    size_t vmo_size = ROUNDUP(mmio->base + mmio->length - vmo_base, PAGE_SIZE);
    zx_handle_t vmo_handle;

    zx_status_t status = zx_vmo_create_physical(mmio->resource, vmo_base, vmo_size,
                                                &vmo_handle);
    if (status != ZX_OK) {
        zxlogf(ERROR, "%s %s: creating vmo failed %d\n", name_, __FUNCTION__, status);
        return status;
    }

    char name[32];
    snprintf(name, sizeof(name), "%s mmio %u", name_, index);
    status = zx_object_set_property(vmo_handle, ZX_PROP_NAME, name, sizeof(name));
    if (status != ZX_OK) {
        zxlogf(ERROR, "%s %s: setting vmo name failed %d\n", name_, __FUNCTION__, status);
        goto fail;
    }

    status = zx_vmo_set_cache_policy(vmo_handle, cache_policy);
    if (status != ZX_OK) {
        zxlogf(ERROR, "%s %s: setting cache policy failed %d\n", name_, __FUNCTION__,status);
        goto fail;
    }

    uintptr_t virt;
    status = zx_vmar_map(zx_vmar_root_self(), 0, vmo_handle, 0, vmo_size,
                         ZX_VM_FLAG_PERM_READ | ZX_VM_FLAG_PERM_WRITE | ZX_VM_FLAG_MAP_RANGE,
                         &virt);
    if (status != ZX_OK) {
        zxlogf(ERROR, "%s %s: mapping vmar failed %d\n", name_, __FUNCTION__, status);
        goto fail;
    }

    *out_size = mmio->length;
    *out_vaddr = (void *)(virt + (mmio->base - vmo_base));
    *out_handle = vmo_handle;
    return ZX_OK;

fail:
    zx_handle_close(vmo_handle);
    return status;
}

zx_status_t ProxyDevice::MapInterrupt(uint32_t index, uint32_t flags, zx_handle_t* out_handle) {
    if (index > irq_count_) {
        return ZX_ERR_INVALID_ARGS;
    }

    pbus_irq_t* irq = &irqs_[index];
    zx_handle_t handle;
    zx_status_t status = zx_interrupt_create(irq->resource, irq->irq, flags, &handle);
    if (status != ZX_OK) {
        zxlogf(ERROR, "%s %s: creating interrupt failed: %d\n", name_, __FUNCTION__, status);
        return status;
    }

    *out_handle = handle;
    return ZX_OK;
}

zx_status_t ProxyDevice::GetBti(uint32_t index, zx_handle_t* out_handle) {
    pdev_req_t req = {};
    req.op = PDEV_GET_BTI;
    req.index = index;
    pdev_resp_t resp;

    return Rpc(&req, sizeof(req), &resp, sizeof(resp), nullptr, 0, out_handle, 1, nullptr);
}

zx_status_t ProxyDevice::GetDeviceInfo(pdev_device_info_t* out_info) {
    pdev_req_t req = {};
    req.op = PDEV_GET_DEVICE_INFO;
    pdev_resp_t resp;

    auto status = Rpc(&req, &resp);
    if (status != ZX_OK) {
        return status;
    }
    memcpy(out_info, &resp.info, sizeof(*out_info));
    return ZX_OK;
}

zx_status_t ProxyDevice::Init() {
    pdev_device_info_t info;
    auto status = GetDeviceInfo(&info);
    if (status != ZX_OK) {
        return status;
    }

    // Request mmio/irq metadata and resource handles from the platform. If
    // this driver dies/exits they will be freed back to the address space
    // allocators when handles are reaped in process teardown.
    mmio_count_ = info.mmio_count;
    irq_count_ = info.irq_count;
    memcpy(name_, info.name, sizeof(name_));

    if (mmio_count_) {
        mmios_ = static_cast<pbus_mmio_t*>(malloc(sizeof(pbus_mmio_t) * mmio_count_));
        if (!mmios_) {
            return status;
        }

        for (uint32_t i = 0; i < mmio_count_; i++) {
            pbus_mmio_t* mmio = &mmios_[i];
            pdev_req_t req = {};
            pdev_resp_t resp;
            zx_handle_t rsrc_handle;
            
            req.op = PDEV_GET_MMIO;
            req.index = i;
            auto status = Rpc(&req, sizeof(req), &resp, sizeof(resp), nullptr, 0, &rsrc_handle, 1,
                              nullptr);
            if (status != ZX_OK) {
                return status;
            }

            mmio->base = resp.mmio.paddr;
            mmio->length = resp.mmio.length;
            mmio->resource = rsrc_handle;

            zxlogf(SPEW, "%s: received MMIO %u (base %#lx length %#lx handle %#x)\n",
                    name_, i, mmio->base, mmio->length, mmio->resource);
        }
    }

    if (irq_count_) {
        irqs_ = static_cast<pbus_irq_t*>(malloc(sizeof(pbus_irq_t) * irq_count_));
        if (!irqs_) {
            return status;
        }

        for (uint32_t i = 0; i < irq_count_; i++) {
            pbus_irq_t* irq = &irqs_[i];
            pdev_req_t req = {};
            pdev_resp_t resp;
            zx_handle_t rsrc_handle;
        
            req.op = PDEV_GET_INTERRUPT;
            req.index = i;
            auto status = Rpc(&req, sizeof(req), &resp, sizeof(resp), nullptr, 0, &rsrc_handle, 1,
                              nullptr);
            if (status != ZX_OK) {
                return status;
            }

            irq->irq = resp.irq;
            irq->resource = rsrc_handle;
            return ZX_OK;

            zxlogf(SPEW, "%s: received IRQ %u (irq %#x handle %#x)\n",
                    name_, i, irq->irq, irq->resource);
        }
    }

    return ZX_OK;
}

zx_status_t ProxyDevice::Create(zx_device_t* parent, const char* name, zx_handle_t rpc_channel) {
    fbl::AllocChecker ac;
    fbl::unique_ptr<platform_bus::ProxyDevice> proxy(new (&ac)
                                                     platform_bus::ProxyDevice(parent, rpc_channel));
    if (!ac.check()) {
        return ZX_ERR_NO_MEMORY;
    }

    auto status = proxy->DdkAdd(name, DEVICE_ADD_INVISIBLE);
    if (status != ZX_OK) {
        return status;
    }
    status = proxy->Init();
    if (status != ZX_OK) {
        proxy->DdkRemove();
        return status;
    }
    proxy->DdkMakeVisible();

    // devmgr is now in charge of the device.
    __UNUSED auto* dummy = proxy.release();
    return ZX_OK;
}

zx_status_t ProxyDevice::DdkGetProtocol(uint32_t proto_id, void* out) {
    switch (proto_id) {
    case ZX_PROTOCOL_PLATFORM_DEV: {
        auto proto = static_cast<platform_device_protocol_t*>(out);
        proto->ctx = this;
        proto->ops = &pdev_proto_ops_;
        return ZX_OK;
    }
    case ZX_PROTOCOL_USB_MODE_SWITCH: {
        auto proto = static_cast<usb_mode_switch_protocol_t*>(out);
        proto->ctx = this;
        proto->ops = &usb_mode_switch_proto_ops_;
        return ZX_OK;
    }
    case ZX_PROTOCOL_GPIO: {
        auto proto = static_cast<gpio_protocol_t*>(out);
        proto->ctx = this;
        proto->ops = &gpio_proto_ops_;
        return ZX_OK;
    }
    case ZX_PROTOCOL_I2C: {
        auto proto = static_cast<i2c_protocol_t*>(out);
        proto->ctx = this;
        proto->ops = &i2c_proto_ops_;
        return ZX_OK;
    }
    case ZX_PROTOCOL_CLK: {
        auto proto = static_cast<clk_protocol_t*>(out);
        proto->ctx = this;
        proto->ops = &clk_proto_ops_;
        return ZX_OK;
    }
    case ZX_PROTOCOL_SCPI: {
        auto proto = static_cast<scpi_protocol_t*>(out);
        proto->ctx = this;
        proto->ops = &scpi_proto_ops_;
        return ZX_OK;
    }
    case ZX_PROTOCOL_CANVAS: {
        auto proto = static_cast<canvas_protocol_t*>(out);
        proto->ctx = this;
        proto->ops = &canvas_proto_ops_;
        return ZX_OK;
    }
    default:
        return ZX_ERR_NOT_SUPPORTED;
    }
}

void ProxyDevice::DdkRelease() {
    delete this;
}

ProxyDevice::~ProxyDevice() {
    // FIXME close resource handles too
    free(mmios_);
    free(irqs_);
}

} // namespace platform_bus

zx_status_t platform_proxy_create(void* ctx, zx_device_t* parent, const char* name,
                                  const char* args, zx_handle_t rpc_channel) {
    return platform_bus::ProxyDevice::Create(parent, name, rpc_channel);
}
