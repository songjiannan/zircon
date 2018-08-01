// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <stdint.h>
#include <ddk/device.h>
#include <ddk/protocol/clk.h>
#include <ddk/protocol/gpio.h>
#include <ddk/protocol/canvas.h>
#include <ddk/protocol/i2c.h>
#include <ddk/protocol/iommu.h>
#include <ddk/protocol/platform-bus.h>
#include <ddk/protocol/platform-device.h>
#include <ddk/protocol/usb-mode-switch.h>
#include <ddk/protocol/mailbox.h>
#include <ddk/protocol/scpi.h>
#include <fbl/vector.h>
#include <lib/sync/completion.h>
#include <zircon/boot/image.h>
#include <zircon/types.h>


#include <ddktl/device.h>
#include <ddktl/protocol/iommu.h>
#include <ddktl/protocol/platform-bus.h>
#include <lib/zx/vmo.h>
#include <fbl/macros.h>


#include "platform-device.h"
#include "platform-i2c.h"

namespace platform_bus {

class PbusDevice;
using PbusDeviceType = ddk::Device<PbusDevice, ddk::GetProtocolable>;

class PbusDevice : public PbusDeviceType, public ddk::PbusProtocol<PbusDevice>,
                   ddk::IommuProtocol<PbusDevice> {
public:
    static zx_status_t Create(zx_device_t* parent, const char* name, zx_handle_t zbi_vmo);

    // device protocol implementation
    zx_status_t DdkGetProtocol(uint32_t proto_id, void* out);
    void DdkRelease();

    // platform bus protocol implementation
    zx_status_t SetProtocol(uint32_t proto_id, void* protocol);
    zx_status_t WaitProtocol(uint32_t proto_id);
    zx_status_t DeviceAdd(const pbus_dev_t* dev, uint32_t flags);
    zx_status_t DeviceEnable(uint32_t vid, uint32_t pid, uint32_t did, bool enable);
    const char* GetBoardName();

    // IOMMU protocol implementation
    zx_status_t GetBti(uint32_t iommu_index, uint32_t bti_id, zx_handle_t* out_handle);

/*
    // platform device protocol implementation
    zx_status_t MapMmio(uint32_t index, uint32_t cache_policy, void** out_vaddr, size_t* out_size,
                        zx_paddr_t* out_paddr, zx_handle_t* out_handle);
    zx_status_t MapInterrupt(uint32_t index, uint32_t flags, zx_handle_t* out_handle);
    zx_status_t GetBti(uint32_t index, zx_handle_t* out_handle);
    zx_status_t GetDeviceInfo(pdev_device_info_t* out_info);

    // canvas protocol implementation
    zx_status_t CanvasConfig(zx_handle_t vmo, size_t offset, canvas_info_t* info, uint8_t* canvas_idx);
    zx_status_t CanvasFree(uint8_t canvas_idx);

    // clock protocol implementation
    zx_status_t ClkEnable(uint32_t index);
    zx_status_t ClkDisable(uint32_t index);

    // GPIO protocol implementation
    zx_status_t GpioConfig(uint32_t index, uint32_t flags);
    zx_status_t GpioSetAltFunction(uint32_t index, uint64_t function);
    zx_status_t GpioRead(uint32_t index, uint8_t* out_value);
    zx_status_t GpioWrite(uint32_t index, uint8_t value);
    zx_status_t GpioGetInterrupt(uint32_t index, uint32_t flags, zx_handle_t* out_handle);
    zx_status_t GpioReleaseInterrupt(uint32_t index);
    zx_status_t GpioSetPolarity(uint32_t index, uint32_t polarity);

    // I2C protocol implementation
    zx_status_t I2cTransact(uint32_t index, const void* write_buf, size_t write_length,
                            size_t read_length, i2c_complete_cb complete_cb, void* cookie);
    zx_status_t I2cGetMaxTransferSize(uint32_t index, size_t* out_size);

    // SCPI protocol implementation
    zx_status_t ScpiGetSensor(const char* name, uint32_t* sensor_id);
    zx_status_t ScpiGetSensorValue(uint32_t sensor_id, uint32_t* sensor_value);
    zx_status_t ScpiGetDvfsInfo(uint8_t power_domain, scpi_opp_t* opps);
    zx_status_t ScpiGetDvfsIdx(uint8_t power_domain, uint16_t* idx);
    zx_status_t ScpiSetDvfsIdx(uint8_t power_domain, uint16_t idx);

    // USB mode switch protocol implementation
    zx_status_t SetUsbMode(usb_mode_t mode);
*/

private:
    explicit PbusDevice(zx_device_t* parent, zx_handle_t zbi_vmo, zx_handle_t iommu);

    DISALLOW_COPY_ASSIGN_AND_MOVE(PbusDevice);

    zx::vmo zbi_vmo_;

    usb_mode_switch_protocol_t ums_;
    gpio_protocol_t gpio_;
    mailbox_protocol_t mailbox_;
    scpi_protocol_t scpi_;
    i2c_impl_protocol_t i2c_;
    clk_protocol_t clk_;
    canvas_protocol_t canvas_;
    zx_handle_t resource_;   // root resource for platform bus
    zbi_platform_id_t platform_id_;
    uint8_t* metadata_;   // metadata extracted from ZBI
    size_t metadata_size_;

    fbl::Vector<platform_dev_t*> devices_;
    fbl::Vector<platform_i2c_bus_t> i2c_buses_;

    zx_handle_t iommu_handle_;

    sync_completion_t proto_completion_;

};

} // namespace platform_bus


// context structure for the platform bus
typedef struct platform_buss {
    zx_device_t* zxdev;
    usb_mode_switch_protocol_t ums;
    gpio_protocol_t gpio;
    mailbox_protocol_t mailbox;
    scpi_protocol_t scpi;
    i2c_impl_protocol_t i2c;
    clk_protocol_t clk;
    iommu_protocol_t iommu;
    canvas_protocol_t canvas;
    zx_handle_t resource;   // root resource for platform bus
    zbi_platform_id_t platform_id;
    zx_handle_t dummy_iommu_handle;
    sync_completion_t proto_completion;

    // metadata extracted from ZBI
    uint8_t* metadata;
    size_t metadata_size;

    fbl::Vector<platform_dev_t*> devices;
    fbl::Vector<platform_i2c_bus_t> i2c_buses;
} platform_bus_t;

// platform-bus.c
zx_status_t platform_bus_get_protocol(void* ctx, uint32_t proto_id, void* protocol);


__BEGIN_CDECLS
zx_status_t platform_bus_create(void* ctx, zx_device_t* parent, const char* name,
                                const char* args, zx_handle_t rpc_channel);
__END_CDECLS
