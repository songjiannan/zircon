// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <stdint.h>
#include <threads.h>
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
#include <ddktl/protocol/canvas.h>
#include <ddktl/protocol/clk.h>
#include <ddktl/protocol/gpio.h>
#include <ddktl/protocol/i2c-impl.h>
#include <ddktl/protocol/iommu.h>
#include <ddktl/protocol/platform-bus.h>
#include <ddktl/protocol/scpi.h>
#include <ddktl/protocol/usb-mode-switch.h>
#include <lib/zx/vmo.h>
#include <fbl/unique_ptr.h>

#include "platform-device.h"

typedef struct pdev_req pdev_req_t;

namespace platform_bus {

typedef struct platform_i2c_bus {
    i2c_impl_protocol_t i2c;
    uint32_t bus_id;
    size_t max_transfer;

    list_node_t queued_txns;
    list_node_t free_txns;
    sync_completion_t txn_signal;

    thrd_t thread;
    mtx_t lock;
} platform_i2c_bus_t;

class PlatformBus;
using PlatformBusType = ddk::Device<PlatformBus, ddk::GetProtocolable>;

class PlatformBus : public PlatformBusType, public ddk::PbusProtocol<PlatformBus>,
                    ddk::IommuProtocol<PlatformBus> {
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
    // FIXME do something with this?
    zx_status_t GetBti(uint32_t iommu_index, uint32_t bti_id, zx_handle_t* out_handle);

    zx_handle_t GetResource() const { return get_root_resource(); }

    fbl::unique_ptr<ddk::CanvasProtocolProxy> canvas_;
    fbl::unique_ptr<ddk::ClkProtocolProxy> clk_;
    fbl::unique_ptr<ddk::GpioProtocolProxy> gpio_;
    fbl::unique_ptr<ddk::IommuProtocolProxy> iommu_;
    fbl::unique_ptr<ddk::I2cImplProtocolProxy> i2c_impl_;
    fbl::unique_ptr<ddk::ScpiProtocolProxy> scpi_;
    fbl::unique_ptr<ddk::UmsProtocolProxy> ums_;

//FIXME private:
    explicit PlatformBus(zx_device_t* parent, zx_handle_t iommu);

    zx_status_t ReadZbi(zx_handle_t vmo);

    zx_status_t I2cInit(i2c_impl_protocol_t* i2c);
    zx_status_t I2cTransact(pdev_req_t* req, pbus_i2c_channel_t* channel,
                                  const void* write_buf, zx_handle_t channel_handle);

    DISALLOW_COPY_ASSIGN_AND_MOVE(PlatformBus);

    zbi_platform_id_t platform_id_;
    uint8_t* metadata_;   // metadata extracted from ZBI
    size_t metadata_size_;

    fbl::Vector<platform_dev_t*> devices_;
    fbl::Vector<platform_i2c_bus_t> i2c_buses_;

    zx_handle_t iommu_handle_;

    sync_completion_t proto_completion_;

};

} // namespace platform_bus

/*
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
*/

// platform-bus.c
zx_status_t platform_bus_get_protocol(void* ctx, uint32_t proto_id, void* protocol);


__BEGIN_CDECLS
zx_status_t platform_bus_create(void* ctx, zx_device_t* parent, const char* name,
                                const char* args, zx_handle_t rpc_channel);
__END_CDECLS
