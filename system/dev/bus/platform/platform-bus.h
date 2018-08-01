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

#include "platform-device.h"
#include "platform-i2c.h"


// context structure for the platform bus
typedef struct platform_bus {
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
    uint8_t* metadata;   // metadata extracted from ZBI
    size_t metadata_size;

    fbl::Vector<platform_dev_t> devices;
    fbl::Vector<platform_i2c_bus_t> i2c_buses;

    zx_handle_t dummy_iommu_handle;

    sync_completion_t proto_completion;
} platform_bus_t;

// platform-bus.c
zx_status_t platform_bus_get_protocol(void* ctx, uint32_t proto_id, void* protocol);

__BEGIN_CDECLS
zx_status_t platform_bus_create(void* ctx, zx_device_t* parent, const char* name,
                                const char* args, zx_handle_t rpc_channel);
__END_CDECLS
