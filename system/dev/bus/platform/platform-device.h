// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/device.h>
#include <ddk/protocol/platform-bus.h>
#include <fbl/vector.h>

typedef struct platform_buss platform_bus_t;

// context structure for a platform device
typedef struct {
    zx_device_t* zxdev;
    platform_bus_t* bus;
    char name[ZX_DEVICE_NAME_MAX + 1];
    uint32_t flags;
    uint32_t vid;
    uint32_t pid;
    uint32_t did;
    serial_port_info_t serial_port_info;
    bool enabled;

    fbl::Vector<pbus_mmio_t> mmios;
    fbl::Vector<pbus_irq_t> irqs;
    fbl::Vector<pbus_gpio_t> gpios;
    fbl::Vector<pbus_i2c_channel_t> i2c_channels;
    fbl::Vector<pbus_clk_t> clks;
    fbl::Vector<pbus_bti_t> btis;
    fbl::Vector<pbus_metadata_t> metadata;
} platform_dev_t;

void platform_dev_free(platform_dev_t* dev);
zx_status_t platform_device_add(platform_bus_t* bus, const pbus_dev_t* dev, uint32_t flags);
zx_status_t platform_device_enable(platform_dev_t* dev, bool enable);
