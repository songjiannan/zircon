// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/device.h>
#include <ddk/protocol/platform-bus.h>

typedef struct platform_bus platform_bus_t;

// context structure for a platform device
typedef struct {
    zx_device_t* zxdev;
    platform_bus_t* bus;
    list_node_t node;
    char name[ZX_DEVICE_NAME_MAX + 1];
    uint32_t flags;
    uint32_t vid;
    uint32_t pid;
    uint32_t did;
    serial_port_info_t serial_port_info;
    bool enabled;

    pbus_mmio_t* mmios;
    pbus_irq_t* irqs;
    pbus_gpio_t* gpios;
    pbus_i2c_channel_t* i2c_channels;
    pbus_clk_t* clks;
    pbus_bti_t* btis;
    pbus_metadata_t* metadata;
    uint32_t mmio_count;
    uint32_t irq_count;
    uint32_t gpio_count;
    uint32_t i2c_channel_count;
    uint32_t clk_count;
    uint32_t bti_count;
    uint32_t metadata_count;
} platform_dev_t;

void platform_dev_free(platform_dev_t* dev);
zx_status_t platform_device_add(platform_bus_t* bus, const pbus_dev_t* dev, uint32_t flags);
zx_status_t platform_device_enable(platform_dev_t* dev, bool enable);
