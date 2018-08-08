// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "device-resources.h"

zx_status_t DeviceResources::Init(const pbus_dev_t* pdev) {
    fbl::AllocChecker ac;

    if (pdev->mmio_count) {
        mmios_.reserve(pdev->mmio_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->mmio_count; i++) {
            mmios_.push_back(pdev->mmios[i]);
        }
    }
    if (pdev->irq_count) {
        irqs_.reserve(pdev->irq_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->irq_count; i++) {
            irqs_.push_back(pdev->irqs[i]);
        }
    }
    if (pdev->gpio_count) {
        gpios_.reserve(pdev->gpio_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->gpio_count; i++) {
            gpios_.push_back(pdev->gpios[i]);
        }
    }
    if (pdev->i2c_channel_count) {
        i2c_channels_.reserve(pdev->i2c_channel_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->i2c_channel_count; i++) {
            i2c_channels_.push_back(pdev->i2c_channels[i]);
        }
    }
    if (pdev->clk_count) {
        clks_.reserve(pdev->clk_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->clk_count; i++) {
            clks_.push_back(pdev->clks[i]);
        }
    }
    if (pdev->bti_count) {
        btis_.reserve(pdev->bti_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->bti_count; i++) {
            btis_.push_back(pdev->btis[i]);
        }
    }
    if (pdev->metadata_count) {
        metadata_.reserve(pdev->metadata_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->metadata_count; i++) {
            metadata_.push_back(pdev->metadata[i]);
        }
    }
    if (pdev->child_count) {
        children_.reserve(pdev->child_count, &ac);
        if (!ac.check()) {
            return ZX_ERR_NO_MEMORY;
        }
        for (uint32_t i = 0; i < pdev->child_count; i++) {
            DeviceResources dr;
            auto status = dr.Init(&pdev->children[i]);
            if (status != ZX_OK) {
                return status;
            }
            children_.push_back(fbl::move(dr));
        }
    }

    return ZX_OK;
}

size_t DeviceResources::DeviceCount() const {
    size_t result = 1;
    for (auto& dr : children_) {
        result += dr.DeviceCount();
    }
    return result;
}

void DeviceResources::BuildDeviceIndex(fbl::Vector<DeviceResources*>& index) {
    index.push_back(this);
    for (auto& dr : children_) {
        dr.BuildDeviceIndex(index);
    }
}
