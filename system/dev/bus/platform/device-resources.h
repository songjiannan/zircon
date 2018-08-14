// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/protocol/platform-bus.h>
#include <fbl/vector.h>

class DeviceResources {
public:
    zx_status_t Init(const pbus_dev_t* pdev);
    size_t DeviceCount() const;
    void BuildDeviceIndex(fbl::Vector<DeviceResources*>& index);

    inline uint32_t device_id() const { return device_id_; }

    inline const pbus_mmio_t& mmio(size_t i) const { return mmios_[i]; }
    inline const pbus_irq_t& irq(size_t i) const { return irqs_[i]; }
    inline const pbus_gpio_t& gpio(size_t i) const { return gpios_[i]; }
    inline const pbus_i2c_channel_t& i2c_channel(size_t i) const { return i2c_channels_[i]; }
    inline const pbus_clk_t& clk(size_t i) const { return clks_[i]; }
    inline const pbus_bti_t& bti(size_t i) const { return btis_[i]; }
    inline const pbus_metadata_t& metadata(size_t i) const { return metadata_[i]; }

    inline size_t mmio_count() const { return mmios_.size(); }
    inline size_t irq_count() const { return irqs_.size(); }
    inline size_t gpio_count() const { return gpios_.size(); }
    inline size_t i2c_channel_count() const { return i2c_channels_.size(); }
    inline size_t clk_count() const { return clks_.size(); }
    inline size_t bti_count() const { return btis_.size(); }
    inline size_t metadata_count() const { return metadata_.size(); }
    inline size_t child_count() const { return children_.size(); }

private:
    // Device ID is index of this device in PlatformDevice::device_index_ 
    uint32_t device_id_;

    // Resources for this device.
    fbl::Vector<pbus_mmio_t> mmios_;
    fbl::Vector<pbus_irq_t> irqs_;
    fbl::Vector<pbus_gpio_t> gpios_;
    fbl::Vector<pbus_i2c_channel_t> i2c_channels_;
    fbl::Vector<pbus_clk_t> clks_;
    fbl::Vector<pbus_bti_t> btis_;
    fbl::Vector<pbus_metadata_t> metadata_;

    // Resources for children of this device.
    fbl::Vector<DeviceResources> children_;
};
