// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "mt-usb.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/platform-defs.h>
#include <ddk/protocol/gpio.h>
#include <ddk/protocol/platform-device.h>
#include <ddk/protocol/platform-device-lib.h>
#include <hw/reg.h>
#include <fbl/algorithm.h>
#include <fbl/unique_ptr.h>

#include "mt-usb-regs.h"
#include "mt-usb-phy-regs.h"

namespace mt_usb {

zx_status_t MtUsb::Create(zx_device_t* parent) {
    pdev_protocol_t pdev;

    auto status = device_get_protocol(parent, ZX_PROTOCOL_PDEV, &pdev);
    if (status != ZX_OK) {
        return status;
    }

    fbl::AllocChecker ac;
    auto mt_usb = fbl::make_unique_checked<MtUsb>(&ac, parent, &pdev);
    if (!ac.check()) {
        return ZX_ERR_NO_MEMORY;
    }

    status = mt_usb->Init();
    if (status != ZX_OK) {
        return status;
    }

    // devmgr is now in charge of the device.
    __UNUSED auto* dummy = mt_usb.release();
    return ZX_OK;
}

zx_status_t MtUsb::Init() {
    auto status = pdev_get_bti(&pdev_, 0, bti_.reset_and_get_address());
    if (status != ZX_OK) {
        return status;
    }

    mmio_buffer_t mmio;
    status = pdev_map_mmio_buffer2(&pdev_, 0, ZX_CACHE_POLICY_UNCACHED_DEVICE, &mmio);
    if (status != ZX_OK) {
        return status;
    }
    usb_mmio_ = ddk::MmioBuffer(mmio);

    status = pdev_map_mmio_buffer2(&pdev_, 1, ZX_CACHE_POLICY_UNCACHED_DEVICE, &mmio);
    if (status != ZX_OK) {
        return status;
    }
    phy_mmio_ = ddk::MmioBuffer(mmio);

    status = pdev_map_interrupt(&pdev_, 0, irq_.reset_and_get_address());
    if (status != ZX_OK) {
        return status;
    }

    int rc = thrd_create_with_name(&irq_thread_,
                                   [](void* arg) -> int {
                                       return reinterpret_cast<MtUsb*>(arg)->IrqThread();
                                   },
                                   reinterpret_cast<void*>(this),
                                   "mt-usb-irq-thread");
    if (rc != thrd_success) {
        return ZX_ERR_INTERNAL;
    }

    status = DdkAdd("mt-usb");
    if (status != ZX_OK) {
        return status;
    }
    return ZX_OK;
}

void MtUsb::InitPhy() {
    volatile uint8_t* regs = static_cast<uint8_t*>(phy_mmio_->get());

    /*
     * swtich to USB function.
     * (system register, force ip into usb mode).
     */
    clr_bitsb(0x04, regs + 0x6b);
    clr_bitsb(0x01, regs + 0x6e);
    clr_bitsb(0x03, regs + 0x21);

    /* RG_USB20_BC11_SW_EN = 1'b0 */
    set_bitsw(0x04, regs + 0x22);
    clr_bitsb(0x80, regs + 0x1a);

    /* RG_USB20_DP_100K_EN = 1'b0 */
    /* RG_USB20_DP_100K_EN = 1'b0 */
    clr_bitsb(0x03, regs + 0x22);

    /*OTG enable*/
    set_bitsw(0x10, regs + 0x20);
    /* release force suspendm */
    clr_bitsb(0x04, regs + 0x6a);

    usleep(800);

    /* force enter device mode */
    clr_bitsb(0x10, regs + 0x6c);
    set_bitsb(0x2E, regs + 0x6c);
    set_bitsb(0x3E, regs + 0x6d);


    /* clean PUPD_BIST_EN */
    /* PUPD_BIST_EN = 1'b0 */
    /* PMIC will use it to detect charger type */
    clr_bitsb(0x10, regs + 0x1d);

    /* force_uart_en = 1'b0 */
    clr_bitsb(0x04, regs + 0x6b);
    /* RG_UART_EN = 1'b0 */
    clr_bitsb(0x01, regs + 0x6e);
    /* force_uart_en = 1'b0 */
    clr_bitsb(0x04, regs + 0x6a);

    clr_bitsb(0x03, regs + 0x21);

    clr_bitsb(0xf4, regs + 0x68);

    /* RG_DATAIN[3:0] = 4'b0000 */
    clr_bitsb(0x3c, regs + 0x69);

    clr_bitsb(0xba, regs + 0x6a);

    /* RG_USB20_BC11_SW_EN = 1'b0 */
    clr_bitsb(0x80, regs + 0x1a);
    /* RG_USB20_OTG_VBUSSCMP_EN = 1'b1 */
    set_bitsb(0x10, regs + 0x1a);

    usleep(800);

    /* force enter device mode */
    //USBPHY_CLR8(0x6c, 0x10);
    //USBPHY_SET8(0x6c, 0x2E);
    //USBPHY_SET8(0x6d, 0x3E);
}

int MtUsb::IrqThread() {
    InitPhy();

    // Enable HSDMA interrupts here?

    // Enable USB level 1 interrupts
//  usb_l1intm = (TX_INT_STATUS | RX_INT_STATUS | USBCOM_INT_STATUS | DMA_INT_STATUS);
//  writel(usb_l1intm, USB_L1INTM);


    while (true) {
        auto status = irq_.wait(nullptr);
        if (status == ZX_ERR_CANCELED) {
            return 0;
        } else if (status != ZX_OK) {
            zxlogf(ERROR, "%s: irq_.wait failed: %d\n", __func__, status);
            return -1;
        }
        zxlogf(INFO, "%s: got interrupt!\n", __func__);
    }
}

void MtUsb::DdkUnbind() {
    irq_.destroy();
    thrd_join(irq_thread_, nullptr);
}

void MtUsb::DdkRelease() {
    delete this;
}

 void MtUsb::UsbDciRequestQueue(usb_request_t* req) {
 printf("%s\n", __func__);
 }
 
 zx_status_t MtUsb::UsbDciSetInterface(const usb_dci_interface_t* interface) {
    memcpy(&dci_intf_, interface, sizeof(dci_intf_));
    return ZX_OK;
}

 zx_status_t MtUsb::UsbDciConfigEp(const usb_endpoint_descriptor_t* ep_desc, const
                            usb_ss_ep_comp_descriptor_t* ss_comp_desc) {
    return ZX_OK;
}

 zx_status_t MtUsb::UsbDciDisableEp(uint8_t ep_address) {
    return ZX_OK;
}

 zx_status_t MtUsb::UsbDciEpSetStall(uint8_t ep_address) {
    return ZX_OK;
}

 zx_status_t MtUsb::UsbDciEpClearStall(uint8_t ep_address) {
    return ZX_OK;
}

 zx_status_t MtUsb::UsbDciGetBti(zx_handle_t* out_bti) {
    *out_bti = bti_.get();
    return ZX_OK;
}

size_t MtUsb::UsbDciGetRequestSize() {
    return 0;
}

} // namespace mt_usb

zx_status_t mt_usb_bind(void* ctx, zx_device_t* parent) {
    return mt_usb::MtUsb::Create(parent);
}
