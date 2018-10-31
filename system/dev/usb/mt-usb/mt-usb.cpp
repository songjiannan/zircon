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
#include <fbl/auto_lock.h>
#include <fbl/unique_ptr.h>
#include <usb/usb-request.h>
#include <zircon/assert.h>

#include "mt-usb-regs.h"
#include "mt-usb-phy-regs.h"

namespace mt_usb {

uint8_t MtUsb::EpAddressToIndex(uint8_t addr) {
    // map 0x01 -> 0, 0x81 -> 2, 0x02 -> 3, 0x82 -> 4, ...
    if (addr & USB_ENDPOINT_DIR_MASK) {
        return static_cast<uint8_t>(2 * (addr & USB_ENDPOINT_NUM_MASK));
    } else {
        return static_cast<uint8_t>(2 * (addr & USB_ENDPOINT_NUM_MASK) - 1);
    }
}

zx_status_t MtUsb::AllocDmaChannel(Endpoint* ep) {
    for (uint8_t i = 0; i < DMA_CHANNEL_COUNT; i++) {
        if ((dma_channel_alloc_ & (1 << i)) == 0) {
            ep->dma_channel = i;
            dma_channel_alloc_ |= (1 << i);
            return ZX_OK;
        }
    }
    return ZX_ERR_NO_RESOURCES;
}

void MtUsb::ReleaseDmaChannel(Endpoint* ep) {
    if (ep->dma_channel < DMA_CHANNEL_COUNT) {
        dma_channel_alloc_ &= ~(1 << ep->dma_channel);
    }
    ep->dma_channel = DMA_CHANNEL_INVALID;
}

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

void MtUsb::InitEndpoints() {
    for (uint8_t i = 0; i < countof(eps_); i++) {
        auto* ep = &eps_[i];
        ep->direction = (i & 1 ? EP_IN : EP_OUT);
        ep->ep_num = static_cast<uint8_t>(i / 2 + 1);
        list_initialize(&ep->queued_reqs);

        fbl::AutoLock lock(&ep->lock);
        ep->current_req = nullptr;
        ep->dma_channel = DMA_CHANNEL_INVALID;
    }
}

zx_status_t MtUsb::Init() {
    InitEndpoints();

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

    status = DdkAdd("mt-usb");
    if (status != ZX_OK) {
        return status;
    }
    return ZX_OK;
}

void MtUsb::InitPhy() {
    volatile uint8_t* regs = static_cast<uint8_t*>(phy_mmio_->get());
printf("PHY regs: %p\n", regs);
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

void MtUsb::HandleSuspend() {
}

void MtUsb::HandleReset() {
    auto* mmio = usb_mmio();

    FADDR::Get()
        .FromValue(0)
        .set_function_address(0)
        .WriteTo(mmio);
    address_ = 0;
    set_address_ = false;
    configuration_ = 0;

/* ???
    INTRTXE::Get()
        .FromValue(0)
        .WriteTo(mmio);
    INTRRXE::Get()
        .FromValue(0)
        .WriteTo(mmio);
*/

    BUSPERF3::Get()
        .FromValue(0)
        .set_ep_swrst(1)
        .set_disusbreset(1)
        .WriteTo(mmio);

    // TODO flush fifos

//    POWER_PERI::Get().ReadFrom(mmio).Print();

    if (POWER_PERI::Get().ReadFrom(mmio).hsmode()) {
        ep0_max_packet_ = 64;
    } else {
        ep0_max_packet_ = 8;
    }

//    INDEX::Get().FromValue(0).WriteTo(mmio);

printf("ep0_max_packet_ %u\n", ep0_max_packet_);
    TXMAP::Get(0)
        .FromValue(0)
        .set_maximum_payload_transaction(ep0_max_packet_)
        .WriteTo(mmio);
    RXMAP::Get(0)
        .FromValue(0)
        .set_maximum_payload_transaction(ep0_max_packet_)
        .WriteTo(mmio);

    // TODO mt_udc_rxtxmap_recover()
}

void MtUsb::HandleEp0() {
    auto* mmio = usb_mmio();

//    INDEX::Get().FromValue(0).WriteTo(mmio);

    // Loop until we explicitly return from this function.
    // This allows us to handle multiple state transitions at once when appropriate.
    while (true) {
        auto csr0 = CSR0_PERI::Get().ReadFrom(mmio);

//printf("HandleEp0 csr0:\n");
//csr0.Print();

        if (csr0.setupend()) {
printf("SETUPEND\n");
            csr0.set_serviced_setupend(1);
            csr0.WriteTo(mmio);
            csr0.ReadFrom(mmio);
            ep0_state_ = EP0_IDLE;
        }

        switch (ep0_state_) {
        case EP0_IDLE: {
printf("case EP0_IDLE\n");

            if (set_address_) {
                FADDR::Get()
                    .FromValue(0)
                    .set_function_address(address_)
                    .WriteTo(mmio);
                set_address_ = false;
            }

            if (!csr0.rxpktrdy()) {
                return;
            }

            usb_setup_t* setup = &cur_setup_;
            size_t actual;
            FifoRead(0, setup, sizeof(*setup), &actual);
            if (actual != sizeof(cur_setup_)) {
                zxlogf(ERROR, "%s: setup read only read %zu bytes\n", __func__, actual);
                return;
            }
            zxlogf(INFO, "SETUP bmRequestType %x bRequest %u wValue %u wIndex %u wLength %u\n",
                   setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex,
                   setup->wLength);
            
            if (setup->wLength > 0 && (setup->bmRequestType & USB_DIR_MASK) == USB_DIR_OUT) {
                ep0_state_ = EP0_READ;
                ep0_data_offset_ = 0;
                ep0_data_length_ = setup->wLength;

// TODO update CSR0_PERI here?
                break;
            } else {
                size_t actual = 0;

                // Handle some special setup requests in this driver.
                if (setup->bmRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE) &&
                    setup->bRequest == USB_REQ_SET_ADDRESS) {
                    zxlogf(TRACE, "SET_ADDRESS %u\n", setup->wValue);
                    address_ = static_cast<uint8_t>(setup->wValue);
                    set_address_ = true;
                } else if (setup->bmRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE) &&
                           setup->bRequest == USB_REQ_SET_CONFIGURATION) {
                    configuration_ = 0;
                    auto status = dci_intf_->Control(setup, nullptr, 0, nullptr, 0, &actual);
                    if (status != ZX_OK) {
                        // TODO error handling
                        zxlogf(ERROR, "%s: USB_REQ_SET_CONFIGURATION Control returned %d\n", __func__, status);
                        return;
                    }
                    configuration_ = static_cast<uint8_t>(setup->wValue);
                    if (configuration_) {
                        StartEndpoints();
                    }
                } else {
                    auto status = dci_intf_->Control(setup, nullptr, 0, ep0_data_, sizeof(ep0_data_), &actual);
                    if (status != ZX_OK) {
                        // TODO error handling
                        zxlogf(ERROR, "%s: Control returned %d\n", __func__, status);
                        return;
                    }
                }

                if (actual > 0) {
                    ep0_state_ = EP0_WRITE;
                    ep0_data_offset_ = 0;
                    ep0_data_length_ = actual;
                } else {
                    ep0_state_ = EP0_IDLE;
                }

                csr0.ReadFrom(mmio);
                csr0.set_serviced_rxpktrdy(1);
                if (actual == 0) {
                    csr0.set_dataend(1);
                }
                csr0.WriteTo(mmio);
                csr0.WriteTo(mmio); // ???

// ????
if (ep0_state_ == EP0_IDLE) {
    return;
}
            }
            break;
        }
        case EP0_READ:
printf("case EP0_READ\n");
            break;
        case EP0_WRITE: {
printf("case EP0_WRITE\n");
            if (csr0.txpktrdy()) {
printf("EP0_WRITE not ready\n");
                return;
            }
            size_t count = ep0_data_length_ - ep0_data_offset_;
            if (count > ep0_max_packet_) {
                count = ep0_max_packet_;
            }
printf("FifoWrite %zu\n", count);
            FifoWrite(0, ep0_data_ + ep0_data_offset_, count);
            ep0_data_offset_ += count;
            if (ep0_data_offset_ == ep0_data_length_) {
printf("flush dataend | txpktrdy\n");
                csr0.set_dataend(1)
                    .set_txpktrdy(1)
                    .WriteTo(mmio);   
                ep0_state_ = EP0_IDLE;
            } else {
                csr0.set_txpktrdy(1)
                    .WriteTo(mmio);
            }
            break;
        }
        }
    }
}

void MtUsb::HandleDma() {
    auto* mmio = usb_mmio();

printf("HandleDma\n");
    auto dma_intr = DMA_INTR::Get().ReadFrom(mmio).WriteTo(mmio);
    dma_intr.Print();
    auto status = dma_intr.status();
    
    for (uint32_t channel = 0; channel < DMA_CHANNEL_COUNT; channel++) {
        if (status & (1 << channel)) {
printf("DMA interrupt for channel %u\n", channel);
        
            auto dma_cntl = DMA_CNTL::Get(channel).ReadFrom(mmio);
            if (dma_cntl.dma_abort()) {
                printf("XXXXX dma_abort for channel %u\n", channel);
            }
        }
    }
}

void MtUsb::EpQueueNextLocked(Endpoint* ep) {
    auto* mmio = usb_mmio();
    usb_request_t* req;

printf("XXXXX EpQueueNextLocked %u\n", ep->ep_num);

    if (ep->current_req == nullptr &&
        (req = list_remove_head_type(&ep->queued_reqs, usb_request_t, node)) != nullptr) {
        ep->current_req = req;
        if (ep->direction == EP_IN) {
            usb_request_cache_flush(req, 0, req->header.length);
        } else {
            usb_request_cache_flush_invalidate(req, 0, req->header.length);
        }

        phys_iter_t iter;
        zx_paddr_t phys;
        usb_request_physmap(req, bti_.get());
        usb_request_phys_iter_init(&iter, req, PAGE_SIZE);
        usb_request_phys_iter_next(&iter, &phys);
        // This controller only supports 32-bit addresses
        ZX_DEBUG_ASSERT(phys < UINT32_MAX);

        // TODO(voydanoff) scatter/gather support
        size_t length = req->header.length;
        ZX_DEBUG_ASSERT(length <= PAGE_SIZE);
//        bool send_zlp = req->header.send_zlp && (length % ep->max_packet_size) == 0;

        uint32_t dma_channel = ep->dma_channel;

printf("XXXXX START DMA channel %u ep_num %u length %zu\n", dma_channel, ep->ep_num, length);

        DMA_ADDR::Get(dma_channel)
            .FromValue(0)
            .set_addr(static_cast<uint32_t>(phys))
            .WriteTo(mmio);
        DMA_COUNT::Get(dma_channel)
            .FromValue(0)
            .set_count(static_cast<uint32_t>(length))
            .WriteTo(mmio);
        DMA_CNTL::Get(dma_channel)
            .FromValue(0)
            .set_burst_mode(3)
            .set_endpoint(ep->ep_num)
            .set_inten(1)
            .set_dir(ep->direction == EP_IN ? 1 : 0)
            .set_enable(1)
            .WriteTo(mmio).Print();

//while (DMA_CNTL::Get(dma_channel).ReadFrom(mmio).enable()) {
//printf("XXXXX spin\n");
//sleep(1);
//}


    }
}

void MtUsb::StartEndpoint(Endpoint* ep) {
    fbl::AutoLock lock(&ep->lock);

    if (ep->enabled) {
        EpQueueNextLocked(ep);
    }
}

void MtUsb::StartEndpoints() {
    for (size_t i = 0; i < countof(eps_); i++) {
        StartEndpoint(&eps_[i]);
    }
}

void MtUsb::FifoRead(uint8_t ep_index, void* buf, size_t buflen, size_t* actual) {
    auto* mmio = usb_mmio();

//    INDEX::Get().FromValue(ep_index).WriteTo(mmio);

    size_t count = RXCOUNT::Get(0).ReadFrom(mmio).rxcount();
printf("RXCOUNT: %zu\n", count);
    if (count > buflen) {
        zxlogf(ERROR, "%s: buffer too small\n", __func__);
        count = buflen;
    }

    auto remaining = count;
    auto dest = static_cast<uint32_t*>(buf);

    while (remaining >= 4) {
        *dest++ = FIFO::Get(ep_index).ReadFrom(mmio).fifo_data();
        remaining -= 4;
    }
    auto dest_8 = reinterpret_cast<uint8_t*>(dest);
    while (remaining > 0) {
        *dest_8++ = FIFO_8::Get(ep_index).ReadFrom(mmio).fifo_data();
        remaining--;
    }

    *actual = count;
}

void MtUsb::FifoWrite(uint8_t ep_index, const void* buf, size_t length) {
    auto* mmio = usb_mmio();

//    INDEX::Get().FromValue(ep_index).WriteTo(mmio);

    auto remaining = length;
    auto src = static_cast<const uint8_t*>(buf);

    while (remaining > 0) {
        FIFO_8::Get(ep_index).FromValue(0).set_fifo_data(*src++).WriteTo(mmio);
        remaining--;
    }
}

int MtUsb::IrqThread() {
    auto* mmio = usb_mmio();

    // Turn off power first
    POWER_PERI::Get()
        .ReadFrom(mmio)
        .set_softconn(0)
        .WriteTo(mmio);

    InitPhy();

    // Enable HSDMA interrupts here?

    // Turn power back on
    POWER_PERI::Get()
        .ReadFrom(mmio)
        .set_softconn(1)
        .set_enablesuspendm(1)
        .set_hsenab(1)
        .WriteTo(mmio);

    // Clear interrupts first
    INTRTX::Get()
        .FromValue(0xffff)
        .WriteTo(mmio);
    INTRRX::Get()
        .FromValue(0xffff)
        .WriteTo(mmio);
    INTRUSB::Get()
        .FromValue(0xff)
        .WriteTo(mmio);

    // Enable interrupts
    uint16_t intrtx = (1 << 0);
    uint16_t intrrx = (1 << 0);

// REMOVE
intrtx = intrrx = 0x1ff;
    
    INTRTXE::Get()
        .FromValue(intrtx)
        .WriteTo(mmio);
    INTRRXE::Get()
        .FromValue(intrrx)
        .WriteTo(mmio);
  
    // Enable USB interrupts
    INTRUSBE::Get()
        .FromValue(0)
        .set_discon_e(1)
        .set_conn_e(1)
        .set_reset_e(1)
        .set_resume_e(1)
        .set_suspend_e(1)
        .WriteTo(mmio);

    // Enable USB level 1 interrupts
    USB_L1INTM::Get()
        .FromValue(0)
        .set_tx(1)
        .set_rx(1)
        .set_usbcom(1)
        .set_dma(1)
        .WriteTo(mmio);

    DMA_INTR::Get()
        .FromValue(0)
        .set_unmask_set(0xff)
        .set_status(0xff)
        .WriteTo(mmio);

    while (true) {
        auto status = irq_.wait(nullptr);
        if (status == ZX_ERR_CANCELED) {
            return 0;
        } else if (status != ZX_OK) {
            zxlogf(ERROR, "%s: irq_.wait failed: %d\n", __func__, status);
            return -1;
        }
        zxlogf(INFO, " \n%s: got interrupt!\n", __func__);

        // Write back these registers to acknowledge the interrupts
        auto intrtx = INTRTX::Get().ReadFrom(mmio).WriteTo(mmio);
        __UNUSED auto intrrx = INTRRX::Get().ReadFrom(mmio).WriteTo(mmio);
        auto intrusb = INTRUSB::Get().ReadFrom(mmio).WriteTo(mmio);
        auto l1ints = USB_L1INTS::Get().ReadFrom(mmio).WriteTo(mmio);

//        intrtx.Print();
//        intrrx.Print();
//        intrusb.Print();
//        l1ints.Print();

        if (intrusb.suspend()) {
            printf("    SUSPEND\n");
            HandleSuspend();
        }
        if (intrusb.reset()) {
            printf("    RESET\n");
            HandleReset();
        }

        if (intrtx.ep_tx() & (1 << 0)) {
            HandleEp0();
        }

        if (l1ints.dma()) {
            HandleDma();
        }

        if (intrusb.discon()) printf("    DISCONNECT\n");
        if (intrusb.conn()) printf("    CONNECT\n");
        if (intrusb.resume()) printf("    RESUME\n");
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

printf("UsbDciRequestQueue address %02x\n", req->header.ep_address);

    uint8_t ep_index = EpAddressToIndex(req->header.ep_address);
    if (ep_index >= countof(eps_)) {
        zxlogf(ERROR, "%s: invalid endpoint address %02x\n", __func__, req->header.ep_address);
        return;
    }
    Endpoint* ep = &eps_[ep_index];

    fbl::AutoLock lock(&ep->lock);

    if (!ep->enabled) {
        usb_request_complete(req, ZX_ERR_BAD_STATE, 0);
        return;
    }

    list_add_tail(&ep->queued_reqs, &req->node);
    EpQueueNextLocked(ep);
}
 
zx_status_t MtUsb::UsbDciSetInterface(const usb_dci_interface_t* interface) {
    // TODO - handle interface == nullptr for tear down path?

    if (dci_intf_.has_value()) {
        zxlogf(ERROR, "%s: dci_intf_ already set\n", __func__);
        return ZX_ERR_BAD_STATE;
    }

    dci_intf_ = ddk::UsbDciInterfaceProxy(interface);

    // Now that the usb-peripheral driver has bound, we can start things up.
    int rc = thrd_create_with_name(&irq_thread_,
                                   [](void* arg) -> int {
                                       return reinterpret_cast<MtUsb*>(arg)->IrqThread();
                                   },
                                   reinterpret_cast<void*>(this),
                                   "mt-usb-irq-thread");
    if (rc != thrd_success) {
        return ZX_ERR_INTERNAL;
    }

    return ZX_OK;
}

 zx_status_t MtUsb::UsbDciConfigEp(const usb_endpoint_descriptor_t* ep_desc,
                                   const usb_ss_ep_comp_descriptor_t* ss_comp_desc) {
    auto* mmio = usb_mmio();
    auto ep_address = ep_desc->bEndpointAddress;
    auto ep_index = EpAddressToIndex(ep_address);
printf("UsbDciConfigEp address %02x configuration_ %u\n", ep_address, configuration_);

    if (ep_index >= countof(eps_)) {
        zxlogf(ERROR, "%s: endpoint address %02x too large\n", __func__, ep_address);
        return ZX_ERR_OUT_OF_RANGE;
    }

    Endpoint* ep = &eps_[ep_index];

    fbl::AutoLock lock(&ep->lock);

    if (ep->enabled) {
        return ZX_ERR_BAD_STATE;
    }
    auto status = AllocDmaChannel(ep);
    if (status != ZX_OK) {
        zxlogf(ERROR, "%s: AllocDmaChannel failed for endpoint %02x: %d\n", __func__,
               ep_desc->bEndpointAddress, status);
        return status;
    }

    // TODO synchronize here?
    if (ep->direction == EP_IN) {    
        auto intrtxe = INTRTXE::Get().ReadFrom(mmio);
        uint16_t mask = intrtxe.ep_tx();
        mask |= static_cast<uint16_t>(1 << ep->ep_num);
        intrtxe.set_ep_tx(mask).WriteTo(mmio);
    } else {
        auto intrrxe = INTRRXE::Get().ReadFrom(mmio);
        uint16_t mask = intrrxe.ep_rx();
        mask |= static_cast<uint16_t>(1 << ep->ep_num);
        intrrxe.set_ep_rx(mask).WriteTo(mmio);
    }

    uint16_t max_packet_size = usb_ep_max_packet(ep_desc);
    if ((ep_address & USB_DIR_MASK) == USB_DIR_IN) {
        TXMAP::Get(ep_index)
            .FromValue(0)
            .set_maximum_payload_transaction(max_packet_size)
            .WriteTo(mmio);
    } else {
        RXMAP::Get(ep_index)
            .FromValue(0)
            .set_maximum_payload_transaction(max_packet_size)
            .WriteTo(mmio);
    }
    ep->max_packet_size = max_packet_size;
    ep->enabled = true;

    if (configuration_) {
        EpQueueNextLocked(ep);
    }

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
