// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <hwreg/bitfields.h>
#include <zircon/types.h>

namespace mt_usb {

// Function Address Register
class FADDR : public hwreg::RegisterBase<FADDR, uint8_t> {
public:
    DEF_FIELD(6, 0, function_address);
    static auto Get() { return hwreg::RegisterAddr<FADDR>(0x00); }
};

// Power Management Register (peripheral mode)
class POWER_PERI : public hwreg::RegisterBase<POWER_PERI, uint8_t> {
public:
    DEF_BIT(7, isoupdate);
    DEF_BIT(6, softconn);
    DEF_BIT(5, hsenab);
    DEF_BIT(4, hsmode);
    DEF_BIT(3, reset);
    DEF_BIT(2, resume);
    DEF_BIT(1, suspendmode);
    DEF_BIT(0, enablesuspendm);
    static auto Get() { return hwreg::RegisterAddr<POWER_PERI>(0x01); }
};

// Power Management Register (host mode)
class POWER_HOST : public hwreg::RegisterBase<POWER_HOST, uint8_t> {
public:
    DEF_BIT(5, hsenab);
    DEF_BIT(4, hsmode);
    DEF_BIT(3, reset);
    DEF_BIT(2, resume);
    DEF_BIT(1, suspendmode);
    DEF_BIT(0, enablesuspendm);
    static auto Get() { return hwreg::RegisterAddr<POWER_HOST>(0x01); }
};

// TX Interrupt Status Register
class INTRTX : public hwreg::RegisterBase<INTRTX, uint16_t> {
public:
    // bit field, one bit per TX endpoint
    DEF_FIELD(7, 0, ep_tx);
    static auto Get() { return hwreg::RegisterAddr<INTRTX>(0x02); }
};

// RX Interrupt Status Register
class INTRRX : public hwreg::RegisterBase<INTRRX, uint16_t> {
public:
    // bit field, one bit per RX endpoint (endpoints 1 - 7)
    DEF_FIELD(7, 0, ep_rx);
    static auto Get() { return hwreg::RegisterAddr<INTRRX>(0x4); }
};

// TX Interrupt Enable Register
class INTRTXE : public hwreg::RegisterBase<INTRTXE, uint16_t> {
public:
    // bit field, one bit per TX endpoint
    DEF_FIELD(7, 0, ep_tx);
    static auto Get() { return hwreg::RegisterAddr<INTRTXE>(0x06); }
};

// RX Interrupt Enable Register
class INTRRXE : public hwreg::RegisterBase<INTRRXE, uint16_t> {
public:
    // bit field, one bit per RX endpoint (endpoints 1 - 7)
    DEF_FIELD(7, 0, ep_rx);
    static auto Get() { return hwreg::RegisterAddr<INTRRXE>(0x8); }
};

// Common USB Interrupt Register
class INTRUSB : public hwreg::RegisterBase<INTRUSB, uint8_t> {
public:
    DEF_BIT(7, vbuserror);
    DEF_BIT(6, sessreq);
    DEF_BIT(5, discon);
    DEF_BIT(4, conn);
    DEF_BIT(3, sof);
    DEF_BIT(2, reset_babble);
    DEF_BIT(1, resume);
    DEF_BIT(0, suspend);
    static auto Get() { return hwreg::RegisterAddr<INTRUSB>(0x0a); }
};

// Common USB Interrupt Enable Register
class INTRUSBE : public hwreg::RegisterBase<INTRUSBE, uint8_t> {
public:
    DEF_BIT(7, vbuserror_e);
    DEF_BIT(6, sessreq_e);
    DEF_BIT(5, discon_e);
    DEF_BIT(4, conn_e);
    DEF_BIT(3, sof_e);
    DEF_BIT(2, reset_babble_e);
    DEF_BIT(1, resume_e);
    DEF_BIT(0, suspend_e);
    static auto Get() { return hwreg::RegisterAddr<INTRUSBE>(0x0b); }
};

// Frame Number Register
class FRAME : public hwreg::RegisterBase<FRAME, uint16_t> {
public:
    DEF_FIELD(10, 0, frame_number);
    static auto Get() { return hwreg::RegisterAddr<FRAME>(0x0c); }
};

// Endpoint Selection Index Register
class INDEX : public hwreg::RegisterBase<INDEX, uint8_t> {
public:
    DEF_FIELD(3, 0, selected_endpoint);
    static auto Get() { return hwreg::RegisterAddr<INDEX>(0x0e); }
};

// Endpoint Selection Index Register
class TESTMODE : public hwreg::RegisterBase<TESTMODE, uint8_t> {
public:
    DEF_BIT(7, force_host);
    DEF_BIT(6, fifo_access);
    DEF_BIT(5, force_fs);
    DEF_BIT(4, force_hs);
    DEF_BIT(3, test_packet);
    DEF_BIT(2, test_k);
    DEF_BIT(1, test_j);
    DEF_BIT(0, test_se0_nak);
    static auto Get() { return hwreg::RegisterAddr<TESTMODE>(0x0f); }
};

// RXMAP Register
class RXMAP : public hwreg::RegisterBase<RXMAP, uint16_t> {
public:
    DEF_FIELD(12, 11, m_1);
    DEF_FIELD(10, 0, maximum_payload_transaction);
    static auto Get() { return hwreg::RegisterAddr<RXMAP>(0x14); }
};

// RX CSR Register (peripheral mode)
class RXCSR_PERI : public hwreg::RegisterBase<RXCSR_PERI, uint16_t> {
public:
    DEF_BIT(15, autoclear);
    DEF_BIT(14, iso);
    DEF_BIT(13, dmareqen);
    DEF_BIT(12, disnyet_piderr);
    DEF_BIT(11, dmareqmode);
    DEF_BIT(9, keeperrstatus);
    DEF_BIT(8, incomprx);
    DEF_BIT(7, clrdatatog);
    DEF_BIT(6, sentstall);
    DEF_BIT(5, sendstall);
    DEF_BIT(4, flushfifo);
    DEF_BIT(3, dataerr);
    DEF_BIT(2, overrun);
    DEF_BIT(1, fifofull);
    DEF_BIT(0, rxpktrdy);
    static auto Get() { return hwreg::RegisterAddr<RXCSR_PERI>(0x16); }
};

// RX CSR Register (host mode)
class RXCSR_HOST : public hwreg::RegisterBase<RXCSR_HOST, uint16_t> {
public:
    DEF_BIT(15, autoclear);
    DEF_BIT(14, autoreq);
    DEF_BIT(13, dmareqenab);
    DEF_BIT(12, piderror);
    DEF_BIT(11, dmareqmode);
    DEF_BIT(10, setreqpkt_twice);
    DEF_BIT(9, keeperrstatus);
    DEF_BIT(8, incomprx);
    DEF_BIT(7, clrdatatog);
    DEF_BIT(6, rxstall);
    DEF_BIT(5, reqpkt);
    DEF_BIT(4, flushfifo);
    DEF_BIT(3, dataerr_naktimeout);
    DEF_BIT(2, error);
    DEF_BIT(1, fifofull);
    DEF_BIT(0, rxpktrdy);
    static auto Get() { return hwreg::RegisterAddr<RXCSR_HOST>(0x16); }
};

// RX Count Register
class RXCOUNT : public hwreg::RegisterBase<RXCOUNT, uint16_t> {
public:
    DEF_FIELD(13, 0, rxcount);
    static auto Get() { return hwreg::RegisterAddr<RXCOUNT>(0x18); }
};

// TX Type Register
class TXTYPE : public hwreg::RegisterBase<TXTYPE, uint8_t> {
public:
    DEF_FIELD(7, 6, tx_speed);
    DEF_FIELD(5, 4, tx_protocol);
    DEF_FIELD(3, 0, tx_target_ep_number);
    static auto Get() { return hwreg::RegisterAddr<TXTYPE>(0x1a); }
};

// TX Interval Register
class TXINTERVAL : public hwreg::RegisterBase<TXINTERVAL, uint8_t> {
public:
    DEF_FIELD(7, 0, tx_polling_interval_nak_limit_m);
    static auto Get() { return hwreg::RegisterAddr<TXINTERVAL>(0x1b); }
};

// RX Type Register
class RXTYPE : public hwreg::RegisterBase<RXTYPE, uint8_t> {
public:
    DEF_FIELD(7, 6, rx_speed);
    DEF_FIELD(5, 4, rx_protocol);
    DEF_FIELD(3, 0, rx_target_ep_number);
    static auto Get() { return hwreg::RegisterAddr<RXTYPE>(0x1c); }
};

// RX Interval Register
class RXINTERVAL : public hwreg::RegisterBase<RXINTERVAL, uint8_t> {
public:
    DEF_FIELD(7, 0, rx_polling_interval_nak_limit_m);
    static auto Get() { return hwreg::RegisterAddr<RXINTERVAL>(0x1d); }
};

// Configured FIFO Size Register
class FIFOSIZE : public hwreg::RegisterBase<FIFOSIZE, uint8_t> {
public:
    DEF_FIELD(7, 4, rxfifosize);
    DEF_FIELD(3, 0, txfifosize);
    static auto Get() { return hwreg::RegisterAddr<FIFOSIZE>(0x1f); }
};

// USB Endpoint n FIFO Register
class FIFO : public hwreg::RegisterBase<FIFO, uint32_t> {
public:
    DEF_FIELD(31, 0, fifo_data);
    static auto Get(uint32_t ep) { return hwreg::RegisterAddr<FIFO>(0x20 + ep * 4); }
};

// Device Control Register
class DEVCTL : public hwreg::RegisterBase<DEVCTL, uint8_t> {
public:
    DEF_BIT(7, b_device);
    DEF_BIT(6, fsdev);
    DEF_BIT(5, lsdev);
    DEF_FIELD(4, 3, vbus);
    DEF_BIT(2, hostmode);
    DEF_BIT(1, hostreq);
    DEF_BIT(0, session);
    static auto Get() { return hwreg::RegisterAddr<DEVCTL>(0x60); }
};

// Power Up Counter Register
class PWRUPCNT : public hwreg::RegisterBase<PWRUPCNT, uint8_t> {
public:
    DEF_FIELD(3, 0, pwrupcnt);
    static auto Get() { return hwreg::RegisterAddr<PWRUPCNT>(0x61); }
};

// TX FIFO Size Register
class TXFIFOSZ : public hwreg::RegisterBase<TXFIFOSZ, uint8_t> {
public:
    DEF_BIT(4, txdpb);
    DEF_FIELD(3, 0, txsz);
    static auto Get() { return hwreg::RegisterAddr<TXFIFOSZ>(0x62); }
};

// RX FIFO Size Register
class RXFIFOSZ : public hwreg::RegisterBase<RXFIFOSZ, uint8_t> {
public:
    DEF_BIT(4, rxdpb);
    DEF_FIELD(3, 0, rxsz);
    static auto Get() { return hwreg::RegisterAddr<RXFIFOSZ>(0x63); }
};

// TX FIFO Address Register
class TXFIFOADD : public hwreg::RegisterBase<TXFIFOADD, uint16_t> {
public:
    DEF_FIELD(12, 0, txfifoadd);
    static auto Get() { return hwreg::RegisterAddr<TXFIFOADD>(0x64); }
};

// RX FIFO Address Register
class RXFIFOADD : public hwreg::RegisterBase<RXFIFOADD, uint16_t> {
public:
    DEF_BIT(15, data_err_intr_en);
    DEF_BIT(14, overrun_intr_en);
    DEF_FIELD(12, 0, rxfifoadd);
    static auto Get() { return hwreg::RegisterAddr<RXFIFOADD>(0x66); }
};

// Hardware Capability Register
class HWCAPS : public hwreg::RegisterBase<HWCAPS, uint16_t> {
public:
    DEF_BIT(15, qmu_support);
    DEF_BIT(14, hub_support);
    DEF_BIT(13, usb20_support);
    DEF_BIT(12, usb11_support);
    DEF_FIELD(11, 10, mstr_wrap_intfx);
    DEF_FIELD(9, 8, slave_wrap_intfx);
    DEF_FIELD(5, 0, usb_version_code);
    static auto Get() { return hwreg::RegisterAddr<HWCAPS>(0x6c); }
};

// Version Register
class HWSVERS : public hwreg::RegisterBase<HWSVERS, uint16_t> {
public:
    DEF_FIELD(7, 0, usb_sub_version_code);
    static auto Get() { return hwreg::RegisterAddr<HWSVERS>(0x6e); }
};

// Number of TX and RX Register
class EPINFO : public hwreg::RegisterBase<EPINFO, uint8_t> {
public:
    DEF_FIELD(7, 4, rxendpoints);
    DEF_FIELD(3, 0, txendpoints);
    static auto Get() { return hwreg::RegisterAddr<EPINFO>(0x78); }
};

} // namespace mt_usb
