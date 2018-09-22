// Copyright 2018 The Fuchsia Authors
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <arch/arm64/periphmap.h>
#include <dev/interrupt.h>
#include <dev/uart.h>
#include <kernel/thread.h>
#include <lib/cbuf.h>
#include <lib/debuglog.h>
#include <pdev/driver.h>
#include <pdev/uart.h>
#include <platform/debug.h>
#include <reg.h>
#include <stdio.h>
#include <trace.h>
#include <zircon/boot/driver-config.h>

// Registers
// clang-format off
#define UART_RBR                    (0x0)   // RX Buffer Register (read-only)
#define UART_THR                    (0x0)   // TX Buffer Register (write-only)
#define UART_IER                    (0x4)   // Interrupt Enable Register
#define UART_IIR                    (0x8)   // Interrupt Identification Register (read-only)
#define UART_FCR                    (0x8)   // FIFO Control Register (write-only)
#define UART_LCR                    (0xc)   // Line Control Register
#define UART_MCR                    (0x10)  // Modem Control Register
#define UART_LSR                    (0x14)  // Line Status Register
#define UART_MSR                    (0x18)  // Modem Status Register
#define UART_SCR                    (0x1c)  // Scratch Register
#define UART_DLL                    (0x0)   // Divisor Latch LS (Only when LCR.DLAB = 1)
#define UART_DLM                    (0x4)   // Divisor Latch MS (Only when LCR.DLAB = 1)
#define UART_EFR                    (0x8)   // Enhanced Feature Register (Only when LCR = 0xbf)
#define UART_XON1                   (0x10)  // XON1 Char Register (Only when LCR = 0xbf)
#define UART_XON2                   (0x14)  // XON2 Char Register (Only when LCR = 0xbf)
#define UART_XOFF1                  (0x18)  // XOFF1 Char Register (Only when LCR = 0xbf)
#define UART_XOFF2                  (0x1c)  // XOFF2 Char Register (Only when LCR = 0xbf)
#define UART_AUTOBAUD_EN            (0x20)  // Auto Baud Detect Enable Register
#define UART_HIGHSPEED              (0x24)  // High Speed Mode Register
#define UART_SAMPLE_COUNT           (0x28)  // Sample Counter Register
#define UART_SAMPLE_POINT           (0x2c)  // Sample Point Register
#define UART_AUTOBAUD_REG           (0x30)  // Auto Baud Monitor Register
#define UART_RATE_FIX_AD            (0x34)  // Clock Rate Fix Register
#define UART_AUTOBAUD_SAMPLE        (0x38)  // Auto Baud Sample Register
#define UART_GUARD                  (0x3c)  // Guard Time Added Register
#define UART_ESCAPE_DAT             (0x40)  // Escape Character Register
#define UART_ESCAPE_EN              (0x44)  // Escape Enable Register
#define UART_SLEEP_EN               (0x48)  // Sleep Enable Register
#define UART_VFIFO_EN               (0x4c)  // DMA Enable Register
#define UART_RXTRI_AD               (0x50)  // RX Trigger Address

/* IER */
#define UART_IER_ERBFI              (1 << 0) /* RX buffer conatins data int. */
#define UART_IER_ETBEI              (1 << 1) /* TX FIFO threshold trigger int. */
#define UART_IER_ELSI               (1 << 2) /* BE, FE, PE, or OE int. */
#define UART_IER_EDSSI              (1 << 3) /* CTS change (DCTS) int. */
#define UART_IER_XOFFI              (1 << 5)
#define UART_IER_RTSI               (1 << 6)
#define UART_IER_CTSI               (1 << 7)

#define UART_IER_ALL_INTS          (UART_IER_ERBFI|UART_IER_ETBEI|UART_IER_ELSI|\
                                    UART_IER_EDSSI|UART_IER_XOFFI|UART_IER_RTSI|\
                                    UART_IER_CTSI)
#define UART_IER_HW_NORMALINTS     (UART_IER_ERBFI|UART_IER_ELSI|UART_IER_EDSSI)
#define UART_IER_HW_ALLINTS        (UART_IER_ERBFI|UART_IER_ETBEI| \
                                    UART_IER_ELSI|UART_IER_EDSSI)

/* FCR */
#define UART_FCR_FIFOE              (1 << 0)
#define UART_FCR_CLRR               (1 << 1)
#define UART_FCR_CLRT               (1 << 2)
#define UART_FCR_DMA1               (1 << 3)
#define UART_FCR_RXFIFO_1B_TRI      (0 << 6)
#define UART_FCR_RXFIFO_6B_TRI      (1 << 6)
#define UART_FCR_RXFIFO_12B_TRI     (2 << 6)
#define UART_FCR_RXFIFO_RX_TRI      (3 << 6)
#define UART_FCR_TXFIFO_1B_TRI      (0 << 4)
#define UART_FCR_TXFIFO_4B_TRI      (1 << 4)
#define UART_FCR_TXFIFO_8B_TRI      (2 << 4)
#define UART_FCR_TXFIFO_14B_TRI     (3 << 4)

#define UART_FCR_FIFO_INIT          (UART_FCR_FIFOE|UART_FCR_CLRR|UART_FCR_CLRT)
#define UART_FCR_NORMAL             (UART_FCR_FIFO_INIT | \
                                     UART_FCR_TXFIFO_4B_TRI| \
                                     UART_FCR_RXFIFO_12B_TRI)

/* LCR */
#define UART_LCR_BREAK              (1 << 6)
#define UART_LCR_DLAB               (1 << 7)

#define UART_WLS_5                  (0 << 0)
#define UART_WLS_6                  (1 << 0)
#define UART_WLS_7                  (2 << 0)
#define UART_WLS_8                  (3 << 0)
#define UART_WLS_MASK               (3 << 0)

#define UART_1_STOP                 (0 << 2)
#define UART_2_STOP                 (1 << 2)
#define UART_1_5_STOP               (1 << 2)    /* Only when WLS=5 */
#define UART_STOP_MASK              (1 << 2)

#define UART_NONE_PARITY            (0 << 3)
#define UART_ODD_PARITY             (0x1 << 3)
#define UART_EVEN_PARITY            (0x3 << 3)
#define UART_MARK_PARITY            (0x5 << 3)
#define UART_SPACE_PARITY           (0x7 << 3)
#define UART_PARITY_MASK            (0x7 << 3)

/* MCR */
#define UART_MCR_DTR    	        (1 << 0)
#define UART_MCR_RTS    	        (1 << 1)
#define UART_MCR_OUT1               (1 << 2)
#define UART_MCR_OUT2               (1 << 3)
#define UART_MCR_LOOP               (1 << 4)
#define UART_MCR_XOFF               (1 << 7)    /* read only */
#define UART_MCR_NORMAL	            (UART_MCR_DTR|UART_MCR_RTS)

/* LSR */
#define UART_LSR_DR                 (1 << 0)
#define UART_LSR_OE                 (1 << 1)
#define UART_LSR_PE                 (1 << 2)
#define UART_LSR_FE                 (1 << 3)
#define UART_LSR_BI                 (1 << 4)
#define UART_LSR_THRE               (1 << 5)
#define UART_LSR_TEMT               (1 << 6)
#define UART_LSR_FIFOERR            (1 << 7)

/* MSR */
#define UART_MSR_DCTS               (1 << 0)
#define UART_MSR_DDSR               (1 << 1)
#define UART_MSR_TERI               (1 << 2)
#define UART_MSR_DDCD               (1 << 3)
#define UART_MSR_CTS                (1 << 4)    
#define UART_MSR_DSR                (1 << 5)
#define UART_MSR_RI                 (1 << 6)
#define UART_MSR_DCD                (1 << 7)

/* EFR */
#define UART_EFR_EN                 (1 << 4)
#define UART_EFR_AUTO_RTS           (1 << 6)
#define UART_EFR_AUTO_CTS           (1 << 7)
#define UART_EFR_SW_CTRL_MASK       (0xf << 0)

#define UART_EFR_NO_SW_CTRL         (0)
#define UART_EFR_NO_FLOW_CTRL       (0)
#define UART_EFR_AUTO_RTSCTS        (UART_EFR_AUTO_RTS|UART_EFR_AUTO_CTS)
#define UART_EFR_XON1_XOFF1         (0xa) /* TX/RX XON1/XOFF1 flow control */
#define UART_EFR_XON2_XOFF2         (0x5) /* TX/RX XON2/XOFF2 flow control */
#define UART_EFR_XON12_XOFF12       (0xf) /* TX/RX XON1,2/XOFF1,2 flow 
control */

#define UART_EFR_XON1_XOFF1_MASK    (0xa)
#define UART_EFR_XON2_XOFF2_MASK    (0x5)

/* IIR (Read Only) */
#define UART_IIR_NO_INT_PENDING     (0x01)
#define UART_IIR_RLS                (0x06) /* Receiver Line Status */
#define UART_IIR_RDA                (0x04) /* Receive Data Available */
#define UART_IIR_CTI                (0x0C) /* Character Timeout Indicator */
#define UART_IIR_THRE               (0x02) /* Transmit Holding Register Empty 
*/
#define UART_IIR_MS                 (0x00) /* Check Modem Status Register */
#define UART_IIR_SW_FLOW_CTRL       (0x10) /* Receive XOFF characters */
#define UART_IIR_HW_FLOW_CTRL       (0x20) /* CTS or RTS Rising Edge */
#define UART_IIR_FIFO_EN          	(0xc0)
#define UART_IIR_INT_MASK           (0x1f)

/* RateFix */
#define UART_RATE_FIX               (1 << 0)
//#define UART_AUTORATE_FIX           (1 << 1)
//#define UART_FREQ_SEL               (1 << 2)
#define UART_FREQ_SEL               (1 << 1)

#define UART_RATE_FIX_13M           (1 << 0) /* means UARTclk = APBclk / 4 */
#define UART_AUTORATE_FIX_13M       (1 << 1) 
#define UART_FREQ_SEL_13M           (1 << 2)
#define UART_RATE_FIX_ALL_13M       (UART_RATE_FIX_13M|UART_AUTORATE_FIX_13M| \
                                     UART_FREQ_SEL_13M)

#define UART_RATE_FIX_26M           (0 << 0) /* means UARTclk = APBclk / 2 */
#define UART_AUTORATE_FIX_26M       (0 << 1) 
#define UART_FREQ_SEL_26M           (0 << 2)
#define UART_RATE_FIX_ALL_26M       (UART_RATE_FIX_26M|UART_AUTORATE_FIX_26M| \
                                     UART_FREQ_SEL_26M)

#define UART_RATE_FIX_32M5          (0 << 0)    /* means UARTclk = APBclk / 2 */
#define UART_FREQ_SEL_32M5          (0 << 1)
#define UART_RATE_FIX_ALL_32M5      (UART_RATE_FIX_32M5|UART_FREQ_SEL_32M5)

#define UART_RATE_FIX_16M25         (0 << 0)    /* means UARTclk = APBclk / 4 */
#define UART_FREQ_SEL_16M25         (0 << 1)
#define UART_RATE_FIX_ALL_16M25     (UART_RATE_FIX_16M25|UART_FREQ_SEL_16M25)


/* Autobaud sample */
#define UART_AUTOBADUSAM_13M         7
#define UART_AUTOBADUSAM_26M        15
#define UART_AUTOBADUSAM_52M        31
//#define UART_AUTOBADUSAM_52M        29  /* 28 or 29 ? */
#define UART_AUTOBAUDSAM_58_5M      31  /* 31 or 32 ? */



// clang-format off


#define RXBUF_SIZE 32

// values read from zbi
static bool initialized = false;
static vaddr_t uart_base = 0;
static uint32_t uart_irq = 0;
static cbuf_t uart_rx_buf;
// static cbuf_t uart_tx_buf;

static bool uart_tx_irq_enabled = false;
static event_t uart_dputc_event = EVENT_INITIAL_VALUE(uart_dputc_event,
                                                      true,
                                                      EVENT_FLAG_AUTOUNSIGNAL);

static spin_lock_t uart_spinlock = SPIN_LOCK_INITIAL_VALUE;

#define UARTREG(reg) (*(volatile uint32_t*)((uart_base) + (reg)))

static void uart_irq_handler(void* arg) {
#if 0 // FIXME
    /* read interrupt status and mask */
    while ((UARTREG(MX8_USR1) & USR1_RRDY)) {
        if (cbuf_space_avail(&uart_rx_buf) == 0) {
            break;
        }
        char c = UARTREG(MX8_URXD) & 0xFF;
        cbuf_write_char(&uart_rx_buf, c);
    }

    /* Signal if anyone is waiting to TX */
    if (UARTREG(MX8_UCR1) & UCR1_TRDYEN) {
        spin_lock(&uart_spinlock);
        if (!(UARTREG(MX8_USR2) & UTS_TXFULL)) {
            // signal
            event_signal(&uart_dputc_event, true);
        }
        spin_unlock(&uart_spinlock);
    }
#endif
}

/* panic-time getc/putc */
static int mt8167_uart_pputc(char c) {
    if (!uart_base) {
        return -1;
    }

    /* spin while fifo is full */
    while (!(UARTREG(UART_LSR) & UART_LSR_THRE))
        ;
    UARTREG(UART_THR) = c;

    return 1;
}

static int mt8167_uart_pgetc(void) {
    if (!uart_base) {
        return ZX_ERR_NOT_SUPPORTED;
    }

    /* spin while fifo is empty */
    while (!(UARTREG(UART_LSR) & UART_LSR_DR))
        ;
    return UARTREG(UART_RBR);
}

static int mt8167_uart_getc(bool wait) {
    if (!uart_base) {
        return ZX_ERR_NOT_SUPPORTED;
    }

    if (initialized) {
        char c;
        if (cbuf_read_char(&uart_rx_buf, &c, wait) == 1) {
            return c;
        }
        return ZX_ERR_INTERNAL;
    } else {
        // Interrupts are not enabled yet. Use panic calls for now
        return mt8167_uart_pgetc();
    }
}

static void mt8167_dputs(const char* str, size_t len,
                         bool block, bool map_NL) {
    spin_lock_saved_state_t state;
    bool copied_CR = false;

    if (!uart_base) {
        return;
    }
    if (!uart_tx_irq_enabled) {
        block = false;
    }
    spin_lock_irqsave(&uart_spinlock, state);

    while (len > 0) {
        // is FIFO full?
        while (!(UARTREG(UART_LSR) & UART_LSR_THRE)) {
            spin_unlock_irqrestore(&uart_spinlock, state);
            if (block) {
                event_wait(&uart_dputc_event);
            } else {
                arch_spinloop_pause();
            }
            spin_lock_irqsave(&uart_spinlock, state);
        }
        if (*str == '\n' && map_NL && !copied_CR) {
            copied_CR = true;
            mt8167_uart_pputc('\r');
        } else {
            copied_CR = false;
            mt8167_uart_pputc(*str++);
            len--;
        }
    }
    spin_unlock_irqrestore(&uart_spinlock, state);
}

static void mt8167_start_panic(void) {
    uart_tx_irq_enabled = false;
}

static const struct pdev_uart_ops uart_ops = {
    .getc = mt8167_uart_getc,
    .pputc = mt8167_uart_pputc,
    .pgetc = mt8167_uart_pgetc,
    .start_panic = mt8167_start_panic,
    .dputs = mt8167_dputs,
};

static void mt8167_uart_init(const void* driver_data, uint32_t length) {
/*    uint32_t regVal;

    // create circular buffer to hold received data
    cbuf_initialize(&uart_rx_buf, RXBUF_SIZE);

    // register uart irq
    register_int_handler(uart_irq, &uart_irq_handler, NULL);

    // set rx fifo threshold to 1 character
    regVal = UARTREG(MX8_UFCR);
    regVal &= ~UFCR_RXTL(UFCR_MASK);
    regVal &= ~UFCR_TXTL(UFCR_MASK);
    regVal |= UFCR_RXTL(1);
    regVal |= UFCR_TXTL(0x2);
    UARTREG(MX8_UFCR) = regVal;

    // enable rx interrupt
    regVal = UARTREG(MX8_UCR1);
    regVal |= UCR1_RRDYEN;
    if (dlog_bypass() == false) {
        // enable tx interrupt
        regVal |= UCR1_TRDYEN;
    }
    UARTREG(MX8_UCR1) = regVal;

    // enable rx and tx transmisster
    regVal = UARTREG(MX8_UCR2);
    regVal |= UCR2_RXEN | UCR2_TXEN;
    UARTREG(MX8_UCR2) = regVal;

    if (dlog_bypass() == true)
        uart_tx_irq_enabled = false;
    else {
        // start up tx driven output
        printf("UART: started IRQ driven TX\n");
        uart_tx_irq_enabled = true;
    }

    initialized = true;

    // enable interrupts
    unmask_interrupt(uart_irq);
*/
}

static void mt8167_uart_init_early(const void* driver_data, uint32_t length) {
    ASSERT(length >= sizeof(dcfg_simple_t));
    auto driver = static_cast<const dcfg_simple_t*>(driver_data);
    ASSERT(driver->mmio_phys && driver->irq);

    uart_base = periph_paddr_to_vaddr(driver->mmio_phys);
    ASSERT(uart_base);
    uart_irq = driver->irq;

    pdev_register_uart(&uart_ops);
}

LK_PDEV_INIT(mt8167_uart_init_early, KDRV_MT8167_UART, mt8167_uart_init_early, LK_INIT_LEVEL_PLATFORM_EARLY);
LK_PDEV_INIT(mt8167_uart_init, KDRV_MT8167_UART, mt8167_uart_init, LK_INIT_LEVEL_PLATFORM);
