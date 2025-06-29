// Copyright (C) 2020 Linar Yusupov. All rights reserved.
// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

//#define DEBUG_TX
//#define DEBUG_RX

#include "board.h"
#include "lmic.h"

// ----------------------------------------
// RADIO STATE
static struct {
    ostime_t irqtime;
    osjob_t irqjob;
    u1_t diomask;
} state;

const SX12XX_ops_t *SX12XX_LL;

void radio_init (void) {
    SX12XX_LL->radio_init();
}

void radio_sleep (void) {
    SX12XX_LL->radio_sleep();
}

void radio_starttx (bool txcontinuous) {
    SX12XX_LL->radio_starttx(txcontinuous);
}

void radio_startrx (bool rxcontinuous) {
    SX12XX_LL->radio_startrx(rxcontinuous);
}

u1_t radio_has_irq (void) {
    return SX12XX_LL->radio_has_irq();
}

bool radio_irq_process (ostime_t irqtime, u1_t diomask) {
    return SX12XX_LL->radio_irq_process(irqtime, diomask);
}

void radio_cca (void) {
    SX12XX_LL->radio_cca();
}

// stop radio, disarm interrupts, cancel jobs
static void radio_stop (void) {
    hal_disableIRQs();
    // put radio to sleep
    radio_sleep();
    // disable antenna switch
    hal_pin_rxtx(-1);
    // power-down TCXO
    hal_pin_tcxo(0);
    // disable IRQs in HAL
    hal_irqmask_set(0);
    // cancel radio job
    os_clearCallback(&state.irqjob);
    // clear state
    state.diomask = 0;
    hal_enableIRQs();
}

// guard timeout in case no completion interrupt is generated by radio
// protected job - runs with irqs disabled!
static void radio_irq_timeout (osjob_t* j) {
    BACKTRACE();

    // stop everything (antenna switch, hal irqs, sleep, irq job)
    radio_stop();

    // enable IRQs!
    hal_enableIRQs();

    debug_printf("WARNING: radio irq timeout!\r\n");

    // indicate timeout
    LMIC.dataLen = 0;

    // run os job (use preset func ptr)
    os_setCallback(&LMIC.osjob, LMIC.osjob.func);
}

void radio_set_irq_timeout (ostime_t timeout) {
    // schedule irq-protected timeout function
    os_setProtectedTimedCallback(&state.irqjob, timeout, radio_irq_timeout);
}

// (run by irqjob)
static void radio_irq_func (osjob_t* j) {
    // call radio-specific processing function
    if( radio_irq_process(state.irqtime, state.diomask) ) {
	// current radio operation has completed
	radio_stop(); // (disable antenna switch and HAL irqs, make radio sleep)

	// run LMIC job (use preset func ptr)
	os_setCallback(&LMIC.osjob, LMIC.osjob.func);
    }

    // clear irq state (job has been run)
    hal_disableIRQs();
    state.diomask = 0;
    hal_enableIRQs();
}

// called by hal exti IRQ handler
// (all radio operations are performed on radio job!)
void radio_irq_handler (u1_t diomask, ostime_t ticks) {
    BACKTRACE();

    // make sure previous job has been run
    ASSERT( state.diomask == 0 );

    // save interrupt source and time
    state.irqtime = ticks;
    state.diomask = diomask;

    // schedule irq job
    // (timeout job will be replaced, intermediate interrupts must rewind timeout!)
    os_setCallback(&state.irqjob, radio_irq_func);
}

void os_radio (u1_t mode) {
    switch (mode) {
	case RADIO_RST:
	    radio_stop();
	    break;

	case RADIO_TX:
	    radio_stop();
#ifdef DEBUG_TX
	    debug_printf("TX[freq=%.1F,sf=%d,bw=%s,pow=%d%+d%+d,len=%d%s]: %h\r\n",
			 LMIC.freq, 6,
			 getSf(LMIC.rps) + 6, ("125\0" "250\0" "500\0" "rfu") + (4 * getBw(LMIC.rps)),
			 LMIC.txpow, LMIC.txPowAdj, TX_ERP_ADJ, LMIC.dataLen,
			 (LMIC.pendTxPort != 0 && (LMIC.frame[OFF_DAT_FCT] & FCT_ADRARQ)) ? ",ADRARQ" : "",
			 LMIC.frame, LMIC.dataLen);
#endif
	    // transmit frame now (wait for completion interrupt)
	    radio_starttx(0);

	    break;

	case RADIO_RX:
	    radio_stop();
#ifdef DEBUG_RX
	    debug_printf("RX_MODE[freq=%.1F,sf=%d,bw=%s,rxtime=%.0F]\r\n",
			 LMIC.freq, 6,
			 getSf(LMIC.rps) + 6, ("125\0" "250\0" "500\0" "rfu") + (4 * getBw(LMIC.rps)),
			 LMIC.rxtime, 0);
#endif
	    // receive frame at rxtime/now (wait for completion interrupt)
	    radio_startrx(0);

	    break;

	case RADIO_RXON:
	    radio_stop();
#ifdef DEBUG_RX
	    debug_printf("RXON_MODE[freq=%.1F,sf=%d,bw=%s]\r\n",
			 LMIC.freq, 6,
			 getSf(LMIC.rps) + 6, ("125\0" "250\0" "500\0" "rfu") + (4 * getBw(LMIC.rps)));
#endif
	    // start scanning for frame now (wait for completion interrupt)
	    radio_startrx(1);
	    break;

	case RADIO_TXCW:
	    radio_stop();
	    // transmit continuous wave (until abort)
	    radio_starttx(1);
	    break;

	case RADIO_CCA:
	    radio_stop();
	    // clear channel assessment
	    radio_cca();
	    radio_stop();
	    break;
    }
}
