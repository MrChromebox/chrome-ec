/* Copyright (c) 2014 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* LPC module for Chrome EC */

#include "acpi.h"
#include "chipset.h"
#include "clock.h"
#include "common.h"
#include "console.h"
#include "gpio.h"
#include "hooks.h"
#include "host_command.h"
#include "keyboard_protocol.h"
#include "lpc.h"
#include "lpc_chip.h"
#include "port80.h"
#include "pwm.h"
#include "registers.h"
#include "system.h"
#include "task.h"
#include "timer.h"
#include "uart.h"
#include "util.h"
#include "system_chip.h"

/* Console output macros */
#if !(DEBUG_LPC)
#define CPUTS(...)
#define CPRINTS(...)
#else
#define CPUTS(outstr) cputs(CC_LPC, outstr)
#define CPRINTS(format, args...) cprints(CC_LPC, format, ## args)
#endif

#define LPC_SYSJUMP_TAG 0x4c50          /* "LP" */

/* PM channel definitions */
#define PMC_ACPI     PM_CHAN_1
#define PMC_HOST_CMD PM_CHAN_2

/* Super-IO index and register definitions */
#define SIO_OFFSET      0x4E
#define INDEX_SID       0x20
#define INDEX_CHPREV    0x24
#define INDEX_SRID      0x27

/*
 * Timeout to wait for host transaction to be completed.
 *
 * For eSPI - it is 200 us.
 * For LPC - it is 5 us.
 */
#ifdef CONFIG_ESPI
#define LPC_HOST_TRANSACTION_TIMEOUT_US 200
#else
#define LPC_HOST_TRANSACTION_TIMEOUT_US 5
#endif

static uint32_t host_events;            /* Currently pending SCI/SMI events */
static uint32_t event_mask[3];          /* Event masks for each type */
static struct	host_packet lpc_packet;
static struct	host_cmd_handler_args host_cmd_args;
static uint8_t	host_cmd_flags;         /* Flags from host command */
static uint8_t	shm_mem_host_cmd[256] __aligned(8);
static uint8_t	shm_memmap[256] __aligned(8);
/* Params must be 32-bit aligned */
static uint8_t params_copy[EC_LPC_HOST_PACKET_SIZE] __aligned(4);
static int init_done;

static uint8_t * const cmd_params = (uint8_t *)shm_mem_host_cmd +
		EC_LPC_ADDR_HOST_PARAM - EC_LPC_ADDR_HOST_ARGS;
static struct ec_lpc_host_args * const lpc_host_args =
		(struct ec_lpc_host_args *)shm_mem_host_cmd;

/*****************************************************************************/
/* IC specific low-level driver */
static void keyboard_irq_assert(void)
{
#ifdef CONFIG_KEYBOARD_IRQ_GPIO
	/*
	 * Enforce signal-high for long enough for the signal to be pulled high
	 * by the external pullup resistor.  This ensures the host will see the
	 * following falling edge, regardless of the line state before this
	 * function call.
	 */
	gpio_set_level(CONFIG_KEYBOARD_IRQ_GPIO, 1);
	udelay(4);
	/* Generate a falling edge */
	gpio_set_level(CONFIG_KEYBOARD_IRQ_GPIO, 0);
	udelay(4);
	/* Set signal high, now that we've generated the edge */
	gpio_set_level(CONFIG_KEYBOARD_IRQ_GPIO, 1);
#else
	/*
	 * SERIRQ is automatically sent by KBC
	 */
#endif
}

static void lpc_task_enable_irq(void)
{
	task_enable_irq(NPCX_IRQ_KBC_IBF);
	task_enable_irq(NPCX_IRQ_PM_CHAN_IBF);
	task_enable_irq(NPCX_IRQ_PORT80);
#ifdef CONFIG_ESPI
	task_enable_irq(NPCX_IRQ_ESPI);
	/* Virtual Wire: SLP_S3/4/5, SUS_STAT, PLTRST, OOB_RST_WARN */
	task_enable_irq(NPCX_IRQ_WKINTA_2);
	/* Virtual Wire: HOST_RST_WARN, SUS_WARN, SUS_PWRDN_ACK, SLP_A */
	task_enable_irq(NPCX_IRQ_WKINTB_2);
	/* Enable eSPI module interrupts */
	NPCX_ESPIIE |= (ESPIIE_GENERIC | ESPIIE_VW);
#endif
}

static void lpc_task_disable_irq(void)
{
	task_disable_irq(NPCX_IRQ_KBC_IBF);
	task_disable_irq(NPCX_IRQ_PM_CHAN_IBF);
	task_disable_irq(NPCX_IRQ_PORT80);
#ifdef CONFIG_ESPI
	task_disable_irq(NPCX_IRQ_ESPI);
	/* Virtual Wire: SLP_S3/4/5, SUS_STAT, PLTRST, OOB_RST_WARN */
	task_disable_irq(NPCX_IRQ_WKINTA_2);
	/* Virtual Wire: HOST_RST_WARN,SUS_WARN, SUS_PWRDN_ACK, SLP_A */
	task_disable_irq(NPCX_IRQ_WKINTB_2);
	/* Disable eSPI module interrupts */
	NPCX_ESPIIE &= ~(ESPIIE_GENERIC | ESPIIE_VW);
#endif
}
/**
 * Generate SMI pulse to the host chipset via GPIO.
 *
 * If the x86 is in S0, SMI# is sampled at 33MHz, so minimum pulse length is
 * 60ns.  If the x86 is in S3, SMI# is sampled at 32.768KHz, so we need pulse
 * length >61us.  Both are short enough and events are infrequent, so just
 * delay for 65us.
 */
static void lpc_generate_smi(void)
{
#ifdef CONFIG_SCI_GPIO
	/* Enforce signal-high for long enough to debounce high */
	gpio_set_level(GPIO_PCH_SMI_L, 1);
	udelay(65);
	/* Generate a falling edge */
	gpio_set_level(GPIO_PCH_SMI_L, 0);
	udelay(65);
	/* Set signal high, now that we've generated the edge */
	gpio_set_level(GPIO_PCH_SMI_L, 1);
#elif defined(CONFIG_ESPI)
	/*
	 * Don't use SET_BIT/CLEAR_BIT macro to toggle SMIB/SCIB to generate
	 * virtual wire. Use NPCX_VW_SMI/NPCX_VW_SCI macro instead.
	 * The reason is - if GPIOC6/CPIO76 are not selected as SMI/SCI, reading
	 * from SMIB/SCIB doesn't really reflect the SMI/SCI status. SMI/SCI
	 * status should be read from bit 1/0 in eSPI VMEVSM(2) register.
	 */
	NPCX_HIPMIC(PMC_ACPI) = NPCX_VW_SMI(1);
	udelay(65);
	/* Generate a falling edge */
	NPCX_HIPMIC(PMC_ACPI) = NPCX_VW_SMI(0);
	udelay(65);
	/* Set signal high */
	NPCX_HIPMIC(PMC_ACPI) = NPCX_VW_SMI(1);
#else
	/* SET SMIB bit to pull SMI_L to high.*/
	SET_BIT(NPCX_HIPMIC(PMC_ACPI), NPCX_HIPMIC_SMIB);
	udelay(65);
	/* Generate a falling edge */
	CLEAR_BIT(NPCX_HIPMIC(PMC_ACPI), NPCX_HIPMIC_SMIB);
	udelay(65);
	/* Set signal high */
	SET_BIT(NPCX_HIPMIC(PMC_ACPI), NPCX_HIPMIC_SMIB);
#endif
	if (host_events & event_mask[LPC_HOST_EVENT_SMI])
		CPRINTS("smi 0x%08x",
			host_events & event_mask[LPC_HOST_EVENT_SMI]);
}

/**
 * Generate SCI pulse to the host chipset via LPC0SCI.
 */
static void lpc_generate_sci(void)
{
#ifdef CONFIG_SCI_GPIO
	/* Enforce signal-high for long enough to debounce high */
	gpio_set_level(CONFIG_SCI_GPIO, 1);
	udelay(65);
	/* Generate a falling edge */
	gpio_set_level(CONFIG_SCI_GPIO, 0);
	udelay(65);
	/* Set signal high, now that we've generated the edge */
	gpio_set_level(CONFIG_SCI_GPIO, 1);
#elif defined(CONFIG_ESPI)
	/*
	 * Don't use SET_BIT/CLEAR_BIT macro to toggle SMIB/SCIB to generate
	 * virtual wire. Use NPCX_VW_SMI/NPCX_VW_SCI macro instead.
	 * The reason is - if GPIOC6/CPIO76 are not selected as SMI/SCI, reading
	 * from SMIB/SCIB doesn't really reflect the SMI/SCI status. SMI/SCI
	 * status should be read from bit 1/0 in eSPI VMEVSM(2) register.
	 */
	NPCX_HIPMIC(PMC_ACPI) = NPCX_VW_SCI(1);
	udelay(65);
	/* Generate a falling edge */
	NPCX_HIPMIC(PMC_ACPI) = NPCX_VW_SCI(0);
	udelay(65);
	/* Set signal high */
	NPCX_HIPMIC(PMC_ACPI) = NPCX_VW_SCI(1);
#else
	/* Set SCIB bit to pull SCI_L to high.*/
	SET_BIT(NPCX_HIPMIC(PMC_ACPI), NPCX_HIPMIC_SCIB);
	udelay(65);
	/* Generate a falling edge */
	CLEAR_BIT(NPCX_HIPMIC(PMC_ACPI), NPCX_HIPMIC_SCIB);
	udelay(65);
	/* Set signal high */
	SET_BIT(NPCX_HIPMIC(PMC_ACPI), NPCX_HIPMIC_SCIB);
#endif

	if (host_events & event_mask[LPC_HOST_EVENT_SCI])
		CPRINTS("sci 0x%08x",
			host_events & event_mask[LPC_HOST_EVENT_SCI]);
}

/**
 * Update the level-sensitive wake signal to the AP.
 *
 * @param wake_events	Currently asserted wake events
 */
static void lpc_update_wake(uint32_t wake_events)
{
	/*
	 * Mask off power button event, since the AP gets that through a
	 * separate dedicated GPIO.
	 */
	wake_events &= ~EC_HOST_EVENT_MASK(EC_HOST_EVENT_POWER_BUTTON);

	/* Signal is asserted low when wake events is non-zero */
	gpio_set_level(GPIO_PCH_WAKE_L, !wake_events);
}

uint8_t *lpc_get_memmap_range(void)
{
	return (uint8_t *)shm_memmap;
}

static void lpc_send_response(struct host_cmd_handler_args *args)
{
	uint8_t *out;
	int size = args->response_size;
	int csum;
	int i;

	/* Ignore in-progress on LPC since interface is synchronous anyway */
	if (args->result == EC_RES_IN_PROGRESS)
		return;

	/* Handle negative size */
	if (size < 0) {
		args->result = EC_RES_INVALID_RESPONSE;
		size = 0;
	}

	/* New-style response */
	lpc_host_args->flags =
			(host_cmd_flags & ~EC_HOST_ARGS_FLAG_FROM_HOST) |
			EC_HOST_ARGS_FLAG_TO_HOST;

	lpc_host_args->data_size = size;

	csum = args->command + lpc_host_args->flags +
			lpc_host_args->command_version +
			lpc_host_args->data_size;

	for (i = 0, out = (uint8_t *)args->response; i < size; i++, out++)
		csum += *out;

	lpc_host_args->checksum = (uint8_t)csum;

	/* Fail if response doesn't fit in the param buffer */
	if (size > EC_PROTO2_MAX_PARAM_SIZE)
		args->result = EC_RES_INVALID_RESPONSE;

	/* Write result to the data byte.  This sets the TOH status bit. */
	NPCX_HIPMDO(PMC_HOST_CMD) = args->result;
	/* Clear processing flag */
	CLEAR_BIT(NPCX_HIPMST(PMC_HOST_CMD), NPCX_HIPMST_F0);
}

static void lpc_send_response_packet(struct host_packet *pkt)
{
	/* Ignore in-progress on LPC since interface is synchronous anyway */
	if (pkt->driver_result == EC_RES_IN_PROGRESS)
		return;

	/* Write result to the data byte.  This sets the TOH status bit. */
	NPCX_HIPMDO(PMC_HOST_CMD) = pkt->driver_result;
	/* Clear processing flag */
	CLEAR_BIT(NPCX_HIPMST(PMC_HOST_CMD), NPCX_HIPMST_F0);
}

int lpc_keyboard_has_char(void)
{
	/* if OBF bit is '1', that mean still have a data in DBBOUT */
	return (NPCX_HIKMST&0x01) ? 1 : 0;
}

int lpc_keyboard_input_pending(void)
{
	/* if IBF bit is '1', that mean still have a data in DBBIN */
	return (NPCX_HIKMST&0x02) ? 1 : 0;
}

/* Put a char to host buffer and send IRQ if specified. */
void lpc_keyboard_put_char(uint8_t chr, int send_irq)
{
	NPCX_HIKDO = chr;
	CPRINTS("KB put %02x", chr);

	/* Enable OBE interrupt to detect host read data out */
	SET_BIT(NPCX_HICTRL, NPCX_HICTRL_OBECIE);
	task_enable_irq(NPCX_IRQ_KBC_OBE);
	if (send_irq) {
		keyboard_irq_assert();
	}
}

/*
 * Check host read is not in-progress and no timeout
 */
static void lpc_sib_wait_host_read_done(void)
{
	timestamp_t deadline;

	deadline.val = get_time().val + LPC_HOST_TRANSACTION_TIMEOUT_US;
	while (IS_BIT_SET(NPCX_SIBCTRL, NPCX_SIBCTRL_CSRD)) {
		if (timestamp_expired(deadline, NULL)) {
			ccprintf("Unexpected time of host read transaction\n");
			break;
		}
	}
}

/*
 * Check host write is not in-progress and no timeout
 */
static void lpc_sib_wait_host_write_done(void)
{
	timestamp_t deadline;

	deadline.val = get_time().val + LPC_HOST_TRANSACTION_TIMEOUT_US;
	while (IS_BIT_SET(NPCX_SIBCTRL, NPCX_SIBCTRL_CSWR)) {
		if (timestamp_expired(deadline, NULL)) {
			ccprintf("Unexpected time of host write transaction\n");
			break;
		}
	}
}

/* Emulate host to read Keyboard I/O */
uint8_t lpc_sib_read_kbc_reg(uint8_t io_offset)
{
	uint8_t data_value;

	/* Disable interrupts */
	interrupt_disable();

	/* Lock host keyboard module */
	SET_BIT(NPCX_LKSIOHA, NPCX_LKSIOHA_LKHIKBD);
	/* Enable Core-to-Host Modules Access */
	SET_BIT(NPCX_SIBCTRL, NPCX_SIBCTRL_CSAE);
	/* Verify Core read/write to host modules is not in progress */
	lpc_sib_wait_host_read_done();
	lpc_sib_wait_host_write_done();
	/* Enable Core access to keyboard module */
	SET_BIT(NPCX_CRSMAE, NPCX_CRSMAE_HIKBDAE);

	/* Specify the io_offset A0 = 0. the index register is accessed */
	NPCX_IHIOA = io_offset;

	/* Start a Core read from host module */
	SET_BIT(NPCX_SIBCTRL, NPCX_SIBCTRL_CSRD);
	/* Wait while Core read operation is in progress */
	lpc_sib_wait_host_read_done();
	/* Read the data */
	data_value = NPCX_IHD;

	/* Disable Core access to keyboard module */
	CLEAR_BIT(NPCX_CRSMAE, NPCX_CRSMAE_HIKBDAE);
	/* Disable Core-to-Host Modules Access */
	CLEAR_BIT(NPCX_SIBCTRL, NPCX_SIBCTRL_CSAE);
	/* unlock host keyboard module */
	CLEAR_BIT(NPCX_LKSIOHA, NPCX_LKSIOHA_LKHIKBD);

	/* Enable interrupts */
	interrupt_enable();

	return data_value;
}

void lpc_keyboard_clear_buffer(void)
{
	/* Clear OBF flag in host STATUS and HIKMST regs */
	if (IS_BIT_SET(NPCX_HIKMST, NPCX_HIKMST_OBF)) {
		/*
		 * Setting HICTRL.FW_OBF clears the HIKMST.OBF and STATUS.OBF
		 * but it does not deassert IRQ1 when it was already asserted.
		 * Emulate a host read to clear these two flags and also
		 * deassert IRQ1
		 */
		lpc_sib_read_kbc_reg(0x0);
	}
}

void lpc_keyboard_resume_irq(void)
{
	if (lpc_keyboard_has_char())
		keyboard_irq_assert();
}

/**
 * Update the host event status.
 *
 * Sends a pulse if masked event status becomes non-zero:
 *   - SMI pulse via EC_SMI_L GPIO
 *   - SCI pulse via LPC0SCI
 */
static void update_host_event_status(void)
{
	int need_sci = 0;
	int need_smi = 0;

	if (!init_done)
		return;

	/* Disable LPC interrupt while updating status register */
	lpc_task_disable_irq();
	if (host_events & event_mask[LPC_HOST_EVENT_SMI]) {
		/* Only generate SMI for first event */
		if (!(NPCX_HIPMST(PMC_ACPI) & NPCX_HIPMST_ST2))
			need_smi = 1;
		SET_BIT(NPCX_HIPMST(PMC_ACPI), NPCX_HIPMST_ST2);
	} else
		CLEAR_BIT(NPCX_HIPMST(PMC_ACPI), NPCX_HIPMST_ST2);

	if (host_events & event_mask[LPC_HOST_EVENT_SCI]) {
		/* Generate SCI for every event */
		need_sci = 1;
		SET_BIT(NPCX_HIPMST(PMC_ACPI), NPCX_HIPMST_ST1);
	} else
		CLEAR_BIT(NPCX_HIPMST(PMC_ACPI), NPCX_HIPMST_ST1);

	/* Copy host events to mapped memory */
	*(uint32_t *)host_get_memmap(EC_MEMMAP_HOST_EVENTS) = host_events;

	lpc_task_enable_irq();

	/* Process the wake events. */
	lpc_update_wake(host_events & event_mask[LPC_HOST_EVENT_WAKE]);

	/* Send pulse on SMI signal if needed */
	if (need_smi)
		lpc_generate_smi();

	/* ACPI 5.0-12.6.1: Generate SCI for SCI_EVT=1. */
	if (need_sci)
		lpc_generate_sci();
}

void lpc_set_host_event_state(uint32_t mask)
{
	if (mask != host_events) {
		host_events = mask;
		update_host_event_status();
	}
}

int lpc_query_host_event_state(void)
{
	const uint32_t any_mask = event_mask[0] | event_mask[1] | event_mask[2];
	int evt_index = 0;
	int i;

	for (i = 0; i < 32; i++) {
		const uint32_t e = (1 << i);

		if (host_events & e) {
			host_clear_events(e);

			/*
			 * If host hasn't unmasked this event, drop it.  We do
			 * this at query time rather than event generation time
			 * so that the host has a chance to unmask events
			 * before they're dropped by a query.
			 */
			if (!(e & any_mask))
				continue;

			evt_index = i + 1;	/* Events are 1-based */
			break;
		}
	}

	return evt_index;
}

void lpc_set_host_event_mask(enum lpc_host_event_type type, uint32_t mask)
{
	event_mask[type] = mask;
	update_host_event_status();
}

uint32_t lpc_get_host_event_mask(enum lpc_host_event_type type)
{
	return event_mask[type];
}

void lpc_set_acpi_status_mask(uint8_t mask)
{
	NPCX_HIPMST(PMC_ACPI) |= mask;
}

void lpc_clear_acpi_status_mask(uint8_t mask)
{
	NPCX_HIPMST(PMC_ACPI) &= ~mask;
}

/* Enable LPC ACPI-EC interrupts */
void lpc_enable_acpi_interrupts(void)
{
	SET_BIT(NPCX_HIPMCTL(PMC_ACPI), NPCX_HIPMCTL_IBFIE);
}

/* Disable LPC ACPI-EC interrupts */
void lpc_disable_acpi_interrupts(void)
{
	CLEAR_BIT(NPCX_HIPMCTL(PMC_ACPI), NPCX_HIPMCTL_IBFIE);
}

/**
 * Handle write to ACPI I/O port
 *
 * @param is_cmd	Is write command (is_cmd=1) or data (is_cmd=0)
 */
static void handle_acpi_write(int is_cmd)
{
	uint8_t value, result;

	/* Set processing flag before reading command byte */
	SET_BIT(NPCX_HIPMST(PMC_ACPI), NPCX_HIPMST_F0);

	/* Read command/data; this clears the FRMH status bit. */
	value = NPCX_HIPMDI(PMC_ACPI);

	/* Handle whatever this was. */
	if (acpi_ap_to_ec(is_cmd, value, &result))
		NPCX_HIPMDO(PMC_ACPI) = result;

	/* Clear processing flag */
	CLEAR_BIT(NPCX_HIPMST(PMC_ACPI), NPCX_HIPMST_F0);

	/*
	 * ACPI 5.0-12.6.1: Generate SCI for Input Buffer Empty / Output Buffer
	 * Full condition on the kernel channel.
	 */
	lpc_generate_sci();
}

/**
 * Handle write to host command I/O ports.
 *
 * @param is_cmd	Is write command (1) or data (0)?
 */
static void handle_host_write(int is_cmd)
{
	/* Set processing flag before reading command byte */
	SET_BIT(NPCX_HIPMST(PMC_HOST_CMD), NPCX_HIPMST_F0);
	/*
	 * Read the command byte.  This clears the FRMH bit in
	 * the status byte.
	 */
	host_cmd_args.command = NPCX_HIPMDI(PMC_HOST_CMD);

	host_cmd_args.result = EC_RES_SUCCESS;
	host_cmd_args.send_response = lpc_send_response;
	host_cmd_flags = lpc_host_args->flags;

	/* See if we have an old or new style command */
	if (host_cmd_args.command == EC_COMMAND_PROTOCOL_3) {
		lpc_packet.send_response = lpc_send_response_packet;

		lpc_packet.request = (const void *)shm_mem_host_cmd;
		lpc_packet.request_temp = params_copy;
		lpc_packet.request_max = sizeof(params_copy);
		/* Don't know the request size so pass in the entire buffer */
		lpc_packet.request_size = EC_LPC_HOST_PACKET_SIZE;

		lpc_packet.response = (void *)shm_mem_host_cmd;
		lpc_packet.response_max = EC_LPC_HOST_PACKET_SIZE;
		lpc_packet.response_size = 0;

		lpc_packet.driver_result = EC_RES_SUCCESS;

		host_packet_receive(&lpc_packet);
		return;

	} else {
		/* Old style command, now unsupported */
		host_cmd_args.result = EC_RES_INVALID_COMMAND;
	}

	/* Hand off to host command handler */
	host_command_received(&host_cmd_args);
}

/*****************************************************************************/
/* Interrupt handlers */
#ifdef HAS_TASK_KEYPROTO
/* KB controller input buffer full ISR */
void lpc_kbc_ibf_interrupt(void)
{
	/* If "command" input 0, else 1*/
	if (lpc_keyboard_input_pending())
		keyboard_host_write(NPCX_HIKMDI, (NPCX_HIKMST & 0x08) ? 1 : 0);
	CPRINTS("ibf isr %02x", NPCX_HIKMDI);
	task_wake(TASK_ID_KEYPROTO);
}
DECLARE_IRQ(NPCX_IRQ_KBC_IBF, lpc_kbc_ibf_interrupt, 3);

/* KB controller output buffer empty ISR */
void lpc_kbc_obe_interrupt(void)
{
	/* Disable KBC OBE interrupt */
	CLEAR_BIT(NPCX_HICTRL, NPCX_HICTRL_OBECIE);
	task_disable_irq(NPCX_IRQ_KBC_OBE);

	CPRINTS("obe isr %02x", NPCX_HIKMST);
	task_wake(TASK_ID_KEYPROTO);
}
DECLARE_IRQ(NPCX_IRQ_KBC_OBE, lpc_kbc_obe_interrupt, 3);
#endif

/* PM channel input buffer full ISR */
void lpc_pmc_ibf_interrupt(void)
{
	/* Channel-1 for ACPI usage*/
	/* Channel-2 for Host Command usage , so the argument data had been
	 * put on the share memory firstly*/
	if (NPCX_HIPMST(PMC_ACPI) & 0x02)
		handle_acpi_write((NPCX_HIPMST(PMC_ACPI)&0x08) ? 1 : 0);
	else if (NPCX_HIPMST(PMC_HOST_CMD) & 0x02)
		handle_host_write((NPCX_HIPMST(PMC_HOST_CMD)&0x08) ? 1 : 0);
}
DECLARE_IRQ(NPCX_IRQ_PM_CHAN_IBF, lpc_pmc_ibf_interrupt, 3);

/* PM channel output buffer empty ISR */
void lpc_pmc_obe_interrupt(void)
{
}
DECLARE_IRQ(NPCX_IRQ_PM_CHAN_OBE, lpc_pmc_obe_interrupt, 3);

void lpc_port80_interrupt(void)
{
	/* Send port 80 data to UART continuously if FIFO is not empty */
	while (IS_BIT_SET(NPCX_DP80STS, 6))
		port_80_write(NPCX_DP80BUF);

	/* If FIFO is overflow */
	if (IS_BIT_SET(NPCX_DP80STS, 7)) {
		SET_BIT(NPCX_DP80STS, 7);
		CPRINTS("DP80 FIFO Overflow!");
	}

	/* Clear pending bit of host writing */
	SET_BIT(NPCX_DP80STS, 5);
}
DECLARE_IRQ(NPCX_IRQ_PORT80, lpc_port80_interrupt, 3);

/**
 * Preserve event masks across a sysjump.
 */
static void lpc_sysjump(void)
{
	lpc_task_disable_irq();

	/* Disable protect for Win 1 and 2. */
	NPCX_WIN_WR_PROT(0) = 0;
	NPCX_WIN_WR_PROT(1) = 0;
	NPCX_WIN_RD_PROT(0) = 0;
	NPCX_WIN_RD_PROT(1) = 0;

	/* Reset base address for Win 1 and 2. */
	NPCX_WIN_BASE(0) = 0xfffffff8;
	NPCX_WIN_BASE(1) = 0xfffffff8;

	system_add_jump_tag(LPC_SYSJUMP_TAG, 1,
			sizeof(event_mask), event_mask);
}
DECLARE_HOOK(HOOK_SYSJUMP, lpc_sysjump, HOOK_PRIO_DEFAULT);

/**
 * Restore event masks after a sysjump.
 */
static void lpc_post_sysjump(void)
{
	const uint32_t *prev_mask;
	int size, version;

	prev_mask = (const uint32_t *)system_get_jump_tag(LPC_SYSJUMP_TAG,
			&version, &size);
	if (!prev_mask || version != 1 || size != sizeof(event_mask))
		return;

	memcpy(event_mask, prev_mask, sizeof(event_mask));
}

/* Super-IO read/write function */
void lpc_sib_write_reg(uint8_t io_offset, uint8_t index_value,
		uint8_t io_data)
{
	/* Disable interrupts */
	interrupt_disable();

	/* Lock host CFG module */
	SET_BIT(NPCX_LKSIOHA, NPCX_LKSIOHA_LKCFG);
	/* Enable Core-to-Host Modules Access */
	SET_BIT(NPCX_SIBCTRL, NPCX_SIBCTRL_CSAE);
	/* Enable Core access to CFG module */
	SET_BIT(NPCX_CRSMAE, NPCX_CRSMAE_CFGAE);
	/* Verify Core read/write to host modules is not in progress */
	lpc_sib_wait_host_read_done();
	lpc_sib_wait_host_write_done();

	/* Specify the io_offset A0 = 0. the index register is accessed */
	NPCX_IHIOA = io_offset;
	/* Write the data. This starts the write access to the host module */
	NPCX_IHD = index_value;
	/* Wait while Core write operation is in progress */
	lpc_sib_wait_host_write_done();

	/* Specify the io_offset A0 = 1. the data register is accessed */
	NPCX_IHIOA = io_offset+1;
	/* Write the data. This starts the write access to the host module */
	NPCX_IHD = io_data;
	/* Wait while Core write operation is in progress */
	lpc_sib_wait_host_write_done();

	/* Disable Core access to CFG module */
	CLEAR_BIT(NPCX_CRSMAE, NPCX_CRSMAE_CFGAE);
	/* Disable Core-to-Host Modules Access */
	CLEAR_BIT(NPCX_SIBCTRL, NPCX_SIBCTRL_CSAE);
	/* unlock host CFG  module */
	CLEAR_BIT(NPCX_LKSIOHA, NPCX_LKSIOHA_LKCFG);

	/* Enable interrupts */
	interrupt_enable();
}

uint8_t lpc_sib_read_reg(uint8_t io_offset, uint8_t index_value)
{
	uint8_t data_value;

	/* Disable interrupts */
	interrupt_disable();

	/* Lock host CFG module */
	SET_BIT(NPCX_LKSIOHA, NPCX_LKSIOHA_LKCFG);
	/* Enable Core-to-Host Modules Access */
	SET_BIT(NPCX_SIBCTRL, NPCX_SIBCTRL_CSAE);
	/* Enable Core access to CFG module */
	SET_BIT(NPCX_CRSMAE, NPCX_CRSMAE_CFGAE);
	/* Verify Core read/write to host modules is not in progress */
	lpc_sib_wait_host_read_done();
	lpc_sib_wait_host_write_done();

	/* Specify the io_offset A0 = 0. the index register is accessed */
	NPCX_IHIOA = io_offset;
	/* Write the data. This starts the write access to the host module */
	NPCX_IHD = index_value;
	/* Wait while Core write operation is in progress */
	lpc_sib_wait_host_write_done();

	/* Specify the io_offset A0 = 1. the data register is accessed */
	NPCX_IHIOA = io_offset+1;
	/* Start a Core read from host module */
	SET_BIT(NPCX_SIBCTRL, NPCX_SIBCTRL_CSRD);
	/* Wait while Core read operation is in progress */
	lpc_sib_wait_host_read_done();
	/* Read the data */
	data_value = NPCX_IHD;

	/* Disable Core access to CFG module */
	CLEAR_BIT(NPCX_CRSMAE, NPCX_CRSMAE_CFGAE);
	/* Disable Core-to-Host Modules Access */
	CLEAR_BIT(NPCX_SIBCTRL, NPCX_SIBCTRL_CSAE);
	/* unlock host CFG  module */
	CLEAR_BIT(NPCX_LKSIOHA, NPCX_LKSIOHA_LKCFG);

	/* Enable interrupts */
	interrupt_enable();

	return data_value;
}

/* For LPC host register initial via SIB module */
void host_register_init(void)
{
	/* enable ACPI*/
	lpc_sib_write_reg(SIO_OFFSET, 0x07, 0x11);
	lpc_sib_write_reg(SIO_OFFSET, 0x30, 0x01);

	/* enable KBC*/
	lpc_sib_write_reg(SIO_OFFSET, 0x07, 0x06);
	lpc_sib_write_reg(SIO_OFFSET, 0x30, 0x01);

	/* Setting PMC2 */
	/* LDN register = 0x12(PMC2) */
	lpc_sib_write_reg(SIO_OFFSET, 0x07, 0x12);
	/* CMD port is 0x200 */
	lpc_sib_write_reg(SIO_OFFSET, 0x60, 0x02);
	lpc_sib_write_reg(SIO_OFFSET, 0x61, 0x00);
	/* Data port is 0x204 */
	lpc_sib_write_reg(SIO_OFFSET, 0x62, 0x02);
	lpc_sib_write_reg(SIO_OFFSET, 0x63, 0x04);
	/* enable PMC2 */
	lpc_sib_write_reg(SIO_OFFSET, 0x30, 0x01);

	/* Setting SHM */
	/* LDN register = 0x0F(SHM) */
	lpc_sib_write_reg(SIO_OFFSET, 0x07, 0x0F);
	/* WIN1&2 mapping to IO */
	lpc_sib_write_reg(SIO_OFFSET, 0xF1,
			lpc_sib_read_reg(SIO_OFFSET, 0xF1) | 0x30);
	/* Host Command on the IO:0x0800 */
	lpc_sib_write_reg(SIO_OFFSET, 0xF7, 0x00);
	lpc_sib_write_reg(SIO_OFFSET, 0xF6, 0x00);
	lpc_sib_write_reg(SIO_OFFSET, 0xF5, 0x08);
	lpc_sib_write_reg(SIO_OFFSET, 0xF4, 0x00);
	/* WIN1 as Host Command on the IO:0x0800 */
	lpc_sib_write_reg(SIO_OFFSET, 0xFB, 0x00);
	lpc_sib_write_reg(SIO_OFFSET, 0xFA, 0x00);
	/* WIN2 as MEMMAP on the IO:0x900 */
	lpc_sib_write_reg(SIO_OFFSET, 0xF9, 0x09);
	lpc_sib_write_reg(SIO_OFFSET, 0xF8, 0x00);
	/* enable SHM */
	lpc_sib_write_reg(SIO_OFFSET, 0x30, 0x01);

	CPRINTS("Host settings are done!");

}

#ifdef CONFIG_CHIPSET_RESET_HOOK
static void lpc_chipset_reset(void)
{
	hook_notify(HOOK_CHIPSET_RESET);
}
DECLARE_DEFERRED(lpc_chipset_reset);
#endif

int lpc_get_pltrst_asserted(void)
{
	/* Read current PLTRST status */
	return IS_BIT_SET(NPCX_MSWCTL1, NPCX_MSWCTL1_PLTRST_ACT);
}

void lpc_host_reset(void)
{
	/* Host Reset Control will assert KBRST# (LPC) or RCIN# VW (eSPI) */
#ifdef CONFIG_ESPI_VW_SIGNALS
	int timeout = 100; /* 100 * 10us = 1ms */

	/* Assert RCIN# VW to host */
	SET_BIT(NPCX_MSWCTL1, NPCX_MSWCTL1_HRSTOB);

	/* Poll for dirty bit to clear to indicate VW read by host */
	while ((NPCX_VWEVSM(2) & VWEVSM_DIRTY(1))) {
		if (!timeout--) {
			CPRINTS("RCIN# VW Timeout");
			break;
		}
		udelay(10);
	}

	/* Deassert RCIN# VW to host */
	CLEAR_BIT(NPCX_MSWCTL1, NPCX_MSWCTL1_HRSTOB);
#else
	SET_BIT(NPCX_MSWCTL1, NPCX_MSWCTL1_HRSTOB);
	udelay(10);
	CLEAR_BIT(NPCX_MSWCTL1, NPCX_MSWCTL1_HRSTOB);
#endif
}

#ifndef CONFIG_ESPI
/* Initialize host settings by interrupt */
void lpc_lreset_pltrst_handler(void)
{
	int pltrst_asserted;

	/* Clear pending bit of WUI */
	SET_BIT(NPCX_WKPCL(MIWU_TABLE_0 , MIWU_GROUP_5), 7);

	/* Ignore PLTRST# from SOC if it is not valid */
	if (chipset_pltrst_is_valid && !chipset_pltrst_is_valid())
		return;

	pltrst_asserted = lpc_get_pltrst_asserted();

	ccprintf("LPC RESET# %sasserted",
		pltrst_asserted ? "" : "de");

	/*
	 * Once LRESET is de-asserted (low -> high), we need to initialize lpc
	 * settings once. If RSTCTL_LRESET_PLTRST_MODE is active, LPC registers
	 * won't be reset by Host domain reset but Core domain does.
	 */
	if (!pltrst_asserted)
		host_register_init();
	else {
		/* Clear processing flag when LRESET is asserted */
		CLEAR_BIT(NPCX_HIPMST(PMC_HOST_CMD), NPCX_HIPMST_F0);
#ifdef CONFIG_CHIPSET_RESET_HOOK
		/* Notify HOOK_CHIPSET_RESET */
		hook_call_deferred(&lpc_chipset_reset_data, MSEC);
#endif
	}
}
#endif

/*****************************************************************************/
/* LPC/eSPI Initialization functions */

static void lpc_init(void)
{
	/* Enable clock for LPC peripheral */
	clock_enable_peripheral(CGC_OFFSET_LPC, CGC_LPC_MASK,
			CGC_MODE_RUN | CGC_MODE_SLEEP);
#ifdef CONFIG_ESPI
	/* Enable clock for eSPI peripheral */
	clock_enable_peripheral(CGC_OFFSET_ESPI, CGC_ESPI_MASK,
		CGC_MODE_RUN | CGC_MODE_SLEEP);
	/* Initialize eSPI IP */
	espi_init();
#else
	/* Switching to LPC interface */
	NPCX_DEVCNT |= 0x04;
#endif
	/* Enable 4E/4F */
	if (!IS_BIT_SET(NPCX_MSWCTL1, NPCX_MSWCTL1_VHCFGA)) {
		NPCX_HCBAL = 0x4E;
		NPCX_HCBAH = 0x0;
	}
	/* Clear Host Access Hold state */
	NPCX_SMC_CTL = 0xC0;

#ifndef CONFIG_ESPI
	/*
	 * Set alternative pin from GPIO to CLKRUN no matter SERIRQ is under
	 * continuous or quiet mode.
	 */
	SET_BIT(NPCX_DEVALT(1), NPCX_DEVALT1_CLKRN_SL);
#endif

	/*
	 * Set pin-mux from GPIOs to SCL/SMI to make sure toggling SCIB/SMIB is
	 * valid if CONFIG_SCI_GPIO isn't defined. eSPI sends SMI/SCI through VW
	 * automatically by toggling them, too. It's unnecessary to set pin mux.
	 */
#if !defined(CONFIG_SCI_GPIO) && !defined(CONFIG_ESPI)
	SET_BIT(NPCX_DEVALT(1), NPCX_DEVALT1_EC_SCI_SL);
	SET_BIT(NPCX_DEVALT(1), NPCX_DEVALT1_SMI_SL);
#endif

	/* Initialize Hardware for UART Host */
#if CONFIG_UART_HOST
	/* Init COMx LPC UART */
	/* FMCLK have to using 50MHz */
	NPCX_DEVALT(0xB) = 0xFF;
	/* Make sure Host Access unlock */
	CLEAR_BIT(NPCX_LKSIOHA, 2);
	/* Clear Host Access Lock Violation */
	SET_BIT(NPCX_SIOLV, 2);
#endif

	/* Don't stall SHM transactions */
	NPCX_SHM_CTL = NPCX_SHM_CTL & ~0x40;
	/* Semaphore and Indirect access disable */
	NPCX_SHCFG = 0xE0;
	/* Disable Protect Win1&2*/
	NPCX_WIN_WR_PROT(0) = 0;
	NPCX_WIN_WR_PROT(1) = 0;
	NPCX_WIN_RD_PROT(0) = 0;
	NPCX_WIN_RD_PROT(1) = 0;
	/* Open Win1 256 byte for Host CMD, Win2 256 for MEMMAP*/
	NPCX_WIN_SIZE = 0x88;
	NPCX_WIN_BASE(0) = (uint32_t)shm_mem_host_cmd;
	NPCX_WIN_BASE(1) = (uint32_t)shm_memmap;
	/* Write protect of Share memory */
	NPCX_WIN_WR_PROT(1) = 0xFF;

	/* We support LPC args and version 3 protocol */
	*(lpc_get_memmap_range() + EC_MEMMAP_HOST_CMD_FLAGS) =
			EC_HOST_CMD_FLAG_LPC_ARGS_SUPPORTED |
			EC_HOST_CMD_FLAG_VERSION_3;

	/*
	 * Clear processing flag before enabling lpc's interrupts in case
	 * it's set by the other command during sysjump.
	 */
	CLEAR_BIT(NPCX_HIPMST(PMC_HOST_CMD), NPCX_HIPMST_F0);

	/* Turn on PMC2 for Host Command usage */
	SET_BIT(NPCX_HIPMCTL(PMC_HOST_CMD), 0);
	SET_BIT(NPCX_HIPMCTL(PMC_HOST_CMD), 1);

	/*
	 * Set required control value (avoid setting HOSTWAIT bit at this stage)
	 */
	NPCX_SMC_CTL = NPCX_SMC_CTL&~0x7F;
	/* Clear status */
	NPCX_SMC_STS = NPCX_SMC_STS;

	/* Restore event masks if needed */
	lpc_post_sysjump();

	/* Create mailbox */

	/*
	 * Init KBC
	 * Clear OBF status flag,
	 * IBF(K&M) INT enable, OBE(K&M) empty INT enable ,
	 * OBF Mouse Full INT enable and OBF KB Full INT enable
	 */
	lpc_keyboard_clear_buffer();
	NPCX_HICTRL = 0x0F;

	/*
	 * Turn on enhance mode on PM channel-1,
	 * enable OBE/IBF core interrupt
	 */
	NPCX_HIPMCTL(PMC_ACPI) |= 0x83;
	/* Normally Polarity IRQ1,12 type (level + high) setting */
	NPCX_HIIRQC = 0x00;

	/*
	 * Init PORT80
	 * Enable Port80, Enable Port80 function & Interrupt & Read auto
	 */
#ifdef CONFIG_ESPI
	NPCX_DP80CTL = 0x2b;
#else
	NPCX_DP80CTL = 0x29;
#endif
	SET_BIT(NPCX_GLUE_SDP_CTS, 3);
#if SUPPORT_P80_SEG
	SET_BIT(NPCX_GLUE_SDP_CTS, 0);
#endif

	/*
	 * Use SMI/SCI postive polarity as default.
	 * Negative polarity must be enabled in the case that SMI/SCI is
	 * generated automatically by hardware. In current design,
	 * SMI/SCI is conntrolled by FW. Use postive polarity is more
	 * intuitive.
	 */
	CLEAR_BIT(NPCX_HIPMCTL(PMC_ACPI), NPCX_HIPMCTL_SCIPOL);
	CLEAR_BIT(NPCX_HIPMIC(PMC_ACPI), NPCX_HIPMIC_SMIPOL);
	/* Set SMIB/SCIB to make sure SMI/SCI are high at init */
	NPCX_HIPMIC(PMC_ACPI) = NPCX_HIPMIC(PMC_ACPI)
			| (1 << NPCX_HIPMIC_SMIB) | (1 << NPCX_HIPMIC_SCIB);
#ifndef CONFIG_SCI_GPIO
	/*
	 * Allow SMI/SCI generated from PM module.
	 * Either hardware autimatically generates,
	 * or set SCIB/SMIB bit in HIPMIC register.
	 */
	SET_BIT(NPCX_HIPMIE(PMC_ACPI), NPCX_HIPMIE_SCIE);
	SET_BIT(NPCX_HIPMIE(PMC_ACPI), NPCX_HIPMIE_SMIE);
#endif
	lpc_task_enable_irq();

	/* Sufficiently initialized */
	init_done = 1;

	/* Update host events now that we can copy them to memmap */
	update_host_event_status();

	/*
	 * TODO: For testing LPC with Chromebox, please make sure LPC_CLK is
	 * generated before executing this function. EC needs LPC_CLK to access
	 * LPC register through SIB module. For Chromebook platform, this
	 * functionality should be done by BIOS or executed in hook function of
	 * HOOK_CHIPSET_STARTUP
	 */
#ifdef BOARD_NPCX_EVB
	/* initial IO port address via SIB-write modules */
	host_register_init();
#else
	/* Initialize LRESET# interrupt */
	/* Set detection mode to edge */
	CLEAR_BIT(NPCX_WKMOD(MIWU_TABLE_0, MIWU_GROUP_5), 7);
	/* Handle interrupting on any edge */
	SET_BIT(NPCX_WKAEDG(MIWU_TABLE_0, MIWU_GROUP_5), 7);
	/* Enable wake-up input sources */
	SET_BIT(NPCX_WKEN(MIWU_TABLE_0, MIWU_GROUP_5), 7);
#endif
}
/*
 * Set prio to higher than default; this way LPC memory mapped data is ready
 * before other inits try to initialize their memmap data.
 */
DECLARE_HOOK(HOOK_INIT, lpc_init, HOOK_PRIO_INIT_LPC);

static void lpc_resume(void)
{
#ifdef CONFIG_POWER_S0IX
	if (chipset_in_state(CHIPSET_STATE_SUSPEND | CHIPSET_STATE_ON))
#endif
	{
		/* Mask all host events until the host unmasks them itself.  */
		lpc_set_host_event_mask(LPC_HOST_EVENT_SMI, 0);
		lpc_set_host_event_mask(LPC_HOST_EVENT_SCI, 0);
		lpc_set_host_event_mask(LPC_HOST_EVENT_WAKE, 0);
	}

	/* Store port 80 event so we know where resume happened */
	port_80_write(PORT_80_EVENT_RESUME);
}
DECLARE_HOOK(HOOK_CHIPSET_RESUME, lpc_resume, HOOK_PRIO_DEFAULT);

/* Get protocol information */
static int lpc_get_protocol_info(struct host_cmd_handler_args *args)
{
	struct ec_response_get_protocol_info *r = args->response;

	memset(r, 0, sizeof(*r));
	r->protocol_versions = (1 << 3);
	r->max_request_packet_size = EC_LPC_HOST_PACKET_SIZE;
	r->max_response_packet_size = EC_LPC_HOST_PACKET_SIZE;
	r->flags = 0;

	args->response_size = sizeof(*r);

	return EC_SUCCESS;
}
DECLARE_HOST_COMMAND(EC_CMD_GET_PROTOCOL_INFO,
		lpc_get_protocol_info,
		EC_VER_MASK(0));
