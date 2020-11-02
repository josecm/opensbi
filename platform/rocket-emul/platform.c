/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_encoding.h>
#include <sbi/sbi_const.h>
#include <sbi/sbi_platform.h>

#include <sbi_utils/irqchip/plic.h>
#include <sbi_utils/sys/clint.h>

#define EMUL_PLIC_ADDR			0xc000000
#define EMUL_PLIC_NUM_SOURCES	2
#define EMUL_HART_COUNT			1
#define EMUL_CLINT_ADDR			0x2000000

static struct plic_data plic = {
	.addr = EMUL_PLIC_ADDR,
	.num_src = EMUL_PLIC_NUM_SOURCES,
};

static struct clint_data clint = {
	.addr = EMUL_CLINT_ADDR,
	.first_hartid = 0,
	.hart_count = EMUL_HART_COUNT,
	.has_64bit_mmio = TRUE,
};

/* clang-format on */

static void putc(char c){
	extern int putchar(int ch);
    putchar(c);
}

static int getc(){
    return 0;
}

static int emul_early_init(bool cold_boot)
{
	return 0;
}

static int emul_final_init(bool cold_boot)
{
	return 0;
}

static int emul_console_init(void)
{
    return 0;
}

static int emul_irqchip_init(bool cold_boot)
{
	int rc;
	u32 hartid = current_hartid();

	if (cold_boot) {
		rc = plic_cold_irqchip_init(&plic);
		if (rc)
			return rc;
	}

	return plic_warm_irqchip_init(&plic, 2 * hartid, 2 * hartid + 1);
}

static int emul_ipi_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = clint_cold_ipi_init(&clint);
		if (rc)
			return rc;
	}

	return clint_warm_ipi_init();
}

static int emul_timer_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = clint_cold_timer_init(&clint, NULL);
		if (rc)
			return rc;
	}

	return clint_warm_timer_init();
}

void tohost_exit(uintptr_t code);
static int emul_system_down(u32 type)
{
	/* For now nothing to do. */
	tohost_exit(0);
	return 0;
}


const struct sbi_platform_operations platform_ops = {
	.early_init		= emul_early_init,
	.final_init		= emul_final_init,
	.console_putc		= putc,
	.console_getc		= getc,
	.console_init		= emul_console_init,
	.irqchip_init		= emul_irqchip_init,
	.ipi_send		= clint_ipi_send,
	.ipi_clear		= clint_ipi_clear,
	.ipi_init		= emul_ipi_init,
	.timer_value		= clint_timer_value,
	.timer_event_stop	= clint_timer_event_stop,
	.timer_event_start	= clint_timer_event_start,
	.timer_init		= emul_timer_init,
	.system_reset		= emul_system_down,
};

const struct sbi_platform platform = {
	.opensbi_version	= OPENSBI_VERSION,
	.platform_version	= SBI_PLATFORM_VERSION(0x0, 0x01),
	.name			= "ESRG Rocket-Hyp EMULATOR",
	.features		= SBI_PLATFORM_DEFAULT_FEATURES,
	.hart_count		= EMUL_HART_COUNT,
	.hart_stack_size	= SBI_PLATFORM_DEFAULT_HART_STACK_SIZE,
	.platform_ops_addr	= (unsigned long)&platform_ops
};
