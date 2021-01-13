/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_encoding.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_trap.h>
#include <sbi/sbi_const.h>
#include <sbi/sbi_platform.h>

#include <sbi_utils/irqchip/plic.h>
#include <sbi_utils/sys/clint.h>
#include <sbi_utils/serial/zynq_uart.h>

#define ZYNQ_PLIC_ADDR			0xc000000
#define ZYNQ_PLIC_NUM_SOURCES	2
#define ZYNQ_HART_COUNT			4
#define ZYNQ_CLINT_ADDR			0x2000000

#define ZYNQ_UART0_ADDR			0xFF000000
#define ZYNQ_UART1_ADDR			0xFF001000
#define ZYNQ_UART_BAUDRATE			115200

static struct plic_data plic = {
	.addr = ZYNQ_PLIC_ADDR,
	.num_src = ZYNQ_PLIC_NUM_SOURCES,
};

static struct clint_data clint = {
	.addr = ZYNQ_CLINT_ADDR,
	.first_hartid = 0,
	.hart_count = ZYNQ_HART_COUNT,
	.has_64bit_mmio = TRUE,
};


static void uart_putc(char c){
    xil_uart_putc((void*)ZYNQ_UART0_ADDR, c);
}

static int uart_getc(){
    return xil_uart_getc((void*)ZYNQ_UART0_ADDR);
}

static int zynq_early_init(bool cold_boot)
{
	return 0;
}

static int zynq_final_init(bool cold_boot)
{
	return 0;
}

static int zynq_console_init(void)
{
    return 0;
}

static int zynq_irqchip_init(bool cold_boot)
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

static int zynq_ipi_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = clint_cold_ipi_init(&clint);
		if (rc)
			return rc;
	}

	return clint_warm_ipi_init();
}

static int zynq_timer_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = clint_cold_timer_init(&clint, NULL);
		if (rc)
			return rc;
	}

	return clint_warm_timer_init();
}

static void zynq_system_down(u32 reset_type, u32 reset_reason)
{

}

static inline void sifive_cflush_dlone(uintptr_t addr, size_t len) {
    asm volatile(
        "csrs   mstatus, %2 \n\t"
        "1:\n\t"
        ".insn r 0x73, 0, 0x7e, x0, %0, x0\n\t"
        "add %0, %0, 64\n\t"
        "blt %0, %1, 1b\n\t"
        "csrc   mstatus, %2 \n\t"
        : "+r"(addr) : "r"(addr+len), "r"(MSTATUS_MPRV): "memory");
}

enum sbi_ext_zynq_fid {
	SBI_EXT_ROCKET_CFLUSHDLONE = 0,
};

/* Vendor-Specific SBI handler */
static int zynq_vendor_ext_provider(long extid, long funcid,
				   const struct sbi_trap_regs *regs,
				   unsigned long *out_value,
				   struct sbi_trap_info *out_trap)
{
	int ret = 0;
	switch (funcid) {
	case SBI_EXT_ROCKET_CFLUSHDLONE:
		sifive_cflush_dlone(regs->a0, regs->a1);
		break;
	default:
		sbi_printf("Unsupported vendor sbi call : %ld\n", funcid);
		asm volatile("ebreak");
	}
	return ret;
}

const struct sbi_platform_operations platform_ops = {
	.early_init		= zynq_early_init,
	.final_init		= zynq_final_init,
	.console_putc		= uart_putc,
	.console_getc		= uart_getc,
	.console_init		= zynq_console_init,
	.irqchip_init		= zynq_irqchip_init,
	.ipi_send		= clint_ipi_send,
	.ipi_clear		= clint_ipi_clear,
	.ipi_init		= zynq_ipi_init,
	.timer_value		= clint_timer_value,
	.timer_event_stop	= clint_timer_event_stop,
	.timer_event_start	= clint_timer_event_start,
	.timer_init		= zynq_timer_init,
	.system_reset		= zynq_system_down,
	.vendor_ext_provider = zynq_vendor_ext_provider
};

const struct sbi_platform platform = {
	.opensbi_version	= OPENSBI_VERSION,
	.platform_version	= SBI_PLATFORM_VERSION(0x0, 0x01),
	.name			= "ESRG Rocket-Hyp FPGA",
	.features		= SBI_PLATFORM_DEFAULT_FEATURES,
	.hart_count		= ZYNQ_HART_COUNT,
	.hart_stack_size	= SBI_PLATFORM_DEFAULT_HART_STACK_SIZE,
	.platform_ops_addr	= (unsigned long)&platform_ops
};
