// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define HAS_DEVICE_TREE 1

static const zbi_cpu_config_t cpu_config = {
    .cluster_count = 1,
    .clusters = {
        {
            .cpu_count = 1, // 4,
        },
    },
};

static const zbi_mem_range_t mem_config[] = {
    {
        .type = ZBI_MEM_RANGE_RAM,
        .paddr = 0x40000000,
        .length = 0x40000000, // 1GB
    },
    {
        .type = ZBI_MEM_RANGE_PERIPHERAL,
        .paddr = 0x10000000,
        .length = 0x10000000,   // ?? what is correct value here?
    },
    {
        // memory to reserve to avoid stomping on bootloader data
        .type = ZBI_MEM_RANGE_RESERVED,
        .paddr = 0x40000000,
        .length = 0x80000,
    },
    {
        // mt8167-atf-reserved-memory
        .type = ZBI_MEM_RANGE_RESERVED,
        .paddr = 0x43000000,
        .length = 0x30000,
    },
    {
        // ram_console
        .type = ZBI_MEM_RANGE_RESERVED,
        .paddr = 0x44400000,
        .length = 0x10000,
    },
    {
        // pstore
        .type = ZBI_MEM_RANGE_RESERVED,
        .paddr = 0x44410000,
        .length = 0xe0000,
    },
    {
        // minirdump
        .type = ZBI_MEM_RANGE_RESERVED,
        .paddr = 0x444f0000,
        .length = 0x10000,
    },
};

static const dcfg_simple_t uart_driver = {
    .mmio_phys = 0x11005000,
    .irq = 84 + 32,
};

static const dcfg_arm_gicv2_driver_t gicv2_driver = {
    .mmio_phys = 0x10310000,
    .gicd_offset = 0x00000,
    .gicc_offset = 0x10000,
    .ipi_base = 9,
};

static const dcfg_arm_psci_driver_t psci_driver = {
    .use_hvc = false,
};

static const dcfg_arm_generic_timer_driver_t timer_driver = {
    .irq_phys = 16 + 14,    // PHYS_NONSECURE_PPI: GIC_PPI 14
    .irq_virt = 16 + 11,    // VIRT_PPI: GIC_PPI 11
};

static const zbi_platform_id_t platform_id = {
    .vid = PDEV_VID_MEDIATEK,
    .pid = PDEV_PID_MEDIATEK_8167S_REF,
    .board_name = "mt8167s_ref",
};

static void append_board_boot_item(zbi_header_t* bootdata) {
    // add CPU configuration
    append_boot_item(bootdata, ZBI_TYPE_CPU_CONFIG, 0, &cpu_config,
                    sizeof(zbi_cpu_config_t) +
                    sizeof(zbi_cpu_cluster_t) * cpu_config.cluster_count);

    // add memory configuration
    append_boot_item(bootdata, ZBI_TYPE_MEM_CONFIG, 0, &mem_config,
                    sizeof(zbi_mem_range_t) * countof(mem_config));

    // add kernel drivers
    append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER, KDRV_MT8167_UART, &uart_driver,
                    sizeof(uart_driver));
    append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER, KDRV_ARM_GIC_V2, &gicv2_driver,
                    sizeof(gicv2_driver));
    append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER, KDRV_ARM_PSCI, &psci_driver,
                    sizeof(psci_driver));
    append_boot_item(bootdata, ZBI_TYPE_KERNEL_DRIVER, KDRV_ARM_GENERIC_TIMER, &timer_driver,
                    sizeof(timer_driver));

    // add platform ID
    append_boot_item(bootdata, ZBI_TYPE_PLATFORM_ID, 0, &platform_id, sizeof(platform_id));
}
