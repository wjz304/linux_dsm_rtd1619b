// Copyright (c) 2000-2012 Synology Inc. All rights reserved.

#include <linux/kernel.h> /* printk() */
#include <linux/errno.h>  /* error codes */
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/cpumask.h>
#include "../i2c/i2c-linux.h"
#include "armada_common.h"


int GetModel(void)
{
	return MODEL_DS115;
}

void GetCPUInfo(SYNO_CPU_INFO *cpu, const unsigned int maxLength)
{
#if defined(CONFIG_SMP)
	int i;

	cpu->core = 0;
	for_each_online_cpu(i) {
		cpu->core++;
	}
#else /* CONFIG_SMP */
	cpu->core = 1;
#endif
	snprintf(cpu->clock, sizeof(char) * maxLength, "%d", 800);
}

int
InitModuleType(struct synobios_ops *ops)
{
	PRODUCT_MODEL model = ops->get_model();
	module_t type_115 = MODULE_T_DS115;
	module_t *pType = NULL;

	switch (model) {
	case MODEL_DS115:
		pType = &type_115;
		armada_hdd_enable_gpio[0] = 24;
		break;
	default:
		break;
	}

	module_type_set(pType);
	return 0;
}

int SetDiskLedStatus(int disknum, SYNO_DISK_LED status)
{
	return SYNO_SOC_HDD_LED_SET(disknum, status);
}

int model_addon_init(struct synobios_ops *ops)
{
        ops->set_hdd_led = set_disk_led_one;

        return 0;
}

int model_addon_cleanup(struct synobios_ops *ops)
{
	return 0;
}
