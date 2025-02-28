// Copyright (c) 2000-2015 Synology Inc. All rights reserved.

#include <linux/kernel.h> /* printk() */
#include <linux/errno.h>  /* error codes */
#include <linux/delay.h>
#include "synobios.h"
#include "broadwell_common.h"

// extern function from broadwell_common
extern int BroadwellRedundantPowerGetPowerStatus(POWER_INFO *power_info);
extern int xsSetBuzzerClear(unsigned char buzzer_cleared);
extern int xsGetBuzzerCleared(unsigned char *buzzer_cleared);
extern int xsCPUFanSpeedMapping(FAN_SPEED speed);
extern int xsFanSpeedMapping(FAN_SPEED speed);
extern char *syno_get_cpu_model_name(void);

SYNO_HWMON_SENSOR_TYPE DS3617xsII_thermal_sensor = {
	.type_name = HWMON_SYS_THERMAL_NAME,
	.sensor_num = 3,
	.sensor[0] = {
		.sensor_name = "Remote1",
	},
	.sensor[1] = {
		.sensor_name = "Local",
	},
	.sensor[2] = {
		.sensor_name = "Remote2",
	},
};

SYNO_HWMON_SENSOR_TYPE DS3617xsII_voltage_sensor = {
	.type_name = HWMON_SYS_VOLTAGE_NAME,
	.sensor_num = 5,
	.sensor[0] = {
		.sensor_name = "VCC",
	},
	.sensor[1] = {
		.sensor_name = "VPP",
	},
	.sensor[2] = {
		.sensor_name = "V33",
	},
	.sensor[3] = {
		.sensor_name = "V5",
	},
	.sensor[4] = {
		.sensor_name = "V12",
	},
};

SYNO_HWMON_SENSOR_TYPE DS3617xsII_fan_speed_rpm = {
	.type_name = HWMON_SYS_FAN_RPM_NAME,
	.sensor_num = 2,
	.sensor[0] = {
		.sensor_name = HWMON_SYS_FAN1_RPM,
	},
	.sensor[1] = {
		.sensor_name = HWMON_SYS_FAN2_RPM,
	},
};

static
int DS3617xsIIInitModuleType(struct synobios_ops *ops)
{
	module_t type_ds3617xsII = MODULE_T_DS3617xsII;
	module_t *pType = &type_ds3617xsII;
	GPIO_PIN Pin;

	if ( strstr(syno_get_cpu_model_name(), "D-1528")) {
		pType->cpu_arch_info = CPU_D_1528;
	} else {
		pType->cpu_arch_info = CPU_D_1527;
	}

	module_type_set(pType);
	return 0;
}

struct model_ops ds3617xsII_ops = {
	.x86_init_module_type = DS3617xsIIInitModuleType,
	.x86_fan_speed_mapping = xsFanSpeedMapping,
	.x86_set_esata_led_status = NULL,
	.x86_cpufan_speed_mapping = xsCPUFanSpeedMapping,
	.x86_get_buzzer_cleared = NULL,
	.x86_get_power_status    = NULL,
	.x86_set_buzzer_clear = NULL,
};

struct hwmon_sensor_list ds3617xsII_sensor_list = {
	.thermal_sensor = &DS3617xsII_thermal_sensor,
	.voltage_sensor = &DS3617xsII_voltage_sensor,
	.fan_speed_rpm = &DS3617xsII_fan_speed_rpm,
	.psu_status = NULL,
	.hdd_backplane = NULL,
};
