#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
/* Copyright (c) 2000-2015 Synology Inc. All rights reserved. */

#include "synobios.h"
#include "../mapping.h"
#include "../common/common.h"

#define SZ_UART_FAN_DUTY_CYCLE     "V"
#define SZ_UART_FAN_FREQUENCY      "W"
#define SZ_UART_CPUFAN_DUTY_CYCLE  "X"
#define SZ_UART_CPUFAN_FREQUENCY   "Y"
#define SZ_UART_PWR_LED_ON         "4"
#define SZ_UART_PWR_LED_OFF        "6"

int GetModel(void);
int SetDiskLedStatus(int disknum, SYNO_DISK_LED status);
int SetFanStatus(FAN_STATUS status, FAN_SPEED speed);
int InitModuleType(struct synobios_ops *ops);
int SetFanSpeedValue(char speed_value);
int SetFanStatus(FAN_STATUS status, FAN_SPEED speed);
int SetHDDActLed(SYNO_LED ledStatus);
int SetPowerLedStatus(SYNO_LED status);
int SYNO_GPIO_READ(int pin);
void SYNO_GPIO_WRITE(int pin, int pValue);
#ifdef MY_DEF_HERE
extern int syno_mv_9235_disk_led_set(const unsigned short hostnum, int iValue);
int SetDiskLedStatusBy9235GPIO(int iDiskNum, SYNO_DISK_LED iStatus);
#endif
#ifdef MY_DEF_HERE
extern int syno_mv_9170_disk_led_set(const unsigned short hostnum, int iValue);
int SetDiskLedStatusBy9170GPIO(int iDiskNum, SYNO_DISK_LED iStatus);
#endif
void GetCPUInfo(SYNO_CPU_INFO *cpu, const unsigned int maxLength);
extern int syno_armada_get_temperature(int *temperature);
// for matching userspace usage, return 0 if button is pressed, else = 1
int GetFanStatusMircopWithGPIOCommon(int fanno, FAN_STATUS *pStatus);
int GetFanStatusActiveLow(int fanno, FAN_STATUS *pStatus);
void VdimmPwrCtrl(SYNO_LED ledStatus);
void syno_gpio_init(void);
void syno_gpio_cleanup(void);

struct hwmon_sensor_list {
	SYNO_HWMON_SENSOR_TYPE *thermal_sensor;
	SYNO_HWMON_SENSOR_TYPE *voltage_sensor;
	SYNO_HWMON_SENSOR_TYPE *fan_speed_rpm;
	SYNO_HWMON_SENSOR_TYPE *psu_status;
	SYNO_HWMON_SENSOR_TYPE *hdd_backplane;
};