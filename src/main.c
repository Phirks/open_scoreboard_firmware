/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
// #include <uart_async_adapter.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
// #include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/sys/reboot.h>

#include <nfc_t2t_lib.h>
#include <nfc/ndef/launchapp_msg.h>

#include <stddef.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/conn.h>

#include <bluetooth/services/lbs.h>

#include <zephyr/pm/device.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>

// #include <dk_buttons_and_leds.h>

#define NDEF_MSG_BUF_SIZE 256
#define NFC_FIELD_LED DK_LED1

char disp[3][6][7];

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

#define PWM_PERIOD_NS 25500

#define STACK_SIZE 1024 // Define the stack size for your thread

K_THREAD_STACK_DEFINE(button_stack, STACK_SIZE);

static struct k_work button_work;
static struct k_work_q work_queue;

static const struct gpio_dt_spec piezo = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), piezo_pin_gpios);

static const struct gpio_dt_spec segment_a = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), segment_a_ctrl_pin_gpios);
static const struct gpio_dt_spec segment_b = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), segment_b_ctrl_pin_gpios);
static const struct gpio_dt_spec segment_c = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), segment_c_ctrl_pin_gpios);
static const struct gpio_dt_spec segment_d = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), segment_d_ctrl_pin_gpios);
static const struct gpio_dt_spec segment_e = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), segment_e_ctrl_pin_gpios);
static const struct gpio_dt_spec segment_f = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), segment_f_ctrl_pin_gpios);
static const struct gpio_dt_spec segment_g = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), segment_g_ctrl_pin_gpios);
static const struct gpio_dt_spec mode_button = GPIO_DT_SPEC_GET(DT_PATH(buttons, mode_button), gpios);
static const struct gpio_dt_spec left_button = GPIO_DT_SPEC_GET(DT_PATH(buttons, left_button), gpios);
static const struct gpio_dt_spec right_button = GPIO_DT_SPEC_GET(DT_PATH(buttons, right_button), gpios);

//(DT_PATH(zephyr_user), red_bttm_ctrl_pin_gpios);
static const struct gpio_dt_spec rb1_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), red_bttm_ctrl_pin_gpios);
static const struct gpio_dt_spec gb1_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), green_bttm_ctrl_pin_gpios);
static const struct gpio_dt_spec bb1_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), blue_bttm_ctrl_pin_gpios);
static const struct gpio_dt_spec rb2_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), red_bttm_ctrl_2_pin_gpios);
static const struct gpio_dt_spec gb2_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), green_bttm_ctrl_2_pin_gpios);
static const struct gpio_dt_spec bb2_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), blue_bttm_ctrl_2_pin_gpios);
static struct gpio_callback button_cb_data;

static const struct pwm_dt_spec rl1 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_rl1));
static const struct pwm_dt_spec gl1 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_gl1));
static const struct pwm_dt_spec bl1 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_bl1));
static const struct pwm_dt_spec rl2 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_rl2));
static const struct pwm_dt_spec gl2 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_gl2));
static const struct pwm_dt_spec bl2 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_bl2));
static const struct pwm_dt_spec rr1 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_rr1));
static const struct pwm_dt_spec gr1 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_gr1));
static const struct pwm_dt_spec br1 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_br1));
static const struct pwm_dt_spec rr2 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_rr2));
static const struct pwm_dt_spec gr2 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_gr2));
static const struct pwm_dt_spec br2 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_br2));

// #define STACKSIZE		 2048
#define PRODUCER_THREAD_PRIORITY 6
#define CONSUMER_THREAD_PRIORITY 7

char mode = 0x00;

int militaryTime = 0;
int alarmOn = 0;
int hour = 0;
int minute = 0;
int second = 0;
int alarmHour = 0;
int alarmMinute = 0;

int timerMinutes = 0;
int timerSeconds = 0;
int timerStarted = 0;

int leftScore = 00;
int leftColor[3] = {255, 0, 0};
int rightScore = 00;
int rightColor[3] = {0, 0, 255};
int clockColor[3] = {255, 0, 0};
int alarmColor[3] = {0, 255, 0};

char hexCache1 = 0x00;
char hexCache2 = 0x00;

static struct k_timer timer0;
static struct k_timer timer4;

uint8_t oldMode = NULL;
uint8_t oldHour = NULL;
uint8_t oldMinute = NULL;
uint8_t oldSecond = NULL;
uint8_t oldTimerMinutes = NULL;
uint8_t oldTimerSeconds = NULL;
uint8_t oldLeftScore = NULL;
uint8_t oldLeftColor0 = NULL;
uint8_t oldLeftColor1 = NULL;
uint8_t oldLeftColor2 = NULL;
uint8_t oldRightScore = NULL;
uint8_t oldRightColor0 = NULL;
uint8_t oldRightColor1 = NULL;
uint8_t oldRightColor2 = NULL;
uint8_t oldClockColor0 = NULL;
uint8_t oldClockColor1 = NULL;
uint8_t oldClockColor2 = NULL;
uint8_t oldAlarmColor0 = NULL;
uint8_t oldAlarmColor1 = NULL;
uint8_t oldAlarmColor2 = NULL;
uint8_t oldHexCache1 = NULL;
uint8_t oldHexCache2 = NULL;
uint8_t oldTimerStarted = NULL;
uint8_t oldAlarmHour = NULL;
uint8_t oldAlarmMinute = NULL;
uint8_t oldMilitaryTime = NULL;
uint8_t oldAlarmOn = NULL;

bool buttonLockout = false;

/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "nrf.h"
#include <stdbool.h>
#include <stdint.h>
// #include "bsp.h"
// #include "nrf_gpio.h"
// #include "nrf_delay.h"
#include "math.h"

// Peripheral channel assignments
#define PWM0_GPIOTE_CH 0
#define PWM0_PPI_CH_A 0
#define PWM0_PPI_CH_B 1
#define PWM0_TIMER_CC_NUM 0

#define PWM1_GPIOTE_CH 1
#define PWM1_PPI_CH_A 2
#define PWM1_TIMER_CC_NUM 1

#define PWMN_GPIOTE_CH {PWM0_GPIOTE_CH, PWM1_GPIOTE_CH, 2, 3, 4}
#define PWMN_PPI_CH_A {PWM0_PPI_CH_A, PWM1_PPI_CH_A, 3, 5, 6}
#define PWMN_PPI_CH_B {PWM0_PPI_CH_B, PWM0_PPI_CH_B, 4, 4, 7}
#define PWMN_TIMER_CC_NUM {PWM0_TIMER_CC_NUM, PWM1_TIMER_CC_NUM, 2, 3, 4}

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE

static uint32_t pwmN_gpiote_ch[] = PWMN_GPIOTE_CH;
static uint32_t pwmN_ppi_ch_a[] = PWMN_PPI_CH_A;
static uint32_t pwmN_ppi_ch_b[] = PWMN_PPI_CH_B;
static uint32_t pwmN_timer_cc_num[] = PWMN_TIMER_CC_NUM;

// TIMER3 reload value. The PWM frequency equals '16000000 / TIMER_RELOAD'
#define TIMER_RELOAD 255
// The timer CC register used to reset the timer. Be aware that not all timers in the nRF52 have 6 CC registers.
#define TIMER_RELOAD_CC_NUM 5

struct uart_data_t
{
	void *fifo_reserved;
	uint8_t data[8];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

// static const struct bt_data sd[] = {
// 	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
// };

// This function initializes timer 3 with the following configuration:
// 24-bit, base frequency 16 MHz, auto clear on COMPARE5 match (CC5 = TIMER_RELOAD)
void timer_init()
{
	NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos;
	NRF_TIMER3->PRESCALER = 0;
	NRF_TIMER3->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk << TIMER_RELOAD_CC_NUM;
	NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
	NRF_TIMER3->CC[TIMER_RELOAD_CC_NUM] = TIMER_RELOAD;
}

// Starts TIMER3
void timer_start()
{
	NRF_TIMER3->TASKS_START = 1;
}

// This function sets up TIMER3, the PPI and the GPIOTE modules to configure a single PWM channel
// Timer CC num, PPI channel nums and GPIOTE channel num is defined at the top of this file
void pwm0_init(uint32_t pinselect)
{
	NRF_GPIOTE->CONFIG[PWM0_GPIOTE_CH] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
										 GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
										 pinselect << GPIOTE_CONFIG_PSEL_Pos |
										 GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos;

	NRF_PPI->CH[PWM0_PPI_CH_A].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[PWM0_TIMER_CC_NUM];
	NRF_PPI->CH[PWM0_PPI_CH_A].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[PWM0_GPIOTE_CH];
	NRF_PPI->CH[PWM0_PPI_CH_B].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[TIMER_RELOAD_CC_NUM];
	NRF_PPI->CH[PWM0_PPI_CH_B].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[PWM0_GPIOTE_CH];

	NRF_PPI->CHENSET = (1 << PWM0_PPI_CH_A) | (1 << PWM0_PPI_CH_B);
}

// ### ------------------ TASK 1: STEP 1 ------------------
// ### Implement a method for initializing PWM channel 1, for a total of 2 individual PWM channels, and call it 'pwm1_init(uint32_t pinselect)'
// ### Hint: You can copy 'pwm0_init(..)' and use it as a starting point
void pwm1_init(uint32_t pinselect)
{
	NRF_GPIOTE->CONFIG[PWM1_GPIOTE_CH] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
										 GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
										 pinselect << GPIOTE_CONFIG_PSEL_Pos |
										 GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos;

	NRF_PPI->CH[PWM1_PPI_CH_A].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[PWM1_TIMER_CC_NUM];
	NRF_PPI->CH[PWM1_PPI_CH_A].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[PWM1_GPIOTE_CH];
	NRF_PPI->FORK[PWM0_PPI_CH_B].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[PWM1_GPIOTE_CH];
	NRF_PPI->CHENSET = (1 << PWM1_PPI_CH_A) | (1 << PWM0_PPI_CH_B);
}
// ### ----------------------------------------------------

// ### ------------------ TASK 1: STEP 2 ------------------
// ### Using the FORK feature of the PPI try to modify 'pwm1_init(..)' to reduce the number of PPI channels used
// ### (in total it should be sufficient to use 3 PPI channels for 2 PWM outputs).
// ### Hint: The FORK feature is useful when the same event needs to trigger several tasks.
// ### ----------------------------------------------------

// ### ------------------ TASK 1: STEP 6 (optional)--------
// ### Make a generic init function that takes both the PWM channel number and the pinselect as arguments,
// ### to avoid having to implement one function for each channel. The function should support up to 4 PWM channels total.
// ### Hint: Don't start on the optional steps until all the required steps are complete.
void pwmN_init(uint32_t N, uint32_t pinselect)
{
	if (N <= 5)
	{
		NRF_GPIO->DIRSET = 1 << pinselect;

		NRF_GPIOTE->CONFIG[pwmN_gpiote_ch[N]] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
												GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
												pinselect << GPIOTE_CONFIG_PSEL_Pos |
												GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos;

		NRF_PPI->CH[pwmN_ppi_ch_a[N]].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[pwmN_timer_cc_num[N]];
		NRF_PPI->CH[pwmN_ppi_ch_a[N]].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[pwmN_gpiote_ch[N]];
		if ((N % 2) == 0)
		{
			NRF_PPI->CH[pwmN_ppi_ch_b[N]].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[TIMER_RELOAD_CC_NUM];
			NRF_PPI->CH[pwmN_ppi_ch_b[N]].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[pwmN_gpiote_ch[N]];
		}
		else
		{
			NRF_PPI->FORK[pwmN_ppi_ch_b[N - 1]].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[pwmN_gpiote_ch[N]];
		}
		NRF_PPI->CHENSET = (1 << pwmN_ppi_ch_a[N]) | (1 << pwmN_ppi_ch_b[N]);
	}
}
// ### ----------------------------------------------------

// Function for changing the duty cycle on PWM channel 0
void pwm0_set_duty_cycle(uint32_t value)
{
	if (value == 0)
	{
		value = 1;
	}
	else if (value >= TIMER_RELOAD)
	{
		value = TIMER_RELOAD - 1;
	}
	NRF_TIMER3->CC[PWM0_TIMER_CC_NUM] = value;
}

// ### ------------------ TASK 1: STEP 3 ------------------
// ### Implement a method for setting the duty cycle on PWM channel 1, and call it 'pwm1_set_duty_cycle(uint32_t value)'
// ### Hint: You can copy 'pwm0_set_duty_cycle(..)' and use it as a starting point
void pwm1_set_duty_cycle(uint32_t value)
{
	if (value == 0)
	{
		value = 1;
	}
	else if (value >= TIMER_RELOAD)
	{
		value = TIMER_RELOAD - 1;
	}
	NRF_TIMER3->CC[PWM1_TIMER_CC_NUM] = value;
}
// ### ----------------------------------------------------

// ### ------------------ TASK 1: STEP 7 (optional) ----------
// ### Make a generic set duty cycle function to support a total of 4 PWM channels.
void pwmN_set_duty_cycle(uint32_t N, uint32_t value)
{
	if (N <= 5)
	{
		uint32_t pwmN_pin_assignment = (NRF_GPIOTE->CONFIG[pwmN_gpiote_ch[N]] & GPIOTE_CONFIG_PSEL_Msk) >> GPIOTE_CONFIG_PSEL_Pos;
		if (value == 0)
		{
			NRF_GPIOTE->CONFIG[pwmN_gpiote_ch[N]] &= ~GPIOTE_CONFIG_MODE_Msk;
			NRF_GPIOTE->CONFIG[pwmN_gpiote_ch[N]] |= GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos;
			NRF_GPIO->OUTCLR = (1 << pwmN_pin_assignment);
		}
		else if (value >= TIMER_RELOAD)
		{
			NRF_GPIOTE->CONFIG[pwmN_gpiote_ch[N]] &= ~GPIOTE_CONFIG_MODE_Msk;
			NRF_GPIOTE->CONFIG[pwmN_gpiote_ch[N]] |= GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos;
			NRF_GPIO->OUTSET = (1 << pwmN_pin_assignment);
		}
		else
		{
			NRF_GPIOTE->CONFIG[pwmN_gpiote_ch[N]] &= ~GPIOTE_CONFIG_MODE_Msk;
			NRF_GPIOTE->CONFIG[pwmN_gpiote_ch[N]] |= GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos;
			NRF_TIMER3->CC[pwmN_timer_cc_num[N]] = value;
		}
	}
}
// ### ----------------------------------------------------

// ### ------------------ TASK 1: STEP 8 (optional) ----------
// ### Find a better workaround for a duty cycle of 0% or 100%, so that it is possible to set the PWM output either constantly high or low.
// ### Implement the workaround in 'pwmN_set_duty_cycle(uint32_t N, uint32_t value)'
// ### -------------------------------------------------------

// Utility function for providing sin values, and converting them to integers.
// input values in the range [0 - input_max] will be converted to 0-360 degrees (0-2*PI).
// output values will be scaled to the range [output_min - output_max].
uint32_t sin_scaled(uint32_t input, uint32_t input_max, uint32_t output_min, uint32_t output_max)
{
	float sin_val = sinf((float)input * 2.0f * 3.141592f / (float)input_max);
	return (uint32_t)(((sin_val + 1.0f) / 2.0f) * (float)(output_max - output_min)) + output_min;
}

void turnOff()
{
	mode = 0x06;
	k_msleep(100);
	gpio_pin_interrupt_configure_dt(&mode_button, GPIO_INT_LEVEL_ACTIVE);
	k_msleep(10);
	sys_poweroff();
}

// int main(void)
// {
//     uint32_t counter = 0;

//     // Initialize the timer
//     timer_init();

//     // Initialize PWM channel 0
//     // pwm0_init(LED_1);

//     // ### ------------------ TASK 1: STEP 4 ------------------
//     // ### Call the init function implemented in STEP 1, and configure the additional PWM channel on LED_2.
//     // pwm1_init(LED_2);
//     // ### ----------------------------------------------------

//     // ### ------------------ TASK 1: STEP 9 (optional) -------
//     // ### Call the generic init function implemented in STEP 6, and configure 2 more PWM channels on LED_3 and LED_4.
//     // pwmN_init(2, LED_3);
//     // pwmN_init(3, LED_4);
//     // ### ----------------------------------------------------

//     // Start the timer
//     timer_start();

//     while (true)
//     {
//         nrf_delay_us(4000);

//         // Update the duty cycle of PWM channel 0 and increment the counter.
//         pwm0_set_duty_cycle(sin_scaled(counter++, 200, 0, TIMER_RELOAD));

//         // ### ------------------ TASK 1: STEP 5 ------------------
//         // ### Update the duty cycle of PWM channel 1, and add an offset to the counter to make the LED's blink out of phase.
//         pwm1_set_duty_cycle(sin_scaled(counter + 50, 200, 0, TIMER_RELOAD));
//         // ### ----------------------------------------------------

//         // ### ------------------ TASK 1: STEP 10 (optional) ------
//         // ### Update the duty cycle of PWM channel 2 and 3, using the generic functions implemented earlier.
//         pwmN_set_duty_cycle(2, sin_scaled(counter + 150, 200, 0, TIMER_RELOAD));
//         pwmN_set_duty_cycle(3, sin_scaled(counter + 100, 200, 0, TIMER_RELOAD));
//         // ### ----------------------------------------------------
//     }
// }

static char digit_bits[16] = {0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01100111, 0b01110111, 0b01111100, 0b01011000, 0b01011110, 0b01111001, 0b01110001};
char bit[8] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000};

void set_digit(int position, int digit, char red, char green, char blue)
{
	for (int i = 0; i < 7; i++)
	{
		if (digit_bits[digit] & bit[i])
		{
			disp[0][position][i] = red;
			disp[1][position][i] = green;
			disp[2][position][i] = blue;
		}
		else
		{
			disp[0][position][i] = 0b00000000;
			disp[1][position][i] = 0b00000000;
			disp[2][position][i] = 0b00000000;
		}
	}
}

void set_raw(int position, char bits, char red, char green, char blue)
{
	for (int i = 0; i < 7; i++)
	{
		if (bits & bit[i])
		{
			disp[0][position][i] = red;
			disp[1][position][i] = green;
			disp[2][position][i] = blue;
		}
		else
		{
			disp[0][position][i] = 0b00000000;
			disp[1][position][i] = 0b00000000;
			disp[2][position][i] = 0b00000000;
		}
	}
}

static void display_time()
{
	int displayHour;
	if (militaryTime == 0)
	{
		if (hour > 11)
		{
			displayHour = hour - 12;
			if (displayHour == 0)
			{
				displayHour = 12;
			}
		}
	}
	else
	{
		displayHour = hour;
	}
	set_raw(4, 0b00000100, clockColor[0] != 0, clockColor[1] != 0, clockColor[2] != 0);
	if (militaryTime == 0 && hour > 11)
	{
		set_raw(5, BIT(2) | alarmOn * BIT(1), clockColor[0], clockColor[1], clockColor[2]);
	}
	else
	{
		set_raw(5, alarmOn * BIT(1), clockColor[0], clockColor[1], clockColor[2]);
	}
	set_digit(0, displayHour / 10, clockColor[0], clockColor[1], clockColor[2]);
	set_digit(1, displayHour % 10, clockColor[0], clockColor[1], clockColor[2]);
	set_digit(2, minute / 10, clockColor[0], clockColor[1], clockColor[2]);
	set_digit(3, minute % 10, clockColor[0], clockColor[1], clockColor[2]);
}
static void display_alarm()
{
	int displayAlarmHour;
	if (militaryTime == 0)
	{
		displayAlarmHour = alarmHour % 12;
		if (displayAlarmHour == 0)
		{
			displayAlarmHour = 12;
		}
	}
	else
	{
		displayAlarmHour = alarmHour;
	}
	set_raw(4, 0b00000000, alarmColor[0] != 0, alarmColor[1] != 0, alarmColor[2] != 0);
	if (militaryTime == 0 && alarmHour > 11)
	{
		set_raw(5, BIT(2) | alarmOn * BIT(1),  alarmColor[0] != 0, alarmColor[2] != 0, alarmColor[1] != 0);
	}
	else
	{
		set_raw(5, alarmOn * BIT(1),  alarmColor[0] != 0, alarmColor[2] != 0, alarmColor[1] != 0);
	}
	set_digit(0, displayAlarmHour / 10, alarmColor[0], alarmColor[1], alarmColor[2]);
	set_digit(1, displayAlarmHour % 10, alarmColor[0], alarmColor[1], alarmColor[2]);
	set_digit(2, alarmMinute / 10, alarmColor[0], alarmColor[1], alarmColor[2]);
	set_digit(3, alarmMinute % 10, alarmColor[0], alarmColor[1], alarmColor[2]);
}

static void display_timer()
{
	set_digit(0, timerMinutes / 10, 128, 128, 128);
	set_digit(1, timerMinutes % 10, 128, 128, 128);
	set_digit(2, timerSeconds / 10, 128, 128, 128);
	set_digit(3, timerSeconds % 10, 128, 128, 128);
	set_raw(4, 0b00000000, 0, 0, 0);
	set_raw(5, 0b00000000, 0, 0, 0);
	if (timerSeconds > 99)
	{
		disp[0][5][3] = rightColor[0];
		disp[1][5][3] = rightColor[2];
		disp[2][5][3] = rightColor[1];
		disp[0][5][6] = rightColor[0];
		disp[1][5][6] = rightColor[2];
		disp[2][5][6] = rightColor[1];
	}
	else
	{
		disp[0][5][3] = 0;
		disp[1][5][3] = 0;
		disp[2][5][3] = 0;
		disp[0][5][6] = 0;
		disp[1][5][6] = 0;
		disp[2][5][6] = 0;
	}
}

void display_score()
{
	set_raw(4, 0b00000000, 0, 0, 0);
	set_raw(5, 0b00000000, 0, 0, 0);
	if (leftScore > 199)
	{
		leftScore = 199;
	}
	if (leftScore < 0)
	{
		leftScore = 0;
	}
	if (rightScore > 199)
	{
		rightScore = 199;
	}
	if (rightScore < 0)
	{
		rightScore = 0;
	}
	if (leftScore > 99)
	{
		disp[0][5][4] = leftColor[0];
		disp[1][5][4] = leftColor[2];
		disp[2][5][4] = leftColor[1];
		disp[0][5][5] = leftColor[0];
		disp[1][5][5] = leftColor[2];
		disp[2][5][5] = leftColor[1];
	}
	else
	{
		disp[0][5][4] = 0;
		disp[1][5][4] = 0;
		disp[2][5][4] = 0;
		disp[0][5][5] = 0;
		disp[1][5][5] = 0;
		disp[2][5][5] = 0;
	}
	set_digit(0, leftScore / 10 % 10, leftColor[0], leftColor[1], leftColor[2]);
	set_digit(1, leftScore % 10, leftColor[0], leftColor[1], leftColor[2]);
	if (rightScore > 99)
	{
		disp[0][5][3] = rightColor[0];
		disp[1][5][3] = rightColor[2];
		disp[2][5][3] = rightColor[1];
		disp[0][5][6] = rightColor[0];
		disp[1][5][6] = rightColor[2];
		disp[2][5][6] = rightColor[1];
	}
	else
	{
		disp[0][5][3] = 0;
		disp[1][5][3] = 0;
		disp[2][5][3] = 0;
		disp[0][5][6] = 0;
		disp[1][5][6] = 0;
		disp[2][5][6] = 0;
	}
	set_digit(2, rightScore / 10 % 10, rightColor[0], rightColor[1], rightColor[2]);
	set_digit(3, rightScore % 10, rightColor[0], rightColor[1], rightColor[2]);
}
void display_off()
{
	set_raw(0, 0b00000000, 0, 0, 0);
	set_raw(1, 0b00000000, 0, 0, 0);
	set_raw(2, 0b00000000, 0, 0, 0);
	set_raw(3, 0b00000000, 0, 0, 0);
	set_raw(4, 0b00000000, 0, 0, 0);
	set_raw(5, 0b00000000, 0, 0, 0);
}

void display_hex(char byte1, char byte2)
{
	set_digit(0, (byte1 & 0b11110000) >> 4, 245, 215, 66);
	set_digit(1, byte1 & 0b00001111, 245, 215, 66);
	set_digit(2, (byte2 & 0b11110000) >> 4, 245, 215, 66);
	set_digit(3, byte2 & 0b00001111, 245, 215, 66);
	set_digit(4, 0b00000000, 0, 0, 0);
	set_digit(5, 0b00000000, 0, 0, 0);
}

static void timer0_handler(struct k_timer *dummy)
{
	second = second + 1;
	if (second > 59)
	{
		minute = minute + 1;
		second = 0;
	}
	if (minute > 59)
	{
		hour++;
		minute = 0;
	}
	if (hour >= 24)
	{
		hour = 0;
	}
}

static void timer4_handler(struct k_timer *dummy)
{
	if (timerMinutes < 1 && timerSeconds < 1)
	{
		timerStarted = 2;
	}
	else if (timerSeconds < 1)
	{
		timerMinutes = timerMinutes - 1;
		timerSeconds = 59;
	}
	else
	{
		timerSeconds = timerSeconds - 1;
	}
}

void display_raw()
{
	set_raw(0, 0b00111111, 0, 255, 0);
	set_raw(1, 0b01110011, 0, 255, 0);
	set_raw(2, 0b01111001, 0, 255, 0);
	set_raw(3, 0b01010100, 0, 255, 0);
	set_digit(4, 0b00000000, 0, 0, 0);
	set_digit(5, 0b00000000, 0, 0, 0);
}

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

// static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
//  static struct k_work_delayable uart_work;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif

static void connected(struct bt_conn *conn, uint8_t err)
{
	// char addr[BT_ADDR_LE_STR_LEN];

	if (err)
	{
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	// bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	// LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);

	// dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn)
	{
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
		// dk_set_led_off(CON_STATUS_LED);
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
							 enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err)
	{
		LOG_INF("Security changed: %s level %u", addr, level);
	}
	else
	{
		LOG_WRN("Security failed: %s level %u err %d", addr,
				level, err);
	}
}
#endif

// #ifdef CONFIG_BT_LBS_SECURITY_ENABLED
// static void security_changed(struct bt_conn *conn, bt_security_t level,
// 			     enum bt_security_err err)
// {
// 	char addr[BT_ADDR_LE_STR_LEN];

// 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// 	if (!err) {
// 		printk("Security changed: %s level %u\n", addr, level);
// 	} else {
// 		printk("Security failed: %s level %u err %d\n", addr, level,
// 			err);
// 	}
// }
// #endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
//.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
						  uint16_t len)
{
	switch (data[0])
	{
	case 0x00: // left++
		leftScore++;
		break;
	case 0x01: // left--
		leftScore--;
		break;
	case 0x02: // right++
		rightScore++;
		break;
	case 0x03: // right--
		rightScore--;
		break;
	case 0x04: // resetScores
		rightScore = 0;
		leftScore = 0;
		break;
	case 0x05: // setMode
		if (len > 1)
		{
			mode = data[1];
		}
		else
		{
			mode = 0x00;
		}
		break;
	case 0x06: // setTime
		if (len > 2)
		{
			timerStarted = 1;

			hour = data[1];
			minute = data[2];
			second = data[3];
		}
		break;
	case 0x07: // displayHex
		mode = 0x02;
		break;
	case 0x08: // setTimer
		timerMinutes = data[1];
		timerSeconds = data[2];
		if (timerSeconds > 59 && timerMinutes > 0)
		{
			timerMinutes = timerMinutes + timerSeconds / 60;
			timerSeconds = timerSeconds % 60;
		}
		break;
	case 0x09: // startTimer
		timerStarted = 3;
		break;
	case 0x0A: // displayTimer
		mode = 0x04;
		break;

	default:
	}
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	// for (uint16_t pos = 0; pos != len;) {
	// 	struct uart_data_t *tx = k_malloc(sizeof(*tx));

	// 	if (!tx) {
	// 		LOG_WRN("Not able to allocate UART send data buffer");
	// 		return;
	// 	}

	// 	/* Keep the last byte of TX buffer for potential LF char. */
	// 	size_t tx_data_size = sizeof(tx->data) - 1;

	// 	if ((len - pos) > tx_data_size) {
	// 		tx->len = tx_data_size;
	// 	} else {
	// 		tx->len = (len - pos);
	// 	}

	// 	memcpy(tx->data, &data[pos], tx->len);
	// 	pos += tx->len;
	// 	/* Append the LF character when the CR character triggered
	// 	 * transmission from the peer.
	// 	 */
	// 	if ((pos == len) && (data[len - 1] == '\r')) {
	// 		tx->data[tx->len] = '\n';
	// 		tx->len++;
	// 	}

	// 	// err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	// 	// if (err) {
	// 	// 	k_fifo_put(&fifo_uart_tx_data, tx);
	// 	// }
	// }
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	// dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true)
	{
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept)
	{
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	}
	else
	{
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	// if (auth_conn) {
	// 	if (buttons & KEY_PASSKEY_ACCEPT) {
	// 		num_comp_reply(true);
	// 	}

	// 	if (buttons & KEY_PASSKEY_REJECT) {
	// 		num_comp_reply(false);
	// 	}
	// }
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

/** .. include_startingpoint_pkg_def_launchapp_rst */
/* Package: no.nordicsemi.android.nrftoolbox */
static const uint8_t android_pkg_name[] = {
	'n', 'o', '.', 'n', 'o', 'r', 'd', 'i', 'c', 's', 'e', 'm', 'i', '.', 'a', 'n', 'd', 'r',
	'o', 'i', 'd', '.', 'n', 'r', 'f', 't', 'o', 'o', 'l', 'b', 'o', 'x'};

/* URI nrf-toolbox://main/ */
static const uint8_t universal_link[] = {
	'n', 'r', 'f', '-', 't', 'o', 'o', 'l', 'b', 'o', 'x', ':', '/', '/', 'm', 'a', 'i', 'n',
	'/'};
/** .. include_endpoint_pkg_def_launchapp_rst */

/* Buffer used to hold an NFC NDEF message. */
static uint8_t ndef_msg_buf[NDEF_MSG_BUF_SIZE];

static void nfc_callback(void *context,
						 nfc_t2t_event_t event,
						 const uint8_t *data,
						 size_t data_length)
{
	ARG_UNUSED(context);
	ARG_UNUSED(data);
	ARG_UNUSED(data_length);

	switch (event)
	{
	case NFC_T2T_EVENT_FIELD_ON:
		// dk_set_led_on(NFC_FIELD_LED);
		break;
	case NFC_T2T_EVENT_FIELD_OFF:
		// dk_set_led_off(NFC_FIELD_LED);
		break;
	default:
		break;
	}
}

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (!buttonLockout)
	{
		k_work_submit_to_queue(&work_queue, &button_work);
	}
}

static void button_work_handler(struct k_work *work)
{
	buttonLockout = true;
	bool pinArray[3] = {gpio_pin_get_dt(&left_button), gpio_pin_get_dt(&right_button), gpio_pin_get_dt(&mode_button)};
	int buttonTimeIter = 0;
	int buttonTimePollingRateMS = 10;
	int buttonTimeShortpress = 50;
	int buttonTimeLongpress = 500;
	int buttonTimeLongpressRepeat = 500;
	int buttonTimeLongpressAcceleration = 50;
	int buttonTimeLongpressRepeatMin = 100;
	while (1) // wait until all buttons are no longer pushed or a button/combination of buttons has been longpressed;
	{
		bool newPinArray[3] = {gpio_pin_get_dt(&left_button), gpio_pin_get_dt(&right_button), gpio_pin_get_dt(&mode_button)};
		if (!newPinArray[0] && !newPinArray[1] && !newPinArray[2])
		{
			break;
		}
		if (pinArray[0] == newPinArray[0] && pinArray[1] == newPinArray[1] && pinArray[2] == newPinArray[2])
		{
			buttonTimeIter = buttonTimeIter + buttonTimePollingRateMS;
		}
		else
		{
			if(buttonTimeIter >= buttonTimeShortpress)
			{
				break;
			}
			pinArray[0] = newPinArray[0];
			pinArray[1] = newPinArray[1];
			pinArray[2] = newPinArray[2];
			buttonTimeIter = 0;
		}
		if (buttonTimeIter >= buttonTimeLongpress)
		{
			break;
		}
		k_msleep(buttonTimePollingRateMS);
	}
	// now pinArray should have the button combination debounced and buttonTimeIter should have the time in ms the button was pushed
	switch (mode)
	{
	case 0x00:
		if (buttonTimeIter < buttonTimeShortpress)
		{
			buttonLockout = false;
			return;
		}
		else if (buttonTimeIter < buttonTimeLongpress)
		{
			if (pinArray[0] == true && pinArray[1] == false && pinArray[2] == false) // left button shortpress
			{
				if (leftScore < 199)
				{
					leftScore++;
				}
			}
			else if (pinArray[0] == false && pinArray[1] == true && pinArray[2] == false) // right button shortpress
			{
				if (rightScore < 199)
				{
					rightScore++;
				}
			}
			else if (pinArray[0] == false && pinArray[1] == false && pinArray[2] == true) // mode button shortpress
			{
				mode = 0x04;
			}
			else if (pinArray[0] == true && pinArray[1] == true && pinArray[2] == false) // right and left button longpress
			{
				rightScore = 0;
				leftScore = 0;
			}
			else
			{
				buttonLockout = false;
				return;
			}
		}
		else if (buttonTimeIter >= buttonTimeLongpress)
		{
			if (pinArray[0] == true && pinArray[1] == false && pinArray[2] == false) // left button longpress
			{
				while (gpio_pin_get_dt(&left_button) && !gpio_pin_get_dt(&right_button) && !gpio_pin_get_dt(&mode_button))
				{
					if (leftScore > 0)
					{
						leftScore--;
					}
					k_msleep(buttonTimeLongpressRepeat);
					buttonTimeLongpressRepeat = buttonTimeLongpressRepeat - buttonTimeLongpressAcceleration;
					if (buttonTimeLongpressRepeat < buttonTimeLongpressRepeatMin)
					{
						buttonTimeLongpressRepeat = buttonTimeLongpressRepeatMin;
					}
				}
			}
			else if (pinArray[0] == false && pinArray[1] == true && pinArray[2] == false) // right buttonlongpress
			{
				while (!gpio_pin_get_dt(&left_button) && gpio_pin_get_dt(&right_button) && !gpio_pin_get_dt(&mode_button))
				{
					if (rightScore > 0)
					{
						rightScore--;
					}
					k_msleep(buttonTimeLongpressRepeat);
					buttonTimeLongpressRepeat = buttonTimeLongpressRepeat - buttonTimeLongpressAcceleration;
					if (buttonTimeLongpressRepeat < buttonTimeLongpressRepeatMin)
					{
						buttonTimeLongpressRepeat = buttonTimeLongpressRepeatMin;
					}
				}
			}
			else if (pinArray[0] == true && pinArray[1] == true && pinArray[2] == false) // right and left button longpress
			{
				rightScore = 0;
				leftScore = 0;
			}
			else if (pinArray[0] == false && pinArray[1] == false && pinArray[2] == true) // mode button longpress
			{
				turnOff();
			}
			else
			{
				buttonLockout = false;
				return;
			}
		}
		else
		{
			buttonLockout = false;
			return;
		}
		break;

	case 0x04: // timer mode
		if (buttonTimeIter < buttonTimeShortpress)
		{
			buttonLockout = false;
			return;
		}
		else if (buttonTimeIter < buttonTimeLongpress)
		{
			if (pinArray[0] == true && pinArray[1] == false && pinArray[2] == false) // left button shortpress
			{
				if (timerStarted == 0 || timerStarted == 2)
				{
					if (timerMinutes < 199)
					{
						timerMinutes = timerMinutes + 1;
					}
				}
			}
			else if (pinArray[0] == false && pinArray[1] == true && pinArray[2] == false) // right button shortpress
			{
				if (timerStarted == 0 || timerStarted == 2)
				{
					if (timerSeconds < 59)
					{
						timerSeconds = timerSeconds + 1;
					}
					else if(timerMinutes<199)
					{
						timerSeconds = 0;
						timerMinutes = timerMinutes + 1;
					}
				}
			}
			else if (pinArray[0] == true && pinArray[1] == true && pinArray[2] == false) // right and left button longpress
			{
				if(timerStarted==0 || timerStarted==2){
					timerStarted=3;
				}
				else if(timerStarted==1 || timerStarted==3){
					timerStarted=2;
				
				}else{
					timerStarted=2;
				}
			}
			else if (pinArray[0] == false && pinArray[1] == false && pinArray[2] == true) // mode button shortpress
			{
				mode = 0x01;
			}
			else
			{
				buttonLockout = false;
				return;
			}
		}
		else if (buttonTimeIter >= buttonTimeLongpress)
		{
			if (pinArray[0] == true && pinArray[1] == false && pinArray[2] == false) // left button longpress
			{
				while (gpio_pin_get_dt(&left_button) && !gpio_pin_get_dt(&right_button) && !gpio_pin_get_dt(&mode_button))
				{
					if (timerStarted == 0 || timerStarted == 2)
					{
						if (timerMinutes > 0)
						{
							timerMinutes = timerMinutes -1;
						}
						else{
							timerSeconds=0;
						}
					}
					k_msleep(buttonTimeLongpressRepeat);
					buttonTimeLongpressRepeat = buttonTimeLongpressRepeat - buttonTimeLongpressAcceleration;
					if (buttonTimeLongpressRepeat < buttonTimeLongpressRepeatMin)
					{
						buttonTimeLongpressRepeat = buttonTimeLongpressRepeatMin;
					}
				}
			}
			else if (pinArray[0] == false && pinArray[1] == true && pinArray[2] == false) // right buttonlongpress
			{
				while (!gpio_pin_get_dt(&left_button) && gpio_pin_get_dt(&right_button) && !gpio_pin_get_dt(&mode_button))
				{
					if (timerStarted == 0 || timerStarted == 2)
					{
						if (timerSeconds > 0)
						{
							timerSeconds = timerSeconds - 1;
						}
						else if(timerMinutes>0)
						{
							timerSeconds = 59;
							timerMinutes = timerMinutes - 1;
						}
					}
					k_msleep(buttonTimeLongpressRepeat);
					buttonTimeLongpressRepeat = buttonTimeLongpressRepeat - buttonTimeLongpressAcceleration;
					if (buttonTimeLongpressRepeat < buttonTimeLongpressRepeatMin)
					{
						buttonTimeLongpressRepeat = buttonTimeLongpressRepeatMin;
					}
				}
			}
			else if (pinArray[0] == true && pinArray[1] == true && pinArray[2] == false) // right and left button longpress
			{
				if(timerStarted==0 || timerStarted==2){
					timerStarted=3;
				}
				else if(timerStarted==1 || timerStarted==3){
					timerStarted=2;
				
				}else{
					timerStarted=2;
				}

			}
			else if (pinArray[0] == false && pinArray[1] == false && pinArray[2] == true) // mode button longpress
			{
				turnOff();
			}
			else
			{
				buttonLockout = false;
				return;
			}
		}
		else
		{
			buttonLockout = false;
			return;
		}
		break;
	case 0x01: // clock mode
		if (buttonTimeIter < buttonTimeShortpress)
		{
			buttonLockout = false;
			return;
		}
		else if (buttonTimeIter < buttonTimeLongpress)
		{
			if (pinArray[0] == true && pinArray[1] == false && pinArray[2] == false) // left button shortpress
			{
				second = 0;
				if (hour < 23)
				{
					hour++;
				}
				else
				{
					hour = 0;
				}
			}
			else if (pinArray[0] == false && pinArray[1] == true && pinArray[2] == false) // right button shortpress
			{
				second = 0;
				if (minute < 59)
				{
					minute++;
				}
				else if (hour < 23)
				{
					hour++;
					minute = 0;
				}
				else
				{
					hour = 0;
					minute = 0;
				}
			}
			else if (pinArray[0] == true && pinArray[1] == true && pinArray[2] == false) // right and left button longpress
			{
								if (militaryTime == 0)
				{
					militaryTime = 1;
				}
				else
				{
					militaryTime = 0;
				}
			}
			else if (pinArray[0] == false && pinArray[1] == false && pinArray[2] == true) // mode button shortpress
			{
				mode = 0x05;
			}
			else
			{
				buttonLockout = false;
				return;
			}
		}
		else if (buttonTimeIter >= buttonTimeLongpress)
		{
			if (pinArray[0] == true && pinArray[1] == false && pinArray[2] == false) // left button longpress
			{

				while (gpio_pin_get_dt(&left_button) && !gpio_pin_get_dt(&right_button) && !gpio_pin_get_dt(&mode_button))
				{
					second = 0;
					 if (hour > 0)
					{
						hour--;
					}
					else
					{
						hour = 23;
					}
					k_msleep(buttonTimeLongpressRepeat);
					buttonTimeLongpressRepeat = buttonTimeLongpressRepeat - buttonTimeLongpressAcceleration;
					if (buttonTimeLongpressRepeat < buttonTimeLongpressRepeatMin)
					{
						buttonTimeLongpressRepeat = buttonTimeLongpressRepeatMin;
					}
				}
			}
			else if (pinArray[0] == false && pinArray[1] == true && pinArray[2] == false) // right buttonlongpress
			{
				while (!gpio_pin_get_dt(&left_button) && gpio_pin_get_dt(&right_button) && !gpio_pin_get_dt(&mode_button))
				{
					second = 0;
					if (minute > 0)
					{
						minute--;
					}
					else if (alarmHour >0)
					{
						hour--;
						minute = 0;
					}
					else
					{
						hour = 23;
						minute = 59;
					}
					k_msleep(buttonTimeLongpressRepeat);
					buttonTimeLongpressRepeat = buttonTimeLongpressRepeat - buttonTimeLongpressAcceleration;
					if (buttonTimeLongpressRepeat < buttonTimeLongpressRepeatMin)
					{
						buttonTimeLongpressRepeat = buttonTimeLongpressRepeatMin;
					}
				}
			}
			else if (pinArray[0] == true && pinArray[1] == true && pinArray[2] == false) // right and left button longpress
			{
				if (militaryTime == 0)
				{
					militaryTime = 1;
				}
				else
				{
					militaryTime = 0;
				}
			}
			else if (pinArray[0] == false && pinArray[1] == false && pinArray[2] == true) // mode button longpress
			{
				turnOff();
			}
			else
			{
				buttonLockout = false;
				return;
			}
		}
		else
		{
			buttonLockout = false;
			return;
		}
		break;

	case 0x05: // alarm set mode
		if (buttonTimeIter < buttonTimeShortpress)
		{
			buttonLockout = false;
			return;
		}
		else if (buttonTimeIter < buttonTimeLongpress)
		{
			if (pinArray[0] == true && pinArray[1] == false && pinArray[2] == false) // left button shortpress
			{
				if (alarmHour < 23)
				{
					alarmHour++;
				}
				else
				{
					alarmHour = 0;
				}
			}
			else if (pinArray[0] == false && pinArray[1] == true && pinArray[2] == false) // right button shortpress
			{
				if (alarmMinute < 59)
				{
					alarmMinute++;
				}
				else if (alarmHour < 23)
				{
					alarmHour++;
					alarmMinute = 0;
				}
				else
				{
					alarmHour = 0;
					alarmMinute = 0;
				}
			}
			else if (pinArray[0] == true && pinArray[1] == true && pinArray[2] == false) // right and left button shortpress
			{
				if (alarmOn == 0)
				{
					alarmOn = 1;
				}
				else
				{
					alarmOn = 0;
				}
			}
			else if (pinArray[0] == false && pinArray[1] == false && pinArray[2] == true) // mode button shortpress
			{
				mode = 0x00;
			}
			else
			{
				buttonLockout = false;
				return;
			}
		}
		else if (buttonTimeIter >= buttonTimeLongpress)
		{
			if (pinArray[0] == true && pinArray[1] == false && pinArray[2] == false) // left button longpress
			{
				while (gpio_pin_get_dt(&left_button) && !gpio_pin_get_dt(&right_button) && !gpio_pin_get_dt(&mode_button))
				{
					 if (alarmHour>0)
					{
						alarmHour--;
					}
					else
					{
						alarmHour = 23;
					}
					k_msleep(buttonTimeLongpressRepeat);
					buttonTimeLongpressRepeat = buttonTimeLongpressRepeat - buttonTimeLongpressAcceleration;
					if (buttonTimeLongpressRepeat < buttonTimeLongpressRepeatMin)
					{
						buttonTimeLongpressRepeat = buttonTimeLongpressRepeatMin;
					}
				}
			}
			else if (pinArray[0] == false && pinArray[1] == true && pinArray[2] == false) // right buttonlongpress
			{
				while (!gpio_pin_get_dt(&left_button) && gpio_pin_get_dt(&right_button) && !gpio_pin_get_dt(&mode_button))
				{
					if (alarmMinute > 0)
					{
						alarmMinute--;
					}
					else if (alarmHour>0)
					{
						alarmHour--;
						alarmMinute = 59;
					}
					else
					{
						alarmHour = 23;
						alarmMinute = 59;
					}
					k_msleep(buttonTimeLongpressRepeat);
					buttonTimeLongpressRepeat = buttonTimeLongpressRepeat - buttonTimeLongpressAcceleration;
					if (buttonTimeLongpressRepeat < buttonTimeLongpressRepeatMin)
					{
						buttonTimeLongpressRepeat = buttonTimeLongpressRepeatMin;
					}
				}
			}
			else if (pinArray[0] == true && pinArray[1] == true && pinArray[2] == false) // right and left button longpress
			{
				if (alarmOn == 0)
				{
					alarmOn = 1;
				}
				else
				{
					alarmOn = 0;
				}
			}
			else if (pinArray[0] == false && pinArray[1] == false && pinArray[2] == true) // mode button longpress
			{
				turnOff();
			}
			else
			{
				buttonLockout = false;
				return;
			}
		}
		else
		{
			buttonLockout = false;
			return;
		}
		break;

	default:
		if (buttonTimeIter < buttonTimeShortpress)
		{
			buttonLockout = false;
			return;
		}
		else if (buttonTimeIter >= buttonTimeLongpress && pinArray[0] == false && pinArray[1] == false && pinArray[2] == true)
		{
			turnOff();
		}
		else
		{
			mode = 0x00;
		}
		break;
	}

	buttonLockout = false;
	return;
}

static void configure_gpio(void)
{
	int err;

	gpio_pin_configure_dt(&segment_a, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&segment_b, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&segment_c, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&segment_d, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&segment_e, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&segment_f, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&segment_g, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&rb1_gpio, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&gb1_gpio, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&bb1_gpio, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&rb2_gpio, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&gb2_gpio, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&bb2_gpio, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&piezo, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&mode_button, GPIO_INPUT);
	gpio_pin_configure_dt(&left_button, GPIO_INPUT);
	gpio_pin_configure_dt(&right_button, GPIO_INPUT);

	gpio_pin_interrupt_configure_dt(&mode_button, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure_dt(&left_button, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_pin_interrupt_configure_dt(&right_button, GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&button_cb_data, button_pressed, BIT(left_button.pin) | BIT(right_button.pin) | BIT(mode_button.pin));
	gpio_add_callback(mode_button.port, &button_cb_data);

	gpio_pin_set_dt(&segment_a, 0);
	gpio_pin_set_dt(&segment_b, 0);
	gpio_pin_set_dt(&segment_c, 0);
	gpio_pin_set_dt(&segment_d, 0);
	gpio_pin_set_dt(&segment_e, 0);
	gpio_pin_set_dt(&segment_f, 0);
	gpio_pin_set_dt(&segment_g, 0);
	gpio_pin_set_dt(&rb1_gpio, 0);
	gpio_pin_set_dt(&gb1_gpio, 0);
	gpio_pin_set_dt(&bb1_gpio, 0);
	gpio_pin_set_dt(&rb2_gpio, 0);
	gpio_pin_set_dt(&gb2_gpio, 0);
	gpio_pin_set_dt(&bb2_gpio, 0);
	gpio_pin_set_dt(&piezo, 0);

	k_work_queue_start(&work_queue, button_stack, K_THREAD_STACK_SIZEOF(button_stack),
					   PRIORITY, NULL);

	k_work_init(&button_work, button_work_handler);
	// #ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	// 	err = dk_buttons_init(button_changed);
	// 	if (err) {
	// 		LOG_ERR("Cannot init buttons (err: %d)", err);
	// 	}
	// #endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	// 	err = dk_leds_init();
	// 	if (err) {
	// 		LOG_ERR("Cannot init LEDs (err: %d)", err);
	// 	}
}

static void configure_pwm()
{
	pwm_set_dt(&rl1, PWM_PERIOD_NS, 255);
	pwm_set_dt(&gl1, PWM_PERIOD_NS, 255);
	pwm_set_dt(&bl1, PWM_PERIOD_NS, 255);
	pwm_set_dt(&rl2, PWM_PERIOD_NS, 255);
	pwm_set_dt(&gl2, PWM_PERIOD_NS, 255);
	pwm_set_dt(&bl2, PWM_PERIOD_NS, 255);
	pwm_set_dt(&rr1, PWM_PERIOD_NS, 255);
	pwm_set_dt(&gr1, PWM_PERIOD_NS, 255);
	pwm_set_dt(&br1, PWM_PERIOD_NS, 255);
	pwm_set_dt(&rr2, PWM_PERIOD_NS, 255);
	pwm_set_dt(&gr2, PWM_PERIOD_NS, 255);
	pwm_set_dt(&br2, PWM_PERIOD_NS, 255);
	pwmN_init(0, 11);
	pwmN_init(2, 12);
	pwmN_init(3, 8);
	pwmN_set_duty_cycle(0, 0);
	pwmN_set_duty_cycle(2, 0);
	pwmN_set_duty_cycle(3, 0);
}

static void bt_tx_manager()
{
	uint8_t buf[2];

	if (oldRightScore != rightScore)
	{
		buf[0] = 0x00;
		buf[1] = rightScore;
		if (!bt_nus_send(NULL, buf, 2))
		{
			oldRightScore = rightScore;
		}
	}

	if (oldLeftScore != leftScore)
	{
		buf[0] = 0x01;
		buf[1] = leftScore;
		if (!bt_nus_send(NULL, buf, 2))
		{
			oldLeftScore = leftScore;
		}
	}

	// uint8_t oldMode=NULL;
	// uint8_t oldHour=NULL;
	// uint8_t oldMinute=NULL;
	// uint8_t oldSecond=NULL;
	// uint8_t oldTimerMinutes=NULL;
	// uint8_t oldTimerSeconds=NULL;
	// uint8_t oldLeftScore=NULL;
	// uint8_t oldLeftColor0=NULL;
	// uint8_t oldLeftColor1=NULL;
	// uint8_t oldLeftColor2=NULL;

	// uint8_t oldRightColor0=NULL;
	// uint8_t oldRightColor1=NULL;
	// uint8_t oldRightColor2=NULL;
	// uint8_t oldHexCache1=NULL;
	// uint8_t oldHexCache2=NULL;
}

int main(void)
{
	int err = 0;

	configure_gpio();
	configure_pwm();
	timer_init();
	timer_start();

	static K_TIMER_DEFINE(timer0, timer0_handler, NULL);
	static K_TIMER_DEFINE(timer4, timer4_handler, NULL);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			for (int k = 0; k < 7; k++)
			{
				disp[i][j][k] = (0b00000000);
			}
		}
	}

	smp_bt_register();
	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED))
	{
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err)
		{
			printk("Failed to register authorization callbacks.\n");
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err)
		{
			printk("Failed to register authorization info callbacks.\n");
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err)
	{
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err)
	{
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
						  ARRAY_SIZE(sd));
	if (err)
	{
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}

	size_t len = sizeof(ndef_msg_buf);

	/* Set up NFC */
	err = nfc_t2t_setup(nfc_callback, NULL);
	if (err)
	{
		printk("Cannot setup NFC T2T library!\n");
		// goto fail;
	}

	/* Encode launch app data  */
	err = nfc_launchapp_msg_encode(android_pkg_name,
								   sizeof(android_pkg_name),
								   universal_link,
								   sizeof(universal_link),
								   ndef_msg_buf,
								   &len);
	if (err)
	{
		printk("Cannot encode message!\n");
		// goto fail;
	}

	/* Set created message as the NFC payload */
	err = nfc_t2t_payload_set(ndef_msg_buf, len);
	if (err)
	{
		printk("Cannot set payload!\n");
		// goto fail;
	}

	/* Start sensing NFC field */
	err = nfc_t2t_emulation_start();
	if (err)
	{
		printk("Cannot start emulation!\n");
		// goto fail;
	}

	k_timer_start(&timer0, K_MSEC(1000), K_MSEC(1000));

	for (;;)
	{
		if (timerStarted == 3)
		{
			k_timer_start(&timer4, K_MSEC(1000), K_MSEC(1000));
			timerStarted = 1;
		}
		else if (timerStarted == 2)
		{
			k_timer_stop(&timer4);
			timerStarted = 0;
		}
		switch (mode)
		{
		case 0x00:
			display_score();
			break;
		case 0x01:
			display_time();
			break;
		case 0x02:
			display_hex(hexCache1, hexCache2);
			break;
		case 0x03:
			display_raw();
			break;
		case 0x04:
			display_timer();
			break;
		case 0x05:
			display_alarm();
			break;
		case 0x06:
			display_off();
			break;
		default:
		}
		bt_tx_manager();
		gpio_pin_set_dt(&segment_a, 1);
		pwm_set_dt(&rl1, PWM_PERIOD_NS, (disp[0][0][0]) * 100);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, (disp[1][0][0]) * 100);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, (disp[2][0][0]) * 100);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, (disp[0][1][0]) * 100);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, (disp[1][1][0]) * 100);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, (disp[2][1][0]) * 100);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, (disp[0][2][0]) * 100);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, (disp[1][2][0]) * 100);
		pwm_set_dt(&br1, PWM_PERIOD_NS, (disp[2][2][0]) * 100);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, (disp[0][3][0]) * 100);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, (disp[1][3][0]) * 100);
		pwm_set_dt(&br2, PWM_PERIOD_NS, (disp[2][3][0]) * 100);
		gpio_pin_set_dt(&rb1_gpio, (disp[0][4][0]) & 0x00000001);
		gpio_pin_set_dt(&gb1_gpio, (disp[1][4][0]) & 0x00000001);
		gpio_pin_set_dt(&bb1_gpio, (disp[2][4][0]) & 0x00000001);
		pwmN_set_duty_cycle(0, (disp[0][5][0]) * 100);
		pwmN_set_duty_cycle(2, (disp[1][5][0]) * 100);
		pwmN_set_duty_cycle(3, (disp[2][5][0]) * 100);
		k_sleep(K_USEC(10));
		pwm_set_dt(&rl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br2, PWM_PERIOD_NS, 0);
		pwmN_set_duty_cycle(0, 0);
		pwmN_set_duty_cycle(2, 0);
		pwmN_set_duty_cycle(3, 0);
		gpio_pin_set_dt(&segment_a, 0);
		gpio_pin_set_dt(&rb1_gpio, 1);
		gpio_pin_set_dt(&gb1_gpio, 1);
		gpio_pin_set_dt(&bb1_gpio, 1);
		k_sleep(K_USEC(1000));
		gpio_pin_set_dt(&rb1_gpio, 0);
		gpio_pin_set_dt(&gb1_gpio, 0);
		gpio_pin_set_dt(&bb1_gpio, 0);
		k_sleep(K_USEC(10));

		gpio_pin_set_dt(&segment_b, 1);
		pwm_set_dt(&rl1, PWM_PERIOD_NS, (disp[0][0][1]) * 100);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, (disp[1][0][1]) * 100);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, (disp[2][0][1]) * 100);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, (disp[0][1][1]) * 100);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, (disp[1][1][1]) * 100);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, (disp[2][1][1]) * 100);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, (disp[0][2][1]) * 100);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, (disp[1][2][1]) * 100);
		pwm_set_dt(&br1, PWM_PERIOD_NS, (disp[2][2][1]) * 100);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, (disp[0][3][1]) * 100);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, (disp[1][3][1]) * 100);
		pwm_set_dt(&br2, PWM_PERIOD_NS, (disp[2][3][1]) * 100);
		gpio_pin_set_dt(&rb1_gpio, (disp[0][4][1]) & 0x00000001);
		gpio_pin_set_dt(&gb1_gpio, (disp[1][4][1]) & 0x00000001);
		gpio_pin_set_dt(&bb1_gpio, (disp[2][4][1]) & 0x00000001);
		pwmN_set_duty_cycle(0, (disp[0][5][1]) * 100);
		pwmN_set_duty_cycle(2, (disp[1][5][1]) * 100);
		pwmN_set_duty_cycle(3, (disp[2][5][1]) * 100);
		k_sleep(K_USEC(10));
		pwm_set_dt(&rl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br2, PWM_PERIOD_NS, 0);
		pwmN_set_duty_cycle(0, 0);
		pwmN_set_duty_cycle(2, 0);
		pwmN_set_duty_cycle(3, 0);
		gpio_pin_set_dt(&segment_b, 0);
		gpio_pin_set_dt(&rb1_gpio, 1);
		gpio_pin_set_dt(&gb1_gpio, 1);
		gpio_pin_set_dt(&bb1_gpio, 1);
		k_sleep(K_USEC(1000));
		gpio_pin_set_dt(&rb1_gpio, 0);
		gpio_pin_set_dt(&gb1_gpio, 0);
		gpio_pin_set_dt(&bb1_gpio, 0);
		k_sleep(K_USEC(10));

		gpio_pin_set_dt(&segment_c, 1);
		pwm_set_dt(&rl1, PWM_PERIOD_NS, (disp[0][0][2]) * 100);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, (disp[1][0][2]) * 100);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, (disp[2][0][2]) * 100);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, (disp[0][1][2]) * 100);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, (disp[1][1][2]) * 100);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, (disp[2][1][2]) * 100);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, (disp[0][2][2]) * 100);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, (disp[1][2][2]) * 100);
		pwm_set_dt(&br1, PWM_PERIOD_NS, (disp[2][2][2]) * 100);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, (disp[0][3][2]) * 100);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, (disp[1][3][2]) * 100);
		pwm_set_dt(&br2, PWM_PERIOD_NS, (disp[2][3][2]) * 100);
		gpio_pin_set_dt(&rb1_gpio, (disp[0][4][2]) & 0x00000001);
		gpio_pin_set_dt(&gb1_gpio, (disp[1][4][2]) & 0x00000001);
		gpio_pin_set_dt(&bb1_gpio, (disp[2][4][2]) & 0x00000001);
		pwmN_set_duty_cycle(0, (disp[0][5][2]) * 100);
		pwmN_set_duty_cycle(2, (disp[1][5][2]) * 100);
		pwmN_set_duty_cycle(3, (disp[2][5][2]) * 100);
		k_sleep(K_USEC(10));
		pwm_set_dt(&rl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br2, PWM_PERIOD_NS, 0);
		pwmN_set_duty_cycle(0, 0);
		pwmN_set_duty_cycle(2, 0);
		pwmN_set_duty_cycle(3, 0);
		gpio_pin_set_dt(&segment_c, 0);
		gpio_pin_set_dt(&rb1_gpio, 1);
		gpio_pin_set_dt(&gb1_gpio, 1);
		gpio_pin_set_dt(&bb1_gpio, 1);
		k_sleep(K_USEC(1000));
		gpio_pin_set_dt(&rb1_gpio, 0);
		gpio_pin_set_dt(&gb1_gpio, 0);
		gpio_pin_set_dt(&bb1_gpio, 0);
		k_sleep(K_USEC(10));

		gpio_pin_set_dt(&segment_d, 1);
		pwm_set_dt(&rl1, PWM_PERIOD_NS, (disp[0][0][3]) * 100);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, (disp[1][0][3]) * 100);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, (disp[2][0][3]) * 100);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, (disp[0][1][3]) * 100);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, (disp[1][1][3]) * 100);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, (disp[2][1][3]) * 100);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, (disp[0][2][3]) * 100);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, (disp[1][2][3]) * 100);
		pwm_set_dt(&br1, PWM_PERIOD_NS, (disp[2][2][3]) * 100);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, (disp[0][3][3]) * 100);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, (disp[1][3][3]) * 100);
		pwm_set_dt(&br2, PWM_PERIOD_NS, (disp[2][3][3]) * 100);
		gpio_pin_set_dt(&rb1_gpio, (disp[0][4][3]) & 0x00000001);
		gpio_pin_set_dt(&gb1_gpio, (disp[1][4][3]) & 0x00000001);
		gpio_pin_set_dt(&bb1_gpio, (disp[2][4][3]) & 0x00000001);
		pwmN_set_duty_cycle(0, (disp[0][5][3]) * 100);
		pwmN_set_duty_cycle(2, (disp[1][5][3]) * 100);
		pwmN_set_duty_cycle(3, (disp[2][5][3]) * 100);
		k_sleep(K_USEC(10));
		pwm_set_dt(&rl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br2, PWM_PERIOD_NS, 0);
		pwmN_set_duty_cycle(0, 0);
		pwmN_set_duty_cycle(2, 0);
		pwmN_set_duty_cycle(3, 0);
		gpio_pin_set_dt(&segment_d, 0);
		gpio_pin_set_dt(&rb1_gpio, 1);
		gpio_pin_set_dt(&gb1_gpio, 1);
		gpio_pin_set_dt(&bb1_gpio, 1);
		k_sleep(K_USEC(1000));
		gpio_pin_set_dt(&rb1_gpio, 0);
		gpio_pin_set_dt(&gb1_gpio, 0);
		gpio_pin_set_dt(&bb1_gpio, 0);
		k_sleep(K_USEC(10));

		gpio_pin_set_dt(&segment_e, 1);
		pwm_set_dt(&rl1, PWM_PERIOD_NS, (disp[0][0][4]) * 100);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, (disp[1][0][4]) * 100);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, (disp[2][0][4]) * 100);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, (disp[0][1][4]) * 100);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, (disp[1][1][4]) * 100);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, (disp[2][1][4]) * 100);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, (disp[0][2][4]) * 100);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, (disp[1][2][4]) * 100);
		pwm_set_dt(&br1, PWM_PERIOD_NS, (disp[2][2][4]) * 100);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, (disp[0][3][4]) * 100);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, (disp[1][3][4]) * 100);
		pwm_set_dt(&br2, PWM_PERIOD_NS, (disp[2][3][4]) * 100);
		gpio_pin_set_dt(&rb1_gpio, (disp[0][4][4]) & 0x00000001);
		gpio_pin_set_dt(&gb1_gpio, (disp[1][4][4]) & 0x00000001);
		gpio_pin_set_dt(&bb1_gpio, (disp[2][4][4]) & 0x00000001);
		pwmN_set_duty_cycle(0, (disp[0][5][4]) * 100);
		pwmN_set_duty_cycle(2, (disp[1][5][4]) * 100);
		pwmN_set_duty_cycle(3, (disp[2][5][4]) * 100);
		k_sleep(K_USEC(10));
		pwm_set_dt(&rl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br2, PWM_PERIOD_NS, 0);
		pwmN_set_duty_cycle(0, 0);
		pwmN_set_duty_cycle(2, 0);
		pwmN_set_duty_cycle(3, 0);
		gpio_pin_set_dt(&segment_e, 0);
		gpio_pin_set_dt(&rb1_gpio, 1);
		gpio_pin_set_dt(&gb1_gpio, 1);
		gpio_pin_set_dt(&bb1_gpio, 1);
		k_sleep(K_USEC(1000));
		gpio_pin_set_dt(&rb1_gpio, 0);
		gpio_pin_set_dt(&gb1_gpio, 0);
		gpio_pin_set_dt(&bb1_gpio, 0);
		k_sleep(K_USEC(10));

		gpio_pin_set_dt(&segment_f, 1);
		pwm_set_dt(&rl1, PWM_PERIOD_NS, (disp[0][0][5]) * 100);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, (disp[1][0][5]) * 100);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, (disp[2][0][5]) * 100);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, (disp[0][1][5]) * 100);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, (disp[1][1][5]) * 100);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, (disp[2][1][5]) * 100);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, (disp[0][2][5]) * 100);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, (disp[1][2][5]) * 100);
		pwm_set_dt(&br1, PWM_PERIOD_NS, (disp[2][2][5]) * 100);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, (disp[0][3][5]) * 100);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, (disp[1][3][5]) * 100);
		pwm_set_dt(&br2, PWM_PERIOD_NS, (disp[2][3][5]) * 100);
		gpio_pin_set_dt(&rb1_gpio, (disp[0][4][5]) & 0x00000001);
		gpio_pin_set_dt(&gb1_gpio, (disp[1][4][5]) & 0x00000001);
		gpio_pin_set_dt(&bb1_gpio, (disp[2][4][5]) & 0x00000001);
		pwmN_set_duty_cycle(0, (disp[0][5][5]) * 100);
		pwmN_set_duty_cycle(2, (disp[1][5][5]) * 100);
		pwmN_set_duty_cycle(3, (disp[2][5][5]) * 100);
		k_sleep(K_USEC(10));
		pwm_set_dt(&rl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br2, PWM_PERIOD_NS, 0);
		pwmN_set_duty_cycle(0, 0);
		pwmN_set_duty_cycle(2, 0);
		pwmN_set_duty_cycle(3, 0);
		gpio_pin_set_dt(&segment_f, 0);
		gpio_pin_set_dt(&rb1_gpio, 1);
		gpio_pin_set_dt(&gb1_gpio, 1);
		gpio_pin_set_dt(&bb1_gpio, 1);
		k_sleep(K_USEC(1000));
		gpio_pin_set_dt(&rb1_gpio, 0);
		gpio_pin_set_dt(&gb1_gpio, 0);
		gpio_pin_set_dt(&bb1_gpio, 0);
		k_sleep(K_USEC(10));

		gpio_pin_set_dt(&segment_g, 1);
		pwm_set_dt(&rl1, PWM_PERIOD_NS, (disp[0][0][6]) * 100);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, (disp[1][0][6]) * 100);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, (disp[2][0][6]) * 100);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, (disp[0][1][6]) * 100);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, (disp[1][1][6]) * 100);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, (disp[2][1][6]) * 100);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, (disp[0][2][6]) * 100);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, (disp[1][2][6]) * 100);
		pwm_set_dt(&br1, PWM_PERIOD_NS, (disp[2][2][6]) * 100);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, (disp[0][3][6]) * 100);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, (disp[1][3][6]) * 100);
		pwm_set_dt(&br2, PWM_PERIOD_NS, (disp[2][3][6]) * 100);
		gpio_pin_set_dt(&rb1_gpio, (disp[0][4][6]) & 0x00000001);
		gpio_pin_set_dt(&gb1_gpio, (disp[1][4][6]) & 0x00000001);
		gpio_pin_set_dt(&bb1_gpio, (disp[2][4][6]) & 0x00000001);
		pwmN_set_duty_cycle(0, (disp[0][5][6]) * 100);
		pwmN_set_duty_cycle(2, (disp[1][5][6]) * 100);
		pwmN_set_duty_cycle(3, (disp[2][5][6]) * 100);
		k_sleep(K_USEC(10));
		pwm_set_dt(&rl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&bl2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br1, PWM_PERIOD_NS, 0);
		pwm_set_dt(&rr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&gr2, PWM_PERIOD_NS, 0);
		pwm_set_dt(&br2, PWM_PERIOD_NS, 0);
		pwmN_set_duty_cycle(0, 0);
		pwmN_set_duty_cycle(2, 0);
		pwmN_set_duty_cycle(3, 0);
		gpio_pin_set_dt(&segment_g, 0);
		gpio_pin_set_dt(&rb1_gpio, 1);
		gpio_pin_set_dt(&gb1_gpio, 1);
		gpio_pin_set_dt(&bb1_gpio, 1);
		k_sleep(K_USEC(1000));
		gpio_pin_set_dt(&rb1_gpio, 0);
		gpio_pin_set_dt(&gb1_gpio, 0);
		gpio_pin_set_dt(&bb1_gpio, 0);
		k_sleep(K_USEC(10));
	}
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);
	struct uart_data_t nus_data = {
		.len = 0,
	};

	for (;;)
	{
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
											 K_FOREVER);

		int plen = MIN(sizeof(nus_data.data) - nus_data.len, buf->len);
		int loc = 0;

		while (plen > 0)
		{
			memcpy(&nus_data.data[nus_data.len], &buf->data[loc], plen);
			nus_data.len += plen;
			loc += plen;

			if (nus_data.len >= sizeof(nus_data.data) ||
				(nus_data.data[nus_data.len - 1] == '\n') ||
				(nus_data.data[nus_data.len - 1] == '\r'))
			{
				if (bt_nus_send(NULL, nus_data.data, nus_data.len))
				{
					LOG_WRN("Failed to send data over BLE connection");
				}
				nus_data.len = 0;
			}

			plen = MIN(sizeof(nus_data.data), buf->len - loc);
		}

		k_free(buf);
	}
}

// K_THREAD_DEFINE(ble_write_thread_id, 8, ble_write_thread, NULL, NULL,
// 				NULL, PRIORITY, 0, 0);