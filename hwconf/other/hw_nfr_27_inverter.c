#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
// #include "drv8301.h"
#include "terminal.h"
#include "commands.h"
#include "mc_interface.h"
#include <pal.h>

// Variables
static volatile bool i2c_running = false;
// Not sure what these are for
static mutex_t shutdown_mutex;
// static float bt_diff = 0.0;


// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};


// static void terminal_shutdown_now(int argc, const char **argv);
// static void terminal_button_test(int argc, const char **argv);
static void terminal_pwm1(int argc, const char **argv);
static void terminal_bs(int argc, const char **argv);
static void terminal_precharge(int argc, const char **argv);
extern volatile bool mcpwm_foc_bs_precharge_enabled;

void hw_init_gpio(void) {

	chMtxObjectInit(&shutdown_mutex);

	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	palSetPadMode(GPIOB, 5,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	ENABLE_GATE();

	// Current filter
	palSetPadMode(GPIOD, 2,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	CURRENT_FILTER_OFF();

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// LEDs. Must come after the hall pins: PC6/PC7 are shared with HALL1/HALL2,
	// and the hall setup above would otherwise turn the LED pins into inputs.
	// NFR27 uses the resolver, so the hall inputs are not needed.
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// Phase filters
#ifdef PHASE_FILTER_GPIO
	palSetPadMode(PHASE_FILTER_GPIO, PHASE_FILTER_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	PHASE_FILTER_OFF();
#endif

	// Sensor port voltage
// #if defined(HW60_IS_MK6)
// 	SENSOR_PORT_3V3();
// 	palSetPadMode(SENSOR_VOLTAGE_GPIO, SENSOR_VOLTAGE_PIN,
// 			PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
// #endif

	// Fault pin
	palSetPadMode(GPIOB, 7, PAL_MODE_INPUT_PULLUP);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);


    // Maybe get rid of this?????? ------------------------------------------------------------------------------------------------------
	// drv8301_init();

	// Add this back for shutdown button functionality -------------------------------------------------------
	// terminal_register_command_callback(
	// 	"shutdown",
	// 	"Shutdown VESC now.",
	// 	0,
	// 	terminal_shutdown_now);

	// terminal_register_command_callback(
	// 	"test_button",
	// 	"Try sampling the shutdown button",
	// 	0,
	// 	terminal_button_test);

	terminal_register_command_callback(
		"nfr_pwm1",
		"Bench test: 50% complementary PWM on one phase only, others off. Motor must be off.",
		"[phase 1-3] [ms 1-5000, default 1000]",
		terminal_pwm1);

	terminal_register_command_callback(
		"nfr_bs",
		"Bench test: mode 1 = low side on 5 ms; mode 2 = that, then one 5 us high-side pulse. Times DRV/vin.",
		"[phase 1-3] [mode 1-2]",
		terminal_bs);

	terminal_register_command_callback(
		"nfr_precharge",
		"Show or set the bootstrap precharge before PWM start (not saved, on after reboot).",
		"[0|1]",
		terminal_precharge);
}

static void terminal_precharge(int argc, const char **argv) {
	if (argc >= 2) {
		int en = 1;
		sscanf(argv[1], "%d", &en);
		mcpwm_foc_bs_precharge_enabled = en != 0;
	}
	commands_printf("Bootstrap precharge: %s\n", mcpwm_foc_bs_precharge_enabled ? "ON" : "OFF");
}

// Single-output drive of one leg. OSSR is set, so a disabled output sits at its
// inactive level (FET off). Only one output of the leg is ever enabled at a time.
static void bs_leg_off(uint16_t c) {
	TIM_SelectOCxM(TIM1, c, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, c, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, c, TIM_CCxN_Disable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}

static void bs_low_on(uint16_t c) {
	TIM_SelectOCxM(TIM1, c, TIM_ForcedAction_Active);
	TIM_CCxCmd(TIM1, c, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, c, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}

static void bs_high_on(uint16_t c) {
	TIM_SelectOCxM(TIM1, c, TIM_ForcedAction_Active);
	TIM_CCxCmd(TIM1, c, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, c, TIM_CCxN_Disable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}

static inline uint32_t bs_us(uint32_t t0) {
	return (DWT->CYCCNT - t0) / (SYSTEM_CORE_CLOCK / 1000000);
}

static void bs_wait_us(uint32_t t0, uint32_t us) {
	while (bs_us(t0) < us) {}
}

/*
 * Bootstrap / high-side event test on one leg, with microsecond timing of when the
 * DRV line goes low and when the bus reading (AMC1311 path) collapses.
 * mode 1: low side on for 5 ms (charges that leg's bootstrap cap), then off.
 * mode 2: as mode 1, then 10 us with both off, then a single 5 us high-side pulse.
 * Monitors until 30 ms after the start.
 */
static void terminal_bs(int argc, const char **argv) {
	int ph = 0;
	int mode = 0;

	if (argc >= 2) { sscanf(argv[1], "%d", &ph); }
	if (argc >= 3) { sscanf(argv[2], "%d", &mode); }

	if (ph < 1 || ph > 3 || mode < 1 || mode > 2) {
		commands_printf("Usage: nfr_bs [phase 1-3] [mode 1-2]\n");
		return;
	}

	if (mc_interface_get_state() != MC_STATE_OFF || mc_interface_get_fault() != FAULT_CODE_NONE) {
		commands_printf("Motor must be off with no active fault.\n");
		return;
	}

	static const uint16_t chs[3] = {TIM_Channel_1, TIM_Channel_2, TIM_Channel_3};
	const uint16_t c = chs[ph - 1];

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	const float v_start = GET_INPUT_VOLTAGE();
	int t_drv = -1, t_vlow = -1, t_ls_off = -1, t_hs_on = -1, t_hs_off = -1;
	bool hs_done = (mode != 2);
	bool ls_on = true;

	const uint32_t t0 = DWT->CYCCNT;
	bs_low_on(c);

	for (;;) {
		uint32_t now = bs_us(t0);

		if (ls_on && now >= 5000) {
			bs_leg_off(c);
			ls_on = false;
			t_ls_off = bs_us(t0);
		}

		if (!ls_on && !hs_done && (t_drv >= 0 || t_vlow >= 0 ||
				mc_interface_get_fault() != FAULT_CODE_NONE)) {
			hs_done = true; // something already failed: don't fire the high side
		}

		if (!ls_on && !hs_done && now >= (uint32_t)t_ls_off + 10) {
			chSysLock();
			t_hs_on = bs_us(t0);
			bs_high_on(c);
			bs_wait_us(t0, t_hs_on + 5);
			bs_leg_off(c);
			t_hs_off = bs_us(t0);
			chSysUnlock();
			hs_done = true;
		}

		if (t_drv < 0 && IS_DRV_FAULT()) { t_drv = bs_us(t0); }
		if (t_vlow < 0 && GET_INPUT_VOLTAGE() < 0.7 * v_start) { t_vlow = bs_us(t0); }

		if (now >= 30000) {
			break;
		}
	}

	bs_leg_off(c);

	commands_printf("nfr_bs phase %d mode %d (vin start %.2f, now %.2f)\n", ph, mode,
			(double)v_start, (double)GET_INPUT_VOLTAGE());
	commands_printf("  t [us]: low on 0, low off %d, high on %d, high off %d\n", t_ls_off, t_hs_on, t_hs_off);
	commands_printf("  DRV low first at %d us, vin < 70%% first at %d us (-1 = never)\n", t_drv, t_vlow);
	commands_printf("  fault now: %s\n", mc_interface_fault_to_string(mc_interface_get_fault()));
}

/*
 * Single-phase PWM bench test. While the motor is off the FOC ISR holds all CCRs at
 * ARR/2, so enabling one channel gives 50% complementary PWM (with dead time) on that
 * leg. Normal fault handling stays active: a DRV or voltage fault calls stop_pwm_hw(),
 * which turns the leg off; this is detected and reported.
 */
static void terminal_pwm1(int argc, const char **argv) {
	int ph = 0;
	int ms = 1000;

	if (argc >= 2) {
		sscanf(argv[1], "%d", &ph);
	}
	if (argc >= 3) {
		sscanf(argv[2], "%d", &ms);
	}

	if (ph < 1 || ph > 3 || ms < 1 || ms > 5000) {
		commands_printf("Usage: nfr_pwm1 [phase 1-3] [ms 1-5000]\n");
		return;
	}

	if (mc_interface_get_state() != MC_STATE_OFF || mc_interface_get_fault() != FAULT_CODE_NONE) {
		commands_printf("Motor must be off with no active fault.\n");
		return;
	}

	static const uint16_t chs[3] = {TIM_Channel_1, TIM_Channel_2, TIM_Channel_3};
	static const uint32_t cne[3] = {TIM_CCER_CC1NE, TIM_CCER_CC2NE, TIM_CCER_CC3NE};
	const uint16_t c = chs[ph - 1];

	const float v_start = GET_INPUT_VOLTAGE();
	float v_min = v_start, v_max = v_start;
	const float ph_scale = (V_REG / 4095.0) * ((VIN_R1 + VIN_R2) / VIN_R2);
	const int ph_ind[3] = {ADC_IND_SENS1, ADC_IND_SENS2, ADC_IND_SENS3};
	float ph_before[3], ph_sum[3] = {0.0, 0.0, 0.0};
	int ph_n = 0;
	for (int k = 0;k < 3;k++) {
		ph_before[k] = (float)ADC_Value[ph_ind[k]] * ph_scale;
	}
	int drv_samples = 0;
	int stopped_at = -1;
	int low_at = -1;

	TIM_SelectOCxM(TIM1, c, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, c, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, c, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	for (int i = 0;i < ms;i++) {
		chThdSleepMilliseconds(1);

		float v = GET_INPUT_VOLTAGE();
		if (v < v_min) { v_min = v; }
		if (v > v_max) { v_max = v; }
		if (IS_DRV_FAULT()) { drv_samples++; }
		for (int k = 0;k < 3;k++) {
			ph_sum[k] += (float)ADC_Value[ph_ind[k]] * ph_scale;
		}
		ph_n++;

		if (!(TIM1->CCER & cne[ph - 1])) {
			stopped_at = i; // the firmware turned the leg off (fault)
			break;
		}

		if (v < 0.7 * v_start) {
			low_at = i; // bus/AMC1311 reading collapsed
			break;
		}
	}

	TIM_SelectOCxM(TIM1, c, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, c, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, c, TIM_CCxN_Disable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	commands_printf("nfr_pwm1 phase %d, %d ms requested\n", ph, ms);
	commands_printf("  vin start %.2f  min %.2f  max %.2f\n", (double)v_start, (double)v_min, (double)v_max);
	commands_printf("  DRV line low in %d samples\n", drv_samples);
	commands_printf("  phases before  U %.1f  V %.1f  W %.1f\n",
			(double)ph_before[0], (double)ph_before[1], (double)ph_before[2]);
	if (ph_n > 0) {
		commands_printf("  phases during  U %.1f  V %.1f  W %.1f  (avg of %d; switching leg ~vin/2)\n",
				(double)(ph_sum[0] / ph_n), (double)(ph_sum[1] / ph_n), (double)(ph_sum[2] / ph_n), ph_n);
	}
	if (stopped_at >= 0) {
		commands_printf("  STOPPED by firmware at %d ms, fault: %s\n", stopped_at,
				mc_interface_fault_to_string(mc_interface_get_fault()));
	} else if (low_at >= 0) {
		commands_printf("  ABORTED at %d ms: vin reading below 70%% of start\n", low_at);
	} else {
		commands_printf("  completed\n");
	}
}

void hw_setup_adc_channels(void) {
	uint8_t t_samp = ADC_SampleTime_15Cycles;
	// Voltage/temp channels are driven through 3.3k (R43) with no hold cap while the
	// phase filter is off, so 15 cycles is too short and they read high (charge sharing
	// with the previous channel). Current channels keep 15 cycles.
	uint8_t t_samp_v = ADC_SampleTime_56Cycles;

	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, t_samp);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, t_samp_v);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, t_samp_v);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, t_samp_v);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, t_samp_v);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, t_samp);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, t_samp_v);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, t_samp_v);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, t_samp_v);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, t_samp_v);

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, t_samp);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 2, t_samp_v);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, t_samp_v);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, t_samp_v);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, t_samp_v);

	// Current oversampling
//	for (int i = 6;i <= 15;i++) {
//		ADC_RegularChannelConfig(ADC1, ADC_Channel_10, i, ADC_SampleTime_56Cycles);
//		ADC_RegularChannelConfig(ADC2, ADC_Channel_11, i, ADC_SampleTime_56Cycles);
//		ADC_RegularChannelConfig(ADC3, ADC_Channel_12, i, ADC_SampleTime_56Cycles);
//	}

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, t_samp);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, t_samp);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, t_samp);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, t_samp);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, t_samp);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, t_samp);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, t_samp);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, t_samp);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, t_samp);
}


void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}


void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}


/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}


// // Not sure if anything past this point is needed
// bool hw_sample_shutdown_button(void) {
// 	chMtxLock(&shutdown_mutex);

// 	bt_diff = 0.0;

// 	for (int i = 0;i < 3;i++) {
// 		palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_INPUT_ANALOG);
// 		chThdSleep(5);
// 		float val1 = ADC_VOLTS(ADC_IND_SHUTDOWN);
// 		chThdSleepMilliseconds(1);
// 		float val2 = ADC_VOLTS(ADC_IND_SHUTDOWN);
// 		palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL);
// 		chThdSleepMilliseconds(1);

// 		bt_diff += (val1 - val2);
// 	}

// 	chMtxUnlock(&shutdown_mutex);

// 	return (bt_diff > 0.12);
// }

// static void terminal_shutdown_now(int argc, const char **argv) {
// 	(void)argc;
// 	(void)argv;
// 	DISABLE_GATE();
// 	HW_SHUTDOWN_HOLD_OFF();
// }

// static void terminal_button_test(int argc, const char **argv) {
// 	(void)argc;
// 	(void)argv;

// 	for (int i = 0;i < 40;i++) {
// 		commands_printf("BT: %d %.2f", HW_SAMPLE_SHUTDOWN(), (double)bt_diff);
// 		chThdSleepMilliseconds(100);
// 	}
// }