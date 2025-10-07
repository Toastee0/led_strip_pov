/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2024 TOKITA Hiroshi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>

#define STRIP_NODE		DT_ALIAS(led_strip)

#if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)
#else
#error Unable to determine length of LED strip
#endif

/* POV Configuration */
#define TARGET_RPM		333	/* Target rotation speed */
#define NUM_SLICES		45	/* Angular slices per revolution (8° resolution) */
					/* At 333 RPM: 180ms/rev, WS2812 refresh ~2.7ms */
					/* Need margin: 4ms per slice = 45 slices max */
					/* Update rate: 333 RPM ÷ 60 × 45 slices = 250 Hz (4ms) */

/* Physical LED Layout - CIRCUMFERENTIAL ARC */
/* 90 LEDs arranged in a circle around the perimeter (NOT radial spokes!) */
#define LED_START_ANGLE		185	/* LED 0: ~5° left of bottom (185°) */
#define LED_END_ANGLE		340	/* LED 89: 340° around circle */
#define LED_ARC_SPAN		335	/* Arc coverage: ~340° (missing 20mm at bottom) */
#define LED_PITCH_MM		2	/* Physical spacing between LEDs */
#define ARC_RADIUS_MM		50	/* Radius of LED arc (100mm diameter / 2) */

/* Each LED covers approximately 335°/90 ≈ 3.7° of arc */
#define DEGREES_PER_LED		((float)LED_ARC_SPAN / STRIP_NUM_PIXELS)

#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

/* POV Frame Buffer - stores all slices for one complete image */
/* When spinning, each slice represents one "frame" or angular position */
/* frame_buffer[slice_index][led_index] where:
 *   - slice_index: Angular position during rotation (0-44 for 45 slices)
 *   - led_index: Position around the circumference (0-89)
 *     - LED 0 at 185° (bottom-left)
 *     - LED 45 at ~0° (top)
 *     - LED 89 at 340° (bottom-right)
 *   - 20mm gap at bottom between LED 89 and LED 0
 */
static struct led_rgb frame_buffer[NUM_SLICES][STRIP_NUM_PIXELS];

/* Current output buffer for LED strip */
static struct led_rgb pixels[STRIP_NUM_PIXELS];

/* Current slice being displayed */
static volatile uint32_t current_slice = 0;
static uint32_t last_slice = 0;

/* Flag to indicate new slice data is ready to draw */
static volatile bool update_flag = false;

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

/* Motor control pins - D5 and D6 on XIAO nRF54L15 
 * DRV8833 Control:
 *   IN1=LOW,  IN2=LOW  -> Coast
 *   IN1=LOW,  IN2=HIGH -> Reverse
 *   IN1=HIGH, IN2=LOW  -> Forward
 *   IN1=HIGH, IN2=HIGH -> Brake
 */
/* D5 = P1.11 (Motor IN1 via PWM20), D6 = P2.08 (Motor IN2 via GPIO m2) */
/* WS2812 is on D10 = P2.02 (no conflict!) */
#define MOTOR_PWM_PERIOD_NS  20000000UL  /* 50 Hz for motor control */

/* Motor control devices: PWM for IN1 (P1.11), GPIO spec for IN2 (P2.08 via m2 alias) */
static const struct device *motor_pwm_in1;  /* PWM20 device */
static const struct gpio_dt_spec motor_gpio_in2 = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), m2_gpios);

/* Kernel timer for POV frame updates */
static struct k_timer pov_timer;

/* Bluetooth advertising data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, "POV_Display", 11),
};

/* Initialize frame buffer with a test pattern */
static void init_test_pattern(void)
{
	/* Create an animated rotating pattern
	 * Each slice shifts the pattern around the circumference
	 * This creates a "spinning" or "chasing" effect through the slices
	 */
	for (int slice = 0; slice < NUM_SLICES; slice++) {
		for (int led = 0; led < STRIP_NUM_PIXELS; led++) {
			struct led_rgb color;
			
			/* Create a position that rotates based on slice number */
			/* This makes the pattern shift as we advance through slices */
			int rotated_pos = (led + (slice * 2)) % STRIP_NUM_PIXELS;  // Rotate 2 LEDs per slice
			
			/* Determine color based on rotated position */
			if (rotated_pos < 30) {
				/* Red section */
				color = (struct led_rgb)RGB(0x30, 0x00, 0x00);
			} else if (rotated_pos < 60) {
				/* Green section */
				color = (struct led_rgb)RGB(0x00, 0x30, 0x00);
			} else {
				/* Blue section */
				color = (struct led_rgb)RGB(0x00, 0x00, 0x30);
			}
			
			/* Add a bright "marker" LED at position 0 of each color section */
			if (rotated_pos == 0 || rotated_pos == 30 || rotated_pos == 60) {
				color.r = (color.r > 0) ? 0xFF : 0;
				color.g = (color.g > 0) ? 0xFF : 0;
				color.b = (color.b > 0) ? 0xFF : 0;
			}
			
			frame_buffer[slice][led] = color;
		}
	}
	
	LOG_INF("Initialized rotating test pattern: %d slices x %d LEDs", NUM_SLICES, STRIP_NUM_PIXELS);
	LOG_INF("Pattern: Rotating RGB sections with bright markers");
}

/* Initialize motor control pins to LOW (motor coast) */
static int init_motor_pins(void)
{
	/* Get PWM20 device for IN1 */
	motor_pwm_in1 = DEVICE_DT_GET(DT_NODELABEL(pwm20));
	if (!device_is_ready(motor_pwm_in1)) {
		LOG_ERR("PWM20 device not ready");
		return -ENODEV;
	}
	
	/* Check GPIO spec for IN2 (m2) is ready */
	if (!gpio_is_ready_dt(&motor_gpio_in2)) {
		LOG_ERR("GPIO m2 device not ready");
		return -ENODEV;
	}
	
	/* Initialize PWM IN1 to LOW (0% duty cycle) - channel 0, period 20ms */
	int ret = pwm_set(motor_pwm_in1, 0, MOTOR_PWM_PERIOD_NS, 0, 0);
	if (ret < 0) {
		LOG_ERR("Failed to initialize PWM20 (IN1): %d", ret);
		return ret;
	}
	
	/* Configure GPIO IN2 (m2) as output, initially LOW */
	ret = gpio_pin_configure_dt(&motor_gpio_in2, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure motor IN2 pin (m2/P2.08): %d", ret);
		return ret;
	}
	
	LOG_INF("Motor control initialized - COAST mode (IN1=LOW, IN2=LOW)");
	LOG_INF("  D5 (P1.11) = IN1 via PWM20, D6 (P2.08) = IN2 via GPIO m2");
	LOG_INF("  WS2812 on D10 (P2.02) - no conflict");
	
	return 0;
}

/* Initialize Bluetooth and start advertising */
static int init_bluetooth(void)
{
	int err;

	/* Initialize the Bluetooth subsystem */
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return err;
	}

	LOG_INF("Bluetooth initialized");

	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return err;
	}

	LOG_INF("Bluetooth advertising started as 'POV_Display'");
	return 0;
}

/* Motor control functions - PWM for IN1, GPIO for IN2 */
static void motor_stop(void)
{
	/* Coast mode: IN1=LOW, IN2=LOW */
	pwm_set(motor_pwm_in1, 0, MOTOR_PWM_PERIOD_NS, 0, 0);  /* 0% duty = LOW */
	gpio_pin_set_dt(&motor_gpio_in2, 0);                   /* GPIO LOW */
	LOG_INF("Motor: COAST");
}

static void motor_forward(void)
{
	/* Forward: IN1=25% PWM, IN2=LOW */
	uint32_t pulse_ns = MOTOR_PWM_PERIOD_NS / 4;  /* 25% duty cycle */
	pwm_set(motor_pwm_in1, 0, MOTOR_PWM_PERIOD_NS, pulse_ns, 0);
	gpio_pin_set_dt(&motor_gpio_in2, 0);  /* GPIO LOW */
	LOG_INF("Motor: FORWARD (25%% PWM)");
}

static void motor_reverse(void)
{
	/* Reverse: IN1=LOW, IN2=HIGH */
	pwm_set(motor_pwm_in1, 0, MOTOR_PWM_PERIOD_NS, 0, 0);  /* 0% = LOW */
	gpio_pin_set_dt(&motor_gpio_in2, 1);                   /* GPIO HIGH */
	LOG_INF("Motor: REVERSE");
}

static void motor_brake(void)
{
	/* Brake: IN1=25% PWM, IN2=HIGH */
	uint32_t pulse_ns = MOTOR_PWM_PERIOD_NS / 4;  /* 25% duty cycle */
	pwm_set(motor_pwm_in1, 0, MOTOR_PWM_PERIOD_NS, pulse_ns, 0);
	gpio_pin_set_dt(&motor_gpio_in2, 1);  /* GPIO HIGH */
	LOG_INF("Motor: BRAKE (25%% PWM)");
}

/* Shell command handlers */
static int cmd_motor_forward(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	motor_forward();
	shell_print(sh, "Motor: FORWARD");
	return 0;
}

static int cmd_motor_reverse(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	motor_reverse();
	shell_print(sh, "Motor: REVERSE");
	return 0;
}

static int cmd_motor_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	motor_stop();
	shell_print(sh, "Motor: COAST (stopped)");
	return 0;
}

static int cmd_motor_brake(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	motor_brake();
	shell_print(sh, "Motor: BRAKE");
	return 0;
}

static int cmd_motor_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	int in2_state = gpio_pin_get_dt(&motor_gpio_in2);
	
	shell_print(sh, "Motor Status:");
	shell_print(sh, "  IN1 (D5/P1.11): PWM20 control");
	shell_print(sh, "  IN2 (D6/P2.08/m2): GPIO %s", in2_state ? "HIGH" : "LOW");
	shell_print(sh, "  Use 'motor forward/reverse/stop/brake' to control");
	
	return 0;
}

/* Register shell commands */
SHELL_STATIC_SUBCMD_SET_CREATE(motor_cmds,
	SHELL_CMD(forward, NULL, "Spin motor forward", cmd_motor_forward),
	SHELL_CMD(reverse, NULL, "Spin motor reverse", cmd_motor_reverse),
	SHELL_CMD(stop, NULL, "Stop motor (coast)", cmd_motor_stop),
	SHELL_CMD(brake, NULL, "Brake motor", cmd_motor_brake),
	SHELL_CMD(status, NULL, "Show motor status", cmd_motor_status),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(motor, &motor_cmds, "Motor control commands", NULL);

/* Timer callback - increments slice counter and sets update flag */
static void timer_callback(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	
	/* Increment slice (wraps around) */
	current_slice = (current_slice + 1) % NUM_SLICES;
	
	/* Signal that a new slice is ready */
	update_flag = true;
}

int main(void)
{
	int rc;

	if (device_is_ready(strip)) {
		LOG_INF("Found LED strip device %s", strip->name);
	} else {
		LOG_ERR("LED strip device %s is not ready", strip->name);
		return 0;
	}

	/* Initialize motor control pins (set to LOW/OFF) */
	rc = init_motor_pins();
	if (rc < 0) {
		LOG_ERR("Failed to initialize motor pins: %d", rc);
		return 0;
	}

	/* Initialize Bluetooth and start advertising */
	rc = init_bluetooth();
	if (rc < 0) {
		LOG_ERR("Failed to initialize Bluetooth: %d", rc);
		/* Continue without Bluetooth - not critical for POV operation */
	}

	/* Step 1: Populate all slice color data (prepare frame buffer) */
	LOG_INF("Preparing frame buffer: %d slices x %d LEDs", NUM_SLICES, STRIP_NUM_PIXELS);
	init_test_pattern();
	LOG_INF("Frame buffer ready");
	
	/* Debug: Check first slice, first few LEDs */
	LOG_INF("Slice 0, LED 0: R=%d G=%d B=%d", 
		frame_buffer[0][0].r, frame_buffer[0][0].g, frame_buffer[0][0].b);
	LOG_INF("Slice 0, LED 5: R=%d G=%d B=%d", 
		frame_buffer[0][5].r, frame_buffer[0][5].g, frame_buffer[0][5].b);
	LOG_INF("Slice 0, LED 10: R=%d G=%d B=%d", 
		frame_buffer[0][10].r, frame_buffer[0][10].g, frame_buffer[0][10].b);
	
	/* Test: Light up all LEDs briefly to verify they work */
	LOG_INF("Testing LEDs - setting all to dim white");
	for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
		pixels[i] = (struct led_rgb)RGB(0x05, 0x05, 0x05);
	}
	rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
	if (rc) {
		LOG_ERR("Test LED update failed: %d", rc);
	} else {
		LOG_INF("Test LED update successful");
	}
	k_sleep(K_MSEC(2000));  /* Keep test pattern for 2 seconds */
	
	/* Step 2: Register timer callback and start timer (10ms tick for safe margin) */
	/* WS2812 update takes ~2.7ms, so 10ms gives plenty of margin */
	LOG_INF("Starting timer with 10ms period");
	k_timer_init(&pov_timer, timer_callback, NULL);
	k_timer_start(&pov_timer, K_MSEC(10), K_MSEC(10));
	LOG_INF("Timer started - animation running (100 Hz, ~22.2 FPS effective)");
	
	/* Test motor control after a few seconds */
	LOG_INF("Waiting 5 seconds before motor test...");
	k_sleep(K_MSEC(5000));
	
	LOG_INF("Testing motor: FORWARD for 2 seconds");
	motor_forward();
	k_sleep(K_MSEC(2000));
	
	LOG_INF("Testing motor: STOP");
	motor_stop();
	k_sleep(K_MSEC(1000));
	
	LOG_INF("Motor test complete. Use shell commands: 'motor forward', 'motor stop', etc.");
	
	/* Step 3: Main loop - draw slice when it changes */
	while (1) {
		if (current_slice != last_slice) {
			/* New slice is ready - draw it to LEDs */
			memcpy(pixels, frame_buffer[current_slice], sizeof(pixels));
			
			rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
			if (rc) {
				LOG_ERR("LED update failed: %d", rc);
			}
			
			/* Update last_slice to current */
			last_slice = current_slice;
			
			/* Log progress occasionally */
			static uint32_t draw_count = 0;
			draw_count++;
			if (draw_count % NUM_SLICES == 0) {
				LOG_INF("Completed rotation %d", draw_count / NUM_SLICES);
			}
		} else {
			/* No new slice - yield to other threads */
			k_yield();
		}
	}

	return 0;
}
