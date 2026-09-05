#include "bio.h"
#include <zephyr/logging/log.h>
#if defined(CONFIG_BIO_ADC_CLOCK)
#include <zephyr/drivers/pwm.h>
static int start_adc_clock(void)
{
	const struct device *pwm = DEVICE_DT_GET(DT_NODELABEL(pwm0));
	uint64_t cycles;
	if (!device_is_ready(pwm)) { return -ENODEV; }
	int err = pwm_get_cycles_per_sec(pwm, 0, &cycles);
	if (err) { return err; }
	if (cycles != 16000000) { return -ENOTSUP; }
	// nRF52840 COUNTERTOP must be >= 3. Four ticks give 4 MHz at 50% duty.
	// A two-tick/8 MHz PWM period would be outside the hardware specification.
	return pwm_set_cycles(pwm, 0, 4, 2, 0);
}
#endif
LOG_MODULE_REGISTER(bio_main, LOG_LEVEL_INF);
int main(void)
{
	LOG_INF("Biosensing v3: %s", CONFIG_BIO_PROFILE);
#if defined(CONFIG_BIO_ADC_CLOCK)
	int clock_err = start_adc_clock();
	if (clock_err) { LOG_ERR("ADC PWM clock failed: %d", clock_err); }
	else { LOG_INF("ADC CLKIN 4 MHz: nominal 250 SPS is approximately 122.07 SPS"); }
#endif
	// Peripheral failures must not prevent access to the command interface.
	int err = bio_ads_init();
	if (err) { LOG_WRN("ADS bring-up: %d (check clock, power and wiring)", err); }
	err = bio_sensors_init();
	if (err) { LOG_WRN("One or more optional I2C sensors absent: %d", err); }
	err = bio_usb_init();
	if (err) { LOG_WRN("USB unavailable: %d", err); }
	err = bio_aux_init();
	if (err) { LOG_WRN("Aux UART unavailable: %d", err); }
	bio_app_start();
	err = bio_ble_init();
	if (err) { LOG_ERR("BLE initialization failed: %d", err); }
	LOG_INF("Sampling is stopped. Use 'bio help' on RTT or BLE text commands.");
	// Reuse the main stack for commands, rather than reserve a second idle stack.
	k_thread_priority_set(k_current_get(), 7);
	bio_app_run();
	return 0;
}
