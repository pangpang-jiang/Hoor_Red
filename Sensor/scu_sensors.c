#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <inttypes.h>
#include "scu_sensors.h"
#include "scu_hci.h"

static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}
static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;
static struct sensor_value pressure, temperature;

extern void process_lps22hb(const struct device *dev)
{
	struct sensor_value press, temp;

	sensor_sample_fetch(dev);
	sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
	sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);	

	pressure = press;
	temperature = temp;
	/* display pressure */
	printf("Pressure:%.1f kPa\n", sensor_value_to_double(&press));

	/* display temperature */
	printf("Temperature:%.1f C\n", sensor_value_to_double(&temp));

}
extern void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig)
{
	static struct sensor_value accel_x, accel_y, accel_z;
	static struct sensor_value gyro_x, gyro_y, gyro_z;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

	/* lsm6dsl gyro */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);
		
	accel_x_out = accel_x;
	accel_y_out = accel_y;
	accel_z_out = accel_z;

	gyro_x_out = gyro_x;
	gyro_y_out = gyro_y;
	gyro_z_out = gyro_z;

}

extern void run_lsm6dsl(const struct device *lsm6dsl_dev)
{
	char out_str[64];
	struct sensor_value odr_attr;
	
	/* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 104;
	odr_attr.val2 = 0;

	sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);

	struct sensor_trigger trig;
	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler);

		/* lsm6dsl accel */
		sprintf(out_str, "accel x:%f ms/2 y:%f ms/2 z:%f ms/2",
							  out_ev(&accel_x_out),
							  out_ev(&accel_y_out),
							  out_ev(&accel_z_out));
		printk("%s\n", out_str);

		/* lsm6dsl gyro */
		sprintf(out_str, "gyro x:%f dps y:%f dps z:%f dps",
							   out_ev(&gyro_x_out),
							   out_ev(&gyro_y_out),
							   out_ev(&gyro_z_out));
		printk("%s\n", out_str);
		printk("\n");
}


#define GPIO_OUTPUT_FLAGS (GPIO_OUTPUT | GPIO_PULL_UP)
#define GPIO_INPUT_FLAGS (GPIO_INPUT | GPIO_PULL_UP)
#define SPEED_OF_SOUND 0.0003432

static struct gpio_dt_spec trigger = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpiob)),
	.pin = 2,
	.dt_flags = 0
};

static struct gpio_dt_spec echo = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpioa)),
	.pin = 4,
	.dt_flags = 0
};

static double hcsr04_distance;

// Through get the time difference between echo and trigger, 
// to calculate the distance between the Ultrasonic Ranger 
// and reference ground
extern void hcsr04_alttitude(){
	gpio_pin_configure_dt(&trigger, GPIO_OUTPUT_FLAGS);
	gpio_pin_configure_dt(&echo, GPIO_INPUT_FLAGS);	
		uint32_t echo_pulse_width;
		int64_t start_time, end_time;

		gpio_pin_set_dt(&trigger,1);
		k_busy_wait(10);
		gpio_pin_set_dt(&trigger,0);

		while(!gpio_pin_get_dt(&echo)){
			// Timeout handling, if needed
		}
		start_time = k_uptime_ticks();
    	while (gpio_pin_get_dt(&echo)) {
        // Timeout handling, if needed
    	}
    	end_time = k_uptime_ticks();
		// Calculate the pulse width in microseconds
    	echo_pulse_width = k_ticks_to_us_floor64((end_time - start_time)/2);

	    // Convert the pulse width to distance in meters
    	hcsr04_distance = echo_pulse_width * SPEED_OF_SOUND*100;
		printk("-------------------------------------\n");
    	printk("hcsr04_distance distance: %.3f cm\n", hcsr04_distance);
}

