#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <inttypes.h>
#include <scu_sensors.h>
#include <scu_hci.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys_clock.h>
#include <limits.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/drivers/uart.h>
#include <string.h>



#define NUM_THREADS 50
#define STACK_SIZE (1024)
#define ANGLE_TIME 2000
K_THREAD_STACK_ARRAY_DEFINE(tstacks, NUM_THREADS, STACK_SIZE);
static struct k_thread t[NUM_THREADS];

void pitch_roll_main(){
    const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
	while(1){
		int data = pitch_roll(lsm6dsl_dev);
		uart_process(data);
		k_sleep(K_MSEC(ANGLE_TIME));
	}
}
void hcsr04_main(){
	while (1) {
		hcsr04_alttitude();
    	k_sleep(K_SECONDS(2));
	}
}
void pressure_alltitude_main(){
	const struct device *const dev = DEVICE_DT_GET_ONE(st_lps22hb_press);
	while (1) {
		pressure_alttitude(dev);
		k_sleep(K_MSEC(2000));
	}
}

void main(void){
    k_tid_t tid[NUM_THREADS];
	tid[0] =
		k_thread_create(&t[0], tstacks[0], STACK_SIZE,
				hcsr04_main, NULL, NULL,
				NULL, K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
	tid[1] =
		k_thread_create(&t[1], tstacks[1], STACK_SIZE,
				pitch_roll_main, NULL, NULL,
				NULL, K_PRIO_PREEMPT(10), 0, K_NO_WAIT);

	tid[2] =
		k_thread_create(&t[2], tstacks[2], STACK_SIZE,
				pressure_alltitude_main, NULL, NULL,
				NULL, K_PRIO_PREEMPT(10), 0, K_NO_WAIT);				
}