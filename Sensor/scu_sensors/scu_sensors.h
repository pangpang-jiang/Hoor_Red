#ifndef SCU_SENSORS_H
#define SCU_SENSORS_H


extern void process_lps22hb(const struct device *dev);
extern void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig);

extern void run_lsm6dsl(const struct device *lsm6dsl_dev);

extern void hcsr04_alttitude();
#endif