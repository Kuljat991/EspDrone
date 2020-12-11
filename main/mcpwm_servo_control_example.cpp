/* servo motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "bluethoot.h"

#include "datatype.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <math.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "I2Cbus.hpp"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"
#include "global.h"

extern "C" {
void app_main(void);
}

static const char* TAG = "bldcTest";

//_____________________________accelerometer mpu6050____________________________//
static constexpr gpio_num_t SDA = GPIO_NUM_21;
static constexpr gpio_num_t SCL = GPIO_NUM_22;
static constexpr uint32_t CLOCK_SPEED = 100000;  // range from 100 KHz ~ 400Hz

float const rad_to_deg = 180/3.141592654;

unsigned long time = 0;
unsigned long timePrev = 0;
unsigned long timerTime = 0;
float elapsedTime;

MPU_t MPU;

i2c_cmd_handle_t cmd;
/*
 * The following registers contain the primary data we are interested in
 * 0x3B MPU6050_ACCEL_XOUT_H
 * 0x3C MPU6050_ACCEL_XOUT_L
 * 0x3D MPU6050_ACCEL_YOUT_H
 * 0x3E MPU6050_ACCEL_YOUT_L
 * 0x3F MPU6050_ACCEL_ZOUT_H
 * 0x50 MPU6050_ACCEL_ZOUT_L
 * 0x41 MPU6050_TEMP_OUT_H
 * 0x42 MPU6050_TEMP_OUT_L
 * 0x43 MPU6050_GYRO_XOUT_H
 * 0x44 MPU6050_GYRO_XOUT_L
 * 0x45 MPU6050_GYRO_YOUT_H
 * 0x46 MPU6050_GYRO_YOUT_L
 * 0x47 MPU6050_GYRO_ZOUT_H
 * 0x48 MPU6050_GYRO_ZOUT_L
 */


void mpu6050_init(){
	printf("$ MPU Driver Example: MPU-I2C\n");
	fflush(stdout);

	// Initialize I2C on port 0 using I2Cbus interface
	i2c0.begin(SDA, SCL, CLOCK_SPEED);

	// Or directly with esp-idf API
	/*
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = SDA;
	conf.scl_io_num = SCL;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = CLOCK_SPEED;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
	*/

	MPU.setBus(i2c0);  // set bus port, not really needed since default is i2c0
	MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);  // set address, default is AD0_LOW

	// Great! Let's verify the communication
	// (this also check if the connected MPU supports the implementation of chip selected in the component menu)
	while (esp_err_t err = MPU.testConnection()) {
		ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	ESP_LOGI(TAG, "MPU connection successful!");

	// Initialize
	ESP_ERROR_CHECK(MPU.initialize());  // initialize the chip and set initial configurations
	// Setup with your configurations
	// ESP_ERROR_CHECK(MPU.setSampleRate(50));  // set sample rate to 50 Hz
	// ESP_ERROR_CHECK(MPU.setGyroFullScale(mpud::GYRO_FS_500DPS));
	// ESP_ERROR_CHECK(MPU.setAccelFullScale(mpud::ACCEL_FS_4G));
}

//______________________________________________________________________________//

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
//static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
//{
//    uint32_t cal_pulsewidth = 0;
//    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
//    return cal_pulsewidth;
//}

/**
 * @brief Configure MCPWM module
 */
void mpu6050_calculate()
{
		MPU.acceleration(&gyro.accelRaw);  // fetch raw data from the registers
		MPU.rotation(&gyro.gyroRaw);       // fetch raw data from the registers
		// MPU.motion(&accelRaw, &gyroRaw);  // read both in one shot
		// Convert
		gyro.accelG = mpud::accelGravity(gyro.accelRaw, mpud::ACCEL_FS_4G);
		gyro.gyroDPS = mpud::gyroDegPerSec(gyro.gyroRaw, mpud::GYRO_FS_500DPS);
		//eulero formula
		/*---X---*/
		gyro.Acceleration_angle[0] = atan((gyro.accelG.y/16384.0)/sqrt(pow((gyro.accelG.x/16384.0),2) + pow((gyro.accelG.z/16384.0),2)))*rad_to_deg;
		/*---Y---*/
		gyro.Acceleration_angle[1] = atan(-1*(gyro.accelG.x/16384.0)/sqrt(pow((gyro.accelG.y/16384.0),2) + pow((gyro.accelG.z/16384.0),2)))*rad_to_deg;
		//
		/*---X---*/
		gyro.Gyro_angle[0] = gyro.gyroDPS.x/131.0;
	    /*---Y---*/
		gyro.Gyro_angle[1] = gyro.gyroDPS.y/131.0;
	    /*---X axis angle---*/
		gyro.Total_angle[0] = 0.98 *(gyro.Total_angle[0] + gyro.Gyro_angle[0]*elapsedTime) + 0.02*gyro.Acceleration_angle[0];
	    /*---Y axis angle---*/
		gyro.Total_angle[1] = 0.98 *(gyro.Total_angle[1] + gyro.Gyro_angle[1]*elapsedTime) + 0.02*gyro.Acceleration_angle[1];
		// Debug
		gyro.angel_x = gyro.Total_angle[0] * 1000;
		gyro.angel_y = gyro.Total_angle[1] * 1000;
	    //printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.x, accelG.y, accelG.z);
		//printf("gyro: [%+7.2f %+7.2f %+7.2f ] (º/s)\n", gyroDPS[0], gyroDPS[1], gyroDPS[2]);

		//printf("total_angel_x: %f total_angel_y: %f (º/s)\n", gyro.Total_angle[0], gyro.Total_angle[1]);


	    //vTaskDelay(2 / portTICK_PERIOD_MS);

}

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(motor_1.mcpwm_unit, motor_1.mcpwm_io_signal, motor_1.pin);    //Set GPIO 18 as PWM0A, to which servo is connected
    mcpwm_gpio_init(motor_2.mcpwm_unit, motor_2.mcpwm_io_signal, motor_2.pin);    //Set GPIO 19 as PWM0A, to which servo is connected
    mcpwm_gpio_init(motor_3.mcpwm_unit, motor_3.mcpwm_io_signal, motor_3.pin);    //Set GPIO 16 as PWM0A, to which servo is connected
    mcpwm_gpio_init(motor_4.mcpwm_unit, motor_4.mcpwm_io_signal, motor_4.pin);    //Set GPIO 17 as PWM0A, to which servo is connected
}

void power_motors(){

	mcpwm_set_duty_in_us(motor_1.mcpwm_unit, motor_1.mcpwm_timer, motor_1.mcpwm_operator, motor_1.pwm_ms);
	mcpwm_set_duty_in_us(motor_2.mcpwm_unit, motor_2.mcpwm_timer, motor_2.mcpwm_operator, motor_2.pwm_ms);
	mcpwm_set_duty_in_us(motor_3.mcpwm_unit, motor_3.mcpwm_timer, motor_3.mcpwm_operator, motor_3.pwm_ms);
	mcpwm_set_duty_in_us(motor_4.mcpwm_unit, motor_4.mcpwm_timer, motor_4.mcpwm_operator, motor_4.pwm_ms);

	//printf("motor_3.pwm_ms = %u \n", motor_3.pwm_ms);
	//printf("motor_4.pwm_ms = %u \n", motor_4.pwm_ms);

	vTaskDelay(1);
}

//stop na motorima!!!
void servo_controle(void *){

//	const char *name = "main";
	preference.begin("main");
	pid.kp = preference.getFloat("kp", 0);
	pid.kd = preference.getFloat("kd", 0);
	pid.ki = preference.getFloat("ki", 0);
	printf("kp: %f	 kd: %f	 ki: %f\n", pid.kp, pid.kd, pid.ki);
	motor_1.pid_setpoint = preference.getUShort("setpoint1", 1000);
	motor_2.pid_setpoint = preference.getUShort("setpoint2", 1000);
	motor_3.pid_setpoint = preference.getUShort("setpoint3", 1000);
	motor_4.pid_setpoint = preference.getUShort("setpoint4", 1000);
	printf("pid_setpoint 1: %u	 pid_setpoint 2: %u	 pid_setpoint 3: %u	pid_setpoint 4: %u\n", motor_1.pid_setpoint, motor_2.pid_setpoint, motor_3.pid_setpoint, motor_4.pid_setpoint);

	mpu6050_init();

	mpud::raw_axes_t accelRaw;   // x, y, z axes as int16
	mpud::raw_axes_t gyroRaw;    // x, y, z axes as int16
	mpud::float_axes_t accelG;   // accel axes in (g) gravity format
	mpud::float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format

	motor_1.pin = 18;
	motor_1.mcpwm_unit =MCPWM_UNIT_0;
	motor_1.mcpwm_io_signal = MCPWM0A;
	motor_1.mcpwm_operator = MCPWM_OPR_A;
	motor_1.mcpwm_timer = MCPWM_TIMER_0;
	motor_1.pwm_ms = 1000;
	motor_1.throttle_compenzation = 0;
	motor_1.state = MOTOR_STATE_RUN;

	motor_2.pin = 19;
	motor_2.mcpwm_unit = MCPWM_UNIT_0;
	motor_2.mcpwm_io_signal = MCPWM0B;
	motor_2.mcpwm_operator = MCPWM_OPR_B;
	motor_2.mcpwm_timer = MCPWM_TIMER_0;
	motor_2.pwm_ms = 1000;
	motor_2.throttle_compenzation = 0;
	motor_2.state = MOTOR_STATE_RUN;

	motor_3.pin = 16;
	motor_3.mcpwm_unit =MCPWM_UNIT_1;
	motor_3.mcpwm_io_signal = MCPWM1A;
	motor_3.mcpwm_operator = MCPWM_OPR_A;
	motor_3.mcpwm_timer = MCPWM_TIMER_1;
	motor_3.pwm_ms = 1000;
	motor_3.throttle_compenzation = 0;
	motor_3.state = MOTOR_STATE_RUN;

	motor_4.pin = 17;
	motor_4.mcpwm_unit =MCPWM_UNIT_1;
	motor_4.mcpwm_io_signal = MCPWM1B;
	motor_4.mcpwm_operator = MCPWM_OPR_B;
	motor_4.mcpwm_timer = MCPWM_TIMER_1;
	motor_4.pwm_ms = 1000;
	motor_4.throttle_compenzation = 0;
	motor_4.state = MOTOR_STATE_RUN;


	//1. mcpwm gpio initialization
	mcpwm_example_gpio_initialize();

	//2. initial mcpwm configuration
	//printf("Configuring Initial Parameters of mcpwm......\n");
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
	pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
	pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
	mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);    //Configure PWM0A & PWM0B with above settings
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	motor_1.pwm_ms=2000;
	motor_2.pwm_ms=2000;
	motor_3.pwm_ms=2000;
	motor_4.pwm_ms=2000;
	power_motors();
	vTaskDelay(4000 / portTICK_PERIOD_MS);
	motor_1.pwm_ms=1000;
	motor_2.pwm_ms=1000;
	motor_3.pwm_ms=1000;
	motor_4.pwm_ms=1000;
	power_motors();
	vTaskDelay(4000 / portTICK_PERIOD_MS);
	motor_1.pwm_ms=1100;
	motor_2.pwm_ms=1100;
	motor_3.pwm_ms=1100;
	motor_4.pwm_ms=1100;
	power_motors();
	vTaskDelay(4000 / portTICK_PERIOD_MS);
	motor_1.pwm_ms=1000;
	motor_2.pwm_ms=1000;
	motor_3.pwm_ms=1000;
	motor_4.pwm_ms=1000;
	power_motors();
//	vTaskDelay(3000 / portTICK_PERIOD_MS);
	float desired_angle = 3;
	time = (esp_timer_get_time());
	for(;;){
		timePrev = time;  // the previous time is stored before the actual time read
		time = esp_timer_get_time();
		elapsedTime = ((float)(time - timePrev)) / 1000000;
		mpu6050_calculate();

		//printf("elapsedTime: %lf  \n", elapsedTime);
		/*First calculate the error between the desired angle and
		*the real measured angle*/

		//TODO: angel offset
		pid.error = gyro.Total_angle[0] - desired_angle;
		//printf("error: %f \n", pid.error);

		/*Next the proportional value of the PID is just a proportional constant
		*multiplied by the error*/

		pid.pid_p = pid.kp*pid.error;
		//printf("pid_p: %f \n", pid.pid_p);
		/*The integral part should only act if we are close to the
		desired position but we want to fine tune the error. That's
		why I've made a if operation for an error between -2 and 2 degree.
		To integrate we just sum the previous integral value with the
		error multiplied by  the integral constant. This will integrate (increase)
		the value each loop till we reach the 0 point*/
		if(pid.eneable)//((-10 <pid.error) && (pid.error <10))
		{
		  pid.pid_i = pid.pid_i+(pid.ki*pid.error);
		}
		//printf("pid_i: %f \n", pid.pid_i);

		/*The last part is the derivate. The derivate acts upon the speed of the error.
		As we know the speed is the amount of error that produced in a certain amount of
		time divided by that time. For taht we will use a variable called previous_error.
		We substract that value from the actual error and divide all by the elapsed time.
		Finnaly we multiply the result by the derivate constant*/

		pid.pid_d = pid.kd*((pid.error - pid.previous_error)/elapsedTime);
		//printf("pid_d: %f \n", pid.pid_d);

		/*The final PID values is the sum of each of this 3 parts*/
		pid.PID = pid.pid_p + pid.pid_i + pid.pid_d;

		/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
		tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
		have a value of 2000us the maximum value taht we could sybstract is 1000 and when
		we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
		to reach the maximum 2000us*/

		if(pid.PID < -1000)
		{
		  pid.PID = -1000;
		}
		if(pid.PID > 1000)
		{
		  pid.PID = 1000;
		}

		pid.previous_error = pid.error;
		//printf("PID: %f \n", pid.PID);

		if(pid.eneable){
			/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
			motor_1.pwm_ms = motor_1.throttle + motor_1.throttle_compenzation + pid.PID;
			motor_2.pwm_ms = motor_2.throttle + motor_2.throttle_compenzation - pid.PID;

			/*Once again we map the PWM values to be sure that we won't pass the min
			and max values. Yes, we've already maped the PID values. But for example, for
			throttle value of 1300, if we sum the max PID value we would have 2300us and
			that will mess up the ESC.*/
			//Right
			if(motor_2.pwm_ms < 1000)
			{
				motor_2.pwm_ms= 1050;
			}
			if(motor_2.pwm_ms > 1900)
			{
				motor_2.pwm_ms=1900;
			}
			if(motor_2.state == MOTOR_STATE_STOP){
				motor_2.pwm_ms=1000;
			}

			//Left
			if(motor_1.pwm_ms < 1000)
			{
				motor_1.pwm_ms= 1050;
			}
			if(motor_1.pwm_ms > 1900)
			{
				motor_1.pwm_ms=1900;
			}
			if(motor_1.state == MOTOR_STATE_STOP){
				motor_1.pwm_ms=1000;
			}
		}else{
			if(motor_1.state == MOTOR_STATE_RUN){
				motor_1.pwm_ms = motor_1.throttle;	//1430;
			}else{
				motor_1.pwm_ms = 1000;
			}

			if(motor_2.state == MOTOR_STATE_RUN){
				motor_2.pwm_ms = motor_2.throttle;	//1300;
			}else{
				motor_2.pwm_ms = 1000;
			}

			if(motor_3.state == MOTOR_STATE_RUN){
				motor_3.pwm_ms = motor_3.throttle;	//1430;
			}else{
				motor_3.pwm_ms = 1000;
			}

			if(motor_4.state == MOTOR_STATE_RUN){
				motor_4.pwm_ms = motor_3.throttle;	//1300;
			}else{
				motor_4.pwm_ms = 1000;
			}

		}
		//printf("left motor: %i \n", pwmLeft);
		//printf("right motor: %i \n", pwmRight);


		power_motors();
	}
}

void app_main()
{
	bt_initialize();

    xTaskCreate(servo_controle, "servo_controle", 4096, NULL, 5, NULL);
}
