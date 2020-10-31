/*
 * datatype.h
 *
 *  Created on: Mar 9, 2020
 *      Author: Ljo
 */

#ifndef MAIN_DATATYPE_H_
#define MAIN_DATATYPE_H_

#include <stdint.h>
#include "driver/mcpwm.h"
#include "MPU.hpp"

typedef enum{
	COMM_PACKET_SET_DUTY = 0,
	COMM_PACKET_SET_ACTIVE_MOTOR,
	COMM_PACKET_SET_IDLE_MOTOR,
	COMM_PACKET_STOP_MOTORS,
	COMM_PACKET_GET_GYRO,
	COMM_PACKET_SET_GYRO_OFFSET,
	COMM_PACKET_PID_ENEABLE,
	COMM_PACKET_PID_DISABLE,
	COMM_PACKET_SET_PID,
	COMM_PACKET_GET_PID,
	COMM_PACKET_GET_VALUE,
	COMM_PACKET_MCU_RESET,
	COMM_PACKET_SET_PID_SETPOINT,
	COMM_PACKET_GET_PID_SETPOINT
}comm_packet_id_e;


typedef enum{
	MOTOR_STATE_STOP = 0,
	MOTOR_STATE_RUN,
	MOTOR_STATE_MAX
}motor_state_e;

typedef enum{
	MOTOR_NUM_1 = 0,
	MOTOR_NUM_2,
	MOTOR_NUM_3,
	MOTOR_NUM_4,
	MOTOR_NUM_MAX
}motor_num_e;

typedef struct{
	mpud::raw_axes_t accelRaw;   // x, y, z axes as int16
	mpud::raw_axes_t gyroRaw;    // x, y, z axes as int16
	mpud::float_axes_t accelG;   // accel axes in (g) gravity format
	mpud::float_axes_t gyroDPS;

	int16_t offset_x;
	int16_t offset_y;
	float Acceleration_angle[2];
	float Gyro_angle[2];
	float Total_angle[2];
	int32_t angel_x;
	int32_t angel_y;
}gyro_s;

typedef struct{
	volatile motor_state_e state;
	volatile uint16_t pwm_ms = 1000;
	volatile uint16_t throttle = 1000;
	volatile uint16_t pid_setpoint = 1000;
	volatile uint16_t throttle_compenzation = 0;
	uint8_t pin;
	mcpwm_unit_t mcpwm_unit;
	mcpwm_io_signals_t mcpwm_io_signal;
	mcpwm_operator_t mcpwm_operator;
	mcpwm_timer_t mcpwm_timer;
}motor_s;

typedef struct{
	float PID, error, previous_error;
	float pid_p=0;
	float pid_i=0;
	float pid_d=0;

	float kp= 3.35;
	float ki= 0.0;//5.0;//0.0005;//0.003
	float kd= 0.0;//0.205;//2.05
	bool eneable = false;
}PID_s;

extern motor_s motor_1;
extern motor_s motor_2;
extern motor_s motor_3;
extern motor_s motor_4;

extern gyro_s gyro;
extern PID_s pid;

#endif /* MAIN_DATATYPE_H_ */
