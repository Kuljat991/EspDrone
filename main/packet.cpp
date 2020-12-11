/*
 * packet.cpp
 *
 *  Created on: Mar 13, 2020
 *      Author: Ljo
 */

#include "packet.h"
#include "datatype.h"
#include "bluethoot.h"
#include "buffer.h"
#include <stdio.h>
#include <string.h>
#include "global.h"


uint8_t write_buffer[1024];
int32_t write_ind = 0;

void process_packet(uint8_t *data, uint16_t len){
	comm_packet_id_e id = (comm_packet_id_e) data[0];
	data++;
	len--;

	switch(id){
	case COMM_PACKET_SET_DUTY:{
		printf("COMM_PACKET_IS_SET_DUTY\n");
		motor_num_e motor_num = (motor_num_e)data[0];
		data++;
		uint16_t pwm = 0;
		int32_t ind = 0;
		pwm = buffer_get_uint16(data, &ind);
		printf("pwm: %i	 \n", pwm);
		switch(motor_num){
		case MOTOR_NUM_1:
			motor_1.throttle = pwm;
			break;
		case MOTOR_NUM_2:
			motor_2.throttle = pwm;
			break;
		case MOTOR_NUM_3:
			motor_3.throttle = pwm;
			break;
		case MOTOR_NUM_4:
			motor_4.throttle = pwm;
			break;
		case MOTOR_NUM_MAX:{
			motor_1.throttle = pwm;
			motor_2.throttle = pwm;
			motor_3.throttle = pwm;
			motor_4.throttle = pwm;
		}break;
		default:
			break;
		}

		break;
	}
	case COMM_PACKET_SET_ACTIVE_MOTOR:{
		motor_num_e motor_num = (motor_num_e)data[0];
		printf("active motor = %u \n", motor_num);
		switch (motor_num){
		case MOTOR_NUM_1:
			motor_1.state = MOTOR_STATE_RUN;
			break;
		case MOTOR_NUM_2:
			motor_2.state = MOTOR_STATE_RUN;
			break;
		case MOTOR_NUM_3:
			motor_3.state = MOTOR_STATE_RUN;
			break;
		case MOTOR_NUM_4:
			motor_4.state = MOTOR_STATE_RUN;
			break;
		default:
//			motor_1.state = MOTOR_STATE_RUN;
//			motor_2.state = MOTOR_STATE_RUN;
//			motor_3.state = MOTOR_STATE_RUN;
//			motor_4.state = MOTOR_STATE_RUN;
			break;
			}
		}
		break;
	case COMM_PACKET_SET_IDLE_MOTOR:{
		motor_num_e motor_num = (motor_num_e)data[0];
		printf("idle motor = %u \n", motor_num);
		switch (motor_num){
		case MOTOR_NUM_1:
			motor_1.state = MOTOR_STATE_STOP;
			printf("MOTOR 1 STOP!!! \n");
			break;
		case MOTOR_NUM_2:
			motor_2.state = MOTOR_STATE_STOP;
			printf("MOTOR 2 STOP!!! \n");
			break;
		case MOTOR_NUM_3:
			motor_3.state = MOTOR_STATE_STOP;
			printf("MOTOR 3 STOP!!! \n");
			break;
		case MOTOR_NUM_4:
			motor_4.state = MOTOR_STATE_STOP;
			printf("MOTOR 4 STOP!!! \n");
			break;
		default:
			break;
		}
		break;
		}
	case COMM_PACKET_STOP_MOTORS:{
		pid.eneable = false;
		motor_1.throttle = 1000;
		motor_2.throttle = 1000;
		motor_3.throttle = 1000;
		motor_4.throttle = 1000;
	}break;
	case COMM_PACKET_GET_GYRO:{
		//printf("COMM_PACKET_GET_GYRO \n");
		write_buffer[write_ind++] = COMM_PACKET_GET_GYRO;
		buffer_append_int32(write_buffer, gyro.angel_x, &write_ind);
		buffer_append_int32(write_buffer, gyro.angel_y, &write_ind);
		bt_send_data(write_buffer, write_ind);
		write_ind = 0;
		}
		break;
	case COMM_PACKET_PID_ENEABLE:{
		pid.eneable = true;
		motor_1.throttle = motor_1.pid_setpoint;
		motor_2.throttle = motor_2.pid_setpoint;
		motor_3.throttle = motor_3.pid_setpoint;
		motor_4.throttle = motor_4.pid_setpoint;
		pid.pid_i = 0;
	}break;
	case COMM_PACKET_PID_DISABLE:{
		pid.eneable = false;
		motor_1.throttle = 1000;
		motor_2.throttle = 1000;
		motor_3.throttle = 1000;
		motor_4.throttle = 1000;
	}break;
	case COMM_PACKET_SET_PID:{
		int32_t ind = 0;
		pid.kp = buffer_get_float32(data, 1000000, &ind);
		pid.kd = buffer_get_float32(data, 1000000, &ind);
		pid.ki = buffer_get_float32(data, 1000000, &ind);
		preference.putFloat("kp", pid.kp);
		preference.putFloat("kd", pid.kd);
		preference.putFloat("ki", pid.ki);
		printf("kp: %f	 kd: %f	 ki: %f\n", pid.kp, pid.kd, pid.ki);
	}break;
	case COMM_PACKET_GET_PID:{
		write_buffer[write_ind++] = COMM_PACKET_GET_PID;
		buffer_append_float32(write_buffer, pid.kp, 1000000 , &write_ind);
		buffer_append_float32(write_buffer, pid.kd, 1000000 , &write_ind);
		buffer_append_float32(write_buffer, pid.ki, 1000000 , &write_ind);
		bt_send_data(write_buffer, write_ind);
		printf("kp: %f	 kd: %f	 ki: %f\n", pid.kp, pid.kd, pid.ki);
		write_ind = 0;
	}break;
	case COMM_PACKET_GET_VALUE:{
		write_buffer[write_ind++] = COMM_PACKET_GET_VALUE;
		buffer_append_int32(write_buffer, gyro.angel_x, &write_ind);
		buffer_append_int32(write_buffer, gyro.angel_y, &write_ind);
		buffer_append_uint16(write_buffer, motor_1.pwm_ms, &write_ind);
		buffer_append_uint16(write_buffer, motor_2.pwm_ms, &write_ind);
		buffer_append_uint16(write_buffer, motor_3.pwm_ms, &write_ind);
		buffer_append_uint16(write_buffer, motor_4.pwm_ms, &write_ind);
		bt_send_data(write_buffer, write_ind);
		write_ind = 0;
	}
	break;
	case COMM_PACKET_MCU_RESET:{
		esp_restart();
	}break;
	case COMM_PACKET_SET_PID_SETPOINT:{
		int32_t ind = 0;
		motor_1.throttle = buffer_get_uint16(data, &ind);
		motor_2.throttle = buffer_get_uint16(data, &ind);
		motor_3.throttle = buffer_get_uint16(data, &ind);
		motor_4.throttle = buffer_get_uint16(data, &ind);
		motor_1.pid_setpoint = motor_1.throttle;
		motor_2.pid_setpoint = motor_2.throttle;
		motor_3.pid_setpoint = motor_3.throttle;
		motor_4.pid_setpoint = motor_4.throttle;
		preference.putUShort("setpoint1", motor_1.throttle);
		preference.putUShort("setpoint2", motor_2.throttle);
		preference.putUShort("setpoint3", motor_3.throttle);
		preference.putUShort("setpoint4", motor_4.throttle);
	}break;
	case COMM_PACKET_GET_PID_SETPOINT:{
		write_buffer[write_ind++] = COMM_PACKET_GET_PID_SETPOINT;
		buffer_append_uint16(write_buffer, motor_1.pid_setpoint, &write_ind);
		buffer_append_uint16(write_buffer, motor_2.pid_setpoint, &write_ind);
		buffer_append_uint16(write_buffer, motor_3.pid_setpoint, &write_ind);
		buffer_append_uint16(write_buffer, motor_4.pid_setpoint, &write_ind);
		bt_send_data(write_buffer, write_ind);
		write_ind = 0;
	}break;
	default:
		break;
	}

}
