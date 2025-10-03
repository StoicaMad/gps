#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif


#include <string.h>
#include <stdio.h>

#define NUM_SENSORS 10

#define USE_HAL_USE_HAL_UART_REGISTER_CALLBACKS 1U

#define SERVO_HEADER_1 		0x20
#define SERVO_HEADER_2 		0x40
#define SERVO_BUFFER_SIZE 	0x20

#define SENSOR_HEADER  		0x04
#define SENSOR_BUFFER_SIZE	0x04
#define SENSOR_TX_BUF_SIZE_6  0x06
#define SENSOR_TX_BUF_SIZE_8  0x08
#define SENSOR_CHNL_MASK 	0x0F
#define SENSOR_CMND_MASK 	0xF0
#define SENSOR_CMD_DSCVR 	0x80
#define SENSOR_CMD_TYPE 	0x90
#define SENSOR_CMD_MEAS 	0xA0
#define TELM_CHECKSUM_CONST 0xFFFF

#define IBUSS_TEMP 0x01    //
#define SENSOR_SPEED 0x7e    // Speed 2bytes km/h
#define SENSOR_GROUND_SPEED  0x13 //2 bytes m/s *100 different unit than build-in sensor
#define SENSOR_TYPE_VERTICAL_SPEED   0x12 //2 bytes m/s *100
#define SENSOR_GPS_STATUS  0x0b
#define SENSOR_GPS_DIST      0x14 //2 bytes dist from home m unsigned
#define SENSOR_GPS_LAT    0x80 //4bytes signed WGS84 in degrees * 1E7
#define SENSOR_GPS_LON    0x81 //4bytes signed WGS84 in degrees * 1E7
#define SENSOR_GPS_ALT    0x82 //4bytes signed!!! GPS alt m*100
#define SENSOR_ALT        0x83 //4bytes signed!!! Alt m*100
#define SENSOR_ARMED      0x15  // unArm , Arm
#define SENSOR_FLIGHT_MODE  0x16//
// salvare , home , reset , wait , manual , return , ready, calib , error , to point ,


#define IBUSS_INTV 0x00 // Internal voltage (in 0.01)
#define IBUSS_EXTV 0x03 // External voltage (in 0.01)
#define SENSOR_TYPE_CMP_HEAD  0x08  //Heading  0..360 deg, 0=north 2bytes


typedef struct TelemetrySensorStruct {
	uint8_t 			SensorType;
	volatile int32_t	SensorMeas;
} TelemetrySensorStruct;

typedef struct TelemetryServoStruct {
	uint16_t 	Header;
	uint16_t 	Channel_1;
	uint16_t 	Channel_2;
	uint16_t 	Channel_3;
	uint16_t 	Channel_4;
	uint16_t 	Channel_5;
	uint16_t 	Channel_6;
	uint16_t 	Channel_7;
	uint16_t 	Channel_8;
	uint16_t 	Channel_9;
	uint16_t 	Channel_10;
	uint16_t 	Channel_11;
	uint16_t 	Channel_12;
	uint16_t 	Channel_13;
	uint16_t 	Channel_14;
	uint16_t 	Checksum;
} TelemetryServoStruct;

struct __FLAGS {
	volatile uint8_t MOTOR_ARMING:1;
	volatile uint8_t FAIL_SAFE:1;
	volatile uint8_t Transiever_RX_Sync:1;
	volatile enum TELEMETRY_SYNC_STATES {
		TELEMETRY_SYNC_SYNC0,
		TELEMETRY_SYNC_SYNC1,
		TELEMETRY_SYNC_SYNCED,
		TELEMETRY_SYNC_VERIFIED} TELEMETRY_SYNC_STATES:2;
	volatile uint8_t UNUSED:3;
};

extern TelemetryServoStruct ServoList;
extern volatile struct __FLAGS FLAGS;

void Sensor_UART_Telemetry_Init(UART_HandleTypeDef *huart);
void Servo_UART_Telemetry_Init(UART_HandleTypeDef *huart);
void Sensor_UART_TxComplete_Callback(struct __UART_HandleTypeDef *huart);
void Sensor_UART_RxComplete_Callback(struct __UART_HandleTypeDef *huart);
void Servo_UART_RxComplete_Callback(struct __UART_HandleTypeDef *huart);
void Sensor_UART_Error_Callback(struct __UART_HandleTypeDef *huart);
void Servo_UART_Error_Callback(struct __UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif
#endif /* __TELEMETRY_H */
