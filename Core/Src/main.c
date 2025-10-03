/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "Telemetry.h"
#include <Telemetry.h>
#include "gps.h"
#include "functii_gps.h"
#include "locatii.h"
#include "voltaj.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "nav_math.h"
#include "circular_filter.h"
#include "cog_stability.h"
#include "pid_heading.h"
#include "gnss.h"
#include "state_machine.h"
#include "speed_profile.h"
#include "pwm_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// The Servo Instance Index Must Start From 0
#define SERVO_ESC   0
#define SERVO_Carma   1


float controlCommand;

TelemetrySensorStruct SensorList[NUM_SENSORS];


// Function to update a specific sensor's value
void updateSensorValue(uint8_t sensorType, int32_t newValue) {
    // Loop through SensorList array
    for (int i = 0; i < NUM_SENSORS; ++i) {
        // Check if SensorType matches the desired type (e.g., IBUSS_TEMP)
        if (SensorList[i].SensorType == sensorType) {
            // Update the SensorMeas value of the found sensor
            SensorList[i].SensorMeas = newValue;
            break; // Exit loop after updating the sensor value
        }
    }
}


//static double targetLat = 45.000000, targetLon = 25.000000;
//static double homeLat   = 45.000000, homeLon   = 25.000000;
static int driveProfile = DRIVE_PERIAT;

/*
static float mapf(float x,float in_min,float in_max,float out_min,float out_max){
  if (x<in_min) x=in_min; if (x>in_max) x=in_max;
  return out_min + (x-in_min)*(out_max-out_min)/(in_max-in_min);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

APContext ap;       // contextul state machine autopilot
float motor_pwm = 0;
float rudder_deg = 0, target_sog = 0;
float cog_std = 999.0f;
GNSSFix fix;
float rudder_offset_us = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


	   SensorList[0].SensorType = IBUSS_EXTV;
	   SensorList[1].SensorType = SENSOR_GPS_STATUS;
	   SensorList[2].SensorType = SENSOR_TYPE_CMP_HEAD;
	   SensorList[3].SensorType = SENSOR_SPEED;
	   SensorList[4].SensorType = SENSOR_GPS_DIST;
	   SensorList[5].SensorType = SENSOR_FLIGHT_MODE;
	   SensorList[6].SensorType = SENSOR_GPS_LAT;
	   SensorList[7].SensorType = SENSOR_GPS_LON;
	   SensorList[8].SensorType = SENSOR_GPS_ALT;
	   SensorList[9].SensorType = IBUSS_TEMP;
	//   SensorList[10].SensorType = SENSOR_ALT;



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  GPS_Init();  // &huart3
  Sensor_UART_Telemetry_Init(&huart1);
  Servo_UART_Telemetry_Init(&huart2);

  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,(250+300)); // servo ESC default
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,(250+300)); // servo dir default

  // init autopilot
  gnss_init(9600);
  cogstab_init(12);
  ap_init(&ap, 0.0f);

  const float dt = 1.0f / CONTROL_HZ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  voltmetru();



		  sateliti = GPS.satelites;
		//  currentHeading = GPS.course_t;
		  currentHeading = GPS.courseT;
		 viteza = GPS.speed_k ;
	//	  curent_lng = kalmanFilter.lon;
	//	  curent_lat =  kalmanFilter.lat ;
		  altitudine = GPS.altitude_ft;

	// viteza = calculateSpeed(curent_lat, curent_lng,save_lat[poi],  save_lng[poi], 1/3600);
		//  int x = map(ServoList.Channel_9,1000,2000,0,360);
		//   HeadingError = calculateHeadingError(x, currentHeading);


		     HeadingError = calculateHeadingError(desiredBearing, currentHeading);
			  updateSensorValue(IBUSS_EXTV, V_filtrat*100);
			  nr_poi_sateliti= (sateliti*256) + poi;
			  updateSensorValue(SENSOR_GPS_STATUS, nr_poi_sateliti);
			  updateSensorValue(SENSOR_TYPE_CMP_HEAD, currentHeading);
			  updateSensorValue(SENSOR_SPEED, viteza*10);
			  updateSensorValue(SENSOR_GPS_DIST, Distance_To_Home);
			  updateSensorValue(SENSOR_FLIGHT_MODE, mod);
		//	  updateSensorValue(SENSOR_GPS_LAT,GPS.nmea_latitude*100000);
		//	  updateSensorValue(SENSOR_GPS_LON, GPS.nmea_longitude*100000);
			  updateSensorValue(SENSOR_GPS_LAT,curent_lat*10000000);
			  updateSensorValue(SENSOR_GPS_LON,curent_lng*10000000);
			  updateSensorValue(SENSOR_GPS_ALT, HeadingError*100);
			  updateSensorValue(IBUSS_TEMP, (desiredBearing*10)+400);



 //HAL_Delay(1000);

// mod pilot  1(manual)   2(catre punct)   3 (return to home)


  poi_sett(ServoList.Channel_4);
   mod_operare (ServoList.Channel_9);


   if (gnss_poll(&fix)){
       cogstab_update(fix.cog_deg, &cog_std);
     }

     poi_sett(ServoList.Channel_4);
     mod_operare(ServoList.Channel_9);

     if(mod_pilot==1){ // manual
        salvare_punct(ServoList.Channel_2);

        // direct control din stick-uri
     //   float servo_norm = map(ServoList.Channel_1,1000,2000,-30,30); // -30..+30°
     //   float motor_norm = mapf(ServoList.Channel_3,1000,2000,0,1);   // 0..1

        pwm_set_servo_us(ServoList.Channel_1);
        pwm_set_motor_us(ServoList.Channel_3);
     }

     else if(mod_pilot==2){ // goto punct selectat
         trimite_catre_punct(ServoList.Channel_2);
         if(pornire_autopilot==1){
             double tgtLat = save_lat[poi];
             double tgtLon = save_lng[poi];

             ap_step(&ap, curent_lat, curent_lng, tgtLat, tgtLon,
                     fix.sog_mps, fix.cog_deg, cog_std,
                     driveProfile,
                     &rudder_offset_us, &target_sog);

             // Servo
             int16_t servo_us = 1500 + (int16_t)rudder_offset_us;
             if (servo_us < 1000) servo_us = 1000;
             if (servo_us > 2000) servo_us = 2000;
             pwm_set_servo_us(servo_us);

             // Motor
             motor_pwm = sog_target_to_pwm(target_sog, motor_pwm, dt, driveProfile);
             float motor_norm = motor_pwm * 2.0f - 1.0f; // 0..1 -> -1..+1
             uint16_t motor_us = 1500 + (int16_t)(motor_norm * 500);
             if (motor_us < 1000) motor_us = 1000;
             if (motor_us > 2000) motor_us = 2000;
             pwm_set_motor_us(motor_us);
         }
     }


     else if(mod_pilot==3){ // misiune: punct -> home
         if(pornire_autopilot==1){
             // Dus la punct selectat
             ap_step(&ap, curent_lat, curent_lng,
                     save_lat[poi], save_lng[poi],
                     fix.sog_mps, fix.cog_deg, cog_std,
                     driveProfile,
                     &rudder_offset_us, &target_sog);

             // Servo (offset µs față de centru)
             int16_t servo_us = 1500 + (int16_t)rudder_offset_us;
             if (servo_us < 1000) servo_us = 1000;
             if (servo_us > 2000) servo_us = 2000;
             pwm_set_servo_us(servo_us);

             // Motor (0..1 → -1..+1 → µs)
             motor_pwm = sog_target_to_pwm(target_sog, motor_pwm, dt, driveProfile);
             float motor_norm = motor_pwm * 2.0f - 1.0f;
             uint16_t motor_us = 1500 + (int16_t)(motor_norm * 500);
             if (motor_us < 1000) motor_us = 1000;
             if (motor_us > 2000) motor_us = 2000;
             pwm_set_motor_us(motor_us);

             // Verifică dacă a ajuns la punct
             if(Distance_To_Home < 2.0){
                 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                 HAL_Delay(10000); // stă 10 secunde

                 // întoarce acasă (index 0 = home)
                 ap_step(&ap, curent_lat, curent_lng,
                         save_lat[0], save_lng[0],
                         fix.sog_mps, fix.cog_deg, cog_std,
                         driveProfile,
                         &rudder_offset_us, &target_sog);


                 // Servo pentru retur
                 servo_us = 1500 + (int16_t)rudder_offset_us;
                 if (servo_us < 1000) servo_us = 1000;
                 if (servo_us > 2000) servo_us = 2000;
                 pwm_set_servo_us(servo_us);

                 // Motor pentru retur
                 motor_pwm = sog_target_to_pwm(target_sog, motor_pwm, dt, driveProfile);
                 motor_norm = motor_pwm * 2.0f - 1.0f;
                 motor_us = 1500 + (int16_t)(motor_norm * 500);
                 if (motor_us < 1000) motor_us = 1000;
                 if (motor_us > 2000) motor_us = 2000;
                 pwm_set_motor_us(motor_us);
             }
         }
     }

//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,(1500) );// servo dir
//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,(1450) );// servo esc .


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
