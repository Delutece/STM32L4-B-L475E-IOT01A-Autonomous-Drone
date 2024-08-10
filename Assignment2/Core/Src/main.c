  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h" //
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"

static void UART1_Init(void);
static void BATTLE_MODE(void);			// declaring battle mode function
static void STANDBY_MODE(void);			// declaring standby mode function
static void CHECK_DOUBLE_PRESS(void);	// declaring checking double press function
void exit(int status);					// declaring exit function for when terminating program when not rescued within 10s

volatile int Lasergun_Energy = 0;		// initial Lazergun Energy

uint8_t Rx_data[1];						// buffer of 1 bytes for UART receive

volatile int press_counter = 0;
volatile int no_of_presses = 0;			// no. of presses status/flag:	1 for single press;	2 for double press

// creating flags for various situations
volatile int TheLastofEE2028 = 0;		// "The Last of EE2028" flag: 0 for w/o "The Last of EE2028";	1 for w/ "The Last of EE2028"

volatile int interrupt = 0;				// interrupt flag:	0 for False/no interrupt/interrupt inactive;	1 for True/carrying out interrupt code/interrupt active

volatile int mode = 0;					// MODES:	0 for STANDBY MODE;		1 for BATTLE MODE;		2 for terminating program mode, when not rescued after 10s in "The Last of EE2028" state

// threshold values
float accel_low_threshold = -9.00;		// acceleration when drone is free-falling
float gyro_high_threshold = 150.00;		// when drone rotates too fast, average drone has limit of 150deg/s rotation speed
float gyro_low_threshold = -150.00;		// when drone rotates too fast, negative for counterclockwise rotation
float mag_high_threshold = 3.00;		// when drop is flipped/near metal objects
float temp_high_threshold = 40.00;		// upper limit of typical suitable drone operating temperature
float temp_low_threshold = 0.00;		// lower limit of typical suitable drone operating temperature
float press_low_threshold = 264.36;		// typical drone max. ceiling height is 10km above sea level, which is around 263.36hPa in pressure
float humid_low_threshold = 20.00;		// humidity levels for comfortable living in temperate climates is >20%rH but <60%rH; however in Singapore, humidity canr each as high as 100%rH

UART_HandleTypeDef huart1;

int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_HSENSOR_Init();
	BSP_MAGNETO_Init();
	BSP_PSENSOR_Init();
	BSP_TSENSOR_Init();
	BSP_LED_Init(LED2);
	BSP_PB_Init(GPIO_PIN_13, BUTTON_MODE_EXTI);

    /* UART initialization  */
    UART1_Init();

    // Drone powering on for the first time
	char message1[] = "Entering STANDBY MODE!!!\r\n";		// Fixed message
	char message_print[32]; 							// UART transmit buffer
    sprintf(message_print, "%s", message1);
    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode

    // UART Receive interrupt
    HAL_UART_Receive_IT (&huart1, Rx_data, 10);

	while (1)
	{
		switch (interrupt)
		{
		case 0:
			switch (mode)
			{
			case 0:							// if (interrupt flag == 0) && (mode == 0), enter STANDBY MODE
				STANDBY_MODE();
				break;
			case 1:							// if (interrupt flag == 0) && (mode == 1), enter BATTLE MODE
				BATTLE_MODE();
				break;
			case 2:							// if (interrupt flag == 0) && (mode == 2), TERMINATE PROGRAM
				exit(EXIT_SUCCESS);
			}
			break;
		case 1:
			CHECK_DOUBLE_PRESS();			// function for checking single/double press, causes 0.5s delay

			// Interrupt code to run after checking if its single or double press
			switch (no_of_presses)
			{
			case 1:							// when no_of_presses flag is 1, single press is detected
				if ((mode == 1) && (TheLastofEE2028 == 0))			// if (interrupt == 1) && (no_of_presses == 1) && (mode == 1) && (TheLastOfEE2028 == 0), charge lasergun with 3/10 units of energy
				{
					char message1[] = "Lasergun Charging! New energy";  // Fixed message
					if ((Lasergun_Energy + 3) > 10)
					{
						Lasergun_Energy = 10;
					} else
					{
						Lasergun_Energy += 3;
					}
		    		char message_print[64];
					sprintf(message_print, "%s: %d/10 units of energy\r\n", message1, Lasergun_Energy);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode
				}
				else if ((mode == 1) && (TheLastofEE2028 == 1))		// else if (interrupt == 1) && (no_of_presses == 1) && (mode == 1) && (TheLastOfEE2028 == 1), ignore single press in battle mode with the state of "The Last Of EE2028"
				{
					BATTLE_MODE();									// BUT do not clear no_of_presses == 1 flag just yet
				}
				else if (mode == 0)									// if (interrupt == 1) && (no_of_presses == 1) && (mode == 0), ignore single press in standby mode
				{
					STANDBY_MODE();									// BUT do not clear no_of_presses == 1 flag just yet
				}
				break;
			case 2:							// when no_of_presses flag is 2, double press is detected
				if ((mode == 1) && (TheLastofEE2028 == 0))			// if (interrupt == 1) && (no_of_presses == 2) && (mode == 1) && (TheLastOfEE2028 == 0), double press in battle mode without the state of "The Last of EE2028" causes drone to enter standby mode
				{
					char message1[] = "Entering STANDBY MODE!!!\r\n";
					char message_print[32];
					sprintf(message_print, "%s", message1);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					mode = 0;					// switch to STANDBY MODE
				}
				else if ((mode == 1) && (TheLastofEE2028 == 1))		// else if (interrupt == 1) && (no_of_presses == 2) && (mode == 1) && (TheLastOfEE2028 == 1), double press in battle mode with the state of "The Last of EE2028" within 10s means drone is rescued and reenters battle mode
				{
					char message1[] = "Entering BATTLE MODE!!!\r\n";
					char message_print[32];
					sprintf(message_print, "%s", message1);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					TheLastofEE2028 = 0;		// clear "The Last of EE2028" flag
				}
				else if (mode == 0) 								// else if (interrupt == 1) && (no_of_presses == 2) && (mode == 0), double press in standby mode causes drone to enter battle mode
				{
					char message1[] = "Entering BATTLE MODE!!!\r\n";
					char message_print[32];
					sprintf(message_print, "%s", message1);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					mode = 1;					// switch to BATTLE MODE
				}
				break;

			// reset no_of_presses and interrupt flag after running interrupt code
			no_of_presses = 0;		// reset no. of presses flag after carrying out the actions for single/double press
			}
			break;
		}
	}
}

// PB interrupt function, set interrupt flag (interrupt active) and no. of presses counter ++
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == BUTTON_EXTI13_Pin)
	{
		interrupt = 1;
		press_counter++;
	}
}

//ENHANCEMENT: UART interrupt: using UART receive to receive commands and set flags to control mode switching and charge lasergun
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, Rx_data, 1);
	interrupt = 1;
	if	((Rx_data[0] == 0) && (mode == 1) && (TheLastofEE2028 == 0)) // receive 0 in battle mode w/o the state of "The Last of EE2028" switches mode to STANDBY MODE
	{
		press_counter = 2;
	}
	else if ((Rx_data[0] == 0) && (mode == 0))						// receive 0 in STANDBY MODE, ignore, similar to ignoring single press
	{
		press_counter = 1;
	}
	else if ((Rx_data[0] == 1) && (mode == 0))						// receive 1 in STANDBY MODE switched mode to BATTLE MODE
	{
		press_counter = 2;
	}
	else if ((Rx_data[0] == 1) && (mode == 1) && (TheLastofEE2028 == 0))	// receive 1 in battle mode w/o the state of "The Last of EE2028" charges lasergun
	{
		press_counter = 1;
	}
	else if ((Rx_data[0] == 1) && (mode == 1) && (TheLastofEE2028 == 1))	// receive 1 in battle mode with the state of "The Last of EE2028" rescues drone and drone enters battle mode
	{
		press_counter = 2;
	}
	else if ((Rx_data[0] == 2) && ((mode == 0) || (mode == 1)))		//receive 2 in battle mode w/o the state of "The Last of EE2028" or standby mode terminates the program
	{
		mode = 2;
	}
}

// function to check if single or double press (after PB interrupt)
static void CHECK_DOUBLE_PRESS(void)
{
	uint32_t doublePress_starttime = HAL_GetTick();
	while (interrupt == 1)
	{
		if (((HAL_GetTick() - doublePress_starttime) >= 500)) // if timeout(>= 500ms); set no. of presses flag to 1(indicating single press); reset press counter and interrupt flag
		{
			no_of_presses = 1;
			press_counter = 0;
			interrupt = 0;
			break;
		}
		else if (press_counter > 1) 		// if press counter > 1; set no. of presses flag is set to 2(indicating double press), reset press counter and interrupt flag
		{
			no_of_presses = 2;
			press_counter = 0;
			interrupt = 0;
			break;
		}
	}
}

// standby mode function
static void STANDBY_MODE(void)
{
    if (no_of_presses == 1)		// when entering function from (interrupt == 1), single press flag is not cleared, ignore single press and jump to polling (TELEMETRY READING, MONITORING and printing)
    {
    	goto ignore_single_standby;
    }
    BSP_LED_On(LED2);			// LED always on in standby mode
	uint32_t standby_starttime = HAL_GetTick();		// timer reset is skipped if entering function from (interrupt == 1), this is to keep interrupt delay to 0.5s. If timer was allowed to be reset, the delay resulted from ignoring standby mode is 1.5s

ignore_single_standby:
	while (interrupt == 0)
	{
		// Polling mode: reading the 4 require sensors and performing calculations
		float press_data;
		press_data = BSP_PSENSOR_ReadPressure(); 	// units hPa %0.2f

    	float gyro_data[3];
    	float gyro_data_i16[3] = { 0 };				// array to store the x, y and z readings.
    	BSP_GYRO_GetXYZ(gyro_data_i16);				// read gyroscope
    	// the function above returns 16 bit integers which are angular rate in mdps (/1000 deg/s).
    	// Converting to float to print the actual acceleration.
    	gyro_data[0] = (float)gyro_data_i16[0]/1000.0f;	// x-axis; roll of drone
    	gyro_data[1] = (float)gyro_data_i16[1]/1000.0f;	// y-axis; pitch of drone

		float humid_data;
		humid_data = BSP_HSENSOR_ReadHumidity(); 	// units rH %0.2f

		float mag_data[3];
		int16_t mag_data_i16[3] = {0};				// array to store x,y,z
		BSP_MAGNETO_GetXYZ(mag_data_i16);
		mag_data[0]= (float)mag_data_i16[0]/1000.0f;
		mag_data[1]= (float)mag_data_i16[1]/1000.0f;
		mag_data[2]= (float)mag_data_i16[2]/1000.0f;
		float mag_magnitude = sqrt(mag_data[0]*mag_data[0] + mag_data[1]*mag_data[1] + mag_data[2]*mag_data[2]);

		// TELEMETRY MONITORING
		char message5[] = "EXCEEDS LOW THRESHOLD OF";
		char message6[] = "EXCEEDS HIGH THRESHOLD OF";
		// send messages every 1s
		if ((HAL_GetTick() - standby_starttime) >= 1000)		// Send messages to UART every 1s
		{
			standby_starttime = HAL_GetTick();

			// threshold violation logs sent every 1s; multiple lines of threshold violation are sent if multiple thresholds are violated
			if (gyro_data[0] > gyro_high_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "Gx: %0.2fdeg/s %s %0.2fdeg/s; Drone is rotating too fast!\r\n",gyro_data[0],message6,gyro_high_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
			if (gyro_data[0] < gyro_low_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "Gx: %0.2fdeg/s %s %0.2fdeg/s; Drone is rotating too fast!\r\n",gyro_data[0],message5,gyro_low_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
			if (gyro_data[1] > gyro_high_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "Gy: %0.2fdeg/s %s %0.2fdeg/s; Drone is rotating too fast!\r\n",gyro_data[1],message6,gyro_high_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
			if (gyro_data[1] < gyro_low_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "Gy: %0.2fdeg/s %s %0.2fdeg/s; Drone is rotating too fast!\r\n",gyro_data[1],message5,gyro_low_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
			if (mag_magnitude > mag_high_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "|M|: %0.2fGauss %s %0.2fGauss; Drone is near metal object!\r\n",mag_magnitude,message6,mag_high_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
			if (press_data < press_low_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "P: %0.2fhPa %s %0.2fhPa; Drone has exceeded max. altitude of 10km!\r\n",press_data,message5,press_low_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
			if (humid_data < humid_low_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "H: %0.2frH %s %0.2frH; Humidity is not suitable for human habitation!\r\n",humid_data,message5,humid_low_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}

			// Send telemetry reading every 1s if no threshold are violated, telemetry readings paused otherwise when any threshold violation logs are violated
			if ((humid_data >= humid_low_threshold) && (mag_magnitude <= mag_high_threshold) && (gyro_data[0] <= gyro_high_threshold) && (gyro_data[0] >= gyro_low_threshold) && (gyro_data[1] <= gyro_high_threshold) && (gyro_data[1] >= gyro_low_threshold) && (press_data >= press_low_threshold))
			{
				standby_starttime = HAL_GetTick();
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "Gx: %0.2fdeg/s, Gy: %0.2fdeg/s, |M|: %0.2fGauss, P: %0.2fhPa, H:%0.2frH\r\n", gyro_data[0],gyro_data[1],mag_magnitude,press_data,humid_data);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
		}
	}
}

static void BATTLE_MODE(void)
{
    if ((no_of_presses == 1) && (TheLastofEE2028 == 1))		// When entering function from (interrupt == 1), single press flag is not cleared, ignore single press and jump to the 10s rescue countdown.
    {
    	goto ignore_single_TheLastOfEE2028;			// This is to avoid resetting any timers when button is pressed once in 0.5s, as the sending of "Drone was attacked!" messages may be delayed insteading of being sent every 1s.
    }

    uint32_t LED_starttime = HAL_GetTick(); 		// starting tick for LED toggle
    uint32_t MSG_starttime = HAL_GetTick(); 		// starting tick for sending threshold violation logs/telemetry reading

    while ((interrupt ==0) && (TheLastofEE2028 == 0))		// While BATTLE MODE w/o the state of "The Last of EE2028"
    {
    	// fire Lasergun whenever it has >= 5/10 units of energy
    	if (Lasergun_Energy >= 5)
    	{
			char message1[] = "Lasergun shoots! Remaining energy";
			char message_print[64];
			Lasergun_Energy -= 5;			// -5/10 units of energy everytime lasergun fires
			sprintf(message_print, "%s: %d/10 units of energy\r\n", message1, Lasergun_Energy);
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
    	}

    	// Polling mode: reading all 6 sensors and performing calculations
    	float temp_data;
    	temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor

    	float humid_data;
    	humid_data = BSP_HSENSOR_ReadHumidity();	// read humidity sensor

    	float press_data;
    	press_data = BSP_PSENSOR_ReadPressure();	// read pressure sensor

    	float accel_data[3];
    	int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
    	BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
    	// the function above returns 16 bit integers which are acceleration in mg (9.8/1000 m/s^2).
    	// Converting to float to print the actual acceleration.
    	accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);	// z-axis
		// if z-axis acceleration is < -9.00m/s^2; drone is likely flipped, enters the state of "The Last of EE2028"
    	if (accel_data[2] < accel_low_threshold)
    	{
    		TheLastofEE2028 = 1;
			char message1[] = "The Last of EE2028! Awaiting rescue!\r\n";
			char message_print[64];
			sprintf(message_print, "%s", message1);
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			break;			// break out of current while loop with battle mode WITHOUT the state of "The Last of EE2028", and go to while loop BELOW with battle mode WITH the state of "The Last of EE2028"
    	}

    	float gyro_data[3];
    	float gyro_data_i16[3] = { 0 };				// array to store the x, y and z readings.
    	BSP_GYRO_GetXYZ(gyro_data_i16);				// read gyroscope
    	// the function above returns 16 bit integers which are angular rate in mdps (/1000 deg/s).
    	// Converting to float to print the actual acceleration.
    	gyro_data[0] = (float)gyro_data_i16[0]/1000.0f;	// x-axis; roll of drone
    	gyro_data[1] = (float)gyro_data_i16[1]/1000.0f;	// y-axis; pitch of drone

    	float mag_data[3];
    	int16_t mag_data_i16[3] = { 0 };		// array to store the x, y and z readings
    	BSP_MAGNETO_GetXYZ(mag_data_i16);		// read magnetometer
    	mag_data[0]= (float)mag_data_i16[0]/1000.0f;
    	mag_data[1]= (float)mag_data_i16[1]/1000.0f;
    	mag_data[2]= (float)mag_data_i16[2]/1000.0f;
    	float mag_magnitude = sqrt(mag_data[0]*mag_data[0] + mag_data[1]*mag_data[1] + mag_data[2]*mag_data[2]); 	// magnitude of M: |M| = sqrt(Mx^2 + My^2 + Mz^2)

    	// toggle LED every 0.5s (blinking at 1Hz)
    	if ((HAL_GetTick() - LED_starttime) >= 500)
    	{
    		BSP_LED_Toggle(LED2);
    		LED_starttime = HAL_GetTick();
    	}

		// TELEMETRY MONITORING
		char message5[] = "EXCEEDS LOW THRESHOLD OF";
		char message6[] = "EXCEEDS HIGH THRESHOLD OF";
		if ((HAL_GetTick() - MSG_starttime) >= 1000)		// Send messages to UART every 1s
		{
			MSG_starttime = HAL_GetTick();

	    	// threshold violation logs sent every 1s; multiple lines of threshold violation are sent if multiple thresholds are violated
	    	if (temp_data > temp_high_threshold)
	    	{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "T: %0.2fdegC %s %0.2fdegC; Suitable drone operating temperature exceeded!\r\n",temp_data,message6,temp_high_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	    	}
	    	if (temp_data < temp_low_threshold)
	    	{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "T: %0.2fdegC %s %0.2fdegC; Suitable drone operating temperature exceeded!\r\n",temp_data,message5,temp_low_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	    	}
	    	if (gyro_data[0] > gyro_high_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "Gx: %0.2fdeg/s %s %0.2fdeg/s; Drone is rotating too fast!\r\n",gyro_data[0],message6,gyro_high_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

			}
			if (gyro_data[0] < gyro_low_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "Gx: %0.2fdeg/s %s %0.2fdeg/s; Drone is rotating too fast!\r\n",gyro_data[0],message5,gyro_low_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

			}
			if (gyro_data[1] > gyro_high_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "Gy: %0.2fdeg/s %s %0.2fdeg/s; Drone is rotating too fast!\r\n",gyro_data[1],message6,gyro_high_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

			}
			if (gyro_data[1] < gyro_low_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "Gy: %0.2fdeg/s %s %0.2fdeg/s; Drone is rotating too fast!\r\n",gyro_data[1],message5,gyro_low_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

			}
			if (mag_magnitude > mag_high_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "|M|: %0.2fGauss %s %0.2fGauss; Drone is near metal object!\r\n",mag_magnitude,message6,mag_high_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
			if (press_data < press_low_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "P: %0.2fhPa %s %0.2fhPa; Drone has exceeded max. altitude of 10km!\r\n",press_data,message5,press_low_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
			if (humid_data < humid_low_threshold)
			{
				char message_print[128]; 							// UART transmit buffer
				sprintf(message_print, "H: %0.2frH %s %0.2frH; Humidity is not suitable for human habitation!\r\n",humid_data,message5,humid_low_threshold);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}

			// Send telemetry reading every 1s if no threshold are violated, telemetry readings paused otherwise when any threshold violation logs are violated
			if ((humid_data >= humid_low_threshold) && (mag_magnitude <= mag_high_threshold) && (gyro_data[0] <= gyro_high_threshold) && (gyro_data[0] >= gyro_low_threshold) && (gyro_data[1] <= gyro_high_threshold) && (gyro_data[1] >= gyro_low_threshold) && (press_data >= press_low_threshold) && (temp_data <= temp_high_threshold) && (temp_data >= temp_low_threshold))
	    	{
	    		char message_print[128]; 							// UART transmit buffer
	    		sprintf(message_print, "T: %0.2fdegC, P: %0.2fhPa, H: %0.2frH, A: %0.2fm/s^2, Gx: %0.2fdeg/s, Gy: %0.2fdeg/s, |M|: %0.2fGauss\r\n", temp_data, press_data, humid_data, accel_data[2], gyro_data[0], gyro_data[1], mag_magnitude);
	    		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode
	    		MSG_starttime = HAL_GetTick();
	    	}
		}
    }

	uint32_t RESCUE_starttime = HAL_GetTick();			// start of 10 seconds rescue timer

ignore_single_TheLastOfEE2028:							// jump to here when ignoring single button press to avoid resetting the 10s countdown

    while ((interrupt ==0) && (TheLastofEE2028 == 1)) 	// if BATTLE MODE w/ "The Last of EE2028"
    {

    	// toggle LED every 0.25s (blinking at 2Hz)
    	if ((HAL_GetTick() - LED_starttime) >= 250)
    	{
    		LED_starttime = HAL_GetTick();
    		BSP_LED_Toggle(LED2);
    	}

    	// Send "Drone was Attacked! every 1s
    	if ((HAL_GetTick() - MSG_starttime) >= 1000)
    	{
    		MSG_starttime = HAL_GetTick();
    		char message1[] = "Drone Was Attacked!\r\n";
    		char message_print[32];
    		sprintf(message_print, "%s", message1);
    		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
    	}

    	// Terminate program when not rescued after 10s countdown
    	if ((HAL_GetTick() - RESCUE_starttime) >= 10000)
    	{
    		char message1[] = "Rescue unsuccessful. Terminating connection.\r\n";
    		char message_print[64];
    		sprintf(message_print, "%s", message1);
    		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
    		mode = 2;			// terminating program mode
    		break;
    	}
    }
}

static void UART1_Init(void)
{
	    //Pin configuration for UART. BSP_COM_Init() can do this automatically
	    __HAL_RCC_GPIOB_CLK_ENABLE();
	    GPIO_InitTypeDef GPIO_InitStruct = {0};
	    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    //Configuring UART1
	    huart1.Instance = USART1;
	    huart1.Init.BaudRate = 115200;
	    huart1.Init.WordLength = UART_WORDLENGTH_8B;
	    huart1.Init.StopBits = UART_STOPBITS_1;
	    huart1.Init.Parity = UART_PARITY_NONE;
	    huart1.Init.Mode = UART_MODE_TX_RX;
	    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	    if (HAL_UART_Init(&huart1) != HAL_OK)
	    {
	      while(1);
	    }
}
