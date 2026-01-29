
#include "DHT11_Driver.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

static void delay_microseconds(uint16_t microseconds) {
// HAL but in micro seconds
	htim1.Instance->CNT = 0;
	while (htim1.Instance->CNT < microseconds) {
		// Blocking Loop
	};
}



// returns 0 if successful otherwise -1
int DHT11_Read_Data(int *temp, int *hum) {
	// --- 1. Send Start Signal ---
	// Pull down 18ms to send start signal to DHT11
	HAL_GPIO_WritePin(DHT11_Data_GPIO_Port, DHT11_Data_Pin, GPIO_PIN_RESET);
	delay_microseconds(18000);

	// Pull up 20-40us wait for DHT11 response
	HAL_GPIO_WritePin(DHT11_Data_GPIO_Port, DHT11_Data_Pin, GPIO_PIN_SET);
	delay_microseconds(20);

	// --- 2. Check for Sensor Response (Handshake) ---
	uint16_t timeout = 0;

	// Verify handshake based on Datasheet 80 millsec hi then low
	while (HAL_GPIO_ReadPin(DHT11_Data_GPIO_Port, DHT11_Data_Pin)
			== GPIO_PIN_RESET) {
		delay_microseconds(1);
		if (timeout++ > 400)
			return -1;
	}

	while (HAL_GPIO_ReadPin(DHT11_Data_GPIO_Port, DHT11_Data_Pin)
			== GPIO_PIN_SET) {
		//Do nothing if its High or timeout
		delay_microseconds(1);
		if (timeout++ > 400)
			return -1;
	}
	// --- 3. Read 40 Bits of Data ---
	/* Start Data Transmission 40 bits
	 * wait for low, then when it goes high, read the bit either 26-28ms or 50 if it was high
	 * Data format: 8bit integral RH data + 8bit decimal RH data + 8bit integral T data + 8bit decimal T
	 * data + 8bit check sum. If the data transmission is right, the check-sum should be the last 8bit of
	 * "8bit integral RH data + 8bit decimal RH data + 8bit integral T data + 8bit decimal T data".
	 */

	// Create 5 blocks of 8 bits at 0
	uint8_t data[5] = { 0 };

	for (int i = 0; i < 5; i++) {
		uint8_t current_byte = 0; //fresh 0000 0000
		for (int j = 0; j < 8; j++) {
			// wait for low -> High Indicating Data
			while (HAL_GPIO_ReadPin(DHT11_Data_GPIO_Port, DHT11_Data_Pin)
					== GPIO_PIN_RESET) {
				// Wait loop
			}

			// Shift left for every new bit for room
			current_byte <<= 1;

			// wait 40 microseconds then read (low is 20ms) high is 70ms
			delay_microseconds(40);

			// if still high after 40 seconds its 1
			if (HAL_GPIO_ReadPin(DHT11_Data_GPIO_Port, DHT11_Data_Pin)
					== GPIO_PIN_SET) {
				// 1 = 0000 0001 bit-shift with Or Operator
				current_byte |= 1;
			}

			// if 0 do nothing since we already left shifted
			// Wait for high pulse to end
			while (HAL_GPIO_ReadPin(DHT11_Data_GPIO_Port, DHT11_Data_Pin)
					== GPIO_PIN_SET) {
				// Wait loop
			}
		}
		// write to array
		data[i] = current_byte;
	}
	// --- 4. Verify Checksum & Convert ---
	// Sum of first 4 bytes must equal the 5th byte (datasheet)
	// data[1] and data [3] contain the float data but outside of
	// tolerance for this sensor

	if (data[4] == (data[0] + data[1] + data[2] + data[3])) {
		*hum = data[0];
		*temp = data[2];
		return 0;
	}

	return -1;
}
