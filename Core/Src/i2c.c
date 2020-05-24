#include "i2c.h"

osSemaphoreId I2C_completeID;
osSemaphoreDef(I2C_complete);

bool I2C_init(void)
{
    if ((I2C_completeID = osSemaphoreCreate(osSemaphore(I2C_complete), 1)) == NULL)
    {
        return false;
    }
    return true;
}

/**
 * Checking if device is ready
 */
void I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,  uint16_t DevAddress, uint32_t TestCount, uint32_t timeout){
    if (osSemaphoreWait(I2C_completeID, 1000) == osOK)
    {
		while (HAL_I2C_IsDeviceReady(hi2c, DevAddress<<1, TestCount, timeout) != HAL_OK) {
			if (DEBUG_EEPROM == 1 || DEBUG_INA226 == 1) printf("\nDevice not found\n");
			osSemaphoreRelease(I2C_completeID);
			Error_Handler();
		}
		if (DEBUG_EEPROM == 1 || DEBUG_INA226 == 1) printf("\nDevice found\n");
		osSemaphoreRelease(I2C_completeID);
    }
}

/**
 * Writing I2C data
 */
void I2C_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t* data, uint16_t Size, uint32_t Timeout){
    if (osSemaphoreWait(I2C_completeID, 1000) == osOK)
    {
		/* -> Start the transmission process */
		/* While the I2C in reception process, user can transmit data through "data" buffer */

		//while (HAL_I2C_Master_Transmit(hi2c, DevAddress<<1, data, Size, Timeout)!= HAL_OK) {
		while (HAL_I2C_Master_Transmit_DMA(hi2c, DevAddress<<1, data, Size)!= HAL_OK) {
			/*
			 * Error_Handler() function is called when Timeout error occurs.
			 * When Acknowledge failure occurs (Slave don't acknowledge it's address)
			 * Master restarts communication
			 */
			if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
				if (DEBUG_EEPROM == 1 || DEBUG_INA226 == 1) printf("In I2C::WriteBuffer -> error");
				osSemaphoreRelease(I2C_completeID);
				Error_Handler();
			}
		}
    /* -> Wait for the end of the transfer */
    /* Before starting a new communication transfer, you need to check the current
     * state of the peripheral; if it’s busy you need to wait for the end of current
     * transfer before starting a new one.
     * For simplicity reasons, this example is just waiting till the end of the
     * transfer, but application may perform other tasks while transfer operation
     * is ongoing.
     */
		while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY){
		}
		osSemaphoreRelease(I2C_completeID);
    }
}

/**
 * Reading I2C data
 */
void I2C_receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t startreg, uint8_t *data, uint16_t Size, uint32_t Timeout){

		I2C_Write(hi2c, DevAddress, &startreg, 1, Timeout);
		/* -> Put I2C peripheral in reception process */

    if (osSemaphoreWait(I2C_completeID, 1000) == osOK)
    {
		//while (HAL_I2C_Master_Receive(hi2c, DevAddress<<1, data, Size, Timeout)!= HAL_OK) {
		while (HAL_I2C_Master_Receive_DMA(hi2c, DevAddress<<1, data, Size)!= HAL_OK) {
			/* Error_Handler() function is called when Timeout error occurs.
				* When Acknowledge failure occurs (Slave don't acknowledge it's address)
				* Master restarts communication
				*/
			if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
				if (DEBUG_EEPROM == 1 || DEBUG_INA226 == 1) printf("In I2C::WriteBuffer -> error");
				osSemaphoreRelease(I2C_completeID);
				Error_Handler();
			}
		}
	   /* -> Wait for the end of the transfer */
	   /* Before starting a new communication transfer, you need to check the current
		* state of the peripheral; if it’s busy you need to wait for the end of current
		* transfer before starting a new one.
		* For simplicity reasons, this example is just waiting till the end of the
		* transfer, but application may perform other tasks while transfer operation
		* is ongoing.
		**/
	   while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
	   }
		osSemaphoreRelease(I2C_completeID);
    }
}

