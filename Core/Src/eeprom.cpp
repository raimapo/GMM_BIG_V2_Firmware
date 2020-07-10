#include <eeprom.h>

/* Public Functions */

/**
 * @brief Constructor
 */

EEPROM::EEPROM(I2C_HandleTypeDef* hi2c){
	_i2c = hi2c;
	Address = 0x50;
}


/**
 * @brief Initialiser
 * @note Sets up the I2C interface
 */

void EEPROM::init(void){

    I2C_IsDeviceReady(_i2c, Address, 3, 1000);

    CompanyID = readCompanyID();
    if (CompanyID != 0x29) {
    	if (DEBUG_EEPROM == 1) printf("Company ID not met\n");
    	Error_Handler();
    } else {
    	if (DEBUG_EEPROM == 1) printf("Code=%#x (Microchip Technology ltd.(c))\n\r",CompanyID);
    }

    EEPROMID = readEEPROMID();
    if (EEPROMID != 0x41) {
    	if (DEBUG_EEPROM == 1) printf("EEPROM Family ID not met\n");
    	Error_Handler();
    } else {
    	if (DEBUG_EEPROM == 1) printf("Code=%#x (2K EEPROM Using i2c)\n\r",EEPROMID);
    }

    UID = readUID();
    if (DEBUG_EEPROM == 1) printf("UID Code=%lx\n\r",UID);
}

uint8_t EEPROM::readData(uint8_t * dataBuffer, uint8_t startAddress, uint8_t bytes){

    readConsecutive(dataBuffer, startAddress, bytes>32?32:bytes);
    if(bytes > 32){
        readData(dataBuffer+32, startAddress+32, bytes-32);
    }

    return *dataBuffer;
}

void EEPROM::writeData(uint8_t * dataBuffer, uint8_t startAddress, uint8_t bytes){
    // Upto 8 bytes can be written at a time, if the data is greater than 8 bytes, divide into 8 byte or smaller chunks

    uint8_t bytesOfPageRemaining = 0;
    bytesOfPageRemaining = 0x20 - (startAddress%0x20);
    bytesOfPageRemaining = bytesOfPageRemaining%MAX_WRITE_PAGE;
    if((bytes > bytesOfPageRemaining) &&(bytesOfPageRemaining != 0)){
        bytesOfPageRemaining = bytesOfPageRemaining%MAX_WRITE_PAGE;
        writePage(dataBuffer, startAddress, bytesOfPageRemaining);
        dataBuffer += bytesOfPageRemaining;
        startAddress += bytesOfPageRemaining;
        bytes -= bytesOfPageRemaining;
    }
    writePage(dataBuffer, startAddress, bytes>MAX_WRITE_PAGE?MAX_WRITE_PAGE:bytes);
    if(bytes > MAX_WRITE_PAGE){
        writeData(dataBuffer + MAX_WRITE_PAGE, startAddress + MAX_WRITE_PAGE, bytes - MAX_WRITE_PAGE);
    }
}


/* Private Functions */


uint8_t EEPROM::readByte(uint8_t dataAddress){

	uint8_t data=0x00;
	I2C_receive(_i2c, Address, dataAddress, &data, 1, 10);
	return data;
}

uint8_t EEPROM::readConsecutive(uint8_t * dataBuffer, uint8_t startAddress, uint8_t bytes){

	I2C_receive(_i2c, Address, startAddress, dataBuffer, bytes, 10);
	return *dataBuffer;
}


void EEPROM::writeByte(uint8_t dataAddress, uint8_t data){

	uint8_t dat[2];
	dat[0]=dataAddress;
	dat[1]=data;
	I2C_Write(_i2c, Address, dat, 1, 10);
}


void EEPROM::writePage(uint8_t * dataBuffer, uint8_t startAddress, uint8_t bytes){
	uint8_t i;
	uint8_t dat[bytes+1];
	dat[0]=startAddress;
	for (i=0; i<(bytes); i++) {
		dat[i+1]=dataBuffer[i];
	}
	I2C_Write(_i2c, Address, dat, bytes+1, 10);
    osDelay(5);   // This is the only intentional piece of blocking code  **********
}

uint32_t EEPROM::readUID(void){
    uint8_t UIDBytes[4]= {0,0,0,0};
    readConsecutive(UIDBytes, 0xFC, 4);
    uint8_t pos;
    UID = 0;        // Probably not needed
    for(pos = 0; pos < 4; pos++){
        UID <<= 8;
        UID |= UIDBytes[pos];
    }
    return UID;
}

uint8_t EEPROM::readCompanyID(void){
	uint8_t ID[1] = {0};
	readConsecutive(ID, 0xFA, 1);
	CompanyID = (uint8_t) ID[0];
	return CompanyID;
}

uint8_t EEPROM::readEEPROMID(void){
	uint8_t ID[1] = {0};
	readConsecutive(ID, 0xFB, 1);
	EEPROMID = (uint8_t) ID[0];
	return EEPROMID;
}

