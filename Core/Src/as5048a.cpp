#include "as5048a.h"

/**
 * @brief Constructor
 */
AS5048A::AS5048A(SPI_HandleTypeDef* hspi, GPIO_TypeDef* arg_ps, uint16_t arg_cs, TIM_HandleTypeDef* htim_timer){
	_cs = arg_cs;
	_ps = arg_ps;
	_spi = hspi;
	_tim_timer = htim_timer;
	errorFlag = 0;
	position = 0;
}

#define EN_SPI HAL_GPIO_WritePin(_ps, _cs, GPIO_PIN_RESET);
#define DIS_SPI HAL_GPIO_WritePin(_ps, _cs, GPIO_PIN_SET);

/**
 * @brief Initialiser
 * @note Sets up the SPI interface
 */
void AS5048A::init(){

	//You can write here various checking functions here

	AS5048A::close();
	AS5048A::open();

	// velocity calculation init
	angle_prev = 0;
	//velocity_calc_timestamp = HAL_GetTick();
	velocity_calc_timestamp = _tim_timer->Instance->CNT;

	// full rotations tracking number
	full_rotation_offset = 0;
	angle_data_prev = getRawRotation();
	zero_offset = 0;
}

/**
 * @brief Closes the SPI connection
 * @note SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void AS5048A::close(){
	if (HAL_SPI_DeInit(_spi) != HAL_OK)
	{
		//User error function
	}
}


/**
 * @brief Openthe SPI connection
 * @note SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void AS5048A::open(){
	if (HAL_SPI_Init(_spi) != HAL_OK)
	{
		//User error function
	}
}

/**
 * @brief Utility function used to calculate even parity of word
 */
uint8_t AS5048A::spiCalcEvenParity(uint16_t value){
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

//#pragma GCC push_options
//#pragma GCC optimize("O0")

/*
 * @brief Read a register from the sensor
 * @note Takes the address of the register as a 16 bit word
 * @note Returns the value of the register
 */
uint16_t AS5048A::read(uint16_t registerAddress){
	volatile uint8_t data[2];

	volatile uint16_t command = 0b0100000000000000; // PAR=0 R/W=R
	command = command | registerAddress;

	//Add a parity bit on the the MSB
	command |= ((uint16_t)spiCalcEvenParity(command)<<15);

	//Split the command into two bytes
	data[1] = command & 0xFF;
	data[0] = ( command >> 8 ) & 0xFF;

	EN_SPI;
	HAL_SPI_Transmit(_spi, (uint8_t *)&data, 2, 0xFFFF);
	while (HAL_SPI_GetState(_spi) != HAL_SPI_STATE_READY) {}
	DIS_SPI;

	EN_SPI;
	HAL_SPI_Receive(_spi, (uint8_t *)&data, 2, 0xFFFF);
	while (HAL_SPI_GetState(_spi) != HAL_SPI_STATE_READY) {}
	DIS_SPI;

	if (data[1] & 0x40) {
		errorFlag = 1;
	} else {
		errorFlag = 0;
	}

	//Return the data, stripping the parity and error bits
	return (( ( data[1] & 0xFF ) << 8 ) | ( data[0] & 0xFF )) & ~0xC000;
}

/*
 * @brief Write to a register
 * @note Takes the 16-bit  address of the target register and the 16 bit word of data
 * @note to be written to that register
 * @note Returns the value of the register after the write has been performed. This
 * @note is read back from the sensor to ensure a sucessful write.
 */
uint16_t AS5048A::write(uint16_t registerAddress, uint16_t data) {

	volatile uint8_t dat[2];

	volatile uint16_t command = 0b0000000000000000; // PAR=0 R/W=W
	command |= registerAddress;

	//Add a parity bit on the the MSB
	command |= ((uint16_t)spiCalcEvenParity(command)<<15);

	//Split the command into two bytes
	dat[1] = command & 0xFF;
	dat[0] = ( command >> 8 ) & 0xFF;

	//Start the write command with the target address
	EN_SPI;
	HAL_SPI_Transmit(_spi, (uint8_t *)&dat, 2, 0xFFFF);
	while (HAL_SPI_GetState(_spi) != HAL_SPI_STATE_READY) {}
	DIS_SPI;

	volatile uint16_t dataToSend = 0b0000000000000000;
	dataToSend |= data;

	//Craft another packet including the data and parity
	dataToSend |= ((uint16_t)spiCalcEvenParity(dataToSend)<<15);
	dat[1] = command & 0xFF;
	dat[0] = ( command >> 8 ) & 0xFF;

	//Now send the data packet
	EN_SPI;
	HAL_SPI_Transmit(_spi, (uint8_t *)&dat, 2, 0xFFFF);
	while (HAL_SPI_GetState(_spi) != HAL_SPI_STATE_READY) {}
	DIS_SPI;

	//Send a NOP to get the new data in the register
	dat[1] = 0x00;
	dat[0] = 0x00;
	EN_SPI;
	HAL_SPI_Transmit(_spi, (uint8_t *)&dat, 2, 0xFFFF);
	while (HAL_SPI_GetState(_spi) != HAL_SPI_STATE_READY) {}
	HAL_SPI_Receive(_spi, (uint8_t *)&dat, 2, 0xFFFF);
	while (HAL_SPI_GetState(_spi) != HAL_SPI_STATE_READY) {}
	DIS_SPI;

	//Return the data, stripping the parity and error bits
	return (( ( dat[1] & 0xFF ) << 8 ) | ( dat[0] & 0xFF )) & ~0xC000;
}

//#pragma GCC pop_options

/**
 * Returns the raw angle directly from the sensor
 */
uint16_t AS5048A::getRawRotation(){
	return AS5048A::read(AS5048A_ANGLE);
}

/**
 * @brief Get the rotation of the sensor relative to the zero position.
 * @return {int} between -2^13 and 2^13
 */
int AS5048A::getRotation(){
	uint16_t data;
	int rotation;

	data = AS5048A::getRawRotation();
	rotation = (int)data - (int)position;
	if(rotation > 8191) rotation = -((0x3FFF)-rotation); //more than -180
	//if(rotation < -0x1FFF) rotation = rotation+0x3FFF;

	return rotation;
}

/**
 * @brief returns the value of the state register
 * @return 16 bit word containing flags
 */
uint16_t AS5048A::getState(){
	return AS5048A::read(AS5048A_DIAG_AGC);
}

/**
 * @brief Check if an error has been encountered.
 */
uint8_t AS5048A::error(){
	return errorFlag;
}

/**
 * @briefReturns the value used for Automatic Gain Control (Part of diagnostic register)
 */
uint8_t AS5048A::getGain(){
	uint16_t data = AS5048A::getState();
	return (uint8_t) data & 0xFF;
}

/**
 * @brief Get and clear the error register by reading it
 */
uint16_t AS5048A::getErrors(){
	return AS5048A::read(AS5048A_CLEAR_ERROR_FLAG);
}

/**
 * @brief Set the zero position
 */
void AS5048A::setZeroPosition(uint16_t arg_position){
	position = arg_position % 0x3FFF;
}

/**
 * @brief Returns the current zero position
 */
uint16_t AS5048A::getZeroPosition(){
	return position;
}

/**
 * @brief Returns normalized angle value
 */
float AS5048A::normalize(float angle) {
	// http://stackoverflow.com/a/11498248/3167294
	#ifdef ANGLE_MODE_1
		angle += 180;
	#endif
	angle = fmod(angle, 360);
	if (angle < 0) {
		angle += 360;
	}
	#ifdef ANGLE_MODE_1
		angle -= 180;
	#endif
	return angle;
}

/**
 * @brief Returns calculated angle value
 */
float AS5048A::read2angle(uint16_t angle) {
	/**
	 * @note 14 bits = 2^(14) - 1 = 16.383
	 */
	return (float)angle * ((float)360 / 16383);
};


/**
 * @brief Shaft angle calculation
 * @note angle is in radians [rad]
 */
float AS5048A::getAngleInRad(){
	taskENTER_CRITICAL();
	// raw data from the sensor
	float angle_data = getRawRotation();

	// tracking the number of rotations
	// in order to expand angle range form [0,2PI]
	// to basically infinity
	float d_angle = angle_data - angle_data_prev;
	// if overflow happened track it as full rotation
	if(abs(d_angle) > (0.8*16384) ) full_rotation_offset += d_angle > 0 ? -_2PI : _2PI;
	// save the current angle value for the next steps
	// in order to know if overflow happened
	angle_data_prev = angle_data;

	// zero offset adding
	angle_data -= (int)zero_offset;
	// return the full angle
	// (number of full rotations)*2PI + current sensor angle
	taskEXIT_CRITICAL();
 	return full_rotation_offset + ( angle_data / (float)16384) * _2PI;
}

/**
 * @brief Shaft velocity calculation
 */
float AS5048A::getVelocity(){
  // calculate sample time
	float Ts;
	uint32_t cur_counter = _tim_timer->Instance->CNT;
	if (cur_counter > velocity_calc_timestamp)
	{
		Ts = ((float)(cur_counter - velocity_calc_timestamp)) * 1e-6;
	}
	else
	{
		Ts = ((float)(cur_counter + (MAX_UINT16_NUMBER - velocity_calc_timestamp))) * 1e-6;
	}
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3;

  // current angle
  float angle_c = getAngleInRad();
  // velocity calculation
  float vel = (angle_c - angle_prev)/Ts;

  // save variables for future pass
  angle_prev = angle_c;
  //velocity_calc_timestamp = HAL_GetTick();
  velocity_calc_timestamp = cur_counter;
  return vel;
}

/*
 * @brief set current angle as zero angle
 * @note return the angle [rad] difference
 */
float AS5048A::initRelativeZero(){
  float angle_offset = -getAngleInRad();
  //zero_offset = getRawRotation();

  // angle tracking variables
  //full_rotation_offset = 0;
  return angle_offset;
}

/**
 * @brief returns 0 if it has no absolute 0 measurement
 * @note 0 - incremental encoder without index
 * @note 1 - encoder with index & magnetic sensors
 */
int AS5048A::hasAbsoluteZero(){
  return 1;
}

/**
 * @brief returns 0 if it does need search for absolute zero
 * @note 0 - magnetic sensor
 * @note 1 - ecoder with index
 */
int AS5048A::needsAbsoluteZeroSearch(){
  return 0;
}

/**
 * @brief set absoule zero angle as zero angle
 * @note return the angle [rad] difference
 */
float AS5048A::initAbsoluteZero(){
  float rotation = -(int)zero_offset;
  // init absolute zero
  zero_offset = 0;

  // angle tracking variables
  full_rotation_offset = 0;
  // return offset in radians
  return rotation / (float)16384 * _2PI;
}

