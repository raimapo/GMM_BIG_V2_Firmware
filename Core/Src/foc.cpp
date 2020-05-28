#include <foc.h>

/**
*	Constructor
*/
BLDCMotor::BLDCMotor(int pp, TIM_HandleTypeDef* htim_motor, TIM_HandleTypeDef* htim_timer)
{
  // Power supply woltage
  voltage_power_supply = DEF_POWER_SUPPLY;

  pole_pairs = pp;
  _tim_motor = htim_motor;
  _tim_timer = htim_timer;

  // Velocity loop config
  // PI contoroller constant
  PI_velocity.P = DEF_PI_VEL_P;
  PI_velocity.I = DEF_PI_VEL_I;
  PI_velocity.timestamp = _tim_timer->Instance->CNT;
  PI_velocity.voltage_limit = voltage_power_supply/2;
  PI_velocity.voltage_ramp = DEF_PI_VEL_U_RAMP;
  PI_velocity.voltage_prev = 0;
  PI_velocity.tracking_error_prev = 0;

  // velocity low pass filter
  LPF_velocity.Tf = DEF_VEL_FILTER_Tf;
  LPF_velocity.timestamp = _tim_timer->Instance->CNT;
  LPF_velocity.prev = 0;

  // position loop config
  // P controller constant
  P_angle.P = DEF_P_ANGLE_P;
  // maximum angular velocity to be used for positioning
  P_angle.velocity_limit = DEF_P_ANGLE_VEL_LIM;

  // index search PI controller
  PI_velocity_index_search.P = DEF_PI_VEL_INDEX_P;
  PI_velocity_index_search.I = DEF_PI_VEL_INDEX_I;
  PI_velocity_index_search.voltage_limit = voltage_power_supply/2;
  PI_velocity_index_search.voltage_ramp = DEF_PI_VEL_INDEX_U_RAMP;
  PI_velocity_index_search.timestamp = _tim_timer->Instance->CNT;
  PI_velocity_index_search.voltage_prev = 0;
  PI_velocity_index_search.tracking_error_prev = 0;

  // index search velocity
  index_search_velocity = DEF_INDEX_SEARCH_TARGET_VELOCITY;

  // electric angle of the zero angle
  zero_electric_angle = 0;

  // default modulation is SinePWM
  //foc_modulation = FOCModulationType::SinePWM;
  foc_modulation = FOCModulationType::SpaceVectorPWM;
}


void BLDCMotor::linkEncoder(AS5048A* enc) {
  encoder = enc;
}


// init hardware pins
void BLDCMotor::init() {

	// if(DEBUG_MAIN) printf("DEBUG: Set high frequency PWM.");
	// Increase PWM frequency to 32 kHz
		// make silent
		// Done By HA

	printf("Initilaise the motor pins\n");
	// PWM pins
	//Enable PWM generation at 25kHz = 76000000/(999*3)
	HAL_TIM_PWM_Start(_tim_motor, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(_tim_motor, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(_tim_motor, TIM_CHANNEL_3);

	// sanity check for the voltage limit configuration
	if(PI_velocity.voltage_limit > voltage_power_supply/2) PI_velocity.voltage_limit =  voltage_power_supply/2;
	if(PI_velocity_index_search.voltage_limit > voltage_power_supply/2) PI_velocity_index_search.voltage_limit = voltage_power_supply/2;
	osDelay(500);

	// enable motor
	printf("Enabling motor\n");
	enable();
	osDelay(500);
}

//disable motor
void BLDCMotor::disable()
{
  // pins low to disable Driver
	//High to enable PWM inputs
	HAL_GPIO_WritePin(MOT_EN1_GPIO_Port, MOT_EN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOT_EN2_GPIO_Port, MOT_EN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOT_EN3_GPIO_Port, MOT_EN3_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOT_DRV_SLEEP_GPIO_Port, MOT_DRV_SLEEP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOT_DRV_RST_GPIO_Port, MOT_DRV_RST_Pin, GPIO_PIN_SET);

}

//enable motor
void BLDCMotor::enable()
{
  // pins high to enable Driver
	//High to enable PWM inputs
	HAL_GPIO_WritePin(MOT_EN1_GPIO_Port, MOT_EN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOT_EN2_GPIO_Port, MOT_EN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOT_EN3_GPIO_Port, MOT_EN3_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(MOT_DRV_SLEEP_GPIO_Port, MOT_DRV_SLEEP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOT_DRV_RST_GPIO_Port, MOT_DRV_RST_Pin, GPIO_PIN_SET);
}


//Encoder alignment to electrical 0 angle
int BLDCMotor::alignEncoder() {
	//printf("Align the encoder and motor electrical 0 angle\n");
	// align the electircal phases of the motor and encoder
	setPwm(1, voltage_power_supply/2.0);
	setPwm(2,0);
	setPwm(3,0);

	osDelay(500);
	// set sensor to zero
	encoder->initRelativeZero();
	osDelay(500);
	setPhaseVoltage(0,0);
	osDelay(200);

	// find the index if available
	int exit_flag = absoluteZeroAlign();
	osDelay(500);
	//if(exit_flag< 0 ) printf("DEBUG: Error: Absolute zero not found!\n");
	//if(exit_flag> 0 ) printf("DEBUG: Success: Absolute zero found!\n");
	//else  printf("DEBUG: Absolute zero not availabe!\n");

  return exit_flag;
}

// Encoder alignment the absolute zero angle
// - to the index
int BLDCMotor::absoluteZeroAlign() {
  // if no absolute zero return
  if(!encoder->hasAbsoluteZero()) return 0;

  //printf("DEBUG: Aligning the absolute zero.\n");

  if(encoder->needsAbsoluteZeroSearch()) printf("DEBUG: Searching for absolute zero.\n");
  // search the absolute zero with small velocity
  while(encoder->needsAbsoluteZeroSearch() && shaft_angle < _2PI){
    loopFOC();
    voltage_q = velocityIndexSearchPI(index_search_velocity - shaftVelocity());
  }
  voltage_q = 0;
  // disable motor
  setPhaseVoltage(0,0);

  // align absoulute zero if it has been found
  if(!encoder->needsAbsoluteZeroSearch()){
    // align the sensor with the absolute zero
    float zero_offset = encoder->initAbsoluteZero();
    // remember zero electric angle
    zero_electric_angle = electricAngle(zero_offset);
  }

  // return bool is zero found
  return !encoder->needsAbsoluteZeroSearch() ? 1 : -1;
}

/*
 * FOC functions
 */

//initialization function
int  BLDCMotor::initFOC() {
  // encoder alignment
  osDelay(500);
  int exit_flag = alignEncoder();
  osDelay(500);
  return exit_flag;
}


//State calculation methods

// shaft angle calculation
float BLDCMotor::shaftAngle() {
	  return encoder->getAngleInRad();
}

// shaft velocity calculation
float BLDCMotor::shaftVelocity() {
	float Ts;
	uint32_t cur_counter = _tim_timer->Instance->CNT;
	if (cur_counter > LPF_velocity.timestamp)
	{
		Ts = ((float)(cur_counter - LPF_velocity.timestamp)) * 1e-6;
	}
	else
	{
		Ts = ((float)(cur_counter + (MAX_UINT16_NUMBER - LPF_velocity.timestamp))) * 1e-6;
	}
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3;
  // calculate the fitering
  float alpha = LPF_velocity.Tf/(LPF_velocity.Tf + Ts);
  float vel = alpha * LPF_velocity.prev + (1 - alpha) * encoder->getVelocity();
  // save the variables
  LPF_velocity.prev = vel;
  LPF_velocity.timestamp = cur_counter;
  return vel;
}

//Electrical angle calculation
float BLDCMotor::electricAngle(float shaftAngle) {
  //return normalizeAngle(shaftAngle * pole_pairs);
  return (shaftAngle * pole_pairs);
}

//Iterative function looping FOC algorithm, setting Uq on the Motor
//The faster it can be run the better
void BLDCMotor::loopFOC() {
  // voltage open loop loop
  shaft_angle = shaftAngle();
  setPhaseVoltage(voltage_q, electricAngle(shaft_angle));
}


 // Iterative funciton running outer loop of the FOC algorithm
  //Bahvior of this funciton is determined by the motor.controller variable
  //It runs either angle, veloctiy, velocity ultra slow or voltage loop
  //- needs to be called iteratively it is asynchronious function

void BLDCMotor::move(float target) {
  // get angular velocity
  shaft_velocity = shaftVelocity();
  switch (controller) {
    case ControlType::voltage:
      voltage_q = target;
      break;
    case ControlType::angle:
      // angle set point
      // include angle loop
      shaft_angle_sp = target;
      shaft_velocity_sp = positionP( shaft_angle_sp - shaft_angle );
      voltage_q = velocityPI(shaft_velocity_sp - shaft_velocity);
      break;
    case ControlType::velocity:
      // velocity set point
      // inlcude velocity loop
      shaft_velocity_sp = target;
      voltage_q = velocityPI(shaft_velocity_sp - shaft_velocity);
      break;
  }
}

//FOC methods

//Method using FOC to set Uq to the motor at the optimal angle
void BLDCMotor::setPhaseVoltage(float Uq, float angle_el) {
	  switch (foc_modulation)
	  {
	  	  case FOCModulationType::SinePWM :
	  		  // angle normalisation in between 0 and 2pi
	  		  // only necessary if using _sin and _cos - approximation funcitons
	  		  angle_el = normalizeAngle(angle_el + zero_electric_angle);
	  		  // Inverse park transform
	  		  Ualpha =  -arm_sin_f32(angle_el) * Uq;  // -sin(angle) * Uq;
	  		  Ubeta =  arm_cos_f32(angle_el) * Uq;    //  cos(angle) * Uq;

	  		  // Clarke transform
	  		  Ua = Ualpha  + voltage_power_supply/2;
	  		  Ub = -0.5 * Ualpha  + _SQRT3_2 * Ubeta  + voltage_power_supply/2;
	  		  Uc = -0.5 * Ualpha - _SQRT3_2 * Ubeta  + voltage_power_supply/2;
	  		  break;

	      case FOCModulationType::SpaceVectorPWM :
	        // Nice video explaining the SpaceVectorModulation (SVPWM) algorithm
	        // https://www.youtube.com/watch?v=QMSWUMEAejg

	        // if negative voltages change inverse the phase
	        // angle + 180degrees
	        if(Uq < 0) angle_el += M_PI;
	        Uq = fabs(Uq);

	        // angle normalisation in between 0 and 2pi
	        // only necessary if using _sin and _cos - approximation functions
	        angle_el = normalizeAngle(angle_el + zero_electric_angle + _PI_2);

	        // find the sector we are in currently
	        int sector = floor(angle_el / _PI_3) + 1;
	        // calculate the duty cycles
	        float T1 = _SQRT3*arm_sin_f32(sector*_PI_3 - angle_el);
	        float T2 = _SQRT3*arm_sin_f32(angle_el - (sector-1.0)*_PI_3);
	        // two versions possible
	        // centered around voltage_power_supply/2
	        float T0 = 1 - T1 - T2;
	        // centered around 0
	        //T0 = 0;

	        // calculate the duty cycles(times)
	        float Ta,Tb,Tc;
	        switch(sector){
	          case 1:
	            Ta = T1 + T2 + T0/2;
	            Tb = T2 + T0/2;
	            Tc = T0/2;
	            break;
	          case 2:
	            Ta = T1 +  T0/2;
	            Tb = T1 + T2 + T0/2;
	            Tc = T0/2;
	            break;
	          case 3:
	            Ta = T0/2;
	            Tb = T1 + T2 + T0/2;
	            Tc = T2 + T0/2;
	            break;
	          case 4:
	            Ta = T0/2;
	            Tb = T1+ T0/2;
	            Tc = T1 + T2 + T0/2;
	            break;
	          case 5:
	            Ta = T2 + T0/2;
	            Tb = T0/2;
	            Tc = T1 + T2 + T0/2;
	            break;
	          case 6:
	            Ta = T1 + T2 + T0/2;
	            Tb = T0/2;
	            Tc = T1 + T0/2;
	            break;
	          default:
	           // possible error state
	            Ta = 0;
	            Tb = 0;
	            Tc = 0;
	        }

	        // calculate the phase voltages
	        Ua = Ta*Uq;
	        Ub = Tb*Uq;
	        Uc = Tc*Uq;
	        break;
	  }

  // set phase voltages
  setPwm(1, Ua);
  setPwm(2, Ub);
  setPwm(3, Uc);
}

//Set voltage to the pwm pin
//- function a bit optimised to get better performance
// Here 1000 is a Timer Period

void BLDCMotor::setPwm(int pinPwm, float U) {

  // max value
  float U_max = voltage_power_supply;

  // sets the voltage [0,12V(U_max)] to pwm [0,1000]
    // - U_max you can set in header file - default 12V
   int U_pwm = 1000.0 * U / U_max;

  // limit the values between 0 and 255
  U_pwm = (U_pwm < 0) ? 0 : (U_pwm >= 1000) ? 1000 : U_pwm;

  // write hardware pwm
  if (pinPwm == 3)
	  _tim_motor->Instance->CCR1 = U_pwm;
  if (pinPwm == 1)
	  _tim_motor->Instance->CCR2 = U_pwm;
  if (pinPwm == 2)
	  _tim_motor->Instance->CCR3 = U_pwm;
}


//Utility funcitons

//normalizing radian angle to [0,2PI]
float BLDCMotor::normalizeAngle(float angle){
	float a = fmod(angle, _2PI);
	return a >= 0 ? a : (a + _2PI);
}

//Motor control functions

// PI controller function
float BLDCMotor::controllerPI(float tracking_error, PI_s& cont){
	float Ts;
	uint32_t cur_counter = _tim_timer->Instance->CNT;
	if (cur_counter > cont.timestamp)
	{
		Ts = ((float)(cur_counter - cont.timestamp)) * 1e-6;
	}
	else
	{
		Ts = ((float)(cur_counter + (MAX_UINT16_NUMBER - cont.timestamp))) * 1e-6;
	}

  // quick fix for strange cases (micros overflow)
  //printf("Ts %f\n", Ts);
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3;
  // u(s) = (P + I/s)e(s)
  // tustin discretisation of the PI controller ( a bit optimised )
  // uk = uk_1  + (I*Ts/2 + P)*ek + (I*Ts/2 - P)*ek_1
  float tmp = cont.I*Ts*0.5;
  float voltage = cont.voltage_prev + (tmp + cont.P) * tracking_error + (tmp - cont.P) * cont.tracking_error_prev;
  // antiwindup - limit the output voltage_q
  if (abs(voltage) > cont.voltage_limit) voltage = voltage > 0 ? cont.voltage_limit : -cont.voltage_limit;
  // limit the acceleration by ramping the the voltage
  float d_voltage = voltage - cont.voltage_prev;
  if (abs(d_voltage)/Ts > cont.voltage_ramp) voltage = d_voltage > 0 ? cont.voltage_prev + cont.voltage_ramp*Ts : cont.voltage_prev - cont.voltage_ramp*Ts;
  //printf("Voltage prev %f\n", cont.voltage_prev);
  //printf("Voltage %f\n", voltage);
  cont.voltage_prev = voltage;
  cont.tracking_error_prev = tracking_error;
  cont.timestamp = cur_counter;
  return voltage;
}

// velocity control loop PI controller
float BLDCMotor::velocityPI(float tracking_error) {
  return controllerPI(tracking_error, PI_velocity);
}

// P controller for position control loop
float BLDCMotor::positionP(float ek) {
  float velk = P_angle.P * ek;
  if (abs(velk) > P_angle.velocity_limit) velk = velk > 0 ? P_angle.velocity_limit : -P_angle.velocity_limit;
  return velk;
}

// index search PI contoller
float BLDCMotor::velocityIndexSearchPI(float tracking_error) {
  return controllerPI(tracking_error, PI_velocity_index_search);
}

