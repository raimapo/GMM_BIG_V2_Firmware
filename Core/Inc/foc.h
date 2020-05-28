/**
  ******************************************************************************
  * @file           : eeprom.h
  * @brief          : Header for eeprom.cpp file.
  *                   This file contains the common defines of the application.
  * @version		: 0.1
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 Raimondas Pomarnacki.
  * All rights reserved.</center></h2>
  *
  * This software component is not licensed,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FOC_HPP
#define __FOC_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "math.h"
#include "stdio.h"

#include "as5048a.h"
#include "arm_math.h"

// default configuration values
// power supply voltage
#define DEF_POWER_SUPPLY 12.0
// velocity PI controller params
#define DEF_PI_VEL_P 0.5
#define DEF_PI_VEL_I 10
#define DEF_PI_VEL_U_RAMP 300
// angle P params
#define DEF_P_ANGLE_P 20
// angle velocity limit default
#define DEF_P_ANGLE_VEL_LIM 20
// index search velocity
#define DEF_INDEX_SEARCH_TARGET_VELOCITY 1
// velocity PI controller params for index search
#define DEF_PI_VEL_INDEX_P 1
#define DEF_PI_VEL_INDEX_I 10
#define DEF_PI_VEL_INDEX_U_RAMP 100
// velocity filter time constant
#define DEF_VEL_FILTER_Tf 0.005

// utility defines
#define _2_SQRT3 1.15470053838
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _SQRT3 1.73205080757
#define _120_D2R 2.09439510239
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038

// controller type configuration enum
enum ControlType{
  voltage,
  velocity,
  angle
};

// FOC Type
enum FOCModulationType{
  SinePWM,
  SpaceVectorPWM
};

// PI controller strucutre
struct PI_s{
  float P;
  float I;
  uint32_t timestamp;
  float voltage_prev, tracking_error_prev;
  float voltage_limit;
  float voltage_ramp;
};

// P controller structure
struct P_s{
  float P;
  uint32_t timestamp;
  float voltage_prev, tracking_error_prev;
  float velocity_limit;
};

// flow pass filter structure
struct LPF_s{
  float Tf;
  uint32_t timestamp;
  float prev;
};

#ifdef __cplusplus

/**
 * @brief Declare to use Encoder class and link to it
 */
class AS5048A;

/**
 * @brief BLDC motor class
 */
class BLDCMotor {

	public:


		uint8_t pole_pairs;
		TIM_HandleTypeDef* _tim_motor;
		TIM_HandleTypeDef* _tim_timer;

	    // Power supply woltage
	    float voltage_power_supply;

	    // state variables
	     // current motor angle
	   	float shaft_angle;
	     // current motor velocity
	   	float shaft_velocity;
	     // current target velocity
	     float shaft_velocity_sp;
	     // current target angle
	     float shaft_angle_sp;
	     // current voltage u_q set
	     float voltage_q;

	     // configuration structures
	     ControlType controller;
	     FOCModulationType foc_modulation;
	     PI_s PI_velocity;
	     PI_s PI_velocity_index_search;
	     P_s P_angle;
	     LPF_s LPF_velocity;

	     // sensor link:
	     // - Encoder
	     // - MagneticSensor
	     AS5048A* sensor;
	     // absolute zero electric angle - if available
	     float zero_electric_angle;
	     // index search velocity
	     float index_search_velocity;

	     float Ua,Ub,Uc;

  		/**
  		 * @brief Constructor. If you do not use some pins leave it to 17.
  		 * @param - Number of poles integer
  		 * @param - Motor PWM timer reference
  		 * @param - Microseconds timer reference
  		 */
	    BLDCMotor(int pp, TIM_HandleTypeDef* htim_motor, TIM_HandleTypeDef* htim_timer);

	    /*
	     * change Driver state
	     */
	  	void init();
	  	void disable();
	    void enable();

	    /*
	     * connect encoder
	     */
		AS5048A* encoder;
		void linkEncoder(AS5048A* enc);

	    /*
	     *  FOC methods
	    */
	    //Method using FOC to set Uq to the motor at the optimal angle
	    void setPhaseVoltage(float Uq, float angle_el);



	    /*
	     * initilise FOC
	     */
	    int initFOC();
	    // iterative method updating motor angles and velocity measurement
	    void loopFOC();
	    // iterative control loop defined by controller
	    void move(float target);

	    /*
	     *  State calculation methods
	    */
	    //Shaft angle calculation
	    float shaftAngle();
	    //Shaft velocity calculation
	    float shaftVelocity();


	  private:

	    // phase voltages
	    float	Ualpha, Ubeta;

	    //Motor and sensor alignement to the sensors absolute 0 angle
	    int absoluteZeroAlign();

	    //Encoder alignment to electrical 0 angle
	    int alignEncoder();
	    int indexSearch();

	    //Set phase voltaget to pwm output
	    void setPwm(int pinPwm, float U);

	    /** Utility funcitons */
	    //normalizing radian angle to [0,2PI]
	    float normalizeAngle(float angle);

	    //Electrical angle calculation
	    float electricAngle(float shaftAngle);

	    /** Motor control functions */
	    float controllerPI(float tracking_error, PI_s &controller);
	    float velocityPI(float tracking_error);
	    float positionP(float ek);
	    float velocityIndexSearchPI(float tracking_error);


};

#endif


#ifdef __cplusplus
}
#endif

#endif /* __FOC_HPP */
