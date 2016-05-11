/**********************************************************************
*  MIT License
*
*  Copyright (c) 2016, Jan., Antonio Petitti and Donato Di Paola
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*  
*  The above copyright notice and this permission notice shall be included in all
*  copies or substantial portions of the Software.
*  
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*  SOFTWARE.
**********************************************************************/

#ifndef ECA_ARM5_MICRO_DRIVER_ARM_H_
#define ECA_ARM5_MICRO_DRIVER_ARM_H_

#include "../lib/serialib.h"

#include <iostream>
#include <stdint.h>



	struct ArmState {
			std::string 	status;
			int		currents[5];
			uint16_t		encoders[5];
			int32_t			positions[5];
			int		speeds[5];
			int		temperatures[5];
			int		demand_type[5];
	};

	class DriverEcaArm {
	private:
		serialib	m_SerialConnection;
		bool        m_connected;
 
		ArmState 	m_armState;
		int			m_current_limits[5];
		int			m_speed_limits[5];

		int     	m_old_bits[5];

		int			m_encoder_flag[5];
		int     	m_position_encoder_bias[5];

		void  	getJointMessage(uint8_t* message, int demand_type, int demand, int speed_limit,  int current_limit);
		void  	getArmMessage(uint8_t* message, int* demand_type, int* demand, int* speed_limit, int* current_limit);		

		float 	bitToRad(float input, bool reverse, int jointId);
		float 	bitToRadSec(int input);
		float 	bitToMilliAmpere(int input);
		float 	bitToCelsius(int input);
		float 	bitToVolts(int input);
		int 	demandToBit(float value);

		int32_t bitNormalization(uint16_t bits, int num_el); 
		void  	setEncoderFlag(uint16_t* bits);
		int   	getArmStatus();
		void	resetFlags();

		float 	deg2rad(float input);
		float 	rad2deg(float input);


	public:
		DriverEcaArm();
		DriverEcaArm(int* current_limits, int* speed_limits);
		~DriverEcaArm();

		int  connect(std::string);
		void disconnect();
		bool isConnected();
		int  sendMotionCommand(int* demand_type, float* demand); // write and send to the arm 
		int	 receiveArmStatus();	       			     		 // read from the arm
		int	 resetEncoders();

		void getArmState_positions(float*);
		void getArmState_currents(float*);
		void getArmState_speeds(float*);
		void getArmState_temperatures(float*);

		int homing();

	};


#endif /* ECA_ARM5_MICRO_DRIVER_ARM_H_ */
