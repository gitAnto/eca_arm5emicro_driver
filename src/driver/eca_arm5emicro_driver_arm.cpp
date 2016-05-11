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

#include "eca_arm5emicro_driver_arm.h"

#include <math.h>
#include <algorithm>



	DriverEcaArm::DriverEcaArm(){
			m_armState.status = "";

			for(int i=0; i<5; i++){
					m_armState.demand_type[i] = 0;
					m_armState.currents[i] = 0;
					m_armState.positions[i] = 0;
					m_armState.encoders[i] = 0;
					m_armState.speeds[i] = 0;
					m_armState.temperatures[i] = 0;

					m_old_bits[i] = 0;
					
					m_speed_limits[i] = 4095;
					m_current_limits[i] = 4095;
					
					m_position_encoder_bias[i] = 0;
					m_encoder_flag[i]          = 0;
			}

			m_connected = false;
	}

	DriverEcaArm::DriverEcaArm(int* current_limits, int* speed_limits){

			DriverEcaArm();

			for(int i=0; i<5; i++){
					m_speed_limits[i]   = current_limits[i];
					m_current_limits[i] = speed_limits[i];
			}

	}

	DriverEcaArm::~DriverEcaArm(){
			 disconnect();
	}

	int DriverEcaArm::connect(std::string device_port){
			int Ret;

			// Open serial port
			std::cout << "Opening port " << device_port << " ..." << std::endl;

			Ret=m_SerialConnection.Open(device_port.c_str(),115200);                										 // Open serial link at 115200 bauds
			if (Ret!=1) {                                                           										 // If an error occured...
					std::cout << "Error while opening port (hint: Permission problem?)" << std::endl;        // ... display a message ...
					return Ret;                                                         										 // ... quit the application
			}
			std::cout << "Serial port opened successfully!" << std::endl; 

			m_connected = true;

			return Ret;
	}

	void DriverEcaArm::disconnect(){
			// Close the connection
			m_SerialConnection.Close();
			std::cout << "Serial connection closed" << std::endl;

			m_connected = false;
	}


	bool DriverEcaArm::isConnected(){
			return m_connected;
	}

	int DriverEcaArm::sendMotionCommand(int* demand_type_, float* demand_){

		  float t_cycle = 0.05; //seconds	
			int   Ret=1;		

			uint8_t message[51];
			int     demand[5] = {0, 0, 0, 0, 0};
			int     demand_type[5];

			for(int i=0; i<5; i++){

				demand_type[i] = demand_type_[i];

				if(demand_type_[i] == 5 )
						demand_type[i] = 0; // position control forbidden (built-in position control doesn't work!)
				else{
						demand[i] = demandToBit(demand_[i]);
				}
				
			}

			
			getArmMessage(message, demand_type, demand, m_speed_limits, m_current_limits);

	                Ret = m_SerialConnection.Write(message, 51);
			if (Ret!=1) {           
					std::cout << "Error while writing on the port" << std::endl;        // ... display a message ...
					return Ret;                                                         // ... quit the application
			}
	   	        sleep(t_cycle);

			return Ret;
	}
 

	 int DriverEcaArm::getArmStatus() {

			uint8_t buffer[51];
			uint8_t offset_position = 5;
			uint8_t offset_speed    = 7;
			uint8_t offset_current  = 9;
			uint8_t offset_temperature  = 10;//11;
			uint8_t width = 9;

			int Ret;

			Ret = m_SerialConnection.Read(buffer,51,0); // int serialib::Read (void *Buffer,unsigned int MaxNbBytes,unsigned int TimeOut_ms)
			if (Ret != 1){

					if (Ret<0){
						std::cout << "Error while reading" << std::endl;
						return Ret;
					}
					else{
						std::cout << "No reply from arm.." << std::endl;
						return Ret;
					}
			 }
			 else
			 {
					
					for(int j=0; j<5; j++)
					{
						m_old_bits[j]              = m_armState.encoders[j];

						m_armState.encoders[j]    = buffer[offset_position     + j*width] + 256*buffer[offset_position + j*width + 1];
						m_armState.speeds[j]       = buffer[offset_speed        + j*width] + 256*buffer[offset_speed    + j*width + 1];
						m_armState.currents[j]     = buffer[offset_current      + j*width] + 256*buffer[offset_current  + j*width + 2];
						m_armState.temperatures[j] = buffer[offset_temperature  + j*width];
					}

			 }

			return 1;
	}  	

    int DriverEcaArm::receiveArmStatus() {

		if(getArmStatus()){

			setEncoderFlag(m_armState.encoders);

			for(int i=0; i<5; i++){      
        m_armState.positions[i] = bitNormalization(m_armState.encoders[i], i);
			};

			return 1;
		}

		return 0;
	}

	int DriverEcaArm::demandToBit( float value ){

			int demand = value;
			if (demand > 4095) //only for currents and velocity references
					demand = 4095;
	
			if (demand < 0)
					demand = 0;
			    //disp('Something went wrong');

			return demand;
			
	}			

	void  DriverEcaArm::getJointMessage(uint8_t* message, int demand_type, int demand, int speed_limit, int current_limit){


		for(int i=0; i<9; i++)
		{
			message[i]=0;
		}

		// demand type
		// 0 = hold	
		// 1 = voltage (%PWM), CW  [16bit, 0-100]
		// 2 = voltage (%PWM), CCW [16bit, 0-100]
		// 3 = speed (rpm), CW [12bit, 0-4095]
		// 4 = speed (rpm), CCW [12bit, 0-4095]
		// 5 = position [16bit]

		message[1] = demand_type;	

		if ( (1==demand_type) || (2==demand_type) )
		{
			int demand_ = std::max(0, std::min(100, (int)demand));

			int      var1          = round( std::max(0.0,(demand_*655.36-1)) );   //max(0,(demand_*(2^16/100)-1))

			int16_t binary_demand  = (int16_t) (var1);      //int16(max(0,(demand_*(2^16/100)-1)))
		
			message[3]=binary_demand & 0xff; // bitwise AND
			message[2]=(binary_demand >> 8); // bitwise right shift
		}

		if ( (3==demand_type) || (4==demand_type) )
		{
			int demand_ = std::max(0, std::min(4095, (int)demand));

			int16_t binary_demand  = (int16_t) (demand_);

			message[3]=binary_demand & 0xff; // bitwise AND
			message[2]=(binary_demand >> 8); // bitwise right shift

		}

		if ( 5==demand_type )
		{

				int demand_            = std::max(0, std::min(65536, (int)demand));

				int16_t binary_demand  = (int16_t) (demand_);     

				message[3]=binary_demand & 0xff; // bitwise AND
				message[2]=(binary_demand >> 8); // bitwise right shift

		}



		int speed_limit_ = std::max(0, std::min(4095, (int)speed_limit));
		int16_t binary_demand  = (int16_t) (speed_limit_);

		message[5]=binary_demand & 0xff; // bitwise AND
		message[4]=(binary_demand >> 8); // bitwise right shift



		int current_limit_ = std::max(0, std::min(4095, (int)current_limit));
		binary_demand      = (int16_t) (current_limit_);

		message[7]=binary_demand & 0xff; // bitwise AND
		message[6]=(binary_demand >> 8); // bitwise right shift

	}

	void  DriverEcaArm::getArmMessage(uint8_t* message, int* demand_type, int* demand, int* speed_limit, int* current_limit){
		 // output: message
		uint8_t header[4] = {231, 0, 0, 0};
		uint8_t footer    = 229;

		message[0]  = header[0];
		message[1]  = header[1];
		message[2]  = header[2];
		message[3]  = header[3];

		message[50] = footer;

		int header_length = 4;

		for(int i=0; i<5; i++)
		{
			uint8_t message_i[9];
			getJointMessage(message_i, demand_type[i], demand[i], speed_limit[i], current_limit[i]);

			message[header_length + (i*9) + 0] = message_i[0];
			message[header_length + (i*9) + 1] = message_i[1];
			message[header_length + (i*9) + 2] = message_i[2];
			message[header_length + (i*9) + 3] = message_i[3];
			message[header_length + (i*9) + 4] = message_i[4];
			message[header_length + (i*9) + 5] = message_i[5];
			message[header_length + (i*9) + 6] = message_i[6];
			message[header_length + (i*9) + 7] = message_i[7];
			message[header_length + (i*9) + 8] = message_i[8];
		}

		int sum_cmd = 0;

		for(int i=0; i<49; i++)
			sum_cmd = sum_cmd + message[i];

		uint8_t checksum = sum_cmd % 255 - floor(float(sum_cmd) / 255.0);

		message[49] = checksum;
	}		

	float DriverEcaArm::bitToRad(float input, bool reverse, int jointId){
			float k_elbow    = 569.88;  // [°/tick]
			float k_shoulder = 798.0;   // [°/tick]
			float k_slew     = 588.14;  // [°/tick]
			float k_wrist    = 8.89;    // [°/tick]

			float L					 = 78;				   	// [mm]
			float k_gripper  = (2*asin(62.5/L))/7000; // [rad/tick]

			float output=0.0;

			switch(jointId){

					case(1): //shoulder
			            if (reverse)
					                output = k_shoulder*input;
			        		else
			                		output = input/k_shoulder;
									break; 

        case(2): //slew
                if (reverse)
                        output = k_slew*input;
                else
                        output = input/k_slew;
                break;

        case(3): //elbow
                if (reverse)
                        output = k_elbow*input;
                else
                        output = input/k_elbow;
                break;

        case(4): //wrist
                if (reverse)
                        output = k_wrist*input;
                else
                        output = input/k_wrist;
                break;


        case(5): //gripper
                if (reverse)
                        ;//TO-DO
                else{
					float alpha = k_gripper*input;
                    output = 2*L*sin(alpha/2);   // [mm]
				}
				return output;
			}

		return deg2rad(output);
	}

	float DriverEcaArm::deg2rad(float input){
		return (input/180.0)*M_PI;
	}

    float DriverEcaArm::rad2deg(float input){
        return (input/M_PI)*180.0;
    }

	float DriverEcaArm::bitToRadSec(int input){
		float k = 1.0;
		return input*k;
	}

	float DriverEcaArm::bitToVolts(int input){
		float k = 0.21;
		return input*k;
	}

	float DriverEcaArm::bitToCelsius(int input){
		float k = 1.96;
		return input*k;
	}

	float DriverEcaArm::bitToMilliAmpere(int input){ //milliAmpere
		float k  = 0.094;
		float k2 = 0.2; 

		return input*k - k2;
	}

	void DriverEcaArm::getArmState_positions(float* positions){
		for(int i=0; i<5; i++)
			positions[i] = bitToRad(m_armState.positions[i], false, i+1);
	}

	void DriverEcaArm::getArmState_currents(float* currents){
		for(int i=0; i<5; i++)
            currents[i] = bitToMilliAmpere(m_armState.currents[i]);
	}

	void DriverEcaArm::getArmState_speeds(float* speeds){
    	for(int i=0; i<5; i++)
        	speeds[i] = bitToRadSec(m_armState.speeds[i]);
	}

	void DriverEcaArm::getArmState_temperatures(float* temperatures){
		for(int i=0; i<5; i++)
        	temperatures[i] = bitToCelsius(m_armState.temperatures[i]);
	}

    int32_t DriverEcaArm::bitNormalization(uint16_t bits, int num_el){
		int32_t bits_out;

		if(m_encoder_flag[num_el] < 0){

			bits_out = -(65536 - bits + m_position_encoder_bias[num_el]);
		
		}
		else{
			bits_out = bits - m_position_encoder_bias[num_el] + m_encoder_flag[num_el]*65536;
		}	

		//else flag is zero
		return bits_out;
	} 
              
	void  DriverEcaArm::setEncoderFlag(uint16_t* bits){

		int threshold = 500;

		for(int i=0; i<5; i++){

				if( bits[i]<threshold && m_old_bits[i]>(65536-threshold) )
						m_encoder_flag[i] = m_encoder_flag[i] + 1;
			
     	if( m_old_bits[i]<threshold && bits[i]>(65536-threshold) )
         	m_encoder_flag[i] = m_encoder_flag[i] - 1;

			//else flag is zero
		}
	}

	void DriverEcaArm::resetFlags(){
			for(int i=0; i<5; i++)
					m_encoder_flag[i] = 0;
	}

	int  DriverEcaArm::resetEncoders(){

		float demand[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
		int demand_type[5] = {0, 0, 0, 0, 0};

		sendMotionCommand(demand_type, demand);

		
		if(getArmStatus()){

				for(int i=0; i<5; i++){
					m_position_encoder_bias[i] = m_armState.encoders[i];		
					m_old_bits[i]              = m_armState.encoders[i]; //forse è inutile
				}

				resetFlags();

				return 1;
		}
		return 0;
    }

    int DriverEcaArm::homing(){

    	float demand[5] = {4000.0, 4000.0, 4000.0, 0.0, 4000.0};
		int demand_type[5] = {3, 0, 3, 0, 0}; 
		int homing[5] = {0, 1, 0, 1, 0}; // can be deleted

		float current_positions[5];
  		float current_currents[5];
		float spike_threshold[5] = {2005.0, 2005.0, 2005.0, 2005.0, 2005.0};

		int tries = 0;

		if(isConnected()){

			while(*std::max_element(demand_type, demand_type+5)  != 0 ){
			
				sendMotionCommand(demand_type, demand); //check if the writing to the arm is ok
				int reading_output = receiveArmStatus();

				if(reading_output>1){
					//error
					if (tries > 5){					      					
						std::cout << "ERROR: TimeOut reached. No data received!" << std::endl;
						return 1;
					}
					tries++;
				} else {
					if(reading_output!=1){
						//error
						return 1;
					} else {

						getArmState_currents(current_currents);
						getArmState_positions(current_positions);

						for(int i=0; i<5; i++){ // Check for current spikes
							if(current_currents[i]>spike_threshold[i]){
								homing[i]      = 1;
								demand_type[i] = 0;
							}
						}
		
						tries = 0;
					}
				}
				
	    	}
		} else {
			std::cout << "ERROR: arm not connected!!" << std::endl;
			return 1;
		}

		// Reset joints' encoders
		getArmState_positions(current_positions);
		if(!resetEncoders()){
			std::cout << "ERROR: encoders reset failed!!" << std::endl;
			return 1;
		}

		return 0;
    }

