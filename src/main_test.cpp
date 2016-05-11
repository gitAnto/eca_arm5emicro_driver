#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <math.h>

#include "lib/serialib.h"
#include "driver/eca_arm5emicro_driver_arm.h"

int main(int argc, char** argv){

	ros::init(argc, argv, "test_driver");
	ros::NodeHandle nh("~");

	std::string device_port;

	nh.param<std::string>("device_port", device_port, "/dev/ttyUSB0");
	
	DriverEcaArm eca_driver;

//	eca_driver.connect(device_port);

	float shoulder_ref, slew_ref, elbow_ref;
	float demand[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
	int demand_type[5] = {5, 5, 5, 0, 0};
	int sp_ref_flag=0, cw=0;

	std::cout << "speed references? [0/1]: ";
	std::cin >> sp_ref_flag;

	if(sp_ref_flag){
		std::cout << " --- Speed Control Mode ---" << std::endl;

		std::cout << "ref speed for the shoulder: ";
		std::cin >> shoulder_ref;
		std::cout << "ref speed for the slew: ";
		std::cin >> slew_ref;
		std::cout << "ref speed for the elbow: ";
		std::cin >> elbow_ref;

		std::cout << "clockwise? [0/1]: ";
		std::cin >> cw;

		if(cw){
			demand_type[0] = 3;
			demand_type[1] = 3;
			demand_type[2] = 3;
		}
		else{
			demand_type[0] = 4;
			demand_type[1] = 4;
			demand_type[2] = 4;
		}
	}
	else{

		std::cout << " --- Position Control Mode ---" << std::endl;
		
		std::cout << "ref angle for the shoulder [°]: ";
		std::cin >> shoulder_ref;
		std::cout << "ref angle for the slew [°]: ";
		std::cin >> slew_ref;
		std::cout << "ref angle for the elbow [°]: ";
		std::cin >> elbow_ref;
	}

	float current_positions[5];
	float current_speeds[5];
	float current_currents[5];
	float current_temperatures[5];

	demand[0] = (shoulder_ref/180.0)*M_PI;
	demand[1] = (slew_ref/180.0)*M_PI;
	demand[2] = (elbow_ref/180.0)*M_PI;

	int tries = 0;
	float angle_tolerance = 0.5;

	eca_driver.connect(device_port);

	for(int i=0; i<1000; i++)
	{

		eca_driver.sendMotionCommand(demand_type, demand);


			int reading_output = eca_driver.receiveArmStatus();

			if(reading_output>1){
				//error
		  	if (tries > 5)
				{    			
					std::cout << "TimeOut reached. No data received!" << std::endl;
					return 1;
				}

				tries++;
			}
			else{

				if(reading_output!=1){
					//error
					return 1;
				}
				else{

					eca_driver.getArmState_positions(current_positions);
					std::cout << "current positions --> shoulder: " << current_positions[0] << " slew: " << current_positions[1] << " elbow: " << current_positions[2] << std::endl; 
		
					eca_driver.getArmState_speeds(current_speeds);
					std::cout << "current speeds --> shoulder: " << current_speeds[0] << " slew: " << current_speeds[1] << " elbow: " << current_speeds[2] << std::endl; 
		
					eca_driver.getArmState_currents(current_currents);
					std::cout << "current currents --> shoulder: " << current_currents[0] << " slew: " << current_currents[1] << " elbow: " << current_currents[2] << std::endl; 
		
					eca_driver.getArmState_temperatures(current_temperatures);
					std::cout << "current temperatures --> shoulder: " << current_temperatures[0] << " slew: " << current_temperatures[1] << " elbow: " << current_temperatures[2] << "\n" << std::endl; 
		

					float alpha_m = current_positions[0];
					float gamma_m = current_positions[1];
					float beta_m  = current_positions[2];
					float alpha_error = shoulder_ref - alpha_m; // 180 / 3.14159265
					if( std::abs(alpha_error) < angle_tolerance)
					{
						demand_type[0] = 0;
						demand[0] = 0;
					}
		
					float gamma_error = slew_ref - gamma_m;
					if( std::abs(gamma_error) < angle_tolerance)
					{
						demand_type[1] = 0;
						demand[1] = 0;
					}
		
					float beta_error = elbow_ref - beta_m;
					if( std::abs(beta_error) < angle_tolerance)
					{
					    demand_type[2] = 0;
					    demand[2] = 0;
					}

					if (*std::max_element(demand_type, demand_type+5)  == 0 )
					{
						printf("alpha: %f\tgamma: %f\tbeta: %f\n",alpha_m,gamma_m,beta_m);
						printf("demand_type = %d %d %d %d %d\n",demand_type[0], demand_type[1], demand_type[2], demand_type[3], demand_type[4]);
						printf("min of demand_type = %d", *std::min_element(demand_type, demand_type+5));
						printf("Completed..\n");
						break;
					}

					tries = 0;
			}
		}
	}


	eca_driver.disconnect();

	return 0;

}
