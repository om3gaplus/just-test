#include "ROV.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
/*******************************************************************************
* Decoding Algorithem                                                          *
*   Parameter:    in_Data: Raw Data input                                      *
*   Return:       ROV Data Decoded                                             *
*******************************************************************************/
ROV_Data Decode(uint8_t *in_Data)
{
//	uint8_t CK_A_Check = 0, CK_B_Check = 0;
	/* 
	Variable Region
	*/
	ROV_Data decoded_data;
	union
	{
		uint8_t hex_data[4];
		float Data;
		int32_t Data32;
		int16_t Data16;
	}toConvertFloat;



//	Fletcher_8(in_Data, &CK_A_Check, &CK_B_Check);



	/*
	Decoding algorithem
	*/
	decoded_data.Mode = in_Data[2];
	decoded_data.Counter = in_Data[6] << 24 | in_Data[5] << 16 |in_Data[4] << 8 | in_Data[3]; // counter
	decoded_data.Data_Len = in_Data[8] << 8 | in_Data[7]; // Length
//	memcpy(toConvertFloat.hex_data, &in_Data[9], 4);
//	decoded_data.Lattitude = toConvertFloat.Data; // lat in_Data[12] << 24 | in_Data[11] << 16 |in_Data[10] << 8 | in_Data[9];
	decoded_data.Lattitude = (double)(in_Data[12] << 24 | in_Data[11] << 16 |in_Data[10] << 8 | in_Data[9])*0.0000001;
//	memcpy(toConvertFloat.hex_data, &in_Data[9], 4);
//	decoded_data.Lattitude = toConvertFloat.Data32*0.0000001;
//	memcpy(toConvertFloat.hex_data, &in_Data[13], 4);
//	decoded_data.Longitude = toConvertFloat.Data; // lon in_Data[16] << 24 | in_Data[15] << 16 |in_Data[14] << 8 | in_Data[13];
	decoded_data.Longitude = (double)(in_Data[16] << 24 | in_Data[15] << 16 |in_Data[14] << 8 | in_Data[13])*0.0000001;
//	memcpy(toConvertFloat.hex_data, &in_Data[13], 4);
//	decoded_data.Longitude = toConvertFloat.Data32;
	memcpy(toConvertFloat.hex_data, &in_Data[17], 4);
	decoded_data.Depth = toConvertFloat.Data; //
	memcpy(toConvertFloat.hex_data, &in_Data[21], 4);
	decoded_data.Velocity_N_Dir = toConvertFloat.Data; //
	memcpy(toConvertFloat.hex_data, &in_Data[25], 4);
	decoded_data.Velocity_E_Dir = toConvertFloat.Data; //
	memcpy(toConvertFloat.hex_data, &in_Data[29], 4);
	decoded_data.Velocity_D_Dir = toConvertFloat.Data; //
	memcpy(toConvertFloat.hex_data, &in_Data[33], 4);
	decoded_data.Roll = toConvertFloat.Data; //
	memcpy(toConvertFloat.hex_data, &in_Data[37], 4);
	decoded_data.Pitch = toConvertFloat.Data; //
	memcpy(toConvertFloat.hex_data, &in_Data[41], 4);
	decoded_data.Yaw = toConvertFloat.Data; //
	memcpy(toConvertFloat.hex_data, &in_Data[45], 4);
	decoded_data.W_X_Ax = toConvertFloat.Data; //
	memcpy(toConvertFloat.hex_data, &in_Data[49], 4);
	decoded_data.W_Y_Ax = toConvertFloat.Data; //
	memcpy(toConvertFloat.hex_data, &in_Data[53], 4);
	decoded_data.W_Z_Ax = toConvertFloat.Data; //
	memcpy(toConvertFloat.hex_data, &in_Data[57], 4);
	decoded_data.Lattitude_Tgt = toConvertFloat.Data; //
	memcpy(toConvertFloat.hex_data, &in_Data[61], 4);
	decoded_data.Longitude_Tgt = toConvertFloat.Data; //
	memcpy(toConvertFloat.hex_data, &in_Data[65], 4);
	decoded_data.Depth_Tgt = toConvertFloat.Data; //
	decoded_data.CK_A = in_Data[69];
	decoded_data.CK_B = in_Data[70];


	return decoded_data;
}
/*******************************************************************************
* Regulator                                                                    *
*   Parameter:    input_Params: Current State of ROV,  PID_K: PID Coefficients *
*   Return:       Controller_output                                            *
*******************************************************************************/
//Controller_output Controller(ROV_Data input_Params, PID_Param PID_K)
//{
//	Controller_output Return_data;
//	struct{
//		int f_u;
//		int tau_p;
//		int tau_r;
//		int tau_y;
//	}PID_FCN;
//	// rov vector is considered Lat == X = VN = WN = Roll = Roll_Tgt = Depth// Y == Lon = VE = WY = Pitch = Pitch_Tgt = Longitude_Tgt// Depth is Z = VD = Yaw == Yaw_TGT == Depth_Tgt
//	PID_FCN.f_u = (int)(- PID_K.K_PU*(input_Params.Lattitude - input_Params.Lattitude_Tgt) - PID_K.K_DU*(input_Params.Velocity_N_Dir));
//	PID_FCN.tau_r = (int)(- PID_K.K_PR*(input_Params.Roll - input_Params.Depth) - PID_K.K_DR*(input_Params.W_X_Ax));
//	PID_FCN.tau_p = (int)(- PID_K.K_PP*(input_Params.Pitch - input_Params.Longitude_Tgt) - PID_K.K_DP*(input_Params.W_Y_Ax));
//	PID_FCN.tau_y = (int)(- PID_K.K_PY*(input_Params.Yaw - input_Params.Depth_Tgt) - PID_K.K_DY*(input_Params.W_Z_Ax));
//	Return_data.PID_M[0] = 1500 + A_1u*PID_FCN.f_u + A_2u*PID_FCN.tau_r + A_3u*PID_FCN.tau_p + A_4u*PID_FCN.tau_y;
//	Return_data.PID_M[1] = 1500 - A_1R*PID_FCN.f_u + A_2R*PID_FCN.tau_r + A_3R*PID_FCN.tau_p - A_4R*PID_FCN.tau_y;
//	Return_data.PID_M[2] = 1500 - A_1P*PID_FCN.f_u + A_2P*PID_FCN.tau_r + A_3P*PID_FCN.tau_p + A_4P*PID_FCN.tau_y;
//	Return_data.PID_M[3] = 1500 + A_1Y*PID_FCN.f_u - A_2Y*PID_FCN.tau_r + A_3Y*PID_FCN.tau_p + A_4Y*PID_FCN.tau_y;

//	//
//	if(Return_data.PID_M[0] > 1900) Return_data.PID_M[0] = 1900;
//	if(Return_data.PID_M[0] < 1100) Return_data.PID_M[0] = 1100;
//	
//	if(Return_data.PID_M[1] > 1900) Return_data.PID_M[1] = 1900;
//	if(Return_data.PID_M[1] < 1100) Return_data.PID_M[1] = 1100;
//	
//	if(Return_data.PID_M[2] > 1900) Return_data.PID_M[2] = 1900;
//	if(Return_data.PID_M[2] < 1100) Return_data.PID_M[2] = 1100;
//	
//	if(Return_data.PID_M[3] > 1900) Return_data.PID_M[3] = 1900;
//	if(Return_data.PID_M[3] < 1100) Return_data.PID_M[3] = 1100;
//	//
//	
//	TIM2->CCR1 = Return_data.PID_M[0];
//	TIM2->CCR2 = Return_data.PID_M[1];
//	TIM2->CCR3 = Return_data.PID_M[2];
//	TIM2->CCR4 = Return_data.PID_M[3];
//	// Return Data
//	Return_data.f_u = PID_FCN.f_u;
//	Return_data.tau_p = PID_FCN.tau_p;
//	Return_data.tau_r = PID_FCN.tau_r;
//	Return_data.tau_y = PID_FCN.tau_y;
//	
//	return Return_data;
//}
/*******************************************************************************
* Controller
* Discription:
		Uses the full-state feedback controller in the previous section to deploy a 
		controller to move the robot from one state (X,Y,Z,Roll,Yaw,Pitch) to anot-
		-her state ensuring stablity.
*   Parameter:    input_Params: Current State of ROV,  PID_K: PID Coefficients *
*   Return:       Controller_output                                            *
*******************************************************************************/
depth_Controller_output depthControl(ROV_Data ROV_current_state)
{
	depth_Controller_output Return_data;
	//////////////// K_P assignment /////////////////////////
	Return_data.k_p = ROV_current_state.W_Z_Ax; // k_p
	///////////// controller ///////////////////////////////
	Return_data.tau = Return_data.k_p*(ROV_current_state.Depth - ROV_current_state.Depth_Tgt); // dive is < 0
	Return_data.PID_M[0] = 1500 - Return_data.tau;
	Return_data.PID_M[3] = 1500 + Return_data.tau;
	///////////// Saturation After tau////////////////////////////////////
	if(Return_data.PID_M[0] > PWM_Max) Return_data.PID_M[1] = PWM_Max;
	if(Return_data.PID_M[0] < PWM_Min) Return_data.PID_M[1] = PWM_Min;
	
	if(Return_data.PID_M[3] > PWM_Max) Return_data.PID_M[2] = PWM_Max; // PWM_Max
	if(Return_data.PID_M[3] < PWM_Min) Return_data.PID_M[2] = PWM_Min; // PWM_Min
	
	////////////////// keepmotion ///////////////////////////////
	Return_data.PID_M[0] = Return_data.PID_M[0] + keepMotion;
	Return_data.PID_M[3] = Return_data.PID_M[3] + keepMotion;
	///////////// action on the thrusters ///////////////////////
	TIM4->CCR1 = Return_data.PID_M[0]; // up
	TIM4->CCR2 = ROV_current_state.Velocity_N_Dir; // left
	TIM4->CCR3 = ROV_current_state.Velocity_E_Dir; // right
	TIM4->CCR4 = Return_data.PID_M[3]; // down
	return Return_data;
}
/*******************************************************************************
* Controller
* Discription:
		Uses the full-state feedback controller in the previous section to deploy a 
		controller to move the robot from one state (X,Y,Z,Roll,Yaw,Pitch) to anot-
		-her state ensuring stablity.
*   Parameter:    input_Params: Current State of ROV,  PID_K: PID Coefficients *
*   Return:       Controller_output                                            *
*******************************************************************************/
Controller_output GPS_Controller(ROV_Data ROV_current_state, ROV_Data Target, ROV_Data Previous_State, ROV_Data origin_state, Saturation Satur)
{
	/* in the case that we get the data of GPS from THEIR boar (this case) we 
	have the params as follows:
	- ROV_current_state = Decoded_Data_ROV
	- Target = Decoded_Data_Raspberry
	- Previous_State = Filterd Data of ROV
	- origin_state = Filterd Data of ROV
	*/
	// longitude = X, latitude = Y
	Controller_output Return_data;
	struct{
		float f_u;
		float tau;
		float k_pf;
		float k_ptau;
		float target_distance;
		float previous_distance;
		float path_angle_error;
		float target_angle;
		float Step_angle;
	}Controller_Params;
	//////////////////// Conversion ///////////////////////////////
//	ROV_current_state.Lattitude = ROV_current_state.Lattitude*Conv;
//	ROV_current_state.Longitude = ROV_current_state.Longitude*Conv;
//	
//	Previous_State.Lattitude = Previous_State.Lattitude*Conv;
//	Previous_State.Longitude = Previous_State.Longitude*Conv;
//	
//	origin_state.Lattitude = origin_state.Lattitude*Conv;
//	origin_state.Longitude = origin_state.Longitude*Conv;
	//////////////////// coeffs ///////////////////////////////////
	Controller_Params.k_ptau = Target.W_X_Ax;
	Controller_Params.k_pf = Target.W_Y_Ax;
	//////////////////// Data to return ///////////////////////////
	Return_data.k_pf = Controller_Params.k_pf;
	Return_data.k_ptau = Controller_Params.k_ptau;
	//////////////////////Converting Data//////////////////////////////////////
//	Target.Lattitude_Tgt = Target.Lattitude_Tgt*Conversion;
	ROV_current_state.Lattitude = (ROV_current_state.Lattitude - origin_state.Lattitude)*Conversion;
//	Target.Longitude_Tgt = Target.Longitude_Tgt*Conversion;
	ROV_current_state.Longitude = (ROV_current_state.Longitude - origin_state.Longitude)*Conversion;
	Previous_State.Lattitude = (Previous_State.Lattitude - origin_state.Lattitude)*Conversion;
	Previous_State.Longitude = (Previous_State.Longitude - origin_state.Longitude)*Conversion;

	Target.Lattitude_Tgt = (Target.Lattitude_Tgt - origin_state.Lattitude)*Conversion;// new
	Target.Longitude_Tgt = (Target.Longitude_Tgt - origin_state.Longitude)*Conversion;// new
	//////////////////////////taking Origin into Account//////////////////////////////////
//	Target.Lattitude_Tgt = Target.Lattitude_Tgt + origin_state.Lattitude*Conversion;
//	Target.Longitude_Tgt = Target.Longitude_Tgt + origin_state.Longitude*Conversion;
	////////////////// calculating parameters ///////////////////
	Controller_Params.target_distance = powf((powf((Target.Lattitude_Tgt - ROV_current_state.Lattitude),2.0)+powf((Target.Longitude_Tgt - ROV_current_state.Longitude),2.0)),.5);
	Controller_Params.previous_distance = powf((powf((ROV_current_state.Lattitude - Previous_State.Lattitude),2.0)+powf((ROV_current_state.Longitude - Previous_State.Longitude),2.0)),.5);
	if(Controller_Params.previous_distance != 0 && Controller_Params.target_distance != 0)
	{
		///////////////////////// calculating angles /////////////////
		if(ROV_current_state.Lattitude-Previous_State.Lattitude>=0)
		{
			Controller_Params.Step_angle = acos((ROV_current_state.Longitude-Previous_State.Longitude)/Controller_Params.previous_distance);
		}
		else if(ROV_current_state.Lattitude-Previous_State.Lattitude<0)
		{
			Controller_Params.Step_angle = -acos((ROV_current_state.Longitude-Previous_State.Longitude)/Controller_Params.previous_distance);
		}
		if(Target.Lattitude_Tgt-ROV_current_state.Lattitude>=0)
		{
			Controller_Params.target_angle = acos((Target.Longitude_Tgt-ROV_current_state.Longitude)/Controller_Params.target_distance);
		}
		else if(Target.Lattitude_Tgt-ROV_current_state.Lattitude<0)
		{
			Controller_Params.target_angle = -acos((Target.Longitude_Tgt-ROV_current_state.Longitude)/Controller_Params.target_distance);
		}
		Return_data.step_angle = Controller_Params.Step_angle; // returning step angle
		Controller_Params.path_angle_error = fmod((Controller_Params.Step_angle - Controller_Params.target_angle+3.1415926),2*3.1415926)-3.1415926;

		// Ezafe Ali.
		if (fabs(Controller_Params.path_angle_error-2*3.1415926)<fabs(Controller_Params.path_angle_error)){
			Controller_Params.path_angle_error = Controller_Params.path_angle_error - 2*3.1415926;
		}
		if (fabs(Controller_Params.path_angle_error+2*3.1415926)<fabs(Controller_Params.path_angle_error)){
			Controller_Params.path_angle_error = Controller_Params.path_angle_error + 2*3.1415926;
		}
		//
		////////////////// calculating Tau and F ////////////////////
		if(cos(Controller_Params.path_angle_error)>=0)
		{
			Controller_Params.f_u = Controller_Params.k_pf*cos(Controller_Params.path_angle_error)*Controller_Params.target_distance;
		}
		else
		{
			Controller_Params.f_u = 0;
		}
		Controller_Params.tau = -Controller_Params.k_ptau*Controller_Params.path_angle_error;
	}
	else
	{
		Controller_Params.path_angle_error = 0;
		Controller_Params.f_u = 1650*2;
		Controller_Params.tau = 0;
	}
	////////////////// Data to return ///////////////////////////////
	Return_data.previous_distance = Controller_Params.previous_distance;
	Return_data.target_distance = Controller_Params.target_distance;
	Return_data.f_u = Controller_Params.f_u;
	Return_data.tau_p = Controller_Params.tau;
	Return_data.path_angle_Error = Controller_Params.path_angle_error;
	// Return_data.step_angle = Controller_Params.Step_angle; // due to false quantity it is moved
	Return_data.X = ROV_current_state.Longitude;
	Return_data.Y = ROV_current_state.Lattitude;
	////////////// calculating motor thrust for f_u /////////////////////
	Return_data.PID_M[1] = PWM_Off + A_N2PWM*Controller_Params.f_u/2; // Left motor
	Return_data.PID_M[2] = PWM_Off + A_N2PWM*Controller_Params.f_u/2; // Right motor
	Return_data.PID_M[3] = PWM_Off + A_N2PWM*Controller_Params.f_u/2; // Bottom motor
	
	///////////// Saturation Befor tau////////////////////////////////////
	if(Return_data.PID_M[1] > Satur.LeftRightUp) Return_data.PID_M[1] = Satur.LeftRightUp;
	if(Return_data.PID_M[1] < PWM_Off) Return_data.PID_M[1] = PWM_Off;
	
	if(Return_data.PID_M[2] > Satur.LeftRightUp) Return_data.PID_M[2] = Satur.LeftRightUp;
	if(Return_data.PID_M[2] < PWM_Off) Return_data.PID_M[2] = PWM_Off; // PWM_Off
	
	if(Return_data.PID_M[3] > Satur.BottomUp) Return_data.PID_M[3] = Satur.BottomUp;
	if(Return_data.PID_M[3] < PWM_Off) Return_data.PID_M[3] = PWM_Off; // PWM_Off
	////////////// calculating motor thrust for taw /////////////////////
	Return_data.PID_M[1] = Return_data.PID_M[1] - A_N2PWM*Controller_Params.tau/0.35; // Left motor
	Return_data.PID_M[2] = Return_data.PID_M[2] + A_N2PWM*Controller_Params.tau/0.35; // Right motor
	// Prep Motor//
	Return_data.PID_M[0] = PWM_Off + A_N2PWM*Controller_Params.tau;
	///////////// Saturation After tau////////////////////////////////////
	if(Return_data.PID_M[0] > PWM_Max) Return_data.PID_M[0] = PWM_Max_prep;// new
	if(Return_data.PID_M[0] < PWM_Min) Return_data.PID_M[0] = PWM_Min_prep;// new

	if(Return_data.PID_M[1] > Satur.LeftRightUp) Return_data.PID_M[1] = Satur.LeftRightUp;
	if(Return_data.PID_M[1] < Satur.LeftRightLow) Return_data.PID_M[1] = Satur.LeftRightLow;
	
	if(Return_data.PID_M[2] > Satur.LeftRightUp) Return_data.PID_M[2] = Satur.LeftRightUp; // PWM_Max
	if(Return_data.PID_M[2] < Satur.LeftRightLow) Return_data.PID_M[2] = Satur.LeftRightLow; // PWM_Min
	
	///////////////////////// keep motion ///////////////////////
//	Return_data.PID_M[0] = PWM_Off;
	Return_data.PID_M[1] = Return_data.PID_M[1]; // left
	Return_data.PID_M[2] = Return_data.PID_M[2]; // Right
	Return_data.PID_M[3] = Return_data.PID_M[3] + keepMotion; // Bottom
	///////////// action on the thrusters ///////////////////////
	TIM4->CCR1 = (int)Return_data.PID_M[0]; // Rotation Motor
	TIM4->CCR2 = (int)Return_data.PID_M[1]; // left
	TIM4->CCR3 = (int)Return_data.PID_M[2]; // right
	TIM4->CCR4 = (int)Return_data.PID_M[3]; // down
	return Return_data;
}

/*******************************************************************************
* Controller
* Discription:
		Uses the full-state feedback controller in the previous section to deploy a 
		controller to move the robot from one state (X,Y,Z,Roll,Yaw,Pitch) to anot-
		-her state ensuring stablity.
*   Parameter:    input_Params: Current State of ROV,  PID_K: PID Coefficients *
*   Return:       Controller_output                                            *
*******************************************************************************/
double Mean(double *array)
{
	double avg = 0;
	for(int i=0; i<Buff_Size; i++)
	{
		avg += array[i];
	}
	avg /= (double)Buff_Size;
	return avg;
}
/*******************************************************************************
* Controller
* Discription:
		Uses the full-state feedback controller in the previous section to deploy a 
		controller to move the robot from one state (X,Y,Z,Roll,Yaw,Pitch) to anot-
		-her state ensuring stablity.
*   Parameter:    input_Params: Current State of ROV,  PID_K: PID Coefficients *
*   Return:       Controller_output                                            *
*******************************************************************************/
double *shiftForward(double *array, double ar)
{
	for(int i=0; i<Buff_Size-1; i++)
	{
		array[i] = array[i+1];
	}
	array[Buff_Size-1] = ar;
	return array;
}

/* Explanation
- 4 modes are available:
	- Depth control = 1
	- Manual Control = 4
	- P2P Control = 3
	- Emergency Stop = 9
- the following parameters are manipulated in each of the modes:
	- Roll = Motor_1 (From raspberry) Mode 4
	- Velocity_N_Dir = Motor_2 (From raspberry) Mode 4
	- Velocity_E_Dir = Motor_3 (From raspberry) Mode 4
	- Velocity_D_Dir = Motor_4 (From raspberry) Mode 4
	- W_X_Ax = K_PTaw (From raspberry) Mode 3
	- W_Y_Ax = K_PF (From raspberry) Mode 3
	- W_Y_Ax = K_Ptaw Depth Control (From raspberry) Mode 1
*/
void Fletcher_8(uint8_t *Buffer, uint8_t *CK_A, uint8_t *CK_B)
{
	uint8_t CK_[2] = {0, 0};
	 for(int I = 0; I<72; I++)
	 {
		 CK_[0] = CK_[0] + Buffer[I];
		 CK_[1] = CK_[1] + CK_[0];
	 }
	 *CK_A = CK_[0];
	 *CK_B = CK_[1];
}


/*******************************************************************************
* ROV Lib Developed in Biomechanic & Moving System Lab                                        *
*   1 Jan 2021                                                                  
*******************************************************************************/
