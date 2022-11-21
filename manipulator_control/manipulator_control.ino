/***********************************************************************************************************************
* Department of Robotics Hoseo University
* LAB509 Senier Project
* Project Name : Concave AI Manipulator System
* Project Period : 2022.01~2022.10
* < Lee Seong Yong > : Responsible for Making the Manipulator Controller
***********************************************************************************************************************/

/**********************************************************************************************************************
* Dynamixel : Extended Position Control Mode(OpenCR Board Use)
* 1) Joint1 : Dynamixel XH540-W270-T
* 2) Joint2 : Dynamixel XM540-W270-T
* 3) Joint3 : Dynamixel XM430-W350-T
* 4) End_Effector_A : Dynamixel 2XL430-W250-T(A)
* 5) End_Effector_B : Dynamixel 2XL430-W250-T(B)
***********************************************************************************************************************/
#include <Arduino.h>
#include <actuator.h>
#include <Dynamixel2Arduino.h>
#include <math.h>

HardwareTimer Timer(TIMER_CH1);

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

// Dynamixel Motor BAUDRATE
#define BAUDRATE 57600

// Dynamixel XH540-W270-T : Joint(1)
#define J1_ID 1
// Dynamixel XM540-W270-T : Joint(2)
#define J2_ID 2
// Dynamixel XH540-W270-T : Joint(3)
#define J3_ID 3
// Dynamixel 2XL430-W250-T : End-Effector Ch(A)-Pitch Orientation, Ch(B)-Yaw Orientation
#define EA_ID 4
#define EB_ID 5

const uint8_t DXL_ID[5] = {J1_ID, J2_ID, J3_ID, EA_ID, EB_ID};
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino DXL(DXL_SERIAL, DXL_DIR_PIN);

// This namespace is required to use Control table item names
using namespace ControlTableItem;

// Dynamixel Postion Estimation Parameter
int Dynamixel_Position[5] = {0,0,0,0,0};
int Position_A[5] = {0,0,0,0,0};
float Position_B[5] = {0.0,0.0,0.0,0.0,0.0};
int Position_Function[5], Position_Sum = 0; // Position_Sum = All Dynamixel Position Error Sum
// Dynamixel Velocity
int Temp_V, Position_V[5], Encoder_E[5], Save_Encoder_E[5], Min_Encoder_E, Dynamixel_V[5];
/**********************************************************************************************************************
* 5DOFs Manipulator Angle Parameter
* radius : radius from Manipulator base to target point
* theta : Default Y Coordinate(theta = pi/2(radian) = 90(degree))
***********************************************************************************************************************/
double j_theta1, j_theta2, j_theta3, e_thetap;
const double e_thetay = 180.0;  //..Constant Yaw Orientation(not use 4DoFs)
double D_theta[5];              //.. = {j_theta1, j_theta2, j_theta3, e_thetap, e_thetay};
double radius, theta;           // radius, theta of coordinate(xv, yv)
/**********************************************************************************************************************
* 5DOFs Manipulator Reference Position Coordinate
***********************************************************************************************************************/
#define xrv 10
#define yrv 19
/**********************************************************************************************************************
* 5DOFs Manipulator Path Planning Parameter
***********************************************************************************************************************/
int N_resolution = 0;
int V_resolution = 0, S_resolution = 0;
double f_radius, f_theta;       // radius, theta of final coordinate(xrv, yrv)
double p_radius, p_theta;       // radius, theta of path planning coordinate
double r_radius, r_theta;       // radius, theta of reference coordinate(xrv, yrv)
/**********************************************************************************************************************
* 5DOFs Manipulator D-H Parameter
* L1,L2,L3 : Link Length[cm]
***********************************************************************************************************************/
double L1 = 23.43, L2 = 32.6, L3 = 28.9; // L1 = 24.13, L2 = 32.5, L3 = 23.8 (Link3 = 23.8 + 5)
double init_x = 0.0, init_y = 0.0;
double x, y, z, xd, yd, zd, rrx, rry, rrz;
double M_d, M_D, M_Ei;
double alpha, beta, r;
/**********************************************************************************************************************
* Square Plate Coordinate Data and 5DOFs Reference Axis Position Data
* base_radius : Manipulator Base Radius
* base_distance : Distance between Board and Manipulator start point
***********************************************************************************************************************/
double base_radius = 9.5, base_distance = 7.4, board_blank = 1.65; // base_distance = 7.4(standard) board_blank = 1.5 + 2.05[cm]
double origin = base_radius+base_distance+board_blank;
// ejector_height : 이젝터 높이, margin_height : 경로 이동을 위한 여유 높이, concave_position : Manipulator의 Base 위치로부터 오목돌의 기준 위치
// end-effector ejector height : 12.15cm
// 오목돌 높이 : 6.5~7.5mm : 0.65~0.75cm -> 평균 : 0.8cm, place_vertical_height : 오목돌 지정 좌표에서 수직 이동을 위한 변수
double stone_height = 0.75, place_vertical_height = 0, save_place_vertical_height = 0; // stone height : 0.7
double board_height = 1.3, board_square = 2.05, ejector_height = 12.15 + board_height + stone_height, vertical_height = ejector_height + 6, dispenser_height = ejector_height + 3, path_height = 0; // end-effector_ejector : 12.15, board_square : 2.05
// (X),(Y),(Z) Origin_Coordinate 대시 상태에서의 초기 기준 위치 선정(initial Position), init_case : Manipulator의 각도 보상기 적용 여부
// X_initial_Position = origin
double X_initial_Position = 10.7 + base_radius, Y_initial_Position = 0.0, Z_initial_Position = vertical_height, init_case = 0;
double X_stone_Position = 10.7 + base_radius, Y_stone_Position = 0.0, Z_stone_Position = vertical_height;

double J1_COMP[19][19] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {-0.5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0.5,0,0,0,0,0},
                          {0,0,0,-0.2,0,0,0,0,0,0,0,0.5,0,0,0.5,0,0,0,0},
                          {0,-0.5,0,0,0,0,0,0,0,0,0,0,0,0.5,0,0,0,0.5,0},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.5,0.5,0,0,0.5},
                          {0,0,0,0,0,0,0,0,0,0,0,0.5,0.5,0.5,0,0.5,0,0,0.5},
                          {0,0,0,0,0,0,0,0.5,0,0,0.5,0,0,0,0.5,0.5,0.5,0,0},
                          {0,0,0,0,0,0,0.5,0,0,0,0,0,0.5,0,0.3,0.3,0,0.3,0.3},
                          {0,0,0,0.5,0,0,0,0.5,0,0,0,0,0.5,0,0.5,0,0,0,0.5},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0.5,1,1},
                          {0,0,0,0,0,0,0,0.5,0,0,0,0.5,0,0,0,0,0,1,1},
                          {0,0,0,0,0.5,0,0,0,0,0,0,1,0,1,1,0.5,0,1,1},
                          {0,0,0.5,0.5,0,0,0,0,0,0,0,1,0,0,1,1,1,1,1},
                          {-0.5,0.5,0,0,-0.5,0.5,0.5,0.5,0.5,0,0,0.5,0.5,0.5,1,1,1,1,0.5},
                          {0,0,0,0.5,0.5,0,0,0.5,0,0,0,1,1,1,1,1,1,1,1}};


double J2_COMP[19][19] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {0,0,0,0,0,0.4,0,0,0,0,0,0,0,0.4,0,0,0,0,0},
                          {0,0,0,0,0.2,0,0,0.2,0,0,0,0.4,0,0,0.4,0,0,0,0.4},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {0,0,0,0,0.2,0,0,0,0.2,0.2,0.2,0,0,0,0.2,0,0,0,0},
                          {0,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2,0},
                          {0,0,0,0,0.2,0,0,0,0,0,0,0,0,0,-0.2,0,0,0,0},
                          {0,-0.2,0,0,0,0,-0.4,-0.2,0,0,0,0,0,0,0,0,0,0,0},
                          {0,-0.2,-0.2,-0.2,-0.2,-0.2,-0.2,-0.2,-0.2,0,-0.2,-0.2,0,0,0,0.2,0,-0.2,-0.2},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.5,-0.2,0,-0.3},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.5,-0.2,0},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

double J3_COMP[19][19] = {{3,3,4,4,4,4,4.5,3,3,3,3,3,4,3.5,4,4,5,5,5},
                          {3,3,4,4,4,2,3,3,2,3,2,4,3,3,4,4,4,3,3},
                          {1.5,2.5,2.5,2.5,3,3,3.5,4.5,2.5,2.5,2.5,4,4,3,3,3,3,3,3},
                          {2.5,2.5,2.5,3.5,3,4.5,3,3,3.5,3.5,3.5,3.5,2.5,5,3.5,3.5,3.5,3,2.5},
                          {3.5,3.5,3.5,2.5,3.5,3,3.5,4.5,2,2,2,4.5,3.5,3.5,4.5,2.5,3.5,4.5,4},
                          {2,2.5,2,3,3,2.5,3.5,2,2.5,2.5,2.5,2.5,3.5,3,3,3,2,2.5,2.5},
                          {2,2.5,2.5,2.5,2.5,2.5,3.5,2,2,2,2,2,3.5,2.5,2.5,3,2.5,2.5,3},
                          {2,2,3,3,3.5,2.5,2.5,3,3.5,3.5,3.5,3.5,3.5,3,4,3.5,3.5,3,3},
                          {2,4,2,1.5,2.5,3.5,1.5,1.5,3,1.5,1.5,2,2,3.5,3,1.5,2,3.5,2.5},
                          {3,2.5,1.5,1.5,3,1.5,2,3.5,3.5,1,3.5,3,2.5,1.5,1.5,2,1.5,2,4},
                          {1.5,0,1.5,4,1,2,1,1,1.5,0.5,1.5,3.5,3,2,2,3.5,2,3.5,2.5},
                          {1.5,1.5,1,1,1,1.5,1.5,1.5,1,1,1,1,2.5,1.5,1.5,2.5,1.5,1.5,1.5},
                          {1,0,1,0,1,0.5,1,1,1,1,1.5,1.5,1.5,1.5,1.5,1.5,1,1,1.5},
                          {0,0.5,0,0,0.5,1,0,0,0,0.5,0,0,0,0.5,0.5,0,0,1,1},
                          {0,0,0.5,0,0.5,0,0,0,0,0.5,0,0,0,0,0.5,0,1,2,0.5},
                          {0.5,0,0.5,0.2,0.5,0,0,0,0,0,0,0,0,0,0.5,0,0,0.5,2},
                          {0,0,0,0,0,0,0,0,0,0,0,0,0,0.5,0,0.5,0,0,0.5},
                          {0,0,0,0,0,0.5,0,0,0,-0.2,0,0,0,1,0,0,0,0,0},
                          {0.5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.5}};

double EP_COMP[19][19] = {{-3,-2,-3,-5,-5,-5,-6,-3,-2,-2,-2,-4,-5,-3,-5,-5,-5,-4.5,-4.5},
                          {-3,-3,-4.5,-6,-4.5,-2,-4,-4,-2,-2,-2,-3,-4,-4,-4.5,-4,-4,-2,-3},
                          {-1,-2,-3,-3,-4,-4,-5,-7,-4,-1,-4,-7,-5,-3,-4,-3,-3,-3,-3},
                          {-2,-3,-1.5,-5,-3.5,-5.5,-2.5,-4,-5,-5.5,-4.5,-4,-2,-6,-4,-5,-2,-2.5,-1},
                          {-4,-5,-5,-3,-5,-4,-5,-7,-1,-1,-0.5,-7.5,-5,-3.0,-6,-1.5,-4,-5.0,-5.5},
                          {-2,-2,0,-4,-4,-3.5,-5,-2,-3,-4,-2.5,-2,-5.2,-3,-3,-4,0,-2,-1},
                          {-3,-3,-2,-3,-2,-3,-4,-2,-2,-1,-3,-2,-4,-2,-2,-3,-1,-1.5,-2},
                          {-4,-1,-6,-4.5,-6,-4,-4,-5,-6.5,-7,-6.5,-5,-6,-3,-6,-4,-5,-2.5,-3.5},
                          {-3,-6,-3,0,-3,-7,-1.5,-1,-3,-2,-2,-3,-1,-6,-3.5,0,-3,-5,-2},
                          {-5,-3,0,-2,-6,-1,-5,-5,-7,0,-7,-6,-4,-1,-1,-2,0,-2.5,-7},
                          {-2,3,-3,-8,0,-3,-1.5,-0.5,-1,0.5,-1,-7,-5,-3,-3,-7,-2,-7,-4},
                          {-4,-1,-1,0,-2,-2,-3,-2,0,-1,0,-2.5,-5,-2,-1,-6,-2,-1,-3},
                          {-1,1,-2,1.5,1,-1,0,-1,-2,-1.5,-1,-2,-3,-4,-2,-2,-2,-2,-1},
                          {1,1,0.2,1,0,-3,1,1,0,0,0,1,0,0,0,1,1,-2.5,0},
                          {3,1,0,1,-1,0,0,1,-1,-1,-1,2,0,1,0,1,-2,-2,1},
                          {0,2,-2,1.5,0,0,0,1,-1,1,-1,0.5,0,1,0,1.5,1,-2,-4},
                          {2,0,-1,2,0,1,0,0,-1,1,-1,0,0,-2,-0.5,-1,0,0,0},
                          {2,-0.5,1,0,0,-2,1,1,-2,0,-2,1,1,-1.5,0,0,2,-0.5,2},
                          {-1,1,-2,-1,-1,-2,-3,-2,-2,-1,-2,-2,-3,-2,-1,-1,-1,2,-1}};

double EY_COMP[19][19] = {{0,0,0,0,0,0,0,1,1,2,2,4,0,-1,-1,0,0,0,0},
                          {1,2,2,3,3,-1,0,0,1,2,3,-1,0,0,0,3,2,2,3},
                          {-2,-2,1,0,0,1,3,0,0,2,-2,0,0,1,-2.5,-2,0,0,0},
                          {0,1,1,3,-2.5,-2,2,4,-1,2,-2,0,2,-3,0,2,3,-3,-2},
                          {0,2,3,-1,1,3,-1,2,-1.5,1,-2,1,-3,-1,2,-2,0.5,2,-3},
                          {2,3.5,-1.5,0,3,0,-3,1,-2,1,-1,2,1,2,0,2,-2,1,2},
                          {1,3,-1,-4,-1,3,1,-2,-2.5,1,-1,3,2,-2,3,0,-3,-1,2},
                          {0,-4,0,-2.5,1.5,-1,-2,-3,2,1,0,-1,-2,1,0,-1.3,2,-1,2},
                          {0,3,0,-2,-3,2,2,2,1,1,1,-1,0,-2,2,3,1,-1,1},
                          {-2,2,0,-1,-2,-2,-1.5,0,0,0,2,2,2,2,-3,2,0,-1,2},
                          {1,-2,-1,0,-2,-1,0,2,-1,0,2,-2,0,0,1,2.5,0,0,-2},
                          {0,-2,-2,-2,-1,1,2,0,-2,-1,-3,0,3,0,1,2,-2,1,0},
                          {1,1,1,1,-2,1,0,1,1,0,-2,-2,0,-1,2,-1,0,-1,-1},
                          {-2,-2,-2,0,-2,0,0,0,0,0,0,0,0,0,2,-1,0,2,2},
                          {-2,-2,-1,-2,-1,0,0,1,-1,0,-2,2,-1,0,0,-2,-2,1,2},
                          {1,-2,0,-2,1,0,1,-2,-3,0,-1,0,-1.5,0,0,-1,-2,0,2},
                          {0,1,2,0,-2,0,-2,-2,-2,-1,0,0,0,-2,-1,0,0,2,3},
                          {-3,1,0,0,-3,0,0,0,0,-1,-2,-1,0,2,0,2,2,0,-2},
                          {0,-2,-3,0,0,0,0,0,-2,-1,-2,0,0,0,0,0,0,0,1}};

double H_COMP[19][19] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0,0.1,0,0.1,0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0,0,0.1,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0.3,0,0,0,0.2,0.2,0,0,0,0,0,0,0,0,0,0},
                         {0.3,0.3,0,0,0,0.3,0,0,0,0,0,0,0,0.2,0,0,0,0.2,0},
                         {0,0.3,0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0.4,0,0},
                         {0.2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.2},
                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                         {0,0,0,0.1,0,0,0,0,0.1,0,0.1,0,0,0,0,0,0,0,0}};
/**********************************************************************************************************************
* Parameter for Communication Data Processing between OpenCR and Python
***********************************************************************************************************************/
String str;
char X_Ard[3] = {0}, Y_Ard[3] = {0}, D_Ard[1];
int xv, yv; // xv : x value, yv : y value

/**********************************************************************************************************************
* Vacuum Ejector Drive Parameter
***********************************************************************************************************************/
const int Ejector_A_PIN = 6, Ejector_B_PIN = 7;

/**********************************************************************************************************************
* DISPENSOR Dynamixel Motor & CDS Sensor Parameter
***********************************************************************************************************************/
#define CDS A0
#define DS_ID 6 // DISPENSOR DYNAMIXEL MOTOR ID
int Dynamixel_Position_DS = 0;
int Position_DSA = 0;
float Position_DSB = 0.0;
int dispenser_sum = 0;
double dispenser_position = 68, ejector_position = 174; // Reference Position Dynamxiel DS_ID
int Stone_OX = 0; // 0 : blank, 1 : stone
int cdsv = 0, cdsv_loop = 0;

void setup() // put your setup code here, to run once:
{
  /*****************************************************************************************************************
  * Vacuum Ejector Set
  ******************************************************************************************************************/
  delay(500);
  Ejector_init();
  DEBUG_SERIAL.begin(BAUDRATE);
  //while(!DEBUG_SERIAL); // Wait for Opening Serial Monitor
  DEBUG_SERIAL.println((String) "< LAB509 Senier Project >\n"+"Project Name : Concave AI Manipulator System\n"+"Project Period : 2022.01~2022.11.08");
  /*****************************************************************************************************************
  * Dynamixel Drive Set
  ******************************************************************************************************************/
  DXL.begin(BAUDRATE); // Set Port baudrate to 57600bps.
  DXL.setPortProtocolVersion(DXL_PROTOCOL_VERSION); // Set Port Protocol Version.
  for(int id = 0;id < 5;id++)
  {
    DXL.ping(DXL_ID[id]);
    // Turn off torque when configuring items in EEPROM area
    DXL.torqueOff(DXL_ID[id]);
    DXL.setOperatingMode(DXL_ID[id], OP_CURRENT_BASED_POSITION);
    DXL.torqueOn(DXL_ID[id]);
    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
    /*****************************************************************************************************************
    * Dynamixel Velocity Range Limiter
    * Joint(1) = XH540-W270-T : 0~167
    * Joint(2) = XM540-W270-T : 0~128
    * Joint(3) = XH540-W270-T : 0~167
    * Pitch, Yaw = 2XL430-W250-T : 0~250
    * DISPENSOR = XL430-W250-T : 0~250
    * Dynamixel PID Default : P(800), I(0), D(0)
    ******************************************************************************************************************/
    // DXL.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[id], 15);
    DXL.writeControlTableItem(POSITION_P_GAIN, DXL_ID[id], 1500);
    DXL.writeControlTableItem(POSITION_I_GAIN, DXL_ID[id], 100);
    DXL.writeControlTableItem(POSITION_D_GAIN, DXL_ID[id], 1000);
    DXL.writeControlTableItem(VELOCITY_LIMIT, DXL_ID[id], 128);
    DXL.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[id], 0);
    switch(id) 
    {
      case 0: DXL.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[id], 2);
              break;
      case 1: DXL.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[id], 15);
              break;
      case 2: DXL.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[id], 15);
              break;
      case 3: DXL.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[id], 15);
              break;       
      default:DXL.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[id], 15);
              break;
    }
  }
  /**********************************************************************************************************************
  * Manipulator : init position setup function
  ***********************************************************************************************************************/
  Manipulator_initial_Position(X_initial_Position, Y_initial_Position, Z_initial_Position, 20);

  // DISPENSER
  DXL.ping(DS_ID);
  DXL.setOperatingMode(DS_ID, OP_CURRENT_BASED_POSITION);
  DXL.torqueOn(DS_ID);
  DXL.writeControlTableItem(POSITION_P_GAIN, DS_ID, 800);
  DXL.writeControlTableItem(POSITION_I_GAIN, DS_ID, 0);
  DXL.writeControlTableItem(POSITION_D_GAIN, DS_ID, 0);
  DXL.writeControlTableItem(VELOCITY_LIMIT, DS_ID, 250);
  DXL.writeControlTableItem(PROFILE_ACCELERATION, DS_ID, 75);
  DXL.writeControlTableItem(PROFILE_VELOCITY, DS_ID, 75);
  Stone_OX = 1;
  Dispenser_DXL_Drive(ejector_position);
}

void loop() // put your main code here, to run repeatedly:
{
  Dispenser_CDS_Sensing_DXL_Drive_Function();
  Checkerboard_Coordinate_Data();
  if(xv != 0 && yv != 0)
  {
    /*
    init_case = 0; // 각도 보상기 X
    init_case = 1; // 각도 보상기 O
    */
    /***********************************************************
    * Manipulator Drive Mode Sequence
    * 1) 좌표 데이터 받기 전 or 로봇팔 작동 전 : 오목돌 디스펜서(OMOK STONE DISPENSER) 항시 작동
    * 2) 좌표계 이동을 위한 DYNAMIXEL 각도 정보 추출 및 저장
    * 3) MANIPULATOR END-EFFECTOR EJECTOR로 오목돌 흡입 위치로 이동 및 집기
    * 4) STANDBY 위치로 복귀하여 REFERENCE 위치(xrv = 10, yrv = 19)로 이동
    * 5) MANIPULATOR FINAL 위치로 PATH PLANNING(PATH CASE = 0) FORWARD MODE로 이동
    * 6) MANIPULATOR FINAL 위치에서 오목돌 놓을 수 있는 EJCETOR POSITION(ej_po)로 이동하여 오목돌 착수
    * 7) MANIPULATOR FINAL 위치를 유지하며 PATH PLAN POSITION인 높이 VERTICAL POSITION(vt_po)로 이동
    * 8) MANIPULATOR REFERENCE 위치로 PATH PLANNING(PATH CASE = 1) INVERSE MODE로 이동
    * 9) STANDBY 위치로 복귀(x_standby_init, y_standby_init, z_standby_init)
    ***********************************************************/
    Save_Position_Calculation(xv,yv);   // Path Planning 데이터 추출하기 위함(이 데이터를 for문 활용하여 구현할 것)
    Data_Verification();                // 시리얼 모니터 데이터 확인용
    init_case = 0;
    for(int n = 0;n < S_resolution;n++)
    {
      place_vertical_height = (double)fabs(vertical_height - dispenser_height)/S_resolution;
      Manipulator_Stone_Position(X_stone_Position, Y_stone_Position, vertical_height - ((n + 1)*place_vertical_height));
      if(n == S_resolution - 1) save_place_vertical_height = vertical_height - ((n + 1)*place_vertical_height);
    }
    Waiting(S_resolution*100);
    Ejector_Logic(Ejector_A_PIN, Ejector_B_PIN);                                          // 오목돌 흡입
    for(int n = 0;n < S_resolution;n++)
    {
      place_vertical_height = (double)fabs(vertical_height - dispenser_height)/S_resolution;
      Manipulator_Stone_Position(X_stone_Position, Y_stone_Position, save_place_vertical_height + ((n + 1)*place_vertical_height));
    }
    Waiting(S_resolution*50);
    cdsv_loop = analogRead(CDS);
    while(cdsv_loop > 1000)
    {
      cdsv_loop = 0;
      for(int n = 0;n < S_resolution;n++)
      {
        place_vertical_height = (double)fabs(vertical_height - dispenser_height)/S_resolution;
        Manipulator_Stone_Position(X_stone_Position, Y_stone_Position, vertical_height - ((n + 1)*place_vertical_height));
        if(n == S_resolution - 1) save_place_vertical_height = vertical_height - ((n + 1)*place_vertical_height);
      }
      Waiting(S_resolution*100);
      Ejector_Logic(Ejector_A_PIN, Ejector_B_PIN);                                          // 오목돌 흡입
      for(int n = 0;n < S_resolution;n++)
      {
        place_vertical_height = (double)fabs(vertical_height - dispenser_height)/S_resolution;
        Manipulator_Stone_Position(X_stone_Position, Y_stone_Position, save_place_vertical_height + ((n + 1)*place_vertical_height));
      }
      Waiting(S_resolution*50);
      cdsv_loop = analogRead(CDS);
    }
    Manipulator_Drive_Reference_Position(xrv, yrv, vertical_height, 25);
    Waiting(2500);
    /*********************************************************** 
    * Path Planning Go Goal Position
    ***********************************************************/
    init_case = 1;
    for(int n = 0;n < N_resolution;n++)
    {
      if(xv == 10 && yv == 19) break;
      radius += p_radius;
      theta += p_theta;
      if(n < (N_resolution/2)) vertical_height += path_height/(N_resolution/2);
      else vertical_height -= path_height/(N_resolution/2);
      Manipulator_Path_Drive_Position(radius, theta, vertical_height);
    }
    Waiting(N_resolution*50);
    for(int n = 0;n < V_resolution;n++)
    {
      place_vertical_height = (double)fabs(vertical_height - (ejector_height + H_COMP[yv-1][xv-1]))/V_resolution;
      Manipulator_Path_Drive_Position(radius, theta, vertical_height - ((n + 1)*place_vertical_height));
      if(n == V_resolution - 1) save_place_vertical_height = vertical_height - ((n + 1)*place_vertical_height);
    }
    Waiting(V_resolution*100);
    for(int n = 0;n < V_resolution;n++)
    {
      if(n < 2) Ejector_Logic(Ejector_B_PIN, Ejector_A_PIN);
      place_vertical_height = (double)fabs(vertical_height - (ejector_height + H_COMP[yv-1][xv-1]))/V_resolution;
      Manipulator_Path_Drive_Position(radius, theta, save_place_vertical_height + ((n + 1)*place_vertical_height));
    }
    /*********************************************************** 
    * Path Planning Back Reference Position
    ***********************************************************/
    for(int n = 0;n < N_resolution;n++)
    {
      if(xv == 10 && yv == 19) break;
      radius -= p_radius;
      theta -= p_theta;
      if(n < (N_resolution/2)) vertical_height += path_height/(N_resolution/2);
      else vertical_height -= path_height/(N_resolution/2);
      Manipulator_Path_Drive_Position(radius, theta, vertical_height);
    }
    Waiting(N_resolution*50);
    init_case = 0;
    Manipulator_initial_Position(X_initial_Position, Y_initial_Position, Z_initial_Position, 20);
    xv = yv = 0;
    
    // 다이나믹셀 속도 데이터 확인용
    for(int i = 0;i < 5;i++)
    {
      DEBUG_SERIAL.print((String)Dynamixel_V[i] + "\t");
    }
    DEBUG_SERIAL.println("");
  }
}

void Save_Position_Calculation(double xfv, double yfv) // final coordinate (x,y) input = (xv,yv)
{
  Coordinate_Position_Calculation(xfv, yfv, vertical_height);
  f_radius = radius;f_theta = theta;
  Coordinate_Position_Calculation(xrv, yrv, vertical_height);
  r_radius = radius;r_theta = theta;
  N_resolution = (int)abs(2*sqrt(sq(10-xfv)+sq(19-yfv))); // 2*(X)
  path_height = N_resolution*0.075; // 0.125
  Manipulator_5DoFs_Path_Planning_Calculation(xfv, N_resolution);
  S_resolution = 8;
  V_resolution = 10;// (int)2 + sqrt(yfv);
  //if(yv < 4) V_resolution = 15;
}

void Data_Verification()
{
  DEBUG_SERIAL.println((String) "...\nInput Concave Coordinate(X,Y) : (" + xv + "," + yv + ")");
  DEBUG_SERIAL.println((String) ">> Joint(1) : " + j_theta1);
  DEBUG_SERIAL.println((String) ">> Joint(2) : " + j_theta2);
  DEBUG_SERIAL.println((String) ">> Joint(3) : " + j_theta3);
  DEBUG_SERIAL.println((String) ">>    Pitch : " + e_thetap);
  DEBUG_SERIAL.println((String) ">>      Yaw : " + e_thetay);
  DEBUG_SERIAL.println((String) "N_resolution: " + N_resolution);
  DEBUG_SERIAL.println((String) "Path Height : " + path_height);
}

void Manipulator_initial_Position(double xi, double yi, double zi, int dvr)
{
  radius = sqrt(sq(xi)+sq(yi));
  theta = PI;
  xd = init_x+(radius*cos(theta));
  yd = init_y+(radius*sin(theta));
  zd = zi;
  Manipulator_5DoFs_Rotation_Orientation_Calculation();
  Dynamixel_Position_Detail_Drive(dvr); // Dynamixel_Position_Estimation_Drive(dvr);
}

void Manipulator_Stone_Position(double xi, double yi, double zi)
{
  radius = sqrt(sq(xi)+sq(yi));
  theta = PI;
  xd = init_x+(radius*cos(theta));
  yd = init_y+(radius*sin(theta));
  zd = zi;
  Manipulator_5DoFs_Rotation_Orientation_Calculation();
  Dynamixel_Position_Estimation_Drive();
}

void Manipulator_Drive_Position(double xm, double ym, double zm)
{
  Coordinate_Position_Calculation(xm, ym, zm);
  Manipulator_5DoFs_Rotation_Orientation_Calculation();
  Dynamixel_Position_Estimation_Drive();
}
/*********************************************************** 
* Reference Position (x,y) : (10,19) Position Control
***********************************************************/
void Manipulator_Drive_Reference_Position(double xm, double ym, double zm, int dvr)
{
  Coordinate_Position_Calculation(xm, ym, zm);
  Manipulator_5DoFs_Rotation_Orientation_Calculation();
  Dynamixel_Position_Detail_Drive(dvr);
}

/*********************************************************** 
* Path Planning Position Control
***********************************************************/
void Manipulator_Path_Drive_Position(double xp, double yp, double zp)
{
  Path_Position_Calculation(xp, yp, zp);
  Manipulator_5DoFs_Rotation_Orientation_Calculation();
  Dynamixel_Position_Estimation_Drive();
}

void Waiting(int st)
{
  delay(st);
}

/*****************************************************************************************************************
* Vacuum Ejector Drive Signal Transfer Function
******************************************************************************************************************/
void Ejector_init()
{
  pinMode(Ejector_A_PIN, OUTPUT);
  pinMode(Ejector_B_PIN, OUTPUT);
  digitalWrite(Ejector_A_PIN, LOW);
  digitalWrite(Ejector_B_PIN, LOW);
}

void Ejector_Logic(int Ejector_HIGH_PIN, int Ejector_LOW_PIN)
{
  digitalWrite(Ejector_HIGH_PIN, HIGH);
  digitalWrite(Ejector_LOW_PIN, LOW);
  delay(500);
  digitalWrite(Ejector_HIGH_PIN, LOW);
  digitalWrite(Ejector_LOW_PIN, LOW);
}

/*****************************************************************************************************************
* Concave Coordinate Data Transfer Function
******************************************************************************************************************/
void Checkerboard_Coordinate_Data()
{
  while(DEBUG_SERIAL.available())
  {
    char wait = Serial.read();
    str.concat(wait);
  }
  str.substring(0,1).toCharArray(D_Ard,2);
  if(D_Ard[0] == 'Q')
  {
    if(str.length() == 5)
    {
      str.substring(1,3).toCharArray(X_Ard,3);
      str.substring(3,5).toCharArray(Y_Ard,3);
      yv = atoi(X_Ard); // xv
      xv = atoi(Y_Ard); // yv
      if((xv < 1 || xv > 19) || (yv < 1 || yv > 19)) xv = yv = 0;
      //if((xv < 0 || yv < 0) || (xv > 19 || yv > 19)) xv = yv = 0;
      str = "";
    }
    else str = "";
  }
  else str = "";
}

/**********************************************************************************************************************
* Matlab Simulation Angle Calculation : [Radian]
* Calculate the Angle to Enter the 5DOFs Manipulator : [degree]
* Convert From Radians to Degrees
* 19X19 Matrix
* x : x coordinate, y : y coordinate
***********************************************************************************************************************/
// 오목판 좌표 위의 위치 추정을 위한 3차원 좌표상의 계산
void Coordinate_Position_Calculation(double xc, double yc, double zc)
{
  int x_case = (xc < 10) ? 0 : (xc > 10) ? 1 : 2;
  switch(x_case)
  {
    case 0: radius = sqrt(sq(board_square*(10-xc))+sq(board_square*(19-yc)+origin));
            theta = (PI/2)+atan2((board_square*(10-xc)),(board_square*(19-yc)+origin));
            break;
    case 1: radius = sqrt(sq(board_square*(xc-10))+sq(board_square*(19-yc)+origin));
            theta = (PI/2)-atan2((board_square*(xc-10)),(board_square*(19-yc)+origin));
            break;
    default:radius = sqrt(sq(board_square*(19-yc)+origin));
            theta = PI/2;
            break;
  }
  /**********************************************************************************************************************
  * d : 원형궤도 정의
  ***********************************************************************************************************************/  
  xd = init_x+radius*cos(theta);
  yd = init_y+radius*sin(theta);
  /**********************************************************************************************************************
  * vacuum_length : vacuum aspirator length(오목판으로부터의 진공흡입기 길이)
  ***********************************************************************************************************************/  
  zd = zc;
}

void Path_Position_Calculation(double p_radi, double p_th, double p_height)
{
  /**********************************************************************************************************************
  * circular trajectory : 원형궤도 정의
  ***********************************************************************************************************************/  
  xd = init_x+p_radi*cos(p_th);
  yd = init_y+p_radi*sin(p_th);
  /**********************************************************************************************************************
  * vacuum_length : vacuum aspirator length(오목판으로부터의 진공흡입기 길이)
  ***********************************************************************************************************************/  
  zd = p_height;
}

// 특수한 경우, 오목판 좌표 외의 위치 추정을 위한 3차원 좌표상의 계산
void Special_Position_Calculation()
{
  
}

void Manipulator_5DoFs_Rotation_Orientation_Calculation()
{
  /**********************************************************************************************************************
  * Inverse Kinematics : 역기구학
  ***********************************************************************************************************************/
  M_d = sqrt(sq(xd)+sq(yd)+sq(zd-L1));
  M_D = (sq(M_d)-sq(L2)-sq(L3))/(2*L2*L3);
  j_theta3 = atan2(-sqrt(1-sq(M_D)),M_D);
  alpha = atan2(zd-L1,sqrt(sq(xd)+sq(yd)));
  M_Ei = (sq(L2)+sq(M_d)-sq(L3))/(2*L2*M_d);
  beta = atan2(sqrt(1-sq(M_Ei)),M_Ei);
  j_theta2 = alpha+beta;
  j_theta1 = atan2(yd,xd);

  /**********************************************************************************************************************
  * Inverse Kinematics : 순기구학
  ***********************************************************************************************************************/
  /*
  rrx = L2*cos(j_theta1);
  rry = L2*sin(j_theta1);
  rrz = L1+L2*sin(j_theta2);
  x = cos(j_theta1)*(L2*cos(j_theta2))+L3*(cos(j_theta2+j_theta3));
  y = sin(j_theta1)*(L2*cos(j_theta2))+L3*(cos(j_theta2+j_theta3));
  z = L1+L2*sin(j_theta2)+L3*(sin(j_theta2+j_theta3));
  */

  /**********************************************************************************************************************
  * Joint(1)(2)(3), End-Effector(pitch, yaw) : But Only End-Effector Pitch Orientation
  ***********************************************************************************************************************/
  j_theta1 = (180/PI)*j_theta1;
  j_theta2 = (180/PI)*j_theta2;
  j_theta3 = (180/PI)*j_theta3;
  if(j_theta2 > 0 && j_theta3 < 0)
  {
    if((fabs(j_theta3)-fabs(j_theta2)) > 90) e_thetap = fabs((fabs(j_theta3)-fabs(j_theta2))-90);
    else if((fabs(j_theta3)-fabs(j_theta2)) < 90) e_thetap = -fabs(90-(abs(j_theta3)-fabs(j_theta2)));
  }
  else if((j_theta2 < 0 && j_theta3 < 0) && ((fabs(j_theta2)+fabs(j_theta3)) < 90)) e_thetap = -fabs(90-(fabs(j_theta2)+fabs(j_theta3)));
}

void Manipulator_5DoFs_Path_Planning_Calculation(double xc, double path_division)
{
  int x_case = (xc < 10) ? 0 : (xc > 10) ? 1 : 2;
  p_radius = fabs((f_radius-r_radius)/path_division);
  switch(x_case)
  {
    case 0: p_theta = (fabs(f_theta-(PI/2))/path_division);
            break;
    case 1: p_theta = -(fabs(f_theta-(PI/2))/path_division);
            break;
    default:p_theta = 0;  // No Angle Change
            break;
  }
}

/**********************************************************************************************************************
* Dynamixel Drive Mode Function : Reference Position Detailed Settings
***********************************************************************************************************************/
void Dynamixel_Position_Detail_Drive(int dvr) // Dynamixel Drive Function
{
  Dynamixel_Rotation_Orientation_Calculation();
  Dynamxiel_Velocity_Redefinition(dvr);
  Dynamixel_Rotation_Orientation_Drive();
}

void Dynamixel_Position_Estimation_Drive() // Dynamixel Drive Function
{
  Dynamixel_Rotation_Orientation_Calculation();
  Dynamixel_Angular_Velocity_Auto_Redefinition();
  Dynamixel_Rotation_Orientation_Drive();
}


void Dynamixel_Rotation_Orientation_Calculation()
{
  D_theta[0] = j_theta1;D_theta[1] = j_theta2;D_theta[2] = j_theta3;D_theta[3] = e_thetap;D_theta[4] = e_thetay;
  for(int id = 0;id < 5;id++)
  {
    // Dynamixel_Position[id] = map(D_theta[id], 0, 360, 0, 4095);
    switch(id) 
    {
      case 0: if(init_case == 0) Dynamixel_Position[id] = map(46+D_theta[id], 0, 360, 0, 4095);
              else Dynamixel_Position[id] = map(46.25+D_theta[id]+J1_COMP[yv-1][xv-1], 0, 360, 0, 4095); 
              break;
      case 1: if(init_case == 0) Dynamixel_Position[id] = map(90.5-D_theta[id], 0, 360, 0, (4095+4096*5));
              else Dynamixel_Position[id] = map(90.5-D_theta[id]+J2_COMP[yv-1][xv-1], 0, 360, 0, (4095+4096*5));
              break;
      case 2: if(init_case == 0) Dynamixel_Position[id] = map(178.5+D_theta[id], 0, 360, 0, 4095); // 177.5
              else Dynamixel_Position[id] = map(178.5+D_theta[id]+J3_COMP[yv-1][xv-1], 0, 360, 0, 4095);
              break;
      case 3: if(init_case == 0) Dynamixel_Position[id] = map(179+D_theta[id], 0, 360, 0, 4095);           
              else Dynamixel_Position[id] = map(180+D_theta[id]+EP_COMP[yv-1][xv-1], 0, 360, 0, 4095); 
              break;
      default:if(init_case == 0) Dynamixel_Position[id] = map(D_theta[id], 0, 360, 0, 4095);
              else Dynamixel_Position[id] = map(D_theta[id]+EY_COMP[yv-1][xv-1], 0, 360, 0, 4095);
              break;
    }
  }
}

void Dynamxiel_Velocity_Redefinition(int dvr)
{
  for(int id = 0;id < 5;id++)
  {
    switch(id) 
    {
      case 0: DXL.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[id], 3*dvr);
              break;
      case 1: DXL.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[id], dvr);
              break;
      case 2: DXL.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[id], 0.5*dvr);
              break;
      case 3: DXL.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[id], dvr);
              break;
      default:DXL.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[id], dvr);
              break;
    }
  }
}

void Dynamixel_Angular_Velocity_Auto_Redefinition()
{
  for(int i = 0;i < 5;i++)
  {
    Position_V[i] = DXL.getPresentPosition(DXL_ID[i]);
    Save_Encoder_E[i] = Encoder_E[i] = abs(Dynamixel_Position[i]-Position_V[i]); // Save_Encoder_Error변수는 자동속도 변속시, 다이나믹셀 id와 일치시키기 위함
  }
  int Size_E = sizeof(Encoder_E)/sizeof(int);
  for(int i = 0;i < Size_E;i++)
  {
    for(int j = 0;j < ((Size_E-1)-i);j++)
    {
      if(Encoder_E[j] > Encoder_E[j+1])
      {
        Temp_V = Encoder_E[j];
        Encoder_E[j] = Encoder_E[j+1];
        Encoder_E[j+1] = Temp_V;
      }
    }
  }
  for(int i = 0;i < Size_E;i++)
  {
    if(Encoder_E[i] =! 0)
    {
      Min_Encoder_E = Encoder_E[i];
      break;
    }
  }
  for(int i = 0;i < Size_E;i++)
  {
    float Division_V = 0, Division_PV = 0;    // V : Velocity Parameter, PV : Planetary Gear Velocity Parameter
    Division_V = 5; Division_PV = 1; // Division_V = 2 + N_resolution*0.15
    if(yv < 4 && init_case != 0) Division_V = 9;

    // Dynamixel Angular Velocity Setting(Planetary_Gear : 1/6)
    if(i != 1) Dynamixel_V[i] = (int)(Save_Encoder_E[i]/(Division_V*Min_Encoder_E));
    // else if(i == 3 || i == 4) Dynamixel_V[i] = (int)(Save_Encoder_E[i]/((Division_PV)*Min_Encoder_E));
    else Dynamixel_V[i] = (int)(Save_Encoder_E[i]/(Division_PV*Min_Encoder_E));
    
    if(Dynamixel_V[i] > 128) Dynamixel_V[i] = 128;
    DXL.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], Dynamixel_V[i]);
  }
}

void Dynamixel_Rotation_Orientation_Drive()
{
  for(int id = 0;id < 5;id++)
  {
    DXL.setGoalPosition(DXL_ID[id], Dynamixel_Position[id]);
  }
  while(Position_Sum > 5)
  {
    for(int id = 0;id < 5;id++)
    {
      if(id == 0) Position_Sum = 0;
      Position_B[id] = DXL.getPresentPosition(DXL_ID[id], UNIT_DEGREE);
      Position_A[id] = DXL.getPresentPosition(DXL_ID[id]);
      Position_Function[id] = abs(Dynamixel_Position[id] - Position_A[id]);
      Position_Sum += Position_Function[id];
    }
  }
}
/**********************************************************************************************************************
* Dynamixel Drive Standby Function : Waiting for Command(Velocity Value(Per 1) = [0.229 rev/min] = [6*0.229 degree/sec])
***********************************************************************************************************************/
void Drive_Standby()
{
  double standby_error[5], standby_time[5]; 
  for(int id = 0;id < 5;id++)
  {
    switch(id)
    {
      case 1:
        standby_error[id] = map(Save_Encoder_E[id], 0, (4095+4096*5), 0, 360);
        break;
      default:
        standby_error[id] = map(Save_Encoder_E[id], 0, 4095, 0, 360);
        break;
    }
    // < RPM to Degree Per Sec >  : V rev/min = V[rev/min] x 2PI[rad/rev] x 1/60[min/sec] = (V x 2PI)/60[rad/sec] x 360/2PI[degree/rad] = 6V [degree/sec]
    standby_time[id] = standby_error[id]/(Dynamixel_V[id]*6*0.229);
  }
  int standby_time_milli = 0, standby_time_sum = 0, standby_division = 5;
  for(int id = 0;id < 5;id++)
  {
    standby_time_sum += 1000*standby_time[id]; // [1000*second => milli second]
    if(standby_time[id] == 0) standby_division = standby_division - 1;
  }
  standby_time_milli = (int)(standby_time_sum/(10*standby_division));
  delay(standby_time_milli);
  DEBUG_SERIAL.println(standby_time_milli);
}

/**********************************************************************************************************************
* DISPENSER CDS Sensor data Processing & Dynamixel Motor Position Control Function
***********************************************************************************************************************/
void Dispenser_CDS_Sensing_DXL_Drive_Function()
{
  if(Stone_OX == 0)
  {
    Dispenser_DXL_Drive(dispenser_position);
    delay(1500);
    Dispenser_DXL_Drive(ejector_position);
    delay(1000);
    cdsv = analogRead(CDS);
    DEBUG_SERIAL.println((String)"CDS : " + cdsv);
    if(cdsv > 1000) Stone_OX = 1;
    else Stone_OX = 0;
  }
  if(analogRead(CDS) < 1000) Stone_OX = 0;
}

void Dispenser_DXL_Drive(double DS_Position)
{
  Dynamixel_Position_DS = map(DS_Position, 0, 360, 0, 4095);
  DXL.setGoalPosition(DS_ID, Dynamixel_Position_DS);
  while(dispenser_sum > 5);
  {
    Position_DSB = DXL.getPresentPosition(DS_ID, UNIT_DEGREE);
    Position_DSA = DXL.getPresentPosition(DS_ID);
    dispenser_sum = abs(Dynamixel_Position_DS - Position_DSA);
    dispenser_sum = 0;
  }
}
