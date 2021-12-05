#ifndef MOTOR_NODE_H
#define MOTOR_NODE_H
#include <pigpiod_if2.h>
#include <ros/ros.h>
#define motor1_DIR 19
#define motor1_PWM 26
#define motor1_ENA 23
#define motor1_ENB 24

#define motor2_DIR 6
#define motor2_PWM 13
#define motor2_ENA 27
#define motor2_ENB 17

#define PI 3.141592

//User_Controller
float Kp_1 = 0.84;
float Ki_1 = 0.32;
float Kd_1 = 0.1;
float Kp_2 = 0.84;
float Ki_2 = 0.28;
float Kd_2 = 0.1;
int con_switch = 0;
int16_t kanu_pwm1 = 0;
int16_t kanu_rpm1 = 0;
bool kanu_dir1 = true;
int16_t kanu_pwm2 = 0;
int16_t kanu_rpm2 = 0;
bool kanu_dir2 = false;
double rpm_err_1 = 0.0;
double rpm_err_2 = 0.0;
double rpm_err_k_1 = 0.0;
double rpm_err_k_2 = 0.0;
double rpm_derr_1 = 0.0;
double rpm_derr_2 = 0.0;
double rpm_err_sum_1 = 0.0;
double rpm_err_sum_2 = 0.0;
double ctrl_up_1 = 0.0;
double ctrl_ui_1 = 0.0;
double ctrl_ud_1 = 0.0;
double ctrl_up_2 = 0.0;
double ctrl_ui_2 = 0.0;
double ctrl_ud_2 = 0.0;
int ctrl_u_1 = 0;
int ctrl_u_2 = 0;
int ipwm_u_1 = 0;
int ipwm_u_2 = 0;
void RPM_Controller(int motor_num, bool direction, int desired_rpm);
void PID_Controller(int motor_num, bool direction, int desired_rpm);

//Text_Input
void Text_Input(void);
int PWM_range;
int PWM_frequency;
int PWM_limit;
double Control_cycle;
int Acceleration_ratio;
double Wheel_radius;
double Robot_radius;
int Encoder_resolution;
double Wheel_round;
double Robot_round;

//Motor_Setup
int Motor_Setup(void);
int pinum;
int current_PWM1;
int current_PWM2;
bool current_Direction1;
bool current_Direction2;
int acceleration;

//Interrupt_Setting
void Interrupt_Setiing(void);
volatile int EncoderCounter1;
volatile int EncoderCounter2;
volatile int EncoderCounter1A;
volatile int EncoderCounter1B;
volatile int EncoderCounter2A;
volatile int EncoderCounter2B;
volatile int EncoderSpeedCounter1;
volatile int EncoderSpeedCounter2;
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
int Motor1_Encoder_Sum();
int Motor2_Encoder_Sum();
void Init_Encoder(void);
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void);

//Motor_Controller
void Motor_Controller(int motor_num, bool direction, int pwm);
void Accel_Controller(int motor_num, bool direction, int desired_pwm);

//Example
bool switch_direction;
int Theta_Distance_Flag;
void Switch_Turn_Example(int PWM1, int PWM2);
void Theta_Turn(double Theta, int PWM);
void Distance_Go(double Distance, int PWM);
void Theta_Distance(double Theta, int Turn_PWM, double Distance, int Go_PWM);

//Utiliy
int Limit_Function(int pwm);
double RPM_Value1;
double RPM_Value2;
void RPM_Calculator();
void Motor_View();

//Whell
ros::Time current_time, last_time;

void Wheel_cal();
double x, y, th;
double v_left, v_right, vth;
double dLeft, dRight, dt;
double delta_distance, delta_th, delta_x, delta_y;
int prev_EncoderCounter1 = 0; 
int prev_EncoderCounter2 = 0;
// int EncoderCounter1
double DistancePerCount;

#endif // MOTOR_NODE_H
