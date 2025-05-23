#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
// #include <iostream>
// #include <vector>
// #include <fstream>
//gcc -o week1 week_1_student.cpp -lwiringPi -lm
// Safety Constants
#define GYRO_MAX 3000
#define ROLL_MAX 45
#define PITCH_MAX 45
#define TIMEOUT 0.75
// Controller Constants
#define THRUST_NEUTRAL 1400
#define THRUST_AMPLITUDE 300
#define PITCH_AMPLITUDE 20
#define ROLL_AMPLITUDE 20
// Pitch PID Gains
#define P_GAIN 18
#define D_GAIN 5
#define I_GAIN 1
// #define P_GAIN 12
// #define D_GAIN 2.5
// #define I_GAIN 1
// #define P_GAIN 0
// #define D_GAIN 0
// #define I_GAIN 0
#define I_SATURATE 200
// Roll PID Gains
#define P_GAIN_ROLL 12
#define D_GAIN_ROLL 2.5
#define I_GAIN_ROLL 1
// #define P_GAIN_ROLL 0
// #define D_GAIN_ROLL 0
// #define I_GAIN_ROLL 0
#define I_SATURATE_ROLL 200
// Motor Constants
#define MOTOR_MAX 1800
int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter();
//global variables
FILE *fp;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //accel xyz, gyro xyz
long time_curr;
long time_prev;
long joy_prev;
long joy_curr;
float joy_diff;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float pitch_accel = 0;
float roll_accel = 0;
float roll_angle=0;
float old_roll = 0;
float old_pitch = 0;
float intl_pitch = 0;
float intl_roll = 0;
float A = 0.02;
float old_gyro_roll;
float old_gyro_pitch;
int prev_sequence = 0;
bool sequence_entered = 0;
int J_thrust;
int J_pitch;
int J_roll;
int motor_thrust;
float desired_pitch;
float desired_roll;
int motor_commands[4];
float p_err;
float r_err;
float I_pitch = 0;
float I_roll = 0;
int motor_address,accel_address,gyro_address;
bool paused = 0;
// std::vector<std::vector<float>> pitchroll;
//global variables to add
struct Joystick
{
 int key0;
 int key1;
 int key2;
 int key3;
 int pitch;
 int roll;
 int yaw;
 int thrust;
 int sequence_num;
};
Joystick* shared_memory;
int run_program=1;
void set_motor(Joystick joystick_data);
void safety_check(Joystick joystick_data, int prev_sequence);
void set_motor(Joystick joystick_data);
void set_motors(int motor0, int motor1, int motor2, int motor3);
//function to add
void setup_joystick()
{
 int segment_id;
 struct shmid_ds shmbuffer;
 int segment_size;
 const int shared_segment_size = 0x6400;
 int smhkey=33222;

 /* Allocate a shared memory segment. */
 segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
 /* Attach the shared memory segment. */
 shared_memory = (Joystick*) shmat (segment_id, 0, 0);
 printf ("shared memory attached at address %p\n", shared_memory);
 /* Determine the segment's size. */
 shmctl (segment_id, IPC_STAT, &shmbuffer);
 segment_size = shmbuffer.shm_segsz;
 printf ("segment size: %d\n", segment_size);
 /* Write a string to the shared memory segment. */
 //sprintf (shared_memory, "test!!!!.");
}
//when cntrl+c pressed, kill motors
void trap(int signal)
{
 // for week 2 graphing
 // std::ofstream file("imu_output.csv");
 // if (file.is_open()) {
 // file << "pa,pg,p,ra,rg,r\n";
 // for (const auto& row : pitchroll) {
 // for (size_t i = 0; i < row.size(); ++i) {
 // file << row[i];
 // if (i != row.size() - 1) file << ",";
 // }
 // file << "\n";
 // }
 // file.close();
 // } else {
 // std::cerr << "Failed to open file.\n";
 // }
 fclose(fp);
 // set motors to 0 when ending
 motor_commands[0] = 0;
 motor_commands[1] = 0;
 motor_commands[2] = 0;
 motor_commands[3] = 0;
 printf("ending program\n\r");
 run_program=0;
}
void calibrate_imu()
{
 float x_gyro_sum = 0.0;
 float y_gyro_sum = 0.0;
 float z_gyro_sum = 0.0;
 float roll_sum = 0.0;
 float pitch_sum = 0.0;
 float z_accel_sum = 0.0;
 for (int i = 0; i < 1000; i++) {
 read_imu();
 // reads IMU values and adds to running sum
 roll_sum += atan2(imu_data[2], imu_data[0])*180/M_PI;
 pitch_sum += atan2(imu_data[1], imu_data[0])*180/M_PI;
 z_accel_sum += imu_data[2];
 x_gyro_sum += imu_data[3];
 y_gyro_sum += imu_data[4];
 z_gyro_sum += imu_data[5];
 }
 // averages past 1000 IMU values and sets as calibration variables
 x_gyro_calibration= x_gyro_sum/1000.0;
 y_gyro_calibration=y_gyro_sum/1000.0;
 z_gyro_calibration=z_gyro_sum/1000.0;
 roll_calibration= roll_sum/1000.0;
 pitch_calibration= pitch_sum/1000.0;
 accel_z_calibration= z_accel_sum/1000.0;
printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);
}
void read_imu()
{
 uint8_t address=0;//todo: set address value for accel x value
 float ax=0;
 float az=0;
 float ay=0;
 int vh=0;
 int vl=0;
 int vw=0;
 old_gyro_pitch = imu_data[5];
 old_gyro_roll = imu_data[4];
 //accel reads
 address=0x12;//use 0x00 format for hex
 vw=wiringPiI2CReadReg16(accel_address,address);
 printf("%d\t\t", vw);
 //convert from 2's complement
 if(vw>0x8000)
 {
 vw=vw ^ 0xffff;
 vw=-vw-1;
 }
 imu_data[0]= (((float)vw)/32768 * 3);//convert to g's //figure this out

 address=0x14;//use 0x00 format for hex
 vw=wiringPiI2CReadReg16(accel_address,address);
 printf("%d\t\t", vw);
 //convert from 2's complement
 if(vw>0x8000)
 {
 vw=vw ^ 0xffff;
 vw=-vw-1;
 }
 imu_data[1]= (((float)vw)/32768 * 3);//convert to g's //figure this out

 address=0x16;//use 0x00 format for hex
 vw=wiringPiI2CReadReg16(accel_address,address);
 printf("%d\r\n", vw);
 //convert from 2's complement
 if(vw>0x8000)
 {
 vw=vw ^ 0xffff;
 vw=-vw-1;
 }
 imu_data[2]= (((float)vw)/32768 * 3);//convert to g's //figure this out


 //gyro reads
 x_gyro_calibration = 0;
 y_gyro_calibration = 0;
 z_gyro_calibration = 0;

 address=0x02;//use 0x00 format for hex
 vw=wiringPiI2CReadReg16(gyro_address,address);
 //convert from 2's complement
 if(vw>0x8000)
 {
 vw=vw ^ 0xffff;
 vw=-vw-1;
 }
 imu_data[3]= ((vw - (-32768)) * (1000 - (-1000)) / (32767 - (-32768))) + (-1000)
- x_gyro_calibration;//convert to degrees/sec

 address=0x04;//use 0x00 format for hex
 vw=wiringPiI2CReadReg16(gyro_address,address);
 //convert from 2's complement
 if(vw>0x8000)
 {
 vw=vw ^ 0xffff;
 vw=-vw-1;
 }
 imu_data[4]= ((vw - (-32768)) * (1000 - (-1000)) / (32767 - (-32768))) + (-1000)
- y_gyro_calibration;//convert to degrees/sec

 address=0x06;//use 0x00 format for hex
 vw=wiringPiI2CReadReg16(gyro_address,address);
 //convert from 2's complement
 if(vw>0x8000)
 {
 vw=vw ^ 0xffff;
 vw=-vw-1;
 }
 imu_data[5]= -(((vw - (-32768)) * (1000 - (-1000)) / (32767 - (-32768))) +
(-1000) - z_gyro_calibration);//convert to degrees/sec

}
void update_filter()
{
 //get current time in nanoseconds
 timespec_get(&te,TIME_UTC);
 time_curr=te.tv_nsec;
 //compute time since last execution
 float imu_diff=time_curr-time_prev;

 //check for rollover
 if(imu_diff<=0)
 {
 imu_diff+=1000000000;
 }
 //convert to seconds
 imu_diff=imu_diff/1000000000;
 time_prev=time_curr;

 //comp. filter for roll, pitch here:
 pitch_accel = (atan2(imu_data[1], imu_data[0])*180/M_PI) - pitch_calibration;
 intl_pitch = (imu_data[5]*imu_diff + old_pitch);
 pitch_angle = pitch_accel * A + (1-A)*(imu_data[5]*imu_diff + pitch_angle);
 old_pitch = intl_pitch;


 roll_accel = (atan2(imu_data[2], imu_data[0])*180/M_PI) - roll_calibration;
 intl_roll = (imu_data[4]*imu_diff + old_roll);
 roll_angle = roll_accel * A + (1-A)*(imu_data[4]*imu_diff + roll_angle);
 old_roll = intl_roll;
}
void set_motor(Joystick joystick_data){
 // printf("hello\r\n");
 // thrust
 J_thrust = joystick_data.thrust;
 J_pitch = joystick_data.pitch;
 J_roll = joystick_data.roll;

 desired_pitch = ( ((float)J_pitch) * (PITCH_AMPLITUDE) / (128.0) ) -
PITCH_AMPLITUDE;
 desired_roll = ( ((float)J_roll) * (ROLL_AMPLITUDE) / (128.0) ) -
ROLL_AMPLITUDE;
 motor_thrust = THRUST_NEUTRAL + THRUST_AMPLITUDE*
(((float)(128.0-J_thrust)/128.0));
 p_err = pitch_angle - desired_pitch;
 r_err = roll_angle - desired_roll;

 // p_err = 0;
 // milestone 1
 // motor_commands[0] = motor_thrust - P_GAIN*p_err;
 // motor_commands[2] = motor_thrust - P_GAIN*p_err;
 // motor_commands[1] = motor_thrust + P_GAIN*p_err;
 // motor_commands[3] = motor_thrust + P_GAIN*p_err;

 /**
 // milestone 2
 motor_commands[0] = motor_thrust - D_GAIN*imu_data[5];
 motor_commands[2] = motor_thrust - D_GAIN*imu_data[5];
 motor_commands[1] = motor_thrust + D_GAIN*imu_data[5];
 motor_commands[3] = motor_thrust + D_GAIN*imu_data[5];

 // milestone 3
 I_pitch += I_GAIN*p_err;

 if(I_pitch > I_SATURATE){
 I_pitch = I_SATURATE;
 }else if(I_pitch < -I_SATURATE){
 I_pitch = -I_SATURATE;
 }
 motor_commands[0] = motor_thrust - I_pitch;
 motor_commands[2] = motor_thrust - I_pitch;
 motor_commands[1] = motor_thrust + I_pitch;
 motor_commands[3] = motor_thrust + I_pitch;
 **/

 // milestone 4
 I_pitch += I_GAIN*p_err;

 if(I_pitch > I_SATURATE){
 I_pitch = I_SATURATE;
 }else if(I_pitch < -I_SATURATE){
 I_pitch = -I_SATURATE;
 }
 I_roll += I_GAIN_ROLL*r_err;
 if(I_roll > I_SATURATE_ROLL){
 I_roll = I_SATURATE_ROLL;
 }else if(I_roll < -I_SATURATE_ROLL){
 I_roll = -I_SATURATE_ROLL;
 }
 motor_commands[1] = motor_thrust - P_GAIN*p_err - D_GAIN*imu_data[5] - I_pitch +
P_GAIN_ROLL*r_err + D_GAIN_ROLL*imu_data[4] + I_roll;
 motor_commands[3] = motor_thrust - P_GAIN*p_err - D_GAIN*imu_data[5] - I_pitch -
P_GAIN_ROLL*r_err - D_GAIN_ROLL*imu_data[4] - I_roll;
 motor_commands[0] = motor_thrust + P_GAIN*p_err + D_GAIN*imu_data[5] + I_pitch +
P_GAIN_ROLL*r_err + D_GAIN_ROLL*imu_data[4] + I_roll;
 motor_commands[2] = motor_thrust + P_GAIN*p_err + D_GAIN*imu_data[5] + I_pitch -
P_GAIN_ROLL*r_err - D_GAIN_ROLL*imu_data[4] - I_roll;


 for(int i = 0; i < 4; i++){
 if(motor_commands[i] > MOTOR_MAX){
 motor_commands[i] = MOTOR_MAX;
 }else if(motor_commands[i] < 0){
 motor_commands[i] = 0;
 }
 }
 set_motors(motor_commands[0], motor_commands[1], motor_commands[2],
motor_commands[3]);
}
int setup_imu()
{
 wiringPiSetup ();
 motor_address=wiringPiI2CSetup (0x56) ;

 //setup imu on I2C
 accel_address=wiringPiI2CSetup (0x19) ;


 gyro_address=wiringPiI2CSetup (0x69) ;

 if(accel_address==-1)
 {
 printf("-----cant connect to accel I2C device %d --------\n",accel_address);
 return -1;
 }
 else if(gyro_address==-1)
 {
 printf("-----cant connect to gyro I2C device %d --------\n",gyro_address);
 return -1;
 }
 else
 {
 printf("all i2c devices detected\n");
 sleep(1);
 wiringPiI2CWriteReg8(accel_address, 0x7d, 0x04); //power on accel
 wiringPiI2CWriteReg8(accel_address, 0x41, 0x00); //accel range to +_3g
 wiringPiI2CWriteReg8(accel_address, 0x40, 0x8A); //high speed filtered accel

 wiringPiI2CWriteReg8(gyro_address, 0x11, 0x00);//power on gyro
 wiringPiI2CWriteReg8(gyro_address, 0x0F, 0x01);//set gyro to +-1000dps
 wiringPiI2CWriteReg8(gyro_address, 0x10, 0x03);//set data rate and bandwith


 sleep(1);
 }
 return 0;
}
void safety_check(Joystick joystick_data, int prev_sequence){
 if (joystick_data.sequence_num == prev_sequence){
 if (!sequence_entered)
 {
 timespec_get(&te,TIME_UTC);
 joy_curr=te.tv_nsec;
 joy_prev = joy_curr;
 sequence_entered = 1;
 }
 //get current time in nanoseconds
 timespec_get(&te,TIME_UTC);
 joy_curr=te.tv_nsec;
 //compute time since last execution
 joy_diff=joy_curr-joy_prev;

 //check for rollover
 if(joy_diff<=0)
 {
 joy_diff+=1000000000;
 }

 //convert to seconds
 joy_diff=joy_diff/1000000000;
 // printf("jd:%10.5f\r\n", joy_diff);
 // if 0.35 seconds have elapsed, die
 if(joy_diff > TIMEOUT){
 printf("timeout failure\r\n");
 run_program = 0;
 }
 }
 else{
 sequence_entered = 0;
 }
 if(abs(imu_data[3]) > GYRO_MAX || abs(imu_data[4]) > GYRO_MAX || abs(imu_data[5])
> GYRO_MAX){
 printf("gyro rate failure\r\n");
 run_program = 0;
 }
 if(abs(roll_angle) > ROLL_MAX){
 printf("roll angle failure\r\n");
 run_program = 0;
 }
 if(abs(pitch_angle) > PITCH_MAX){
 printf("pitch angle failure\r\n");
 run_program = 0;
 }
 if(joystick_data.key1 == 1){
 printf("you pressed B failure\r\n");
 run_program = 0;
 }
 if(joystick_data.key0 == 1){
 printf("paused\r\n");
 paused = 1;
 }
 if(joystick_data.key3 == 1){
 printf("unpaused\r\n");
 paused = 0;
 }

}
void motor_enable()
{

 uint8_t motor_id=0;
 uint8_t special_command=0;
 uint16_t commanded_speed_0=1000;
 uint16_t commanded_speed_1=0;
 uint16_t commanded_speed=0;
 uint8_t data[2];

 int cal_delay=50;

 for(int i=0;i<1000;i++)
 {

 motor_id=0;
 commanded_speed=0;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(cal_delay);
 wiringPiI2CWrite(motor_address,data[1]);


 usleep(cal_delay);
 motor_id=1;
 commanded_speed=0;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(cal_delay);
 wiringPiI2CWrite(motor_address,data[1]);

 usleep(cal_delay);
 motor_id=2;
 commanded_speed=0;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(cal_delay);
 wiringPiI2CWrite(motor_address,data[1]);


 usleep(cal_delay);
 motor_id=3;
 commanded_speed=0;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(cal_delay);
 wiringPiI2CWrite(motor_address,data[1]);
 usleep(cal_delay);
 }

 for(int i=0;i<2000;i++)
 {

 motor_id=0;
 commanded_speed=50;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(cal_delay);
 wiringPiI2CWrite(motor_address,data[1]);


 usleep(cal_delay);
 motor_id=1;
 commanded_speed=50;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(cal_delay);
 wiringPiI2CWrite(motor_address,data[1]);

 usleep(cal_delay);
 motor_id=2;
 commanded_speed=50;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(cal_delay);
 wiringPiI2CWrite(motor_address,data[1]);


 usleep(cal_delay);
 motor_id=3;
 commanded_speed=50;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(cal_delay);
 wiringPiI2CWrite(motor_address,data[1]);
 usleep(cal_delay);
 }


 for(int i=0;i<500;i++)
 {

 motor_id=0;
 commanded_speed=0;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(cal_delay);
 wiringPiI2CWrite(motor_address,data[1]);


 usleep(cal_delay);
 motor_id=1;
 commanded_speed=0;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(cal_delay);
 wiringPiI2CWrite(motor_address,data[1]);

 usleep(cal_delay);
 motor_id=2;
 commanded_speed=0;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(cal_delay);
 wiringPiI2CWrite(motor_address,data[1]);


 usleep(cal_delay);
 motor_id=3;
 commanded_speed=0;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(cal_delay);
 wiringPiI2CWrite(motor_address,data[1]);
 usleep(cal_delay);
 }
}
void set_motors(int motor0, int motor1, int motor2, int motor3)
{
 if(motor0<0)
 motor0=0;
 if(motor0>2000)
 motor0=2000;
 if(motor1<0)
 motor1=0;
 if(motor1>2000)
 motor1=2000;
 if(motor2<0)
 motor2=0;
 if(motor2>2000)
 motor2=2000;
 if(motor3<0)
 motor3=0;
 if(motor3>2000)
 motor3=2000;



 uint8_t motor_id=0;
 uint8_t special_command=0;
 uint16_t commanded_speed_0=1000;
 uint16_t commanded_speed_1=0;
 uint16_t commanded_speed=0;
 uint8_t data[2];

 // wiringPiI2CWriteReg8(motor_address, 0x00,data[0] );
 //wiringPiI2CWrite (motor_address,data[0]) ;
 int com_delay=500;

 motor_id=0;
 commanded_speed=motor0;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(com_delay);
 wiringPiI2CWrite(motor_address,data[1]);

 usleep(com_delay);
 motor_id=1;
 commanded_speed=motor1;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(com_delay);
 wiringPiI2CWrite(motor_address,data[1]);

 usleep(com_delay);
 motor_id=2;
 commanded_speed=motor2;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(com_delay);
 wiringPiI2CWrite(motor_address,data[1]);

 usleep(com_delay);
 motor_id=3;
 commanded_speed=motor3;
 data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
 data[1]=commanded_speed&0x7f;
 wiringPiI2CWrite(motor_address,data[0]);
 usleep(com_delay);
 wiringPiI2CWrite(motor_address,data[1]);
 usleep(com_delay);
}
int main (int argc, char *argv[])
{

    fp = fopen("output.csv", "w");
    fprintf(fp, "Front PWM, Back PWM, Roll Angle, Desired Roll\n");
    setup_imu();
    calibrate_imu();
    motor_enable();
    setup_joystick();
    signal(SIGINT, &trap);
    //to refresh values from shared memory first
    Joystick joystick_data=*shared_memory;
    int prev_sequence = joystick_data.sequence_num;
    //be sure to update the while(1) in main to use run_program instead
    while(run_program == 1)
 {
 read_imu();
 update_filter();
 //printf("pa:%10.5f\tpg:%10.5f\tp:%10.5f\tra:%10.5f\trg:%10.5f\tp:%10.5f\n\r",pitch_accel,intl_pitch,pitch_angle,roll_accel, intl_roll, roll_angle);
 joystick_data = *shared_memory;
 safety_check(joystick_data, prev_sequence);
 printf("\tmc1: %d\tmc2: %d\tpitch: %.3f\t\tdpitch: %.3f\n", motor_commands[0], motor_commands[2], 10*roll_angle, 10*desired_roll);

 prev_sequence = joystick_data.sequence_num;
 if(paused == 0){
 set_motor(joystick_data);
 }else{
 set_motors(1, 1, 1, 1);
 }
 // printf("M1:%d\t\tM2:%d\t\tM3:%d\t\tM4:%d\r\n", motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]);  fprintf(fp, "%d,%d,%f,%f\n", motor_commands[0], motor_commands[2],10*roll_angle, 10*desired_roll);
 }
 return 0;
}