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

//gcc -o week1 week_1_student.cpp -lwiringPi  -lm


int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();

//global variables
int accel_address,gyro_address;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0; 
float accel_z_calibration=0;
float imu_data[6]; //accel xyz,  gyro xyz, 
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
float old_roll = 0;
float old_pitch = 0;
float intl_pitch = 0;
float intl_roll = 0;
float A = 0.02;

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

//function to add
void setup_joystick()
{

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  int smhkey=33222;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  shared_memory = (Joystick*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 

}


//when cntrl+c pressed, kill motors

void trap(int signal)

{

   
 
   printf("ending program\n\r");

   run_program=0;
}

 
int main (int argc, char *argv[])
{

    setup_imu();
    calibrate_imu();    
    //in main before while(1) loop add...
      setup_joystick();
      signal(SIGINT, &trap);

      //to refresh values from shared memory first 
      Joystick joystick_data=*shared_memory;

      //be sure to update the while(1) in main to use run_program instead 
    while(run_program == 1)
    {
      read_imu();    
      printf("x:%10.5f\ty:%10.5f\tz:%10.5f\tp:%10.5f\tr:%10.5f\n\r",imu_data[3],imu_data[4],imu_data[5],pitch_angle, roll_angle);

      // printf("x:%10.5f\ty:%10.5f\tz:%10.5f\tp:%10.5f\tr:%10.5f\n\r",imu_data[3],imu_data[4],imu_data[5],atan2(imu_data[1], imu_data[0])*180/M_PI, atan2(imu_data[2], imu_data[0])*180/M_PI);
    }
  
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


  //accel reads

  address=0x12;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);    
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[0]= (((float)vw)/32768 * 3);//convert to g's  //figure this out 
  
  address=0x14;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);   
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[1]= (((float)vw)/32768 * 3);//convert to g's  //figure this out 
  
  address=0x16;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);   
  //convert from 2's complement     
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[2]= (((float)vw)/32768 * 3);//convert to g's  //figure this out
  


     

  //gyro reads

  address=0x02;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);   
  //convert from 2's complement          
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[3]= ((vw - (-32768)) * (1000 - (-1000)) / (32767 - (-32768))) + (-1000) - x_gyro_calibration;//convert to degrees/sec
  
  address=0x04;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);    
  //convert from 2's complement              
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4]= ((vw - (-32768)) * (1000 - (-1000)) / (32767 - (-32768))) + (-1000) - y_gyro_calibration;//convert to degrees/sec
  
  address=0x06;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);   
  //convert from 2's complement               
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5]= -(((vw - (-32768)) * (1000 - (-1000)) / (32767 - (-32768))) + (-1000) - z_gyro_calibration);//convert to degrees/sec  

  


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
  intl_pitch = (imu_data[5]*imu_diff + old_pitch)
  pitch_angle = pitch_accel * A + (1-A)*intl_pitch;
  old_pitch = pitch_angle;
  roll_accel = (atan2(imu_data[2], imu_data[0])*180/M_PI) - roll_calibration;
  intl_roll = (imu_data[4]*imu_diff + old_roll)
  roll_angle = roll_accel * A + (1-A)*;
  old_roll = roll_angle;
}

int setup_imu()
{
  wiringPiSetup ();
  
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
    wiringPiI2CWriteReg8(accel_address, 0x40, 0x89); //high speed filtered accel
    
    wiringPiI2CWriteReg8(gyro_address, 0x11, 0x00);//power on gyro
    wiringPiI2CWriteReg8(gyro_address, 0x0F, 0x01);//set gyro to +-1000dps
    wiringPiI2CWriteReg8(gyro_address, 0x10, 0x03);//set data rate and bandwith
    
    
    sleep(1);
  }
  return 0;
}


