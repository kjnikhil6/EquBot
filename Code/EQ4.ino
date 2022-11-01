
#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro



float pi = 3.1415926;


float thetaB_set = 0.0;
float thetaBdot_set=0.0;

float thetaB;// tilt angle
float thetaBdot;

float prev_thetaB=0; 
float control_output;// Control Signal
float thetaB_dot;

float errorThetaB;    
float errorThetaBdot;



float K[2]={ -227.8253,-21.3778};
//float K[2]={ 27.3818,372.5674};//1ms

static float wrap(float angle,float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}


void calcOffsets(bool is_calc_gyro=true, bool is_calc_acc=true);

#define MPU6050_GYRO_OUT_REGISTER     0x43
#define MPU6050_ACCEL_OUT_REGISTER    0x3B

#define RAD_2_DEG             57.29578 // [deg/rad]
#define CALIB_OFFSET_NB_MES   500
#define TEMP_LSB_2_DEGREE     340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET       12412.0


#define DEFAULT_GYRO_COEFF    0.98

uint8_t address = 0x68;
float gyroXoffset, gyroYoffset, gyroZoffset;
float accXoffset, accYoffset, accZoffset;
float gyro_lsb_to_degsec, acc_lsb_to_g;
float filterGyroCoef=0.98; 
float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
float angleAccX, angleAccY;
float angleX, angleY, angleZ;
long preInterval;

bool upsideDownMounting = true;



int gyro_address = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)
int acc_calibration_value = 8878;                            //Enter the accelerometer calibration value

//Various settings
float pid_p_gain = 15;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 0.5;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                       //Gain setting for the D-controller (30)
float turning_speed = 30;                                    //Turning speed (20)
float max_target_speed = 150;                                //Max target speed (100)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw,gyro_roll_data_raw, accelerometer_data_raw;

long gyro_roll_calibration_value, gyro_pitch_calibration_value,gyro_yaw_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle;
float self_balance_pid_setpoint =0.0;
float pid_error_temp, pid_i_mem,  gyro_input;
float pid_last_d_error=0.0;
float pid_output=0.0;
float pid_setpoint=0.0;
float pid_output_left, pid_output_right;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(115200);                                                       //Start the serial port at 9600 kbps
  Wire.begin();                                                             //Start the I2C bus as master
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode
  
  //By default the MPU-6050 sleeps. So we have to wake it up.
                                                 //End the transmission with the gyro 

  calcOffsets();

  pinMode(2, OUTPUT);                                                       //Configure digital poort 2 as output
  pinMode(3, OUTPUT);                                                       //Configure digital poort 3 as output
  pinMode(4, OUTPUT);                                                       //Configure digital poort 4 as output
  pinMode(5, OUTPUT);                                                       //Configure digital poort 5 as output
  pinMode(13, OUTPUT);                                                      //Configure digital poort 6 as output

                                       //Divide the total value by 500 to get the avarage gyro offset

  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
    Wire.beginTransmission(address);
  Wire.write(MPU6050_ACCEL_OUT_REGISTER);
  Wire.endTransmission(false);
  Wire.requestFrom(address,(uint8_t) 14);

  int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]

  for(int i=0;i<7;i++){
  rawData[i]  = Wire.read() << 8;
    rawData[i] |= Wire.read();
  }

  accX = ((float)rawData[0]) / acc_lsb_to_g - accXoffset;
  accY = ((float)rawData[1]) / acc_lsb_to_g - accYoffset;
  accZ = (!upsideDownMounting - upsideDownMounting) * ((float)rawData[2]) / acc_lsb_to_g - accZoffset;
  //temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gyroX = ((float)rawData[4]) / gyro_lsb_to_degsec - gyroXoffset;
  gyroY = ((float)rawData[5]) / gyro_lsb_to_degsec - gyroYoffset;
  gyroZ = ((float)rawData[6]) / gyro_lsb_to_degsec - gyroZoffset;
float sgZ = (accZ>=0)-(accZ<0); // allow one angle to go from -180 to +180 degrees
  //angleAccX =   atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG; // [-180,+180] deg
  angleAccY = - atan2(accX,     sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG; // [- 90,+ 90] deg

  unsigned long Tnew = millis();
  float dt = (Tnew - preInterval) * 1e-3; 
  //Serial.println(dt*1e3);
  preInterval = Tnew;

  //angleX = wrap(filterGyroCoef*(angleAccX + wrap(angleX +     gyroX*dt - angleAccX,180)) + (1.0-filterGyroCoef)*angleAccX,180);
  angleY = wrap(filterGyroCoef*(angleAccY + wrap(angleY + sgZ*gyroY*dt - angleAccY, 90)) + (1.0-filterGyroCoef)*angleAccY, 90);
                            //Calculate the traveled during this loop angle and add this to the angle_gyro variable
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  thetaB=angleY;//(thetaB_degree)* (pi / 180);
Serial.println(thetaB);
 if(thetaB-prev_thetaB>-1 && thetaB-prev_thetaB<1 ) thetaB_dot =0;
 else thetaB_dot = (thetaB - prev_thetaB)*250;
  //thetaB_dot = (thetaB - prev_thetaB)*dt;
  //Serial.print("\tThetaB_dot:");
  //Serial.print(thetaB_dot,3);
  
  prev_thetaB=thetaB;
     
  //error vector
  errorThetaB     = (thetaB_set-thetaB);
  errorThetaBdot  = (thetaBdot_set-thetaB_dot) ;
  pid_output =  -(-K[0]*errorThetaB-K[1]*errorThetaBdot);
  //pid_output-=pid_output*0.000003;
  //Serial.print("\tLQR:");
  //Serial.println(pid_output);
  
//if(thetaB<0.5 || thetaB>-0.50){
//  Serial.println(1);
//  pid_output=0;
//}
  if(pid_output <5 && pid_output > -5){
    //if(pid_output > 0)self_balance_pid_setpoint += 0.000015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    //if(pid_output < 0)self_balance_pid_setpoint -= 0.000015;
    pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced
  }
//Serial.println( pid_output);
  if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1){    //If the robot tips over or the start variable is zero or the battery is empty
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                          //Reset the I-controller memory
    start = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Control calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Motor pulse calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 2500;
  else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 2500;


  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)left_motor = 470 - pid_output_left;
  else if(pid_output_left < 0)left_motor = -470 - pid_output_left;
  else left_motor = 0;

  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  while(loop_timer > micros());//{Serial.println(12);}
  loop_timer += 4000;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine  TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect){
  //Left motor pulse calculations
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(throttle_counter_left_motor > throttle_left_motor_memory){             //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0){                                     //If the throttle_left_motor_memory is negative
     PORTD |= 0b00001000;                                                 //Set output 3 low to reverse the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else PORTD &= 0b11110111;                                              //Set output 3 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_left_motor == 1)PORTD |= 0b00000100;             //Set output 2 high to create a pulse for the stepper controller
  else if(throttle_counter_left_motor == 2)PORTD &= 0b11111011;             //Set output 2 low because the pulse only has to last for 20us 
  
 //Set output 4 low because the pulse only has to last for 20us
}


void setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}
void setAccOffsets(float x, float y, float z){
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}
void calcOffsets(bool is_calc_gyro, bool is_calc_acc){
  if(is_calc_gyro){ setGyroOffsets(0,0,0); }
  if(is_calc_acc){ setAccOffsets(0,0,0); }
  float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro
  
  for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
    Wire.beginTransmission(address);
  Wire.write(MPU6050_ACCEL_OUT_REGISTER);
  Wire.endTransmission(false);
  Wire.requestFrom(address,(uint8_t) 14);

  int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]

  for(int i=0;i<7;i++){
  rawData[i]  = Wire.read() << 8;
    rawData[i] |= Wire.read();
  }

  accX = ((float)rawData[0]) / acc_lsb_to_g - accXoffset;
  accY = ((float)rawData[1]) / acc_lsb_to_g - accYoffset;
  accZ = (!upsideDownMounting - upsideDownMounting) * ((float)rawData[2]) / acc_lsb_to_g - accZoffset;
  //temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gyroX = ((float)rawData[4]) / gyro_lsb_to_degsec - gyroXoffset;
  gyroY = ((float)rawData[5]) / gyro_lsb_to_degsec - gyroYoffset;
  gyroZ = ((float)rawData[6]) / gyro_lsb_to_degsec - gyroZoffset;
  ag[0] += accX;
  ag[1] += accY;
  ag[2] += (accZ-1.0);
  ag[3] += gyroX;
  ag[4] += gyroY;
  ag[5] += gyroZ;
  delay(1); // wait a little bit between 2 measurements
  }
  
  if(is_calc_acc){
    accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
    accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
    accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
  }
  
  if(is_calc_gyro){
    gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
    gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
    gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
  }
}
