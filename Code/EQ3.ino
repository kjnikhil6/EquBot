#include <Wire.h>

//MPU6050
#define MPU6050_ADDR                  0x68
#define MPU6050_SMPLRT_DIV_REGISTER   0x19
#define MPU6050_CONFIG_REGISTER       0x1a
#define MPU6050_GYRO_CONFIG_REGISTER  0x1b
#define MPU6050_ACCEL_CONFIG_REGISTER 0x1c
#define MPU6050_PWR_MGMT_1_REGISTER   0x6b

#define MPU6050_GYRO_OUT_REGISTER     0x43
#define MPU6050_ACCEL_OUT_REGISTER    0x3B

#define RAD_2_DEG             57.29578 // [deg/rad]
#define CALIB_OFFSET_NB_MES   500
#define TEMP_LSB_2_DEGREE     340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET       12412.0


#define DEFAULT_GYRO_COEFF    0.98

uint8_t address = MPU6050_ADDR;
float gyroXoffset, gyroYoffset, gyroZoffset;
float accXoffset, accYoffset, accZoffset;
float gyro_lsb_to_degsec, acc_lsb_to_g;
float filterGyroCoef;
float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
float angleAccX, angleAccY;
float angleX, angleY, angleZ;
long preInterval;

bool upsideDownMounting = true;


byte writeData(byte reg, byte data);
// MPU CONFIG SETTER
byte setGyroConfig(int config_num);
byte setAccConfig(int config_num);

void setGyroOffsets(float x, float y, float z);
void setAccOffsets(float x, float y, float z);

void setFilterGyroCoef(float gyro_coeff);
void setFilterAccCoef(float acc_coeff);

/* Wrap an angle in the range [-limit,+limit] (special thanks to Edgar Bonet!) */
static float wrap(float angle, float limit) {
  while (angle >  limit) angle -= 2 * limit;
  while (angle < -limit) angle += 2 * limit;
  return angle;
}

void calcOffsets(bool is_calc_gyro = true, bool is_calc_acc = true);


///LQR
float thetaB_set = 0.0;
float thetaBdot_set = 0.0;
float thetaB, thetaBdot, errorThetaB, errorThetaBdot, LQR_output;
float prev_thetaB = 0;
float K[2] = { -227.8253, -21.3778};


//motor
int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;

//timer
unsigned long loop_timer;

void setup() {

  Serial.begin(115200);
  Wire.begin();                                                             //Start the I2C bus as master
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);

  //MPU6050 init
  setFilterGyroCoef(DEFAULT_GYRO_COEFF);
  setAccOffsets(0, 0, 0);
  setGyroOffsets(0, 0, 0);
  //setAccOffsets(0.07,-0.04,0.08);
  //setGyroOffsets(-4.34,-4.34,1.56);
  //
  byte status = writeData(MPU6050_PWR_MGMT_1_REGISTER, 0x01); // check only the first connection with status
  writeData(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  writeData(MPU6050_CONFIG_REGISTER, 0x00);
  gyro_lsb_to_degsec = 131.0;           // range = +- 250 deg/s
  status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x00);
  acc_lsb_to_g = 16384.0;              //  // range = +- 2 g
  status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x00);
  setGyroConfig(0);
  setAccConfig(0);
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);

  calcOffsets();
  //  Serial.println("Done!\n");
  //  Serial.println(accXoffset);
  //  Serial.println(accYoffset);
  //  Serial.println(accZoffset);
  //  Serial.println(gyroXoffset);
  //  Serial.println(gyroXoffset);
  //  Serial.println(gyroZoffset);

  pinMode(2, OUTPUT);                                                       //Configure digital poort 2 as output
  pinMode(3, OUTPUT);                                                       //Configure digital poort 3 as output
  pinMode(4, OUTPUT);                                                       //Configure digital poort 4 as output
  pinMode(5, OUTPUT);                                                       //Configure digital poort 5 as output
  pinMode(13, OUTPUT);                                                      //Configure digital poort 6 as output

  loop_timer = micros() + 4000;



}

void loop() {
  update();
  thetaB = angleY; //(thetaB_degree)* (pi / 180);
  //Serial.println(thetaB);
  thetaBdot = gyroY;
  if(thetaBdot>-2.5 && thetaBdot<2.5)thetaBdot = 0;
  if(thetaB>-1 && thetaB<1 ) thetaBdot =0;

  //error vector
  errorThetaB     = (thetaB_set - thetaB);
  errorThetaBdot  = (thetaBdot_set - thetaBdot);
  
  LQR_output =  (-K[0] * errorThetaB - K[1] * errorThetaBdot);
  if (LQR_output > -12.5 && LQR_output < 12.5)LQR_output = 0;
  LQR_output = constrain(LQR_output, -1200, 1200);
  //Serial.println(LQR_output);

  //Motor pulse calculations
  if (LQR_output > 0)LQR_output = 405 - (1 / (LQR_output + 9)) * 13550;
  else if (LQR_output < 0)LQR_output = -405 - (1 / (LQR_output - 9)) * 13550;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if (LQR_output > 0)left_motor = 600 - LQR_output;
  else if (LQR_output < 0)left_motor = -600 - LQR_output;
  else left_motor = 0;
  if (thetaB > -1.0 && thetaB < 1.0)left_motor = 0;



  throttle_left_motor = left_motor;
  Serial.println(throttle_left_motor);

  while (loop_timer > micros());
  loop_timer += 4000;

}

void setMotorPeriod() {

}



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
}




byte writeData(byte reg, byte data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  byte status = Wire.endTransmission();
  return status; // 0 if success
}

void setGyroOffsets(float x, float y, float z) {
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}
void setAccOffsets(float x, float y, float z) {
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}

void setFilterGyroCoef(float gyro_coeff) {
  if ((gyro_coeff < 0) or (gyro_coeff > 1)) {
    gyro_coeff = DEFAULT_GYRO_COEFF;  // prevent bad gyro coeff, should throw an error...
  }
  filterGyroCoef = gyro_coeff;
}
void setFilterAccCoef(float acc_coeff) {
  setFilterGyroCoef(1.0 - acc_coeff);
}


byte setGyroConfig(int config_num) {
  byte status;
  switch (config_num) {
    case 0: // range = +- 250 deg/s
      gyro_lsb_to_degsec = 131.0;
      status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x00);
      break;
    case 1: // range = +- 500 deg/s
      gyro_lsb_to_degsec = 65.5;
      status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x08);
      break;
    default: // error
      status = 1;
      break;
  }
  return status;
}

byte setAccConfig(int config_num) {
  byte status;
  switch (config_num) {
    case 0: // range = +- 2 g
      acc_lsb_to_g = 16384.0;
      status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x00);
      break;
    case 1: // range = +- 4 g
      acc_lsb_to_g = 8192.0;
      status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x08);
      break;
    default: // error
      status = 1;
      break;
  }
  return status;
}

/* CALC OFFSET */

void calcOffsets(bool is_calc_gyro, bool is_calc_acc) {
  if (is_calc_gyro) {
    setGyroOffsets(0, 0, 0);
  }
  if (is_calc_acc) {
    setAccOffsets(0, 0, 0);
  }
  float ag[6] = {0, 0, 0, 0, 0, 0}; // 3*acc, 3*gyro

  for (int i = 0; i < CALIB_OFFSET_NB_MES; i++) {
    fetchData();
    ag[0] += accX;
    ag[1] += accY;
    ag[2] += (accZ - 1.0);
    ag[3] += gyroX;
    ag[4] += gyroY;
    ag[5] += gyroZ;
    delay(1); // wait a little bit between 2 measurements
  }

  if (is_calc_acc) {
    accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
    accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
    accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
  }

  if (is_calc_gyro) {
    gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
    gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
    gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
  }
}


/* UPDATE */

void fetchData() {
  Wire.beginTransmission(address);
  Wire.write(MPU6050_ACCEL_OUT_REGISTER);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t) 14);

  int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]

  for (int i = 0; i < 7; i++) {
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
}

void update() {
  // retrieve raw data
  fetchData();

  // estimate tilt angles: this is an approximation for small angles!
  float sgZ = (accZ >= 0) - (accZ < 0); // allow one angle to go from -180 to +180 degrees
  //angleAccX =   atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG; // [-180,+180] deg
  angleAccY = - atan2(accX,     sqrt(accZ * accZ + accY * accY)) * RAD_2_DEG; // [- 90,+ 90] deg

  unsigned long Tnew = millis();
  float dt = (Tnew - preInterval) * 1e-3;
  //Serial.println(dt*1e3);
  preInterval = Tnew;

  //angleX = wrap(filterGyroCoef*(angleAccX + wrap(angleX +     gyroX*dt - angleAccX,180)) + (1.0-filterGyroCoef)*angleAccX,180);
  angleY = wrap(filterGyroCoef * (angleAccY + wrap(angleY + sgZ * gyroY * dt - angleAccY, 90)) + (1.0 - filterGyroCoef) * angleAccY, 90);
  //angleZ += gyroZ*dt; // not wrapped (to do???)

}
