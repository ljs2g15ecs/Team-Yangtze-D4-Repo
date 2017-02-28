/*

MPU6050 -   Arduino
VCC     -   3.3V
GND     -   GND
SDA     -   2
SCL     -   3

*//////////////////////////////////////////////////////////////////////////////////////

//Include I2C library
#include <Wire.h>
#include <Servo.h>

//Declaring some global variables
int gx, gy, gz;
long ax, ay, az, acc_magnitude;
int temp;
long gx_cal, gy_cal, gz_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_angle;
float angle_roll_acc, angle_pitch_acc;
float filtered_pitch, filtered_roll;
float kpx, kix, kdx, servo_out_x, measured_x, setpoint_x, prev_error_x, p_error_x, i_error_x, d_error_x;
float kpy, kiy, kdy, servo_out_y, measured_y, setpoint_y, prev_error_y, p_error_y, i_error_y, d_error_y;

int Throttle=0;
byte Roll, Pitch, Yaw;

Servo pitch,roll;                                                           //Initialize 'servo' as a servo

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  kpx = 0.7;
  kix = 0.01;
  kdx = 0.2;
  setpoint_x = 0;

  kpy = 0.7;
  kiy = 0.01;
  kdy = 0.2;
  setpoint_y = 0;  
  
  delay(10);                                                           //Startup delay to allow for settling
  pinMode(5, OUTPUT);                                                  //Set pin 5 as output
  pitch.attach(5);                                                     //Attach 'pitch servo' to pin 5
  Wire.begin();                                                        //start I2C. No address means master
  //Wire.onReceive(receiveEvent);
  //Serial.begin(57600);                                               //Serial for debugging
  
  setup_registers();                                                   //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  calibrate_MPU6050();                                                 //Run calibration procedure for gyro offsets
  
  loop_timer = micros();                                               //Reset the loop timer. To be used to keep a constant loop-time. Replacement for interrupts in this situation
}

void loop(){
  read_mpu6050();                                                      //Read raw acc and gyro data from MPU-6050

  gx -= gx_cal;                                                //Subtract calibrated offsets from read gyro data
  gy -= gy_cal;                                                
  gz -= gz_cal;                                                
  
  //Gyro angle calculations
  // 0.000061068 = (1 / 250Hz)/ 65.5
  angle_pitch += gx * 0.000061068;                                        //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gy * 0.000061068;                                         //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001065 = 0.000061068 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gz * 0.000001065);                    //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gz * 0.000001065);                    //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_magnitude = sqrt((ax*ax)+(ay*ay)+(az*az));                        //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)ay/acc_magnitude)* 57.296;              //Calculate the pitch angle
  angle_roll_acc = asin((float)ax/acc_magnitude)* -57.296;              //Calculate the roll angle
  
  //Accelerometer calibration offsets
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_angle){
    angle_pitch = angle_pitch * 0.999 + angle_pitch_acc * 0.001;           //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.999 + angle_roll_acc * 0.001;              //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_angle = true;
  }
  //To dampen the pitch and roll angles a complementary filter is used
  filtered_pitch = (filtered_pitch * 0.9) + (angle_pitch * 0.1);   //Take 90% of the output pitch value and add 10% of the raw pitch value
  filtered_roll = (filtered_roll * 0.9) + (angle_roll * 0.1);      //Take 90% of the output roll value and add 10% of the raw roll value
  
  //Serial.print(filtered_pitch);
  //Serial.print("  ");
  //Serial.println(filtered_roll);
  //Serial.println(micros() - loop_timer);

  servo_out_x = pid_x(setpoint_x,filtered_pitch);
  
  if(servo_out_x>180){
    servo_out_x = 180;
  }
  else if(servo_out_x<0){
    servo_out_x = 0;
  }
  pitch.write(servo_out_x);
  //Serial.print(Throttle);
  //Serial.print("xc");
  //Serial.print(Roll);
  //Serial.print(" ");
  //Serial.print(Pitch);
  //Serial.print(" ");
  //Serial.println(Yaw);
  
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer
}

void setup_registers(){
  //Activate  MPU6050
  Wire.beginTransmission(0x68);                                        //Address the MPU6050
  Wire.write(0x6B);                                                    //Address the PWR_MGMT_1 register
  Wire.write(0x00);                                                    //Reset all values to zero
  Wire.endTransmission();                                              
  //Configure accelerometer for +/-8g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();
  //Configure gyro for 500 degrees per second
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();
}

void calibrate_MPU6050(){
  //digitalWrite(5, HIGH);                                               //Turn LED on pin 5 on for duration of startup procedure
  
  //for (int cal_int = 0; cal_int < 10000 ; cal_int ++){                 //Take 10000 samples for a good average reading
  //  read_mpu6050();                                                    //Read from MPU6050
  //  gx_cal += gx;                                                      //Set calibration offsets for x y z
  //  gy_cal += gy;                                              
  //  gz_cal += gz;
  //}
  //gx_cal /= 10000;                                                     //Average all the samples
  //gy_cal /= 10000;                                                  
  //gz_cal /= 10000;                                                  

  gx_cal = -158;                                                         //Using previously calculated calibration data from 10,000 samples
  gy_cal = 90;
  gz_cal = -77;
  
  //Serial.print("gx_cal:  ");
  //Serial.print(gx_cal);
  //Serial.print(" gy_cal:  ");
  //Serial.print(gy_cal);
  //Serial.print(" gz_cal:  ");
  //Serial.println(gz_cal);
  
  //digitalWrite(5, LOW);                                                //Turn LED off when startup is complete
}

void read_mpu6050(){
  Wire.beginTransmission(0x68);                                        //Start communicating with MPU6050, using its default address 0x68. Found in datasheet
  Wire.write(0x3B);                                                    //Tell MPU6050 to start at register 0x3B. Again found from data sheet
  Wire.endTransmission();                                              //Finish writing
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from MPU6050
  while(Wire.available() < 14);                                        //Wait until all bytes are received
  ax = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the ax variable
  ay = Wire.read()<<8|Wire.read();                                  
  az = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   //Add the low and high byte to the temp variable
  gx = Wire.read()<<8|Wire.read();                                 
  gy = Wire.read()<<8|Wire.read();                                 
  gz = Wire.read()<<8|Wire.read();                                 
}

float pid_x(float meas,float set){
  p_error_x = set-meas;
  i_error_x += p_error_x*0.004;
  d_error_x = (prev_error_x-p_error_x)0.004;
  return (kpx*p_error_x)+(kix*i_error_x)-(kdx*d_error_x);
  prev_error_x = p_error_x;
}

//void receiveEvent(int howMany) {
//  while(Wire.available() < 4);
//  Throttle  = Wire.read()<<1|Wire.read();
//  Roll      = Wire.read()<<1|Wire.read();
//  Pitch     = Wire.read()<<1|Wire.read();
//  Yaw       = Wire.read()<<1|Wire.read();
//}
