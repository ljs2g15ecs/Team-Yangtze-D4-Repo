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
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_angle;
float angle_roll_acc, angle_pitch_acc;
float filtered_pitch, filtered_roll;
float kpx, kix, kdx, servo_out_x, measured_x, setpoint_x, prev_error_x, p_error_x, i_error_x, d_error_x;
float kpy, kiy, kdy, servo_out_y, measured_y, setpoint_y, prev_error_y, p_error_y, i_error_y, d_error_y;
float kpz, kiz, kdz, servo_out_z, measured_z, setpoint_z, prev_error_z, p_error_z, i_error_z, d_error_z;
float pid_output_x, pid_output_y, pid_output_z;


int pid_max_x = 400;
int pid_max_y = 400;
int pid_max_z = 400;

int set_throttle = 90, set_roll=0, set_pitch=0, set_yaw=0;                                        // setpoint variables to store the angle at which the quad is meant to be at
float lf_power, rf_power, lb_power, rb_power;

#define lf_pin 11
Servo lf, rf, lb, rb;                                                           //Initialize the 4 motors as servo outputs

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  set_throttle = (1000+(set_throttle*3.137255));
  kpx = 1.3;
  kix = 0.04;
  kdx = 18;
  //setpoint_x = 0;

  kpy = 1.3;
  kiy = 0.04;
  kdy = 18;
//  setpoint_y = 0;

  kpz = 4;
  kiz = 0.02;
  kdz = 0;
 // setpoint_z = 0; 

  pinMode(lf_pin, OUTPUT);                                                  //Set pin 5 as output
  lf.attach(lf_pin);                                                        //Attach 'pitch servo' to pin 5
  Wire.begin();                                                               //start I2C. No address means master
  Serial.begin(57600);                                                        //Serial for debugging
  
  setup_IMU_registers();                                                      //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  calibrate_MPU6050();                                                        //Run calibration procedure for gyro offsets
  
  loop_timer = micros();                                                      //Reset the loop timer. To be used to keep a constant loop-time. Replacement for interrupts in this situation
}

void loop(){
  
  read_mpu6050();                                                      //Read raw acc and gyro data from MPU-6050

  gx -= gx_cal;                                                //Subtract calibrated offsets from read gyro data
  gy -= gy_cal;                                                
  gz -= gz_cal;                                                
  
  calc_angles();
  
  //Serial.print(filtered_pitch);
  //Serial.print("  ");
  //Serial.println(filtered_roll);
  
  //receiveControl();
  
  pid_x(filtered_pitch,setpoint_x);
  pid_y(filtered_roll,setpoint_y);
  pid_z(angle_yaw,setpoint_z);
  mix();
  lf.writeMicroseconds(lf_power);
  
  Serial.print(lf_power);
  Serial.print("\t");
  Serial.print(rf_power);
  Serial.print("\t");
  Serial.print(lb_power);
  Serial.print("\t");
  Serial.print(rb_power);
  Serial.print("           "); 
  Serial.print(set_throttle); 
  Serial.print("\t"); 
  Serial.print(pid_output_x);
  Serial.print("\t");
  Serial.print(pid_output_y);
  Serial.print("\t");
  Serial.println(pid_output_z);

  //Serial.println(micros() - loop_timer);
  
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer

//  if(startup_counter<100){
//    startup_counter += 1;
//  }
}

void setup_IMU_registers(){
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
  //Configure gyro for 1000 degrees per second (0x08 for 500 degrees per second)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x0C);                                                    //Set the requested starting register
  Wire.endTransmission();
}

void calibrate_MPU6050(){ 
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
///////////////////////////////////////////////      PID       //////////////////////////////////////////////////////////////////////////
float pid_x(float meas,float set){
  prev_error_x = p_error_x;
  p_error_x = set-meas;
  i_error_x += p_error_x;
  if(i_error_x > pid_max_x) i_error_x = pid_max_x;
  else if(i_error_x < (-1*pid_max_x)) i_error_x = (-1*pid_max_x);
  d_error_x = p_error_x-prev_error_x;
  pid_output_x = (kpx*p_error_x)+(kix*i_error_x)+(kdx*d_error_x);
}

float pid_y(float meas,float set){
  prev_error_y = p_error_y;
  p_error_y = set-meas;
  i_error_y += p_error_y;
  if(i_error_y > pid_max_y) i_error_y = pid_max_y;
  else if(i_error_y < (-1*pid_max_y)) i_error_y = (-1*pid_max_y);
  d_error_y = p_error_y-prev_error_y;
  pid_output_y = (kpy*p_error_y)+(kiy*i_error_y)+(kdy*d_error_y);
}

float pid_z(float meas,float set){
  prev_error_z = p_error_z;
  p_error_z = set-meas;
  i_error_z += p_error_z;
  if(i_error_z > pid_max_z) i_error_z = pid_max_z;
  else if(i_error_z < (-1*pid_max_z)) i_error_z = (-1*pid_max_z);
  d_error_z = p_error_z-prev_error_z;
  pid_output_z = (kpz*p_error_z)+(kiz*i_error_z)+(kdz*d_error_z);
}

void calc_angles(){
  //Gyro angle calculations
  // 0.000061068 = (1 / 250Hz)/ 65.5 for 500 degrees per second gyro configuration
  // 0.000121951 = (1 / 250Hz)/ 32.8 for 1000 degrees per second gyro configuration
  angle_pitch += gx * 0.000121951;                                        //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gy * 0.000121951;                                        //Calculate the traveled roll angle and add this to the angle_roll variable
  angle_yaw += gz * 0.000121951;

  //0.000002128 = 0.000121951 * (3.142(PI) / 180degr) for 500 degrees per second gyro configuration
  //0.000001065 = 0.000061068 * (3.142(PI) / 180degr) for 1000 degrees per second gyro configuration (The Arduino sin function is in radians)
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
}

void receiveControl(){
  Wire.requestFrom(1,4);
  while(Wire.available() < 4);
  set_throttle  = Wire.read();
  set_roll      = Wire.read();
  set_pitch     = Wire.read();
  set_yaw       = Wire.read();

}

void mix(){
  rf_power = set_throttle - pid_output_x + pid_output_y - pid_output_z; //Calculate the pulse for esc 1 (front-right - CCW)
  rb_power = set_throttle + pid_output_x + pid_output_y + pid_output_z; //Calculate the pulse for esc 2 (rear-right - CW)
  lb_power = set_throttle + pid_output_x - pid_output_y - pid_output_z; //Calculate the pulse for esc 3 (rear-left - CCW)
  lf_power = set_throttle - pid_output_x - pid_output_y + pid_output_z; //Calculate the pulse for esc 4 (front-left - CW)
  
  if(rf_power > 2000)rf_power = 2000;                                           //Limit the esc-1 pulse to 2000us.
  if(rb_power > 2000)rb_power = 2000;                                           //Limit the esc-2 pulse to 2000us.
  if(lb_power > 2000)lb_power = 2000;                                           //Limit the esc-3 pulse to 2000us.
  if(lf_power > 2000)lf_power = 2000; 

  if (rf_power < 1000) rf_power = 1000;                                         //Keep the motors running.
  if (rb_power < 1000) rb_power = 1000;                                         //Keep the motors running.
  if (lb_power < 1000) lb_power = 1000;                                         //Keep the motors running.
  if (lf_power < 1000) lf_power = 1000;   
}

