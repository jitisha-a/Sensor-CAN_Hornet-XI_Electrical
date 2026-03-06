#include <FlexCAN_T4.h>
#include <Wire.h>
#include <MS5837.h>
#include <Arduino.h>
#include "Adafruit9DOF.h"
#include "MadgwickAHRS.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define WIRE Wire

// Create object
Adafruit9DOF imu;
BMI270 imu2;
MS5837 bar30;

int val;
float calc;
float scaledup;
float pres;
float depth_ini = 0.0f;// ADDED NEW

uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR;

// Timing for 50Hz updates
const unsigned long period = 10; // milliseconds (50Hz)
static unsigned long last_update = 0;

/*
TLDR
Each of the 5 sensor values have been matched to one diff frame each
This is cause for roll pitch yaw, if i convert decimal into  int, it becomes a 32 bit unsigned integer which takes up 4 bytes alr out of 8 for the can bus frame
Im also adding a counter for debugging so thats 2 more bytes and then rest 2 are rserrved
So to be safe, 1 frame per value is better, esp since we have leeway for 3703 frames / second at 50% bus utilisation recommendation
*/

// can controller
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// CAN set up
static const uint32_t CAN_BITRATE = 1000000; // 1 Mbps

// messages ids
static const uint32_t CAN_ID_ROLL = 0x300;
static const uint32_t CAN_ID_PITCH = 0x301;
static const uint32_t CAN_ID_YAW = 0x302;
static const uint32_t CAN_ID_PRESSURE = 0x303;
static const uint32_t CAN_ID_DEPTH = 0x304;
static const uint32_t CAN_ID_LIN_ACC = 0x305; // ADDED NEW
static const uint32_t CAN_ID_ANG_VEL = 0x306; // ADDED NEW
static const uint32_t CAN_ID_QUATERNION = 0x307; // ADDED NEW

// scaling factors to convert float to integer
static const float SCALE_RPY = 10000.0f; // for 4 dp
static const float SCALE_PD = 100.0f;    // for 2 dp


// timings
const unsigned long period_ms = 20; // 50 Hz corresponds to a period of 20ms

// separate counters per message ID (will help debugging any frame drops per stream, especially as sending more data now)
static uint16_t ctr_roll = 0;
static uint16_t ctr_pitch = 0;
static uint16_t ctr_yaw = 0;
static uint16_t ctr_pressure = 0;
static uint16_t ctr_depth = 0;
// mag hard iron and soft iron biases
static const float bias[3] = {
  -14.049434f,  2.764056f, -60.358348f
};

static const float Ainv[3][3] = {
  { 1.869998f, -0.049957f, -0.064656f },
  { -0.049957f,  1.439493f,  0.174199f },
  { -0.064656f,  0.174199f,  1.460782f }
};
// callibration matrix to correct magneto 
static void mag_calibrate_ut(float magneto[3], float cal[3]) {
  float v0 = magneto[0] - bias[0];
  float v1 = magneto[1] - bias[1];
  float v2 = magneto[2] - bias[2];
  cal[0] = Ainv[0][0]*v0 + Ainv[0][1]*v1 + Ainv[0][2]*v2;
  cal[1] = Ainv[1][0]*v0 + Ainv[1][1]*v1 + Ainv[1][2]*v2;
  cal[2] = Ainv[2][0]*v0 + Ainv[2][1]*v1 + Ainv[2][2]*v2;
}
// helpers to convert our numercial sensor values into CANBUS byte formats
// Function to convert quaternion to Euler angles (roll, pitch, yaw) with gimbal lock protection
void quaternionToEuler(float q0, float q1, float q2, float q3, float &roll, float &pitch, float &yaw)
{
  // Normalize quaternion
  float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

  // Pitch (Y-axis rotation) 
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  sinp = fmaxf(-1.0f, fminf(1.0f, sinp)); // Clamp to [-1, 1]
  pitch = asinf(sinp);

  // Check for gimbal lock
  const float GIMBAL_LOCK_THRESHOLD = 0.4f; // ~sin(23.5°)
  
  if (fabsf(sinp) > GIMBAL_LOCK_THRESHOLD) {
    
    roll = 0.0f;
    
    // Compute yaw using alternative formula that's stable at high pitch
    // yaw = atan2(q1*q3 - q0*q2, q0*q3 + q1*q2) at pitch = 90°
    // General stable formula: yaw = atan2(-2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
    yaw = atan2f(-2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2));
  } else {
    // Normal case: no gimbal lock
    // Roll (X-axis rotation)
    roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    
    // Yaw (Z-axis rotation)
    yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
  }

  // Convert to degrees
  roll *= 180.0f / M_PI;
  pitch *= 180.0f / M_PI;
  yaw *= 180.0f / M_PI;
  
  // Wrap yaw to [-180, 180]
  while (yaw > 180.0f) yaw -= 360.0f;
  while (yaw <= -180.0f) yaw += 360.0f;
}

// helper: to write uint16 as little-endian into buf at index i
static inline void put_u16_le(uint8_t *buf, int i, uint16_t v)
{                                          // buf - pointer to can data array (msg.buf[i]) // i - starting index // v - the sensor numerical
  buf[i + 0] = (uint8_t)(v & 0xFF);        // extracts the lowest 8 bits thus creating LSB (least significant byte)
  buf[i + 1] = (uint8_t)((v >> 8) & 0xFF); // shifts right by 8 and masks with 0xFF to extract only 8 bits, thus creating MSB most significant byte
}

// helper: to write int16 as little-endian into buf at index i
static inline void put_i16_le(uint8_t *buf, int i, int16_t v)
{
  uint16_t u = (uint16_t)v; // reinterpret bits
  put_u16_le(buf, i, u);
}

// Helper: write int32 little-endian into buf at index i
static inline void put_i32_le(uint8_t *buf, int i, int32_t v)
{
  uint32_t u = (uint32_t)v; // reinterpret bits
  buf[i + 0] = (uint8_t)(u & 0xFF);
  buf[i + 1] = (uint8_t)((u >> 8) & 0xFF);
  buf[i + 2] = (uint8_t)((u >> 16) & 0xFF);
  buf[i + 3] = (uint8_t)((u >> 24) & 0xFF);
}

// function for sending roll, pitch and yaw frames

void send_rpy_frame(uint32_t can_id, float angle_deg, uint16_t &counter_ref)
{

  // convert float to an integer, specifically an int32 since 3 digits + 4 dp = int32
  int32_t scaled = (int32_t)lroundf(angle_deg * SCALE_RPY);

  CAN_message_t msg; // struct from can library
  msg.id = can_id;   // identifier
  msg.len = 8;       // message length

  // putting the int32 value in bytes 0,1,2,3
  put_i32_le(msg.buf, 0, scaled);

  // putting counter in bytes 4,5
  put_u16_le(msg.buf, 4, counter_ref);

  // reserved bytes 6,7 LOL
  msg.buf[6] = 0;
  msg.buf[7] = 0;

  Can0.write(msg); // sending acc can message

  counter_ref++;
}

// function for sending pressure frames

void send_pressure_frame(float pressure, uint16_t &counter_ref)
{
  // pressure is never negative has been assumed

  float p = pressure;

  // scaling by 100 to remove 2p and converting float into int16
  uint16_t scaled = (uint16_t)lroundf(p * SCALE_PD);

  CAN_message_t msg;        // struct from can library
  msg.id = CAN_ID_PRESSURE; // identifier
  msg.len = 8;              // message length

  // putting the int16 value in bytes 0,1
  put_u16_le(msg.buf, 0, scaled);

  // putting counter in bytes 2,3
  put_u16_le(msg.buf, 2, counter_ref);

  // remaining bytes setting to 0
  for (int i = 4; i < 8; i++)
    msg.buf[i] = 0;

  Can0.write(msg); // sending acc can message

  counter_ref++;
}

void send_depth_frame(float depth_m, uint16_t &counter_ref)
{
  // scaling by 100 to remove 2p and converting float into int16
  int16_t scaled = (int16_t)lroundf(depth_m * SCALE_PD);

  CAN_message_t msg;     // struct from can library
  msg.id = CAN_ID_DEPTH; // identifier
  msg.len = 8;           // message length

  // putting the int16 value in bytes 0,1
  put_i16_le(msg.buf, 0, scaled);

  // putting counter in bytes 2,3
  put_u16_le(msg.buf, 2, counter_ref);

  // rremaining bytes setting to 0
  for (int i = 4; i < 8; i++)
    msg.buf[i] = 0;

  Can0.write(msg); // sending acc can message

  counter_ref++;
}

void send_linear_accleration_frame(float a_x, float a_y, float a_z)
{

  int16_t ax_scaled = (int16_t)lroundf(a_x * SCALE_PD);
  int16_t ay_scaled = (int16_t)lroundf(a_y * SCALE_PD);
  int16_t az_scaled = (int16_t)lroundf(a_z * SCALE_PD);

  CAN_message_t msg;        // struct from can library
  msg.id = CAN_ID_LIN_ACC; // identifier
  msg.len = 8;              // message length

  // putting the int16 ax_scaled in bytes 0,1
  put_i16_le(msg.buf, 0, ax_scaled);

  // putting the int16 ay_scaled in bytes 2,3
  put_i16_le(msg.buf, 2, ay_scaled);

  // putting the int16 az_scaled in bytes 4,5
  put_i16_le(msg.buf, 4, az_scaled);

  // remaining bytes setting to 0
  for (int i = 6; i < 8; i++)
    msg.buf[i] = 0;

  Can0.write(msg); // sending acc can message
}

void send_angular_velocity_frame(float g_x, float g_y, float g_z)
{

  int16_t gx_scaled = (int16_t)lroundf(g_x * SCALE_PD);
  int16_t gy_scaled = (int16_t)lroundf(g_y * SCALE_PD);
  int16_t gz_scaled = (int16_t)lroundf(g_z * SCALE_PD);

  CAN_message_t msg;        // struct from can library
  msg.id = CAN_ID_ANG_VEL; // identifier
  msg.len = 8;              // message length

  // putting the int16 gx_scaled in bytes 0,1
  put_i16_le(msg.buf, 0, gx_scaled);
  // putting the int16 gy_scaled in bytes 2,3
  put_i16_le(msg.buf, 2, gy_scaled);

  // putting the int16 gz_scaled in bytes 4,5
  put_i16_le(msg.buf, 4, gz_scaled);

  // remaining bytes setting to 0
  for (int i = 6; i < 8; i++)
    msg.buf[i] = 0;

  Can0.write(msg); // sending acc can message
}

void send_quaternion_frame(float q1, float q2, float q3, float q0)
{

  
  int16_t q1_scaled = (int16_t)lroundf(q1 * SCALE_PD);
  int16_t q2_scaled = (int16_t)lroundf(q2 * SCALE_PD);
  int16_t q3_scaled = (int16_t)lroundf(q3 * SCALE_PD);
  int16_t q0_scaled = (int16_t)lroundf(q0 * SCALE_PD);

  CAN_message_t msg;        // struct from can library
  msg.id = CAN_ID_QUATERNION; // identifier
  msg.len = 8;              // message length

  // putting the int16 q0_scaled in bytes 0,1
  put_i16_le(msg.buf, 0, q1_scaled);

  // putting the int16 q1_scaled in bytes 2,3
  put_i16_le(msg.buf, 2, q2_scaled);

  // putting the int16 q2_scaled in bytes 4,5
  put_i16_le(msg.buf, 4, q3_scaled);

  // putting the int16 q3_scaled in bytes 6,7
  put_i16_le(msg.buf, 6, q0_scaled);

  Can0.write(msg); // sending acc can message
}

void setup()
{
  //Initialize Serial, I2C, and Sensors
  Serial.begin(115200);
  Wire.begin();
  imu.begin();
  imu2.beginI2C();
  Serial.println("9DOF IMU init OK!");
  bar30.init();
  Serial.println("Bar30 init OK!");
  bar30.setFluidDensity(992); // freshwater is 997 air is 1225

  Can0.begin();                  // turns on can hardware
  Can0.setBaudRate(CAN_BITRATE); // sets can timing to match can bus - 1 mbps rn
   // ADDED NEW CODE - 10/02
  const int N = 20;
  float depth_sum = 0.0f;

  for (int i = 0; i < N; i++)
  {
    bar30.read();                 // get fresh sensor data
    depth_sum += bar30.depth();   
    delay(10);                    
  }

  depth_ini = depth_sum / N;// mean of initial values so the initial reference is not a noisy value
  Serial.println("Teensy CAN started");
}

void loop()
{
  // Run at 50 Hz (every 20ms)
  if (millis() - last_update >= period_ms)
  {
    //read Depth sensors
    bar30.read();
    // Read sensor data
    float ax, ay, az; // accelerometer
    float gx, gy, gz; // gyroscope
    float mx, my, mz; // magnetometer
    float magneto[3];
    float mag_cal[3];
    // imu.readAll(ax, ay, az, gx, gy, gz, mx, my, mz); // the axis might be wrong
    imu.readAll(ay, ax, az, gy, gx, gz, my, mx, mz); // the axis might be wrong
    imu2.getSensorData();

    // float ax2 = imu2.data.accelX; // might be flipped
    // float ay2 = imu2.data.accelY;
    // float az2 = imu2.data.accelZ;
    float ax2 = -imu2.data.accelY; // might be flipped
    float ay2 = imu2.data.accelX;
    float az2 = imu2.data.accelZ;

    // float gx2 = imu2.data.gyroX * DEG_TO_RAD;                                    
    // float gy2 = imu2.data.gyroY * DEG_TO_RAD;
    // float gz2 = imu2.data.gyroZ * DEG_TO_RAD;
    float gx2 = -imu2.data.gyroY * DEG_TO_RAD;
    float gy2 = imu2.data.gyroX * DEG_TO_RAD;
    float gz2 = imu2.data.gyroZ * DEG_TO_RAD;

    magneto[0] = -mx;
    magneto[1] = my;
    magneto[2] = mz;
    mag_calibrate_ut(magneto,mag_cal);

    bar30.read();
    float roll, pitch, yaw;
    
    // Use imu2 (BMI270) consistently for gyro and accel with imu magnetometer
    // gx2, gy2, gz2 already in rad/s from BMI270
    // ax2, ay2, az2 are from BMI270
    MadgwickAHRSupdate(gx2, gy2, gz2, ax2, ay2, az2, mag_cal[0], mag_cal[1], mag_cal[2]);
    
    // Convert quaternion to Euler angles
    quaternionToEuler(q0, q1, q2, q3, roll, pitch, yaw);

    

    // read internal hull pressure
    val = analogRead(A0);
    calc = (val / 1023.0) * (3.3);
    scaledup = calc * 1.5;
    pres = (scaledup + 0.204) / 0.0204;
    //Depth Value
    float depth_m = bar30.depth() - depth_ini +0.32;

    // 3) Send 8 separate frames (one per value)
    send_rpy_frame(CAN_ID_ROLL, roll, ctr_roll);
    send_rpy_frame(CAN_ID_PITCH, pitch, ctr_pitch);
    send_rpy_frame(CAN_ID_YAW, yaw, ctr_yaw);
    send_pressure_frame(pres, ctr_pressure);
    send_depth_frame(depth_m, ctr_depth);
    send_linear_accleration_frame(ax2, ay2, az2);
    send_angular_velocity_frame(gx2, gy2, gz2);
    send_quaternion_frame(q1, q2, q3, q0);
    
    
    //For Checking Purpose
    
    //Serial.print("T:"); Serial.print(roll, 4);
    //Serial.print(",V:"); Serial.print(pitch, 4);
    //Serial.print(",I:"); Serial.print(yaw, 4);
    //Serial.print(",P:"); Serial.println(depth_m, 3);

    //Serial.print(ax2); Serial.print(","); Serial.print(ay2); Serial.print(","); Serial.print(az2);
    //Serial.println('\n');
    //Serial.print(gx2); Serial.print(","); Serial.print(gy2); Serial.print(","); Serial.print(gz2);
    //Serial.println('\n');

    Serial.print(q0); Serial.print(","); Serial.print(q1); Serial.print(","); Serial.print(q2); Serial.print(","); Serial.print(q3);
    Serial.println('\n');
    //Serial.print(pres);
    //Serial.println(depth_m);
   
    last_update = millis(); 
  }
}