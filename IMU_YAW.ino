#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

float  yaw;
void setup() 
{
Serial.begin(9600);
bno.begin();

  bno.setExtCrystalUse(true);
}

void loop() 
{
 // sensors_event_t event; 
 // bno.getEvent(&event);

 // Serial.print("X: ");
 // Serial.print(event.orientation.x, 4);
 // Serial.print("\tY: ");
 // Serial.print(event.orientation.y, 4);
 // Serial.print("\tZ: ");
 // Serial.print(event.orientation.z, 4);
 // Serial.println("");

 imu::Quaternion quat = bno.getQuat();
//Serial.print("W: "); Serial.print(quat.w());
//Serial.print(" X: "); Serial.print(quat.x());
//Serial.print(" Y: "); Serial.print(quat.y());
//Serial.print(" Z: "); Serial.println(quat.z());

float q_w = quat.w();
float q_x = quat.x();
float q_y = quat.y();
float q_z = quat.z();

 imu::Vector<3> euler = quat.toEuler();

yaw = atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y * q_y + q_z * q_z)); 
yaw = yaw*180.0 / M_PI;

Serial.print(" Yaw: "); Serial.println(yaw);

  delay(100);

}
