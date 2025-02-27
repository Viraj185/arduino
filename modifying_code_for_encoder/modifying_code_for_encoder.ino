#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define ROBOT_RADIUS 0.15f 
#define WHEEL_RADIUS 0.05f   
#define COUNTS_TO_RAD 0.001f 
#define DT 0.02f

float Q_pos = 0.01f;    
float Q_vel = 0.1f;     
float R_encoder = 0.1f; 
float R_imu = 0.05f;    


const float K_MATRIX[3][3] = {
    {-0.866f,  0.5f,   ROBOT_RADIUS}, 
    { 0.866f,  0.5f,   ROBOT_RADIUS}, 
    { 0.0f,   -1.0f,   ROBOT_RADIUS}  
};

struct RobotState {
    float x, y, theta;
    float vx, vy, omega;
};

struct EncoderCounts {
    long Encoder1, Encoder2, Encoder3;
};

RobotState currentState = {0};
EncoderCounts lastCounts = {0};

void SensorFusion_Init();
void SensorFusion_Update();
RobotState SensorFusion_GetState();

int Encoder1 = 0, Encoder2 = 0, Encoder3 = 0; 


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

    if (!bno.begin()) {
        Serial.println("No BNO055 detected. Check wiring!");
        while (1);
    }

}

void loop(){
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); 
    int values[3] = {0, 0, 0}; 

    int numRead = sscanf(input.c_str(), "%d %d %d", &values[0], &values[1], &values[2]);

    if (numRead > 0) {
      Serial.print("Encoder1: ");
      Serial.println(values[0]);
    if (numRead > 1) {
      Serial.print("Encoder2: ");
      Serial.println(values[1]);
    }
    if (numRead > 2) {
      Serial.print("Encoder3: ");
      Serial.println(values[2]);
      }
    }
  }

SensorFusion_Update();
    
RobotState state = SensorFusion_GetState();
Serial.print("X: "); Serial.print(state.x);
Serial.print(" Y: "); Serial.print(state.y);
Serial.print(" Theta: "); Serial.println(state.theta);
    
delay(20); // Run at 50Hz
}

void SensorFusion_Reset() {
    currentState.x = 0.0f;
    currentState.y = 0.0f;
    currentState.theta = 0.0f;
    currentState.vx = 0.0f;
    currentState.vy = 0.0f;
    currentState.omega = 0.0f;
    lastCounts.Encoder1 = 0;
    lastCounts.Encoder2 = 0;
    lastCounts.Encoder3 = 0;
}

void SensorFusion_Update() {
    // Read encoders
    EncoderCounts currentCounts;
    currentCounts.Encoder1 = Encoder1;
    currentCounts.Encoder2 = Encoder2;
    currentCounts.Encoder3 = Encoder3;

    // Get IMU data
    sensors_event_t orientationData, gyroData, accelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    // Calculate encoder deltas
    float deltaCounts[3] = {
        (float)(currentCounts.Encoder1 - lastCounts.Encoder1),
        (float)(currentCounts.Encoder2 - lastCounts.Encoder2),
        (float)(currentCounts.Encoder3 - lastCounts.Encoder3)
    };

 float wheelVel[3];
    for (int i = 0; i < 3; i++) {
        wheelVel[i] = (deltaCounts[i] * COUNTS_TO_RAD * WHEEL_RADIUS) / DT;
    }

    // Calculate robot velocities from encoders
    float vel_x_enc = 0.0f, vel_y_enc = 0.0f, omega_enc = 0.0f;
    for (int i = 0; i < 3; i++) {
        vel_x_enc += K_MATRIX[i][0] * wheelVel[i] / 3.0f;
        vel_y_enc += K_MATRIX[i][1] * wheelVel[i] / 3.0f;
        omega_enc += K_MATRIX[i][2] * wheelVel[i] / (3.0f * ROBOT_RADIUS);
    }

    // Fuse encoder and IMU data using complementary filter
    float alpha = 0.85f; // Weight for encoder data

    currentState.vx = alpha * vel_x_enc + (1.0f - alpha) * accelData.acceleration.x * DT;
    currentState.vy = alpha * vel_y_enc + (1.0f - alpha) * accelData.acceleration.y * DT;
    currentState.omega = alpha * omega_enc + (1.0f - alpha) * gyroData.gyro.z;

    // Update heading using IMU (more accurate than encoder integration)
    currentState.theta = orientationData.orientation.x * (PI / 180.0f); // Convert degrees to radians

    // Update position
    float cosTheta = cosf(currentState.theta);
    float sinTheta = sinf(currentState.theta);

    // Transform velocities to global frame and integrate
    currentState.x += (currentState.vx * cosTheta - currentState.vy * sinTheta) * DT;
    currentState.y += (currentState.vx * sinTheta + currentState.vy * cosTheta) * DT;

    // Normalize theta to -π to π
    while (currentState.theta > PI) currentState.theta -= 2.0f * PI;
    while (currentState.theta < -PI) currentState.theta += 2.0f * PI;

    // Save current counts for next update
    lastCounts = currentCounts;
}

RobotState SensorFusion_GetState() {
    return currentState;
}
