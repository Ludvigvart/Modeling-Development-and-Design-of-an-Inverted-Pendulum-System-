/* ========= Copyright (C) CCBY 2023 by Ivo Tredal, Leidulv Tønnesland, Ludvig Vartdal & Bjørn K.T. Solheim ==========*/

/*==================================================== Libraries =====================================================*/
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "BluetoothSerial.h"
/*========================================== Global Variables & Definitions ==========================================*/

// Pinouts
#define i2cSda 33
#define i2cScl 32
#define in1 25
#define in2 26
#define in3 27
#define in4 21
#define d1 23
#define d2 22
#define d3 13
#define d4 4
#define fs1 34
#define fs2 35
#define muxA 19
#define muxB 18
#define muxC 12
#define muxCOM 14
#define motor1EncoderChannelA 36
#define motor1EncoderChannelB 39
#define motor2EncoderChannelA 16
#define motor2EncoderChannelB 17

// General code definitions
#define oneMinute 60               // milliseconds
#define absoluteZero -273.15       // degrees celcius 
#define gravityAccel 9.81          // meters pr. second squared
const uint16_t OneRotation = 360;  // degrees  
const uint8_t noRotation = 0;                     
const uint8_t cwRotation = 1;                       
const int8_t ccwRotation = -1;
                    
// Temperature sensor properties
const float R10k = 10000.0; 
const float beta25 = 3977.0; 
const float temp25 = 298.15; 
const float R25 = 10000.0; 
const float analogmax = 4095.0; 
const float Voltage = 3.3; 
const float A = 0.001152861;
const float B = 0.000227214;
const float C = 0.000000610;
const float D = 0.000000072;
const uint8_t numberOfTempSensor = 5;
float temp[5];
float tempCelsius;

// Enocder variables
volatile uint8_t currStateMotor1EncoderA;
volatile uint8_t currStateMotor2EncoderA;
volatile uint8_t prevStateMotor1EncoderA;
volatile uint8_t prevStateMotor2EncoderA;

// Timers & Intervals
unsigned long prevRPMTimer = 0;
unsigned long prevIMUTimer = 0;
unsigned long process0StartTime = 0;
unsigned long prevBluetoothTimer = 0;
unsigned long prevParamterizeMotorTimer = 0;
unsigned long prevTemperatureTimer = 0;
unsigned long prevRegulator1SampleTime = 0;
unsigned long prevRegulator2SampleTime = 0;
const unsigned long imuSampleTime = 10000;                 // microseconds
const unsigned long regulationSampleTime = 10000;          // microseconds
const unsigned long rpmSampleTime = 100000;                // microseconds
const unsigned long bluetoothUpdateInterval = 10000;       // microseconds
const unsigned long temperatureUpdateInterval = 1000000;   // microseconds
const unsigned long parameterizeMotorSampleTime = 2000000; // microseconds

// Motor settings & definitions
#define freqPWM 1000
#define motor1Channel 0
#define motor2Channel 1
#define resolutionPWM 8
const uint8_t minDutyCycle = 0;
const uint8_t maxDutyCycle = 255;
const uint8_t dutyCycleOffset = 30;
const uint8_t angleIncrement = 3;  // approximation in degrees
  
// Motor structure & properties
typedef union motor_t {
    struct motorProperties{
        double rpm;
        double angle;
        double rotation;
        double runningAngle;
        double temperature;
        double driverTemperature;
    }data;
    double motor_a[6];
};

// IMU structure & properties
struct gyroProperties{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float xError;
    float yError;
    float zError;
    float dRoll;
    float dPitch;
    float calibrSumX;
    float calibrSumY;
    float calibrSumZ;
};

struct accelProperties{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float xError;
    float yError;
    float zError;
    float xFiltered;
    float yFiltered;
    float zFiltered;
    float xPrev;
    float yPrev;
    float zPrev;
    float calibrSumX;
    float calibrSumY;
    float calibrSumZ;
};

typedef union imu_t {
    struct imu_s{
        gyroProperties gyro;
        accelProperties accel;
        float roll;
        float pitch;
        float rollComp;
        float pitchComp;    
        float rollPrev;
        float pitchPrev;
        float nu;
        float alpha;
    }data;
    float imu_a[34];
};

typedef union batteries_t {
    struct batteries_s {
        float battery1temp;
        float battery2temp;
        float battery3temp;
    }data;
    float batteries_a[3];
};

typedef union controller1_t {
    struct controller1_s{
        //Parameters
        float Kp;  
        float Ti;  
        float Td;
        float Tt;
        float N;
        
        //Signals
        float uP;
        float uI[2]; // uI[n], uI[n-1]
        float uD[2]; // uD[n], uD[n-1]
        float y[2];  // y[n], y[n-1]
        float u;
        float v;
        float uSat;  
    }value;
    float array[12];
};

typedef union controller2_t {
    struct controller2_s{
        //Parameters
        float Kp;  
        float Ti;  
        float Td;
        float Tt;
        float N;

        //Signals
        float uP;
        float uI[2]; // uI[n], uI[n-1]
        float uD[2]; // uD[n], uD[n-1]
        float y[2];  // y[n], y[n-1]
        float u;
        float v;
        float uSat;
    }value;
    float array[12];
};

// Transmission properties
float receiveDataArray[11];
uint8_t receiveDataCounter = 0;
const uint8_t decimalAccuracy = 3;
const uint8_t imuDataElements = 6;
const uint8_t motorDataElements = 3;
const uint8_t sendDataElements = 15;
const uint8_t motor1DataElementOffset = 6;
const uint8_t motor2DataElementOffset = 9;
const uint8_t receiveDataElements = 13;
const uint8_t receiveDataControllerElements = 3;
const uint8_t receiveDataOffset = 3;
const uint8_t batterElementOffset = 12;
const uint8_t batteryDataElements = 3;
const uint8_t PIDelementsToReceive[3] = {0,1,2};                       // PID elements to receive
const uint8_t imuElementsToSend[imuDataElements] = {3,4,16,17,30,31};  // IMU data elements to send
const uint8_t motorElementsToSend[motorDataElements] = {0,4,5};        // Motor data elements to send

// Objects
BluetoothSerial SerialBT;
TwoWire i2cImuSensor = TwoWire(0);
Adafruit_MPU6050 imuSensor;
controller1_t c1;
controller2_t c2;
imu_t imu;
batteries_t bat;
volatile motor_t motor1;
volatile motor_t motor2;

// Flags
bool runFlag = false;
bool runController1Flag = false;
bool runController2Flag = false;

// Misc. global variables for testing
uint16_t paramterizeDuctyCyle = 0;

/*====================================================== Setup =======================================================*/

void setup () {
    // Pinout configurations
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(d1, OUTPUT);
    pinMode(d2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(d3, OUTPUT);
    pinMode(d4, OUTPUT);
    pinMode(fs1, INPUT);
    pinMode(fs2, INPUT);
    pinMode(motor1EncoderChannelA, INPUT);
    pinMode(motor1EncoderChannelB, INPUT);
    pinMode(motor2EncoderChannelA, INPUT);
    pinMode(motor2EncoderChannelB, INPUT);
    pinMode(muxA, OUTPUT);
    pinMode(muxB, OUTPUT);
    pinMode(muxC, OUTPUT);
    pinMode(muxCOM, INPUT);
    attachInterrupt(motor1EncoderChannelA, isrMotor1Encoder, CHANGE);  
    attachInterrupt(motor2EncoderChannelA, isrMotor2Encoder, CHANGE); 
    ledcSetup(motor1Channel, freqPWM, resolutionPWM);
    ledcSetup(motor2Channel, freqPWM, resolutionPWM);
    
    // Motor pins default configuration
    digitalWrite(d1, LOW);
    digitalWrite(d2, HIGH);
    digitalWrite(d3, LOW);
    digitalWrite(d4, HIGH);

    // Multiplexer pins default configuration
    digitalWrite(muxA, LOW);
    digitalWrite(muxB, LOW);
    digitalWrite(muxC, LOW);
    
    // Serial initializes
    SerialBT.begin("ESP32-E2306");
    Serial.begin(115200);
    
    // IMU initialize & setting ranges 
    i2cImuSensor.begin(i2cSda, i2cScl);
    imuSensor.begin(0x68, &i2cImuSensor);
    imuSensor.setAccelerometerRange(MPU6050_RANGE_2_G);
    imuSensor.setGyroRange(MPU6050_RANGE_250_DEG);
    imuSensor.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Update inital encoder states
    prevStateMotor1EncoderA = digitalRead(motor1EncoderChannelA);
    prevStateMotor2EncoderA = digitalRead(motor2EncoderChannelA);

    // Set defualt initialize values:
    imu.data.alpha = 0.05;
    imu.data.nu = 0.3;
}

/*==================================================== Functions =====================================================*/

void motorControl(uint8_t motor, float regulatorOutput) {
    int8_t direction;
    uint16_t dutyCycle;
    
    // Rounds an maps output from regulator
    int16_t regulatorOutputRounded = round((float)regulatorOutput);
    int16_t dutyCycleWithDirection = map(regulatorOutputRounded, -maxDutyCycle,maxDutyCycle, 
                                        -(maxDutyCycle-dutyCycleOffset), maxDutyCycle-dutyCycleOffset);

    // Identifies direction and sets duty cycle as positive
    if (dutyCycleWithDirection > 0) {
        direction = ccwRotation;
        dutyCycle = dutyCycleWithDirection + dutyCycleOffset;
    }
    if (dutyCycleWithDirection < 0) {
        direction = cwRotation; 
        dutyCycle = abs(dutyCycleWithDirection) + dutyCycleOffset;
    }

    // Upper and lower limit for duty cycle
    if (dutyCycleWithDirection == 0) {dutyCycle = minDutyCycle;}
    if (dutyCycle > maxDutyCycle || dutyCycle < -maxDutyCycle ) {dutyCycle = maxDutyCycle;}

    switch(motor) {
        case 1:
            if (direction == ccwRotation) {
                motor1.data.rotation = ccwRotation;
                ledcDetachPin(in3);
                digitalWrite(in3, LOW);
                ledcAttachPin(in4, motor1Channel);
                ledcWrite(motor1Channel, dutyCycle);    
            }
            if (direction == cwRotation) {
                motor1.data.rotation = cwRotation;
                ledcDetachPin(in4);
                digitalWrite(in4, LOW);
                ledcAttachPin(in3, motor1Channel);
                ledcWrite(motor1Channel, dutyCycle);            
            }
            break;
        case 2:
            if (direction == ccwRotation) {  
                motor2.data.rotation = ccwRotation;
                ledcDetachPin(in2);
                digitalWrite(in2, LOW);
                ledcAttachPin(in1, motor2Channel);
                ledcWrite(motor2Channel, dutyCycle);
            }
            if (direction == cwRotation) {
                motor2.data.rotation = cwRotation;
                ledcDetachPin(in1);
                digitalWrite(in1, LOW);
                ledcAttachPin(in2, motor2Channel);
                ledcWrite(motor2Channel, dutyCycle);  
            }
            break;
    }
}


void imuGetData(){
    unsigned long currentIMUTimer = esp_timer_get_time();

    if(currentIMUTimer-prevIMUTimer >= imuSampleTime) {
        sensors_event_t a, g, temp;
        imuSensor.getEvent(&a, &g, &temp);
          
        // Get raw data and correct for error
        imu.data.accel.x = a.acceleration.x - imu.data.accel.xError;
        imu.data.accel.y = a.acceleration.y - imu.data.accel.yError;
        imu.data.accel.z = a.acceleration.z - imu.data.accel.zError;
        imu.data.gyro.x = g.gyro.x - imu.data.gyro.xError;
        imu.data.gyro.y = g.gyro.y - imu.data.gyro.yError;
        imu.data.gyro.z = g.gyro.z - imu.data.gyro.zError;

        // Filter accelerometer data (EWMA)
        imu.data.accel.xFiltered = imu.data.nu * imu.data.accel.x + (1 - imu.data.nu) * imu.data.accel.xPrev;
        imu.data.accel.yFiltered = imu.data.nu * imu.data.accel.y + (1 - imu.data.nu) * imu.data.accel.yPrev;
        imu.data.accel.zFiltered = imu.data.nu * imu.data.accel.z + (1 - imu.data.nu) * imu.data.accel.zPrev;

        // Compute Euler angles from accelerometer
        imu.data.accel.roll = atan2(imu.data.accel.yFiltered , imu.data.accel.zFiltered);
        imu.data.accel.pitch = atan2(-imu.data.accel.xFiltered , sqrt(imu.data.accel.yFiltered * 
                                    imu.data.accel.yFiltered + imu.data.accel.zFiltered * imu.data.accel.zFiltered));

        // Transform body rate to Euler rate
        imu.data.gyro.dRoll = imu.data.gyro.x + tanf(imu.data.pitchComp) * (imu.data.gyro.y * sinf(imu.data.rollComp) 
                              + imu.data.gyro.z * cosf(imu.data.rollComp));
        imu.data.gyro.dPitch = imu.data.gyro.y * cosf(imu.data.rollComp) - imu.data.gyro.z * sinf(imu.data.rollComp);
    
        // Compute Euler angles from gyroscope
        imu.data.gyro.roll = imu.data.rollPrev + (imuSampleTime/1e6) * imu.data.gyro.dRoll;
        imu.data.gyro.pitch = imu.data.pitchPrev + (imuSampleTime/1e6) * imu.data.gyro.dPitch;
        
        // Applying complementary filter
        imu.data.roll = imu.data.alpha * imu.data.accel.roll + (1 - imu.data.alpha) * imu.data.gyro.roll;
        imu.data.pitch = imu.data.alpha * imu.data.accel.pitch + (1 - imu.data.alpha) * imu.data.gyro.pitch;

        // Update signals
        imu.data.accel.xPrev = imu.data.accel.xFiltered;
        imu.data.accel.yPrev = imu.data.accel.yFiltered;
        imu.data.accel.zPrev = imu.data.accel.zFiltered;
        imu.data.rollPrev = imu.data.roll;
        imu.data.pitchPrev = imu.data.pitch;
       
        prevIMUTimer = currentIMUTimer;  
    } 
}

void getMotorsRPM() {
    unsigned long currentRPMTimer = esp_timer_get_time();

    if (currentRPMTimer - prevRPMTimer >= rpmSampleTime) {
        motor1.data.rpm = ((float)abs(motor1.data.runningAngle)/(float)OneRotation)*(float)oneMinute*
                          ((float)rpmSampleTime/(float)1e4);
        motor2.data.rpm = ((float)abs(motor2.data.runningAngle)/(float)OneRotation)*(float)oneMinute*
                          ((float)rpmSampleTime/(float)1e4);
        motor1.data.runningAngle = 0;
        motor2.data.runningAngle = 0;
        
        prevRPMTimer = esp_timer_get_time();
    } 
}

// Interrupt Routine for motor 1 encoder
void isrMotor1Encoder() {
    currStateMotor1EncoderA = digitalRead(motor1EncoderChannelA); 
    if (currStateMotor1EncoderA != prevStateMotor1EncoderA) {
        if (digitalRead(motor1EncoderChannelB) != currStateMotor1EncoderA) {
            motor1.data.angle += angleIncrement;
            motor1.data.runningAngle += angleIncrement;  
            motor1.data.rotation = cwRotation;
        }
        else {
            motor1.data.angle -= angleIncrement;
            motor1.data.runningAngle -= angleIncrement;  
            motor1.data.rotation = ccwRotation;  
        }
    }
    if (motor1.data.angle >= OneRotation || motor1.data.angle <= -(OneRotation)) {
        motor1.data.angle = 0;
    }
    prevStateMotor1EncoderA = currStateMotor1EncoderA;
}

// Interrupt Routine for motor 2 encoder
void isrMotor2Encoder() {
    currStateMotor2EncoderA = digitalRead(motor2EncoderChannelA); 
    if (currStateMotor2EncoderA != prevStateMotor2EncoderA) {
        if (digitalRead(motor2EncoderChannelB) != currStateMotor2EncoderA) {
            motor2.data.angle += angleIncrement;
            motor2.data.runningAngle += angleIncrement;   
            motor2.data.rotation = cwRotation; 
        }
        else {
            motor2.data.angle -= angleIncrement;
            motor2.data.runningAngle -= angleIncrement;  
            motor2.data.rotation = ccwRotation;
        }
    }
    if (motor2.data.angle >= OneRotation || motor2.data.angle <= -(OneRotation)) {
        motor2.data.angle = 0;
    }
    prevStateMotor2EncoderA = currStateMotor2EncoderA;
}

void getTemperatures(){
    if (esp_timer_get_time() - prevTemperatureTimer >= temperatureUpdateInterval){
        for(int i = 0; i < numberOfTempSensor; i++){
            temp[i]= readMux(i);  
        }
        bat.data.battery1temp = temp[2];   
        motor1.data.driverTemperature = temp[3];
        motor2.data.driverTemperature = temp[4];
        motor1.data.temperature = temp[1];
        motor2.data.temperature = temp[0]; 
        prevTemperatureTimer = esp_timer_get_time();  
    }
}

float readMux(int channel){
    int controlPin[] = {muxA, muxB, muxC};

    int muxChannel[5][3]={
        {0,0,0}, // channel 0
        {1,0,0}, // channel 1
        {0,1,0}, // channel 2
        {1,1,0}, // channel 3
        {0,0,1}, // channel 4
    };

    // Loop through each signal
    for(int i = 0; i < 3; i ++){
        digitalWrite(controlPin[i], muxChannel[channel][i]);
    }
    // Read the value of the muxCOM pin and calculate temperature by Steinhart-Hart equation
    int raw = analogRead(muxCOM);
    float Rt = R10k / (analogmax /raw - 1.0);
    float tempCelsius = (1.0 / (A + (B * log(Rt)) + (C * pow(log(Rt), 2.0)) + (D * pow(log(Rt), 3.0)))) + absoluteZero;
    return tempCelsius;
}

// Simple manual parameterization function for motor to find correlation between duty cycle and RPM
void parameterizeMotor() {
    
    if (esp_timer_get_time() - prevParamterizeMotorTimer >= parameterizeMotorSampleTime) {
        // Motor 1 cw;
        ledcDetachPin(in4);
        digitalWrite(in4, LOW);
        ledcAttachPin(in3, motor1Channel);
        ledcWrite(motor1Channel, paramterizeDuctyCyle);      
               
        SerialBT.println("Duty cycle: " + (String)paramterizeDuctyCyle + " , RPM: " + (String)motor1.data.rpm);
        paramterizeDuctyCyle++;
        prevParamterizeMotorTimer = esp_timer_get_time();
    } 
}

void calibrateImu() {
    // Find errors values for gyroscope and accelerometer
    for (float i=1.00; i < 1000; i++) {
        sensors_event_t a, g, temp;
        imuSensor.getEvent(&a, &g, &temp);
        imu.data.gyro.calibrSumX += g.gyro.x;
        imu.data.gyro.xError = imu.data.gyro.calibrSumX/i;
        imu.data.gyro.calibrSumY += g.gyro.y;
        imu.data.gyro.yError = imu.data.gyro.calibrSumY/i;
        imu.data.gyro.calibrSumZ += g.gyro.z;
        imu.data.gyro.zError = imu.data.gyro.calibrSumZ/i;       
        imu.data.accel.calibrSumX += a.acceleration.x;
        imu.data.accel.xError = imu.data.accel.calibrSumX/i;
        imu.data.accel.calibrSumY += a.acceleration.y;
        imu.data.accel.yError = imu.data.accel.calibrSumY/i;
        imu.data.accel.calibrSumZ += a.acceleration.z - gravityAccel;
        imu.data.accel.zError = imu.data.accel.calibrSumZ/i;         
    }
    SerialBT.println(1); // Indicate that calibartion is done            
}


// Single-axis SISO PID control
void singleAxis() {
    unsigned long currentRegulationTimer = esp_timer_get_time();
    unsigned long Ts = currentRegulationTimer - prevRegulator1SampleTime;

    if(Ts >= regulationSampleTime){
        // Parameters
        c1.value.N = 10;
        c1.value.Tt = sqrt(c1.value.Td*c1.value.Ti);
        float chi = (Ts/1e6) / (c1.value.Tt);
        float sigma = (Ts/1e6) / (c1.value.Ti);
        float xi = ((Ts/1e6)*c1.value.N - 2*c1.value.Td) / ((Ts/1e6)*c1.value.N + 2*c1.value.Td);
        float zeta = (2*c1.value.Td*c1.value.N) / ((Ts/1e6)*c1.value.N + 2*c1.value.Td);

        //Fetch output data
        c1.value.y[0] = imu.data.pitch;

        //Compute inputs
        c1.value.uP = -c1.value.Kp * c1.value.y[0];
        c1.value.uI[0] = c1.value.uI[1] - c1.value.Kp * sigma * c1.value.y[0];
        c1.value.uD[0] = -xi * c1.value.uD[1] - c1.value.Kp * zeta * (c1.value.y[0] - c1.value.y[1]);
        c1.value.v = c1.value.uP + c1.value.uI[0] + c1.value.uD[0];  
        c1.value.uI[0] += chi * (constrain(c1.value.v, -maxDutyCycle, maxDutyCycle) - c1.value.v);
        c1.value.u = c1.value.uP + c1.value.uI[0] + c1.value.uD[0]; 
        c1.value.uSat = constrain(c1.value.u, -maxDutyCycle, maxDutyCycle);
        
        motorControl(1,c1.value.uSat);

        // Update signals
        c1.value.y[1] = c1.value.y[0];
        c1.value.uI[1] = c1.value.uI[0];
        c1.value.uD[1] = c1.value.uD[0];

        prevRegulator1SampleTime = currentRegulationTimer;
    }
}


// Dual-axis MIMO PID control
void dualAxis() {
    unsigned long currentRegulationTimer = esp_timer_get_time();
    unsigned long Ts = currentRegulationTimer - prevRegulator2SampleTime;

    if(Ts >= regulationSampleTime){
        // Controller 1 parameters
        c1.value.N = 10;
        c1.value.Tt = sqrt(c1.value.Td*c1.value.Ti);
        float chi1 = (Ts/1e6) / (c1.value.Tt);
        float sigma1 = (Ts/1e6) / (c1.value.Ti);
        float xi1 = ((Ts/1e6)*c1.value.N - 2*c1.value.Td) / ((Ts/1e6)*c1.value.N + 2*c1.value.Td);
        float zeta1 = (2*c1.value.Td*c1.value.N) / ((Ts/1e6)*c1.value.N + 2*c1.value.Td);

        // Controller 2 parameters
        c2.value.N = 10;
        c2.value.Tt = sqrt(c2.value.Td*c2.value.Ti);
        float chi2 = (Ts/1e6) / (c2.value.Tt);
        float sigma2 = (Ts/1e6) / (c2.value.Ti);
        float xi2 = ((Ts/1e6)*c2.value.N - 2*c2.value.Td) / ((Ts/1e6)*c2.value.N + 2*c2.value.Td);
        float zeta2 = (2*c2.value.Td*c2.value.N) / ((Ts/1e6)*c2.value.N + 2*c2.value.Td);

        // Fetch output data
        c1.value.y[0] = imu.data.pitch;
        c2.value.y[0] = imu.data.roll;

        // Compute controller 1 inputs
        c1.value.uP = -c1.value.Kp * c1.value.y[0];
        c1.value.uI[0] = c1.value.uI[1] - c1.value.Kp * sigma1 * c1.value.y[0];
        c1.value.uD[0] = -xi1 * c1.value.uD[1] - c1.value.Kp * zeta1 * (c1.value.y[0] - c1.value.y[1]);
        c1.value.v = c1.value.uP + c1.value.uI[0] + c1.value.uD[0];  
        c1.value.uI[0] += chi1 * (constrain(c1.value.v, -maxDutyCycle, maxDutyCycle) - c1.value.v);
        c1.value.u = c1.value.uP + c1.value.uI[0] + c1.value.uD[0]; 
        c1.value.uSat = constrain(c1.value.u, -maxDutyCycle, maxDutyCycle);

        motorControl(1,c1.value.uSat);

        // Compute controller 2 inputs
        c2.value.uP = -c2.value.Kp * c2.value.y[0];
        c2.value.uI[0] = c2.value.uI[1] - c2.value.Kp * sigma2 * c2.value.y[0];
        c2.value.uD[0] = -xi2 * c2.value.uD[1] - c2.value.Kp * zeta2 * (c2.value.y[0] - c2.value.y[1]);
        c2.value.v = c2.value.uP + c2.value.uI[0] + c2.value.uD[0];  
        c2.value.uI[0] += chi2 * (constrain(c2.value.v, -maxDutyCycle, maxDutyCycle) - c2.value.v);
        c2.value.u = c2.value.uP + c2.value.uI[0] + c2.value.uD[0];
        c2.value.uSat = constrain(c2.value.u, -maxDutyCycle, maxDutyCycle);

        motorControl(2,c2.value.uSat);

        // Update signals
        c1.value.y[1] = c1.value.y[0];
        c1.value.uI[1] = c1.value.uI[0];
        c1.value.uD[1] = c1.value.uD[0];
        c2.value.y[1] = c2.value.y[0];
        c2.value.uI[1] = c2.value.uI[0];
        c2.value.uD[1] = c2.value.uD[0];

        prevRegulator2SampleTime = currentRegulationTimer;
    }
}


void sendDataOverBluetooth() {
    if (esp_timer_get_time() - prevBluetoothTimer >= bluetoothUpdateInterval) {
        float dataToSend[sendDataElements];
         
        // Update IMU data to send
        for (int i = 0; i < imuDataElements; i++) {
            dataToSend[i] = imu.imu_a[imuElementsToSend[i]];
        }
        // Update Motor data to send
        for (int i = 0; i < motorDataElements; i++) {
            dataToSend[i+motor1DataElementOffset] = motor1.motor_a[motorElementsToSend[i]];  
            dataToSend[i+motor2DataElementOffset] = motor2.motor_a[motorElementsToSend[i]];
        }
        // Update Battery data to send
        for (int i = 0; i < batteryDataElements; i++) {
             dataToSend[i+batterElementOffset] = bat.batteries_a[i];
        }
        // Send all specified data elements
        for (int i = 0; i < sendDataElements; i++) {
            SerialBT.print(dataToSend[i], decimalAccuracy);
            if (i < sendDataElements - 1) {
                SerialBT.print(" , ");
            }
            else {
                SerialBT.println("");
            }
        }
        prevBluetoothTimer = esp_timer_get_time();
    }
}

void receiveDataOverBluetooth() {
  if (SerialBT.available()) { 
    float input = SerialBT.parseFloat();
    receiveDataArray[receiveDataCounter] = input;
    receiveDataCounter += 1;
    Serial.println(receiveDataArray[receiveDataCounter]);
    // Check if all data is received
    if (receiveDataCounter == receiveDataElements) {
        // Update PID parameters 
        for (int i = 0; i < receiveDataControllerElements;  i++) {
            c1.array[PIDelementsToReceive[i]] = receiveDataArray[i];
            c2.array[PIDelementsToReceive[i]] = receiveDataArray[i + receiveDataOffset];
        }
        // Update IMU constants if not 0. 
        if (receiveDataArray[6] != 0 && receiveDataArray[7] != 0) {
            imu.data.nu = receiveDataArray[6];
            imu.data.alpha = receiveDataArray[7];
        }
        // Check for start flag
        if (receiveDataArray[8] == 1.00) {
            runFlag = true;     
        }
        // Check for stop flag
        if (receiveDataArray[9] == 1.00) {
            runFlag = false;
            motorControl(1, 0);
            motorControl(2, 0);
        }
        // Check for calibrate flag
        if (receiveDataArray[10] == 1.00) {
            calibrateImu();    
        }
        // Check controller 1 flag
        if (receiveDataArray[11] == 1.00) {
            runController1Flag = true;   
        }
        // Check controller 1 flag
        if (receiveDataArray[11] == 0.00) {
            runController1Flag = false;   
        }
        // Check controller 2 flag
        if (receiveDataArray[12] == 1.00) {
            runController2Flag = true;  
        }
        // Check controller 1 flag
        if (receiveDataArray[12] == 0.00) {
            runController2Flag = false;   
        }
        receiveDataCounter = 0;
        }  
    } 
}

/*==================================================== Main Loop =====================================================*/
void loop () {
    // Wait for command from PC-program 
    while (runFlag == false) {
       SerialBT.println("0"); // Indicate bluetooth presence while waiting
       receiveDataOverBluetooth();
       if (runFlag == true) {
            break;
       }
    } 
    if (runController1Flag == true && runController2Flag != true) {singleAxis();}
    if (runController1Flag == true && runController2Flag == true) {dualAxis();}
    imuGetData();
    getMotorsRPM();
    sendDataOverBluetooth();
    receiveDataOverBluetooth();
    getTemperatures(); 
}
/*====================================================================================================================*/
