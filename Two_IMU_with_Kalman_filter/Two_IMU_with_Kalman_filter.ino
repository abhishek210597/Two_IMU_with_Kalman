#include <Wire.h>

float RateRoll1, RatePitch1, RateYaw1;
float RateRoll2, RatePitch2, RateYaw2;

float RateCalibrationRoll1, RateCalibrationPitch1, RateCalibrationYaw1;
float RateCalibrationRoll2, RateCalibrationPitch2, RateCalibrationYaw2;
int RateCalibrationNumber;

float AccX1, AccY1, AccZ1;
float AccX2, AccY2, AccZ2;

float AngleRoll1, AnglePitch1;
float AngleRoll2, AnglePitch2;

uint32_t LoopTimer;

float KalmanAngleRoll1=0, KalmanUncertaintyAngleRoll1=2*2;
float KalmanAnglePitch1=0, KalmanUncertaintyAnglePitch1=2*2;
float KalmanAngleRoll2=0, KalmanUncertaintyAngleRoll2=2*2;
float KalmanAnglePitch2=0, KalmanUncertaintyAnglePitch2=2*2;

float Kalman1DOutput[]={0,0};

void kalman_1d(float& KalmanState, float& KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

void gyro_signals(int address, float& RateRoll, float& RatePitch, float& RateYaw, float& AccX, float& AccY, float& AccZ, float& AngleRoll, float& AnglePitch) {
  Wire.beginTransmission(address);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(address,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(address);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(address,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void setup_mpu(int address) {
  Wire.beginTransmission(address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  setup_mpu(0x68);
  setup_mpu(0x69);

  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++) {
    gyro_signals(0x68, RateRoll1, RatePitch1, RateYaw1, AccX1, AccY1, AccZ1, AngleRoll1, AnglePitch1);
    RateCalibrationRoll1+=RateRoll1;
    RateCalibrationPitch1+=RatePitch1;
    RateCalibrationYaw1+=RateYaw1;

    gyro_signals(0x69, RateRoll2, RatePitch2, RateYaw2, AccX2, AccY2, AccZ2, AngleRoll2, AnglePitch2);
    RateCalibrationRoll2+=RateRoll2;
    RateCalibrationPitch2+=RatePitch2;
    RateCalibrationYaw2+=RateYaw2;

    delay(1);
  }
  RateCalibrationRoll1/=2000;
  RateCalibrationPitch1/=2000;
  RateCalibrationYaw1/=2000;
  RateCalibrationRoll2/=2000;
  RateCalibrationPitch2/=2000;
  RateCalibrationYaw2/=2000;

  LoopTimer=micros();
}

void loop() {
  gyro_signals(0x68, RateRoll1, RatePitch1, RateYaw1, AccX1, AccY1, AccZ1, AngleRoll1, AnglePitch1);
  RateRoll1-=RateCalibrationRoll1;
  RatePitch1-=RateCalibrationPitch1;
  RateYaw1-=RateCalibrationYaw1;

  kalman_1d(KalmanAngleRoll1, KalmanUncertaintyAngleRoll1, RateRoll1, AngleRoll1);
  KalmanAngleRoll1=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll1=Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch1, KalmanUncertaintyAnglePitch1, RatePitch1, AnglePitch1);
  KalmanAnglePitch1=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch1=Kalman1DOutput[1];

  gyro_signals(0x69, RateRoll2, RatePitch2, RateYaw2, AccX2, AccY2, AccZ2, AngleRoll2, AnglePitch2);
  RateRoll2-=RateCalibrationRoll2;
  RatePitch2-=RateCalibrationPitch2;
  RateYaw2-=RateCalibrationYaw2;

  kalman_1d(KalmanAngleRoll2, KalmanUncertaintyAngleRoll2, RateRoll2, AngleRoll2);
  KalmanAngleRoll2=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll2=Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch2, KalmanUncertaintyAnglePitch2, RatePitch2, AnglePitch2);
  KalmanAnglePitch2=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch2=Kalman1DOutput[1];

  Serial.print("MPU1 - Roll Angle [°] ");
  Serial.print(KalmanAngleRoll1);
  Serial.print(" Pitch Angle [°] ");
  Serial.print(KalmanAnglePitch1);
  Serial.print("  ====  ");
  Serial.print("MPU2 - Roll Angle [°] ");
  Serial.print(KalmanAngleRoll2);
  Serial.print(" Pitch Angle [°] ");
  Serial.println(KalmanAnglePitch2);

  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}
