// MPU-6050
//Accelerometer http://tom.pycke.be/mav/69/accelerometer-to-attitude
//Gyro Drift//http://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data

#include<Wire.h>
#include<math.h>
#define alpha 0.1

float Gyro_cal_x, Gyro_cal_y, Gyro_cal_z, Accel_cal_x, Accel_cal_y, Accel_cal_z;
float gpitch=0,gyaw=0,groll=0;
float pitch=0,roll=0,yaw=0;

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
float gX,gY,gZ,aX,aY,aZ;
float gXp=0,gYp=0,gZp=0,aXp=0,aYp=0,aZp=0;
int yawp,pitchp,rollp;


void getGyro()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,6,true);  // request a total of 6 registers
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  gX=(GyX*(250.0/32768));
  gY=(GyY*(250.0/32768));
  gZ=(GyZ*(250.0/32768));
}

void getAcc()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,6,true);  // request a total of 6 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  aX=AcX*(2*9.8/32768);
  aY=(AcY*(2*9.8/32768));
  aZ=(AcZ*(2*9.8/32768));
}


void setup(){
  int Gyro_cal_x_sample = 0;
  int Gyro_cal_y_sample = 0;
  int Gyro_cal_z_sample = 0;
  int Accel_cal_x_sample = 0;
  int Accel_cal_y_sample = 0;
  int Accel_cal_z_sample = 0;
  int i;
  delay(5);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  delay(5000);
  int div=100;
  for (i = 0; i <= div; i += 1) {

    getGyro();
    getAcc();

    Gyro_cal_x_sample += gX;
    Gyro_cal_y_sample += gY;
    Gyro_cal_z_sample += gZ;
    Accel_cal_x_sample += aX;
    Accel_cal_y_sample += aY;
    Accel_cal_z_sample += aZ;
    delay(50);
  }

  Gyro_cal_x = Gyro_cal_x_sample / div;
  Gyro_cal_y = Gyro_cal_y_sample / div;
  Gyro_cal_z = Gyro_cal_z_sample / div;
  Accel_cal_x = Accel_cal_x_sample / div;
  Accel_cal_y = Accel_cal_y_sample / div;
  Accel_cal_z = Accel_cal_z_sample / div;

  Serial.println("Calibration Values");
  Serial.print("Gyro_cal_x = "); 
  Serial.print(Gyro_cal_x);
  Serial.print("  | Gyro_cal_y = "); 
  Serial.print(Gyro_cal_y);
  Serial.print("  | Gyro_cal_z = "); 
  Serial.print(Gyro_cal_z);
  Serial.print("  | Accel_cal_x_sample = ");
  Serial.print(Accel_cal_x);
  Serial.print("  | Accel_cal_y_sample = ");
  Serial.print(Accel_cal_y);
  Serial.print("  | Accel_cal_z_sample = ");
  Serial.println(Accel_cal_z);

}

void loop(){

  getAcc();
  getGyro();

  //Acceleration Calibration
  if((aX<0&&Accel_cal_x<0)||(aX>0&&Accel_cal_x>0))
    aX=aX-Accel_cal_x;
  else
    aX=aX+Accel_cal_x;

  if((aY<0&&Accel_cal_y<0)||(aY>0&&Accel_cal_y>0))
    aY=aY-Accel_cal_y;
  else
    aY=aY+Accel_cal_y;

  if((aZ<0&&Accel_cal_z<0)||(aZ>0&&Accel_cal_z>0))
    aZ=aZ-Accel_cal_z;
  else
    aZ=aZ+Accel_cal_z;

  //Gyroscope Calibration
  if((gX<0&&Gyro_cal_x<0)||(gX>0&&Gyro_cal_x>0))
    gX=gX-Gyro_cal_x;
  else
    gX=gX+Gyro_cal_x;

  if((gY<0&&Gyro_cal_y<0)||(gY>0&&Gyro_cal_y>0))
    gY=gY-Gyro_cal_y;
  else
    gY=gY+Gyro_cal_y;

  if((gZ<0&&Gyro_cal_z<0)||(gZ>0&&Gyro_cal_z>0))
    gZ=gZ-Gyro_cal_z;
  else
    gZ=gZ+Gyro_cal_z;

//  //error calibration with previous error for GYROSCOPE
//  float e1,e2,e3;
//  e1=gX-gXp;
//  e2=gY-gYp;
//  e3=gZ-gZp;
//  if(-1.0<e1<1.0)
//    gX=0;
//  if(-1.0<e2<1.0)
//    gY=0;
//  if(-0.5<e3<0.5)
//    gZ=0;

//Serial.print("AcX = "); Serial.print(aX);
//Serial.print(" | AcY = "); Serial.print(aY);
//Serial.print(" | AcZ = "); Serial.print(aZ);
  //  Serial.print(" | GyX = ");Serial.print(gX);
  //  Serial.print(" | GyY = ");Serial.print(gY);
  //  Serial.print(" | GyZ = ");Serial.print(gZ/50);


  //Gyproscope
  //gpitch=gpitch+(gX*(finish-start)/10);
  //groll=groll+(gY*(finish-start)/10);
  if(gZ>1.0||gZ<-1.0)
  gyaw=gyaw+(gZ/50);
  //    gyaw=gyaw+(gZ*(finish-start)/10);


  //Accelerometer
  roll=asin((((aX))/9.8));
  pitch=asin((((aY))/9.8)); 
  //yaw=asin((sqrt((aZ*aZ))/9.8)); 
  //pitch = (180 * atan (aX/sqrt((aY*aY)+(aZ*aZ)))/M_PI)+50;
  //roll = (180 * atan (aY/sqrt((aX*aX)+(aZ*aZ)))/M_PI)+8;
  //yaw = (180 * atan (aZ/sqrt((aX*aX) + (aY*aY)))/M_PI)-38;
  
  float ro=roll;
  float pi=pitch;

  //mapping
  roll=roll*90.0/1.5;
  pitch=pitch*90.0/1.5;
  yaw=gyaw*(90/30);
  
  int y=yaw;
  int p=pitch;
  int r=roll;
  
  y= (y * alpha) + (yawp* (1.0 - alpha));
  r = (r * alpha) + (rollp * (1.0 - alpha));
  p = (p * alpha) + (pitchp * (1.0 - alpha));

  //Serial.print("Pitch = "); Serial.print(pi);
  //Serial.print("  | Roll= "); Serial.print(ro);
    Serial.print("  | Yaw= "); Serial.print(y);
    Serial.print("  | Pitch = ");Serial.print(p);
    Serial.print("  | Roll= "); Serial.println(r);
//    Serial.print("  | Yaw= "); Serial.println(yaw);

  gXp=gX,gYp=gY,gZp=gZ,aXp=aX,aYp=aY,aZp=aZ;
  yawp=y;
  rollp=r;
  pitchp=p;
  //  delay(100);
}


