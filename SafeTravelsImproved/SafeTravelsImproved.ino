#include <NewPing.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include  "stdlib.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINzO_WIRE
    #include "Wire.h"
#endif

// ================================================================
// ===               VARIABLE DECLARATION                       ===
// ================================================================

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define trigPin 7
#define echoPin 8
#define ECHO_PIN     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar = NewPing(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); //

int PIEZO_PIN = 13;  
bool blinkState = false;
int countDistance = 0;
double lastTenReadings[10];
double newReadings[10];
double upperTukeyFence = 0;
double lowerTukeyFence = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                 SETUP OF MPU AND PINS                    ===
// ================================================================

void setup() {
    /*
     * Everything below in the setup is standard code for setting up the accelerometer
     * The baudrate is set to 115200 but could be changed
     */
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
  
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if (devStatus == 0) 
    {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 

    pinMode(PIEZO_PIN, OUTPUT);
}

// ================================================================
// ===               LOOP: CALL SONAR METHOD                    ===
// ===          CHECK FIFO AND OUTPUT YPR CALCULATION RESULTS   ===
// ================================================================
  
void loop() 
{
  //double currentPitch = ypr[2] * 180/M_PI;
  //Serial.println(currentPitch);

  double tempDistance = checkDistance();
  
  if (tempDistance < 400 && tempDistance > 1) {
    lastTenReadings[countDistance] = tempDistance;
    countDistance++;
  }
  if (countDistance == 9) {
    double newSomething = getFilteredDistance();
    Serial.println(newSomething);
    countDistance = 0;
  }
   
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
  } 
  
  else if (mpuIntStatus & 0x02) 
  {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      #endif
  } 
}

// ================================================================
// ===                MEASUREMENT SANITIZATION                  ===
// ================================================================

int getFilteredDistance()
{
  sortLastTenReadings();
  upperTukeyFence = getUpperTukeyFence();
  lowerTukeyFence = getLowerTukeyFence();
  
  for(int i = 0; i < 10; i++)
  {
    newReadings[i] = 0; 
    if((lastTenReadings[i] <= upperTukeyFence) && (lastTenReadings[i] >= lowerTukeyFence)) {
      newReadings[i] = lastTenReadings[i];
    }
  }
  return(calculateNewAverage()); 
}

double calculateNewAverage() 
{
  int amountOfReadings = 0;
  double newAverage = 0;

  for(int i = 0; i < 10; i++)
  {
    if (newReadings[i] > 0) {
      newAverage += newReadings[i];
      amountOfReadings++;
    }
    newReadings[i] = 0;
  }   
  return newAverage / amountOfReadings;
}

//sorting the array. Call this before determining tukey fences!
void sortLastTenReadings()
{
  qsort(lastTenReadings, 10, sizeof (double), compare_doubles);
}

int compare_doubles (const void *a, const void *b)
{
  const double *da = (const double *) a;
  const double *db = (const double *) b;

  return (*da > *db) - (*da < *db);
}

double getQ1()
{
  return lastTenReadings[2];
}

double getQ3()
{
  return lastTenReadings[7];
}

//determining upper and lower tukey fences
double getInterQuartileRange()
{
  return getQ3() - getQ1();
}

double getUpperTukeyFence()
{
  return getQ3() + (getInterQuartileRange() * 1.5);
}

double getLowerTukeyFence()
{
  return getQ1() - (getInterQuartileRange() * 1.5);
}

// ================================================================
// ===               Ultrasonic Sensor Methods                  ===
// ================================================================

long checkDistance()
{
  delay(20);
  return(sonar.ping_cm());
}



