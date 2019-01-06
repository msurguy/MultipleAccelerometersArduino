// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 mpu;
MPU6050 refmpu(0x68);
MPU6050 tipmpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

#define REFINTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define TIPINTERRUPT_PIN 3  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool refdmpReady = false;  // set true if DMP init was successful
bool tipdmpReady = false;  // set true if DMP init was successful

uint8_t refmpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t tipmpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t refdevStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t tipdevStatus;      // return status after each device operation (0 = success, !0 = error)

uint16_t refpacketSize;    // expected DMP packet size (default is 42 bytes)
uint16_t reffifoCount;     // count of all bytes currently in FIFO
uint8_t reffifoBuffer[64]; // FIFO storage buffer

uint16_t tippacketSize;    // expected DMP packet size (default is 42 bytes)
uint16_t tipfifoCount;     // count of all bytes currently in FIFO
uint8_t tipfifoBuffer[64]; // FIFO storage buffer


// orientation/motion vars
Quaternion refq;           // [w, x, y, z]         quaternion container
VectorInt16 refaa;         // [x, y, z]            accel sensor measurements
VectorInt16 refaaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 refaaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat refgravity;    // [x, y, z]            gravity vector

Quaternion tipq;           // [w, x, y, z]         quaternion container
VectorInt16 tipaa;         // [x, y, z]            accel sensor measurements
VectorInt16 tipaaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 tipaaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat tipgravity;    // [x, y, z]            gravity vector

float refeuler[3];         // [psi, theta, phi]    Euler angle container
float tipeuler[3];         // [psi, theta, phi]    Euler angle container
float refypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float tipypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool refmpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void refdmpDataReady() {
    refmpuInterrupt = true;
}

volatile bool tipmpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void tipdmpDataReady() {
    tipmpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    refmpu.initialize();
    tipmpu.initialize();

    pinMode(REFINTERRUPT_PIN, INPUT);
    pinMode(TIPINTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(refmpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    refdevStatus = refmpu.dmpInitialize();
    tipdevStatus = tipmpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    refmpu.setXGyroOffset(220);
    refmpu.setYGyroOffset(76);
    refmpu.setZGyroOffset(-85);
    refmpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // supply your own gyro offsets here, scaled for min sensitivity
    tipmpu.setXGyroOffset(220);
    tipmpu.setYGyroOffset(76);
    tipmpu.setZGyroOffset(-85);
    tipmpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (refdevStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling REF DMP..."));
        refmpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(REFINTERRUPT_PIN), refdmpDataReady, RISING);
        refmpuIntStatus = refmpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        refdmpReady = true;

        // get expected DMP packet size for later comparison
        refpacketSize = refmpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(refdevStatus);
        Serial.println(F(")"));
    }

    // make sure it worked (returns 0 if so)
    if (tipdevStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling TIP DMP..."));
        tipmpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(TIPINTERRUPT_PIN), tipdmpDataReady, RISING);
        tipmpuIntStatus = tipmpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        tipdmpReady = true;

        // get expected DMP packet size for later comparison
        tippacketSize = tipmpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(tipdevStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!refdmpReady) return;
    if (!tipdmpReady) return;


    // wait for MPU interrupt or extra packet(s) available
    while (!refmpuInterrupt && reffifoCount < refpacketSize) {}
    while (!tipmpuInterrupt && tipfifoCount < tippacketSize) {}

    // reset interrupt flag and get INT_STATUS byte
    refmpuInterrupt = false;
    tipmpuInterrupt = false;

    refmpuIntStatus = refmpu.getIntStatus();
    tipmpuIntStatus = tipmpu.getIntStatus();

    // get current FIFO count
    reffifoCount = refmpu.getFIFOCount();
    tipfifoCount = refmpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((refmpuIntStatus & 0x10) || reffifoCount == 1024) {
        // reset so we can continue cleanly
        refmpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (refmpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (reffifoCount < refpacketSize) reffifoCount = refmpu.getFIFOCount();

        // read a packet from FIFO
        refmpu.getFIFOBytes(reffifoBuffer, refpacketSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        reffifoCount -= refpacketSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            refmpu.dmpGetQuaternion(&refq, reffifoBuffer);
            refmpu.dmpGetGravity(&refgravity, &refq);
            refmpu.dmpGetYawPitchRoll(refypr, &refq, &refgravity);
            Serial.print("ref ypr\t");
            Serial.print(refypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(refypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(refypr[2] * 180/M_PI);
            Serial.print("\t");

        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }


    // check for overflow (this should never happen unless our code is too inefficient)
    if ((tipmpuIntStatus & 0x10) || tipfifoCount == 1024) {
        // reset so we can continue cleanly
        tipmpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (tipmpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (tipfifoCount < tippacketSize) tipfifoCount = tipmpu.getFIFOCount();

        // read a packet from FIFO
        tipmpu.getFIFOBytes(tipfifoBuffer, tippacketSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        tipfifoCount -= tippacketSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            tipmpu.dmpGetQuaternion(&tipq, tipfifoBuffer);
            tipmpu.dmpGetGravity(&tipgravity, &tipq);
            tipmpu.dmpGetYawPitchRoll(tipypr, &tipq, &tipgravity);
            Serial.print("tip ypr\t");
            Serial.print(tipypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(tipypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(tipypr[2] * 180/M_PI);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
