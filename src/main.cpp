#include <Arduino.h>
#include <SPI.h>

// ================================================================
// ===               ROLE DEFINE DECLARATIONS                   ===
// ================================================================
//Uncommente only the relative role define
//#define RF_DATASENDER
#define RF_DATALOGGER

#ifdef RF_DATASENDER
    #include "I2Cdev.h"
    #include "MPU6050_6Axis_MotionApps20.h"
#endif

#ifdef RF_DATALOGGER
    #include <U8g2lib.h>
#endif

#include "Wire.h"

#include <nRF24L01.h>
#include <RF24.h>

// ================================================================
// ===               GENERAL DEFINE DECLARATIONS                ===
// ================================================================
#define INTERRUPT_PIN 34
#define LED_PIN 2

#ifdef RF_DATASENDER
    #define M_PI		3.14159265358979323846

    MPU6050 mpu;
#endif

#ifdef RF_DATALOGGER
    #include <Adafruit_GFX.h>
    #include <Adafruit_SSD1306.h>

    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels

    #define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
    Adafruit_SSD1306 display(-1);
#endif

TaskHandle_t IMUTaskHandle;
TaskHandle_t RFTaskHandle;
TaskHandle_t SDTaskHandle;
TaskHandle_t LCDTaskHandle;

bool blinkState = false;
bool imustarted = false;

#ifdef RF_DATASENDER
// ================================================================
// ===               MPU6050 DECLARATIONS                       ===
// ================================================================

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
float yprDeg[3];        // [yaw, pitch, roll]   Degree yaw/pitch/roll container
int32_t imuTemp;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
#endif

// ================================================================
// ===               nRF24L01 DECLARATIONS                       ===
// ================================================================
RF24 radio(4, 5); // CE pin, CSN pin

// Let these addresses be used for the pair
uint8_t address[][6] = {"Pipe1", "Pipe2"};

// ================================================================
// ===                    TASKS PROGRAM LOOP                    ===
// ================================================================
#ifdef RF_DATASENDER
void IMUTask( void * pvParameters )
{
    Serial.printf("IMUTask running on core: %d\r\n",xPortGetCoreID());

    for(;;)
    {
        if (!dmpReady)
        {
            imustarted = false;
            return;
        }
        // read a packet from FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
        {   // Get the Latest packet 
            // display Euler angles in degrees
            imustarted = true;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yprDeg[0] = ypr[0] * 180/M_PI;
            yprDeg[1] = ypr[1] * 180/M_PI;
            yprDeg[2] = ypr[2] * 180/M_PI;
            /*
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            */
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
#endif

void RFTrasmitTask( void * pvParameters )
{
    Serial.printf("RFTrasmitTask running on core: %d\r\n",xPortGetCoreID());
    char rfDataBuffer[18];

    for(;;)
    {
        #ifdef RF_DATASENDER
        if (imustarted==true)
        {
            Serial.print("RF data trasmition\r\n");
            radio.stopListening();
            sprintf(rfDataBuffer,"%.2f,%.2f,%.2f\r\n",yprDeg[0],yprDeg[1],yprDeg[2]);
            Serial.printf("rfDataBuffer length: %d\r\n",sizeof(rfDataBuffer));
            radio.write(&rfDataBuffer, sizeof(rfDataBuffer));

            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
        }
        else
        {
            Serial.print("Waiting data to trasmit\r\n");
        }
        #endif

        #ifdef RF_DATALOGGER
        Serial.print("RF data receiving\r\n");
        radio.startListening();
        while(radio.available())
        {
            radio.read(&rfDataBuffer, sizeof(rfDataBuffer));
            Serial.printf("Data received: %s\r\n",rfDataBuffer);
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
        }
        #endif
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void SDTask( void * pvParameters )
{
    Serial.printf("SDTask running on core: %d\r\n",xPortGetCoreID());

    for(;;)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void LcdTask( void * pvParameters )
{
    Serial.printf("LcdTask running on core: %d\r\n",xPortGetCoreID());

    for(;;)
    {
        /*
        uint16_t fps;
        fps = execute_with_fps(draw_clip_test);
        show_result("draw clip test", fps);
        delay(5000);
        fps = execute_with_fps(draw_set_screen);
        show_result("clear screen", fps);
        delay(5000);
        fps = execute_with_fps(draw_char);
        show_result("draw @", fps);
        delay(5000);  
        fps = execute_with_fps(draw_pixel);
        show_result("draw pixel", fps);
        delay(5000);
        fps = execute_with_fps(draw_line);
        show_result("draw line", fps);
        */
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(1);
        display.setCursor(0,0);
        display.println("Rectangle");
        display.drawRect(0, 15, 60, 40, 1);
        display.display();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() 
{
    // ===                    CONFIGURE I2C BUS                     === //
    Wire.begin();
    Wire.setClock(400000);

    // ===                    CONFIGURE SERIAL COM                     === //
    Serial.begin(115200);
    while (!Serial);

    // ===                    CONFIGURE IMU                     === //
    #ifdef RF_DATASENDER
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);  
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed")); 
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();    
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-72);
    mpu.setYGyroOffset(171);
    mpu.setZGyroOffset(393);
    mpu.setZAccelOffset(1400);  
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);    
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();  
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;    
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else 
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }   
    #endif

    // ===                    CONFIGURE RF LINK                     === //
    radio.begin();
    #ifdef RF_DATASENDER
        radio.openWritingPipe(address[0]);
        radio.openReadingPipe(0, address[1]);
    #endif
    #ifdef RF_DATALOGGER
        radio.openWritingPipe(address[1]);
        radio.openReadingPipe(0, address[0]);
    #endif
    radio.setPALevel(RF24_PA_MIN);

    // ===                    CONFIGURE IO                     === //
    pinMode(LED_PIN, OUTPUT);
    
    // ===                    CONFIGURE OLED                     === //
    #ifdef RF_DATALOGGER
        display.begin(0x02, 0x3C);
    #endif

    // ===                    CONFIGURE TASKS                     === //
    #ifdef RF_DATASENDER
        xTaskCreatePinnedToCore(
                        IMUTask,   /* Task function. */
                        "MPU6050 Reading Task",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        24,           /* priority of the task */
                        &IMUTaskHandle,      /* Task handle to keep track of created task */
                        0);          /* pin task to core 0 */ 
    #endif

    #ifdef RF_DATALOGGER
        xTaskCreatePinnedToCore(
                        LcdTask,   /* Task function. */
                        "LCD Manager",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        2,           /* priority of the task */
                        &LCDTaskHandle,      /* Task handle to keep track of created task */
                        0);          /* pin task to core 0 */ 
        xTaskCreatePinnedToCore(
                        SDTask,   /* Task function. */
                        "SD data store task",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        3,           /* priority of the task */
                        &SDTaskHandle,      /* Task handle to keep track of created task */
                        0);          /* pin task to core 0 */ 
    #endif
    
    xTaskCreatePinnedToCore(
                    RFTrasmitTask,   /* Task function. */
                    "RF trasmission manager task",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    24,           /* priority of the task */
                    &RFTaskHandle,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */ 
}

void loop() 
{
    ;
}