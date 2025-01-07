// #include <Arduino.h>

// // void setup(){Serial.begin(9600);}
// // void loop() {Serial.println("test");}

// #include "I2Cdev.h"
// #include "MPU6050.h"
// #include "Wire.h"

// // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// // is used in I2Cdev.h
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//     #include "Wire.h"
// #endif

// // class default I2C address is 0x68
// // specific I2C addresses may be passed as a parameter here
// // AD0 low = 0x68 (default for InvenSense evaluation board)
// // AD0 high = 0x69
// // TwoWire I2C_IMU((uint8_t)PB9, (uint8_t)PB8);

// // MPU6050 accelgyro((uint8_t)0x68, I2C_IMU);
// MPU6050 accelgyro(0x68); // <-- use for AD0 high


// const char LBRACKET = '[';
// const char RBRACKET = ']';
// const char COMMA    = ',';
// const char BLANK    = ' ';
// const char PERIOD   = '.';

// const int iAx = 0;
// const int iAy = 1;
// const int iAz = 2;
// const int iGx = 3;
// const int iGy = 4;
// const int iGz = 5;

// const int usDelay = 3150;   // empirical, to hold sampling to 200 Hz
// const int NFast =  1000;    // the bigger, the better (but slower)
// const int NSlow = 10000;    // ..
// const int LinesBetweenHeaders = 5;
//       int LowValue[6];
//       int HighValue[6];
//       int Smoothed[6];
//       int LowOffset[6];
//       int HighOffset[6];
//       int Target[6];
//       int LinesOut;
//       int N;
      
// void ForceHeader()
//   { LinesOut = 99; }
    
// void GetSmoothed()
//   { int16_t RawValue[6];
//     int i;
//     long Sums[6];
//     for (i = iAx; i <= iGz; i++)
//       { Sums[i] = 0; }
// //    unsigned long Start = micros();

//     for (i = 1; i <= N; i++)
//       { // get sums
//         accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz], 
//                              &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
//         if ((i % 500) == 0)
//           Serial.print(PERIOD);
//         delayMicroseconds(usDelay);
//         for (int j = iAx; j <= iGz; j++)
//           Sums[j] = Sums[j] + RawValue[j];
//       } // get sums
// //    unsigned long usForN = micros() - Start;
// //    Serial.print(" reading at ");
// //    Serial.print(1000000/((usForN+N/2)/N));
// //    Serial.println(" Hz");
//     for (i = iAx; i <= iGz; i++)
//       { Smoothed[i] = (Sums[i] + N/2) / N ; }
//   } // GetSmoothed

//   void ShowProgress()
//   { if (LinesOut >= LinesBetweenHeaders)
//       { // show header
//         Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
//         LinesOut = 0;
//       } // show header
//     Serial.print(BLANK);
//     for (int i = iAx; i <= iGz; i++)
//       { Serial.print(LBRACKET);
//         Serial.print(LowOffset[i]),
//         Serial.print(COMMA);
//         Serial.print(HighOffset[i]);
//         Serial.print("] --> [");
//         Serial.print(LowValue[i]);
//         Serial.print(COMMA);
//         Serial.print(HighValue[i]);
//         if (i == iGz)
//           { Serial.println(RBRACKET); }
//         else
//           { Serial.print("]\t"); }
//       }
//     LinesOut++;
//   } // ShowProgress

//   void SetOffsets(int TheOffsets[6])
//   { accelgyro.setXAccelOffset(TheOffsets [iAx]);
//     accelgyro.setYAccelOffset(TheOffsets [iAy]);
//     accelgyro.setZAccelOffset(TheOffsets [iAz]);
//     accelgyro.setXGyroOffset (TheOffsets [iGx]);
//     accelgyro.setYGyroOffset (TheOffsets [iGy]);
//     accelgyro.setZGyroOffset (TheOffsets [iGz]);
//   } // SetOffsets

// void PullBracketsOut()
//   { boolean Done = false;
//     int NextLowOffset[6];
//     int NextHighOffset[6];

//     Serial.println("expanding:");
//     ForceHeader();
 
//     while (!Done)
//       { Done = true;
//         SetOffsets(LowOffset);
//         GetSmoothed();
//         for (int i = iAx; i <= iGz; i++)
//           { // got low values
//             LowValue[i] = Smoothed[i];
//             if (LowValue[i] >= Target[i])
//               { Done = false;
//                 NextLowOffset[i] = LowOffset[i] - 1000;
//               }
//             else
//               { NextLowOffset[i] = LowOffset[i]; }
//           } // got low values
      
//         SetOffsets(HighOffset);
//         GetSmoothed();
//         for (int i = iAx; i <= iGz; i++)
//           { // got high values
//             HighValue[i] = Smoothed[i];
//             if (HighValue[i] <= Target[i])
//               { Done = false;
//                 NextHighOffset[i] = HighOffset[i] + 1000;
//               }
//             else
//               { NextHighOffset[i] = HighOffset[i]; }
//           } // got high values
//         ShowProgress();
//         for (int i = iAx; i <= iGz; i++)
//           { LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
//             HighOffset[i] = NextHighOffset[i]; // ..
//           }
//      } // keep going
//   } // PullBracketsOut

// void SetAveraging(int NewN)
//   { N = NewN;
//     Serial.print("averaging ");
//     Serial.print(N);
//     Serial.println(" readings each time");
//    } // SetAveraging

// void Initialize()
//   {
//     // join I2C bus (I2Cdev library doesn't do this automatically)
//     #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//         Wire.begin();
//     #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//         Fastwire::setup(400, true);
//     #endif

//     Serial.begin(9600);

//     // initialize device
//     Serial.println("Initializing I2C devices...");
//     accelgyro.initialize();

//     // verify connection
//     Serial.println("Testing device connections...");
//     Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
//     Serial.println("PID tuning Each Dot = 100 readings");
//   /*A tidbit on how PID (PI actually) tuning works. 
//     When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and 
//     integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral 
//     uses the error from set-point (set-point is zero), it takes a fraction of this error (error * ki) and adds it 
//     to the integral value. Each reading narrows the error down to the desired offset. The greater the error from 
//     set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the 
//     integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the 
//     noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100 
//     readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to 
//     the fact it reacts to any noise.
//   */
//         accelgyro.CalibrateAccel(6);
//         accelgyro.CalibrateGyro(6);
//         Serial.println("\nat 600 Readings");
//         accelgyro.PrintActiveOffsets();
//         Serial.println();
//         accelgyro.CalibrateAccel(1);
//         accelgyro.CalibrateGyro(1);
//         Serial.println("700 Total Readings");
//         accelgyro.PrintActiveOffsets();
//         Serial.println();
//         accelgyro.CalibrateAccel(1);
//         accelgyro.CalibrateGyro(1);
//         Serial.println("800 Total Readings");
//         accelgyro.PrintActiveOffsets();
//         Serial.println();
//         accelgyro.CalibrateAccel(1);
//         accelgyro.CalibrateGyro(1);
//         Serial.println("900 Total Readings");
//         accelgyro.PrintActiveOffsets();
//         Serial.println();    
//         accelgyro.CalibrateAccel(1);
//         accelgyro.CalibrateGyro(1);
//         Serial.println("1000 Total Readings");
//         accelgyro.PrintActiveOffsets();
//      Serial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:"); 
//   } // Initialize




// void PullBracketsIn()
//   { boolean AllBracketsNarrow;
//     boolean StillWorking;
//     int NewOffset[6];
  
//     Serial.println("\nclosing in:");
//     AllBracketsNarrow = false;
//     ForceHeader();
//     StillWorking = true;
//     while (StillWorking) 
//       { StillWorking = false;
//         if (AllBracketsNarrow && (N == NFast))
//           { SetAveraging(NSlow); }
//         else
//           { AllBracketsNarrow = true; }// tentative
//         for (int i = iAx; i <= iGz; i++)
//           { if (HighOffset[i] <= (LowOffset[i]+1))
//               { NewOffset[i] = LowOffset[i]; }
//             else
//               { // binary search
//                 StillWorking = true;
//                 NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
//                 if (HighOffset[i] > (LowOffset[i] + 10))
//                   { AllBracketsNarrow = false; }
//               } // binary search
//           }
//         SetOffsets(NewOffset);
//         GetSmoothed();
//         for (int i = iAx; i <= iGz; i++)
//           { // closing in
//             if (Smoothed[i] > Target[i])
//               { // use lower half
//                 HighOffset[i] = NewOffset[i];
//                 HighValue[i] = Smoothed[i];
//               } // use lower half
//             else
//               { // use upper half
//                 LowOffset[i] = NewOffset[i];
//                 LowValue[i] = Smoothed[i];
//               } // use upper half
//           } // closing in
//         ShowProgress();
//       } // still working
   
//   } // PullBracketsIn


// void setup()
//   { Initialize();
//     for (int i = iAx; i <= iGz; i++)
//       { // set targets and initial guesses
//         Target[i] = 0; // must fix for ZAccel 
//         HighOffset[i] = 0;
//         LowOffset[i] = 0;
//       } // set targets and initial guesses
//     Target[iAz] = 16384;
//     SetAveraging(NFast);
    
//     PullBracketsOut();
//     PullBracketsIn();
    
//     Serial.println("-------------- done --------------");
//   } // setup
 
// void loop()
//   {
//   } // loop






// #include <Arduino.h>
// #include "Wire.h"

// // I2Cdev and HMC5883L must be installed as libraries, or else the .cpp/.h files
// // for both classes must be in the include path of your project
// #include "I2Cdev.h"
// #include "HMC5883L.h"

// // class default I2C address is 0x1E
// // specific I2C addresses may be passed as a parameter here
// // this device only supports one I2C address (0x1E)
// // // TwoWire I2C_IMU((uint8_t)PB9, (uint8_t)PB8);

// // // MPU6050 accelgyro((uint8_t)0x68, I2C_IMU);
// HMC5883L mag;

// int16_t mx, my, mz;
// TwoWire WIRE1 (PD13, PD12);
// #define LED_PIN 13
// bool blinkState = false;

// void setup() {
//     // join I2C bus (I2Cdev library doesn't do this automatically)
//     Wire.begin();

//     // initialize serial communication
//     // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
//     // it's really up to you depending on your project)
//     Serial.begin(38400);

//     // initialize device
//     Serial.println("Initializing I2C devices...");
//     mag.initialize();

//     // verify connection
//     Serial.println("Testing device connections...");
//     Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

//     // configure Arduino LED pin for output
//     pinMode(LED_PIN, OUTPUT);
// }

// void loop() {
//     // read raw heading measurements from device
//     mag.getHeading(&mx, &my, &mz);

//     // display tab-separated gyro x/y/z values
//     Serial.print("mag:\t");
//     Serial.print(mx); Serial.print("\t");
//     Serial.print(my); Serial.print("\t");
//     Serial.print(mz); Serial.print("\t");
    
// // To calculate heading in degrees. 0 degree indicates North
//     float heading = atan2(my, mx);
//     if(heading < 0)
//       heading += 2 * M_PI;
//     Serial.print("heading:\t");
//     Serial.println(heading * 180/M_PI);

//     // blink LED to indicate activity
//     blinkState = !blinkState;
//     digitalWrite(LED_PIN, blinkState);
// }


#include <Arduino.h>
#include "qmc5883l.h"
#include "imu.h"

void setup(){Serial.begin(115200); hmc5883l_init(); imu_init(false);}
void loop (){hmc5883l_GetData_calibed();hmc5883l_computeHeading();imu_update();
    // Serial.print("mx : \t");
    // Serial.print(mx);
    // Serial.print("\tmy : \t");
    // Serial.print(my);
    // Serial.print("\tmz : \t");
    // Serial.print(mz);
    // Serial.print("\theading : \t");
    // heading = atan2(my, mx);
    // if(heading < 0)
    //   heading += 2 * M_PI;
    // Serial.print(heading);
    // Serial.print("\ttrue_heading : \t");
    // Serial.println(trueHeading);
    Serial.print("r : \t");
    Serial.print(roll);
    Serial.print("\tp : \t");
    Serial.print(pitch);
    Serial.print("\ty : \t");
    Serial.print(yaw);
    Serial.print("\ttrue_heading : \t");
    Serial.println(trueHeading);
}


// #include <Wire.h>
// #define I2C_SDA PB9

// //use IIC2
// // #define I2C_SCL PB8
// TwoWire WIRE1 (PB7, PB6);
// // TwoWire WIRE1 (PD13, PD12);

// #define Wire WIRE1


// void setup() {
//   delay(5000*2);
//   Serial.begin(115200);
 
//   Wire.begin();
//   Serial.println("\nI2C Scanner");
// }


// void loop() {
//   byte error, address;
//   int nDevices;

//   Serial.println("Scanning...");

//   nDevices = 0;
//   for(address = 1; address < 127; address++) {
//     // The i2c_scanner uses the return value of
//     // the Write.endTransmisstion to see if
//     // a device did acknowledge to the address.

//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();
    
//     if (error == 0) {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16) 
//         Serial.print("0");
//       Serial.println(address, HEX);

//       nDevices++;
//     }
//     else if (error == 4) {
//       Serial.print("Unknown error at address 0x");
//       if (address < 16) 
//         Serial.print("0");
//       Serial.println(address, HEX);
//     }    
//   }
//   if (nDevices == 0)
//     Serial.println("No I2C devices found");
//   else
//     Serial.println("done");

//   delay(5000);           // wait 5 seconds for next scan
// }