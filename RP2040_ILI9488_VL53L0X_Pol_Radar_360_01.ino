/*
 Adapted from the Adafruit graphicstest sketch, see original header at end of sketch.

 This sketch uses the GLCD font (font 1) only.

 Make sure all the display driver and pin connections are correct by
 editing the User_Setup.h file in the TFT_eSPI library folder.

 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 #########################################################################
*/
//#########################################################################################################
// //  User_Setup.h        : line 54: //#define ILI9488_DRIVER           //####
//   User_Setup_Select.h : line  89: #include <User_Setups/Setup63_RP2040_ILI9488.h>    
//                         line 284...: deactivate SPI-Pins ESP8266         
//   User_Setups/Setup63_RP2040_ILI9488.h : 
//                         line  17: #define SPI_0
//                         line  33: #define ILI9488_DRIVER         
//                         line 124...: 
//                                  //For the Pico use these #define lines
//                                  #ifdef SPI_0          // SPI0
//                                  #define TFT_MISO   16 //
//                                  #define TFT_MOSI   19 //
//                                  #define TFT_SCLK   18 //
//                                  #define TFT_CS     17 // Chip select control pin
//                                  #define TFT_DC     20 // Data Command control pin
//                                  #define TOUCH_CS   22 // Chip select pin (T_CS) of touch screen
//                                  #else                 // RP2040 to Arduino-Nano Adapter
//                                  #define TFT_MISO    4 //
//                                  #define TFT_MOSI    7 // 
//                                  #define TFT_SCLK    6 //
//                                  #define TFT_CS      5 // Chip select control pin
//                                  #define TFT_DC     21 // Data Command control pin
//                                  #define TOUCH_CS   22 // Chip select pin (T_CS) of touch screen
//                                  #endif
//                                  #define TFT_RST    -1 // Reset pin (could connect to Arduino RESET pin)                     
//                        line 188: #define SPI_FREQUENCY  40000000
//
//###########################################################################################################
//                                     
#include "SPI.h"
#include "TFT_eSPI.h"

#define RGB(r, g, b)  (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

//#define TFT_AQUA           0x07FF
#define TFT_CORNFLOWERBLUE 0x64BD
#define TFT_GREEN2         0x0400
#define TFT_TURQUOISE      0x471A
#define TFT_AZURE          0xF7FF
#define TFT_DARKBLUE       0x0011
//efine TFT_CYAN           0x07FF /*   0, 255, 255 */  == TFT_AQUA
#define TFT_DARKCYAN       0x03EF /*   0, 128, 128 */
#define TFT_DARKCYAN2      0x0451
#define TFT_DARKCYAN3      0x0551
#define TFT_DARKCYAN4      0x0651
#define TFT_DARKCYAN5      0x0699
#define TFT_TURQ           RGB(0,245,190)
#define TFT_BLUES          RGB(135,242,255)

TFT_eSPI tft = TFT_eSPI();        //###############################################

#include <Wire.h>
#include <VL53L0X.h>              // Pololu

const uint8_t sensorCount = 2;    // The number of sensors in your system. 

VL53L0X sensors[sensorCount];

//#define XSHUT1_PIN         0
//#define XSHUT2_PIN         1
const uint8_t xshutPins[sensorCount] = { 0, 1};    //    Gpio: 0,1
//      SDA0                 4
//      SCL0                 5  

#define SWITCH_1             8  // switch for clear-screen
//                           9  // Servo
#define ROT1A               10  // rotary encoder1 A
#define ROT1B               13  // rotary encoder1 B

/* Sweep
  by BARRAGAN <http://barraganstudio.com>
  This example code is in the public domain.

  modified 28 May 2015
  by Michael C. Miller
  modified 8 Nov 2013
  by Scott Fitzgerald

  http://arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

unsigned long total = 0;
unsigned long tn = 0;

int16_t wi_h = TFT_WIDTH / 2;

int32_t distance0,distance1,vect0,vect1;
int32_t dist0_old = 99,dist1_old=99,vect0_old=200,vect1_old,x0_old,x1_old,y0_old,y1_old;

int max_dist= 500;           //####################################################
int max_sens=2000;           // 2000 mm : VL53lL0x #################################
int pos_old;
float rad_p;

volatile uint8_t sw_1 = 0;
volatile long time_sw1 = 0;

volatile int  e1_new = 0,e1_old=99;
volatile long time_1=0;
volatile int last_state_1,curr_state_1;
 
uint16_t pos0_x[180],pos0_y[180],pos0_c[180];
uint16_t pos1_x[180],pos1_y[180],pos1_c[180];


void readSW_1() {       // switch_1  change interrupt handler
 
  if (millis() - time_sw1 < 15) return;
 
  sw_1 = 1;
  time_sw1 = millis(); 
  
}


void read_Enc1()
 {
  if (sw_1 == 1) return;                                        //##########################################
 // if (millis() - time_1 < 5) return;  // 10
  curr_state_1 = digitalRead(ROT1A); 
 // if (curr_state_1 == last_state_1)        return;

  if (curr_state_1 == 1 and digitalRead(ROT1B) == 0) e1_new++;
  
  if (curr_state_1 == 0 and digitalRead(ROT1B) == 1) e1_new--;  //###############################
  if (e1_new > 3) e1_new = 0;
  if (e1_new < 0) e1_new = 3;
//  last_state_1 = curr_state_1;
//  time_1 = millis();
  sw_1 = 1;
  
 }



void setup() {

  Serial.begin(115200);
 // while (!Serial) delay(10);
  delay(200);

  Serial.println(F("TFT_eSPI ILI9341 VL53L1X  180° Radar !"));

  pinMode(SWITCH_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SWITCH_1), readSW_1, FALLING);
 
  pinMode(ROT1A, INPUT_PULLUP);
  pinMode(ROT1B, INPUT_PULLUP);
     
  last_state_1 = digitalRead(ROT1A);  	// Read the initial state of ROT1A
  attachInterrupt(digitalPinToInterrupt(ROT1A), read_Enc1, FALLING);
  attachInterrupt(digitalPinToInterrupt(ROT1B), read_Enc1, FALLING);
 

  myservo.attach(9,350,2500);  // 800,2450  attaches the servo on GIO2 to the servo object

  tft.init();
  tft.setRotation(2);
  tn = micros();
  tft.fillScreen(TFT_BLACK);

  yield(); Serial.println(F("Benchmark                Time (microseconds)"));

  yield(); Serial.print(F("Screen fill              "));
  yield(); Serial.println(testFillScreen());
  //total+=testFillScreen();
  //delay(500);

  yield(); Serial.print(F("Text                     "));
  yield(); Serial.println(testText());
  //total+=testText();
  //delay(3000);

  yield(); Serial.print(F("Lines                    "));
  yield(); Serial.println(testLines(TFT_CYAN));
  //total+=testLines(TFT_CYAN);
  //delay(500);

  yield(); Serial.print(F("Horiz/Vert Lines         "));
  yield(); Serial.println(testFastLines(TFT_RED, TFT_BLUE));
  //total+=testFastLines(TFT_RED, TFT_BLUE);
  //delay(500);

  yield(); Serial.print(F("Rectangles (outline)     "));
  yield(); Serial.println(testRects(TFT_GREEN));
  //total+=testRects(TFT_GREEN);
  //delay(500);

  yield(); Serial.print(F("Rectangles (filled)      "));
  yield(); Serial.println(testFilledRects(TFT_YELLOW, TFT_MAGENTA));
  //total+=testFilledRects(TFT_YELLOW, TFT_MAGENTA);
  //delay(500);

  yield(); Serial.print(F("Circles (filled)         "));
  yield(); Serial.println(testFilledCircles(10, TFT_MAGENTA));
  //total+= testFilledCircles(10, TFT_MAGENTA);

  yield(); Serial.print(F("Circles (outline)        "));
  yield(); Serial.println(testCircles(10, TFT_WHITE));
  //total+=testCircles(10, TFT_WHITE);
  //delay(500);

  yield(); Serial.print(F("Triangles (outline)      "));
  yield(); Serial.println(testTriangles());
  //total+=testTriangles();
  //delay(500);

  yield(); Serial.print(F("Triangles (filled)       "));
  yield(); Serial.println(testFilledTriangles());
  //total += testFilledTriangles();
  //delay(500);

  yield(); Serial.print(F("Rounded rects (outline)  "));
  yield(); Serial.println(testRoundRects());
  //total+=testRoundRects();
  //delay(500);

  yield(); Serial.print(F("Rounded rects (filled)   "));
  yield(); Serial.println(testFilledRoundRects());
  //total+=testFilledRoundRects();
  //delay(500);

  yield(); Serial.println(F("Done!")); yield();
  //Serial.print(F("Total = ")); Serial.println(total);
  
  //yield();Serial.println(millis()-tn);


// "RADAR"  //##################################################################################

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
// Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }


// Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(10);
     
    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print(F("Failed to detect and initialize sensor: "));
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    //if (i == 1) sensors[i].setAddress(0x2A);  // 0x29 0x2A
   // if (i == 0) sensors[i].setAddress(0x2B);  // 0x29 0x2A
    sensors[i].setAddress(0x2A + i);
    uint8_t adr_x;
    adr_x = sensors[i].getAddress();
    Serial.print(F(" i ")); Serial.print(i);Serial.print(F(" adr: ")); Serial.println(adr_x);   
   

    sensors[i].startContinuous(50); //50);

  }
 

  sw_1 = 1;
    
}

void testFilledCircles() {
 // unsigned long start;
  int           i, i2,
                cx = 120 - 1,
                cy = 200 - 1;

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_BLUES); // TFT_DARKCYAN);
  tft.setTextSize(2);
  tft.setCursor(15,15);
  tft.println("  VL53L0X 360  Radar");
  tft.setCursor(173,8);
  tft.println("o");
  tft.setTextColor(TFT_CORNFLOWERBLUE); // TFT_DARKCYAN);
  tft.setCursor(10,55);  tft.println(" angle [ ]    :");
  tft.setCursor(106,51); tft.println("o");
  tft.setCursor(10,75);  tft.println("distance [mm] :");
 /*
  for (i = 120; i > 4; i -= 4) {  // 120, 116, 112, 108
    i2 = i / 2;
 //   tft.fillRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(0, i, 0));
    tft.fillCircle(cx, cy, i, tft.color565(0, i, 0));
    yield();
  }
  delay(400);
  */
//#define TFT_WIDTH  ILI9341 = 240, ILI9488 = 320
//#define TFT_HEIGHT ILI9341 = 320, ILI9488 = 480
// wi_h = TFT_WIDTH / 2; 
  tft.fillCircle( wi_h, TFT_HEIGHT - wi_h-1, wi_h - 1, TFT_DARKBLUE);  // 120, 200, 119, TFT_DARKBLUE);//TURQUOISE); //BLACK);
  tft.drawCircle( wi_h, TFT_HEIGHT - wi_h-1, wi_h,     TFT_SILVER);    // 120, 200, 120, TFT_SILVER);
  tft.drawCircle( wi_h, TFT_HEIGHT - wi_h, wi_h / 2, TFT_SILVER);      // 120, 200, 60, TFT_SILVER);
  tft.drawLine( 0, TFT_HEIGHT - wi_h, TFT_WIDTH - 1, TFT_HEIGHT - wi_h, TFT_SILVER); //0,200,239,200,TFT_SILVER);  // horizontal
  tft.drawLine( wi_h, TFT_HEIGHT - TFT_WIDTH, wi_h,  TFT_HEIGHT - 1,    TFT_SILVER); //120,80,120,319,TFT_SILVER); // vertical

  int ir, xr, yr, xz = wi_h, yz = TFT_HEIGHT - wi_h;

 
  for (ir = 0; ir <=90; ir=ir+15) {                                 
 
    rad_p = 0.01745*(float)ir;            // < 90   rad_p = 0.01745*(float)(180-pos);
    xr =  (int) (cos(rad_p) * (float)wi_h);
    yr =  (int) (sin(rad_p) * (float)wi_h);
 // Serial.print(F(" xr "));Serial.print(xr);Serial.print(F(" yr "));Serial.println(yr);
    tft.drawLine( xz, yz, wi_h + xr, yz - yr, TFT_SILVER);            // 1. quadrant
    tft.drawLine( xz, yz, wi_h - xr, yz - yr, TFT_SILVER);            // 2. quadrant
    tft.drawLine( xz, yz, wi_h - xr, yz + yr, TFT_SILVER);            // 3. quadrant
    tft.drawLine( xz, yz, wi_h + xr, yz + yr, TFT_SILVER);            // 4. quadrant 
    
  }


  tft.setTextSize(1);
  tft.setTextColor(TFT_SILVER);
  tft.setCursor(9,TFT_HEIGHT - wi_h - 9);
  tft.println("0");
  tft.setCursor(3,TFT_HEIGHT - wi_h + 5);
  tft.println("180");
  tft.setCursor(TFT_WIDTH - 25, TFT_HEIGHT - wi_h - 10);
  tft.println("180");
  tft.setCursor(TFT_WIDTH - 11, TFT_HEIGHT - wi_h +  6);
  tft.println("0");
  tft.setCursor(wi_h + 4,TFT_HEIGHT - 11);
  tft.println("90");
  tft.setCursor(wi_h + 4,TFT_HEIGHT - TFT_WIDTH + 10);
  tft.println("90");
  //tft.setTextColor(TFT_CYAN);
  tft.setCursor(wi_h - 26, TFT_HEIGHT - TFT_WIDTH + 10);
  tft.println(max_dist);     // bot
  tft.setCursor(wi_h - 26, TFT_HEIGHT - 11);
  tft.println(max_dist);
  
  tft.setTextColor(TFT_YELLOW);

  tft.setCursor(233,120);           // x
  tft.println("x:"); 
  tft.setCursor(233,138);           // y
  tft.println("y:"); 

  tft.setTextSize(2);

}

void readDistance(int pos) {
  int x0,x1,y0,y1,s_limit0,s_limit1;

  if (pos <= 90) rad_p = 0.01745*(float)pos;  
  else           rad_p = 0.01745*(float)(180-pos);

start_dist0:
 
    // new measurement for the taking!
    distance0 = sensors[0].readRangeContinuousMillimeters();   // Sensor 0  ################## Pololu
    
    if (sensors[0].timeoutOccurred()) { 
      Serial.print(" TIMEOUT Sensor 0 "); 
   
      goto start_dist0;//  return;
    }
    s_limit0 = 0;                                            
    if (distance0 > max_dist) {
      s_limit0 = 1;
      distance0 = max_dist;  // max_sens ??
    } 
    vect0 = map(distance0,0,max_dist,0,wi_h); //,200,319);     // Sensor 0

 start_dist1:  

  distance1 = sensors[1].readRangeContinuousMillimeters();    //  Sensor 1    ################## Pololu
  if (sensors[1].timeoutOccurred()) {  
    Serial.print(" TIMEOUT Sensor 1 "); 
    goto start_dist1;//  return;
  }
  s_limit1 = 0;                                          
  if (distance1 > max_dist) {
    s_limit1 = 1;
    distance1 = max_dist;  // max_sens ??
  } 
  vect1 = map(distance1,0,max_dist,0,wi_h);                     // Sensor 1
 


  y0 = (int) (sin(rad_p) * (float) vect0);                     // Sensor 0
  x0 = (int) (cos(rad_p) * (float) vect0);
  y0 = TFT_HEIGHT - wi_h + y0;
  if (pos < 90) x0 = wi_h - x0;
  else          x0 = wi_h + x0;
   // Serial.print(F("pos: ")); Serial.print(pos); Serial.print(F(" rad: ")); Serial.println(rad_p);
    if (pos0_y[pos] == 0); 
    else tft.drawLine(wi_h , TFT_HEIGHT - wi_h, pos0_x[pos], pos0_y[pos], TFT_DARKBLUE); //120,200,pos0_x[pos], pos0_y[pos], TFT_DARKBLUE); //pos0_c[pos]); // erase old
// tft.drawPixel(pos0_x[pos], pos0_y[pos], pos0_c[pos]);        // erase old

  uint16_t z_col = TFT_CYAN;     // = TFT_AQUA N##########################################################################
 // if (s_limit0 == 1) z_col = TFT_DARKBLUE; //BLACK; //MAGENTA;
                            
  if (s_limit0 == 0) tft.drawLine( wi_h, TFT_HEIGHT - wi_h, x0, y0, z_col); //tft.drawLine(120,200,x0, y0, z_col);    // draw  new
 // tft.drawPixel(x0,y0,z_col);                                  // draw  new
 
  if (s_limit0 == 0) {
    pos0_x[pos] = x0;
    pos0_y[pos] = y0;
  }  
  else {
    pos0_x[pos] = 0; 
    pos0_y[pos] = 0; 
  } 
  pos0_c[pos] = tft.color565(0,vect0, 0);

  
  y1 = (int) (sin(rad_p) * (float) vect1);                     // Sensor 1
  x1 = (int) (cos(rad_p) * (float) vect1);
  y1 = TFT_HEIGHT - wi_h - y1; 
  if (pos < 90) x1 = wi_h + x1;
  else          x1 = wi_h - x1;

  //  Serial.print(F("pos: ")); Serial.print(pos); Serial.print(F(" rad: ")); Serial.print(rad_p);
  //  Serial.print(F(" x1: ")); Serial.print(x1); Serial.print(F(" y1: ")); Serial.println(y1);
  if (pos1_y[pos] == 0); 
  else tft.drawLine( wi_h, TFT_HEIGHT - wi_h, pos1_x[pos], pos1_y[pos], TFT_DARKBLUE); //tft.drawLine(120,200,pos1_x[pos], pos1_y[pos], TFT_DARKBLUE); //pos1_c[pos]); // erase old          
 // tft.drawPixel( pos1_x[pos], pos1_y[pos], pos1_c[pos]);      // erase old
  z_col = TFT_TURQ;             // #####################################################    
 // z_col = TFT_DARKCYAN3;      //      0x0551
 // z_col = TFT_DARKCYAN2;      //      0x0451
  //z_col = TFT_CORNFLOWERBLUE;  // 0x64BDDARKCYAN;
  //if (s_limit1 == 1) z_col = TFT_DARKBLUE;// BLACK; //MAGENTA;

  
  if (s_limit1 == 0) tft.drawLine( wi_h, TFT_HEIGHT - wi_h, x1, y1, z_col); // 120,200,x1, y1, z_col);     // draw  new
 // tft.drawPixel(x1,y1,z_col);                                  // draw  new
 
  if (s_limit1 == 0)  {
    pos1_x[pos] = x1;
    pos1_y[pos] = y1;
  }  
  else {
    pos1_x[pos] = 0; 
    pos1_y[pos] = 0; 
  } 
  pos1_c[pos] = tft.color565(0,vect1, 0);

      tft.setCursor(190,55);           // angle: 0...180°
      tft.setTextColor(TFT_BLACK);
      tft.println(pos_old);
      tft.setCursor(190,55);
      tft.setTextColor(TFT_YELLOW);
      tft.println(pos);
      pos_old = pos;

      tft.setCursor(190,75);           // distance1
      tft.setTextColor(TFT_BLACK);
      tft.println(dist1_old);
      tft.setCursor(190,75);
      tft.setTextColor(TFT_YELLOW);
      tft.println(distance1);
      dist1_old = distance1;

      tft.setTextSize(1);

      

      tft.setCursor(250,120);           // x0
      tft.setTextColor(TFT_BLACK);
      tft.println(x0_old); 
      tft.setCursor(250,120);
      tft.setTextColor(TFT_YELLOW);
      tft.println(x0);
      x0_old = x0;

      tft.setCursor(250,138);           // y0
      tft.setTextColor(TFT_BLACK);
      tft.println(y0_old); 
      tft.setCursor(250,138);
      tft.setTextColor(TFT_YELLOW);
      tft.println(y1);
      y0_old = y0;

      tft.setCursor(280,120);           // x1
      tft.setTextColor(TFT_BLACK);
      tft.println(x1_old); 
      tft.setCursor(280,120);
      tft.setTextColor(TFT_YELLOW);
      tft.println(x1);
      x1_old = x1;

      tft.setCursor(280,138);           // y1
      tft.setTextColor(TFT_BLACK);
      tft.println(y1_old); 
      tft.setCursor(280,138);
      tft.setTextColor(TFT_YELLOW);
      tft.println(y1);
      y1_old = y1;
        
      tft.setTextSize(2);

 
 
  
}
void loop(void) {

 change_max: 
 
  if (sw_1 == 1) {
   // Serial.print(e1_new);Serial.print(" old ");Serial.println(e1_old);
    if (e1_old != e1_new) {
   // Serial.print(e1_new);Serial.print(" old ");Serial.println(e1_old);
     
      if      (e1_new == 3) max_dist = 2000;
      else if (e1_new == 2) max_dist = 1500; 
      else if (e1_new == 1) max_dist = 1000;
      else                  max_dist =  500; 
      e1_old = e1_new;
    }   
    testFilledCircles(); 
    pos0_y[0] = 0;
    pos1_y[0] = 0;
    sw_1 = 0;
  }  

  tft.drawLine( 0, TFT_HEIGHT - wi_h, TFT_WIDTH - 1, TFT_HEIGHT - wi_h, TFT_SILVER); //0,200,239,200,TFT_SILVER);  // horizontal
  tft.drawLine( wi_h, TFT_HEIGHT - TFT_WIDTH, wi_h,  TFT_HEIGHT - 1,    TFT_SILVER); //120,80,120,319,TFT_SILVER); // vertical

  for (int pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
   
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    readDistance(pos);
    if (sw_1 == 1)  goto change_max;
  }
  tft.drawLine( 0, TFT_HEIGHT - wi_h, TFT_WIDTH - 1, TFT_HEIGHT - wi_h, TFT_SILVER); //0,200,239,200,TFT_SILVER);  // horizontal
  tft.drawLine( wi_h, TFT_HEIGHT - TFT_WIDTH, wi_h,  TFT_HEIGHT - 1,    TFT_SILVER); //120,80,120,319,TFT_SILVER); // vertical

  for (int pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    readDistance(pos);
    if (sw_1 == 1)  goto change_max;
 
  }

  
}


unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(TFT_BLACK);
  tft.fillScreen(TFT_RED);
  tft.fillScreen(TFT_GREEN);
  tft.fillScreen(TFT_BLUE);
  tft.fillScreen(TFT_BLACK);
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(TFT_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(TFT_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(TFT_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  //tft.setTextColor(TFT_GREEN,TFT_BLACK);
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(TFT_BLACK);

  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing
  yield();
  tft.fillScreen(TFT_BLACK);

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;
  yield();
  tft.fillScreen(TFT_BLACK);

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;
  yield();
  tft.fillScreen(TFT_BLACK);

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  yield();
  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(TFT_BLACK);
  start = micros();
  for (y = 0; y < h; y += 5) tft.drawFastHLine(0, y, w, color1);
  for (x = 0; x < w; x += 5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(TFT_BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for (i = 2; i < n; i += 6) {
    i2 = i / 2;
    tft.drawRect(cx - i2, cy - i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(TFT_BLACK);
  n = min(tft.width(), tft.height());
  for (i = n - 1; i > 0; i -= 6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx - i2, cy - i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx - i2, cy - i2, i, i, color2);
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(TFT_BLACK);
  start = micros();
  for (x = radius; x < w; x += r2) {
    for (y = radius; y < h; y += r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                      w = tft.width()  + radius,
                      h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for (x = 0; x < w; x += r2) {
    for (y = 0; y < h; y += r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(TFT_BLACK);
  n     = min(cx, cy);
  start = micros();
  for (i = 0; i < n; i += 5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(0, 0, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(TFT_BLACK);
  start = micros();
  for (i = min(cx, cy); i > 10; i -= 5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     tft.color565(0, i, i));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     tft.color565(i, i, 0));
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(TFT_BLACK);
  w     = min(tft.width(), tft.height());
  start = micros();
  for (i = 0; i < w; i += 6) {
    i2 = i / 2;
    tft.drawRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(TFT_BLACK);
  start = micros();
  for (i = min(tft.width(), tft.height()); i > 20; i -= 6) {
    i2 = i / 2;
    tft.fillRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(0, i, 0));
    yield();
  }

  return micros() - start;
}

/***************************************************
  Original Adafruit text:

  This is an example sketch for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

