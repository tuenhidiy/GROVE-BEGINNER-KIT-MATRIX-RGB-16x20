/*
 *********** THE 16x20 RGB LED MATRIX & GROVE BEGINNER KIT FOR ARDUINO - SEEED STUDIO *************
 * Scrolling message of sensors value on the RGB led with some funtions:
 * 1. Background color of led table: based on 3-axis acceleration sensors on the Grove Begginer Kit.
 *    For each different pose of led table, it will show a different color.
 * 
 * 2. Letter and number color: based on potentiometer on the Grove Begginer Kit.   
 *    Its color is displayed in "colorwheel", corresponding to the position of potentiometer knob.
 *    
 * 3. The letters and numbers can be shown in 4 fonts sizes
 *    - Font 3x5
 *    - Font 5x7
 *    - Font 8x8
 *    - Font 8x16
 */

#include <SPI.h>
#include <Wire.h>
#include <Seeed_BMP280.h>
#include <LIS3DHTR.h>

#include "font3x5.h"
#include "font5x7.h"
#include "font8x8.h"
#include "font8x16.h"

#define WIRE Wire

BMP280 bmp280; 
LIS3DHTR <TwoWire> accelemeter;

#define BLANK_PIN   7     // Blank Pin, PIN 7.
#define LATCH_PIN   2     // Latch Pin, PIN 2.
#define CLOCK_PIN   13    // SPI, PIN 13 - SCK.
#define DATA_PIN    11    // SPI, PIN 11 - MOSI.

// Common variables
#define myPI              3.14159265358979323846
#define myDPI             1.2732395
#define myDPI2            0.40528473
#define dist(a, b, c, d)  sqrt(double((a - c) * (a - c) + (b - d) * (b - d)))

// Potentiometer variables
#define POTPIN            0
#define LIMIT_BAND_POT    10
int POT;
int OLD_POT;

// Pressure sensor BMP280 variables
float rawpressure;
int pressure;
char Pressure[25];

// Three-Axis Acceleration sensor variables
float ax, ay, az; 
float prev_ax, prev_ay, prev_az;
#define LIMIT_BAND_ACC  0.1
unsigned long samplingtime  = 0;

//*********** RGB Matrix variables *************//

#define BAM_RESOLUTION 4    // 4 bit colour = 15 variation of R, G & B (16^3 = 4096 colours)
const  byte Size_X = 20;    // No. of Column - X axis
const  byte Size_Y = 16;    // No of Row - Y axis

byte red[4][48];
byte green[4][48];
byte blue[4][48];

// Anode low and high byte for shifting out.
byte anode[16][2]= {{B11111111, B11111110}, {B11111111, B11111101}, {B11111111, B11111011}, {B11111111, B11110111}, {B11111111, B11101111}, {B11111111, B11011111}, {B11111111, B10111111}, {B11111111, B01111111}, 
                    {B11111110, B11111111}, {B11111101, B11111111}, {B11111011, B11111111}, {B11110111, B11111111}, {B11101111, B11111111}, {B11011111, B11111111}, {B10111111, B11111111}, {B01111111, B11111111}};

int level = 0;
int row = 0;
int BAM_Bit, BAM_Counter=0; // Bit Angle Modulation variables

//*********** Colorwheel variables *************//

#define COLOUR_WHEEL_LENGTH 256

uint8_t colourR[COLOUR_WHEEL_LENGTH];
uint8_t colourG[COLOUR_WHEEL_LENGTH];
uint8_t colourB[COLOUR_WHEEL_LENGTH];

int16_t ColPos = 0;
uint16_t colourPos;
uint8_t R, G, B;

//*********** Color template *************//

struct Color
{
  unsigned char red, green, blue;
  Color(int r, int g, int b) : red(r), green(g), blue(b) {}
  Color() : red(0), green(0), blue(0) {}
};

const Color redcolor        = Color(0x0F, 0x00, 0x00);
const Color orangecolor     = Color(0x0F, 0x0F, 0x00);
const Color yellowcolor     = Color(0x0F, 0x09, 0x00);
const Color greencolor      = Color(0x00, 0x0F, 0x00);
const Color tealcolor       = Color(0x00, 0x0F, 0x04);
const Color bluecolor       = Color(0x00, 0x00, 0x0F);
const Color purplecolor     = Color(0x0F, 0x00, 0x0F);
const Color whitecolor      = Color(0x0F, 0x0F, 0x0F);
const Color blackcolor      = Color(0x00, 0x00, 0x00);

#define RED                         0x0F,0x00,0x00
#define ORANGE                      0x0F,0x04,0x00
#define YELLOW                      0x0F,0x09,0x00
#define GREEN                       0x00,0x0F,0x00
#define TEAL                        0x00,0x0F,0x04
#define BLUE                        0x00,0x00,0x0F
#define PURPLE                      0x0F,0x00,0x0F
#define WHITE                       0x0F,0x0F,0x0F
#define BLACK                       0x00,0x00,0x00

//*********** Fonts *************//

#define FONT3x5         0
#define FONT5x7         1
#define FONT8x8         2
#define FONT8x16        3


void setup()
{
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  
  noInterrupts();
  
  TCCR1A = B00000000;
  TCCR1B = B00001011;
  TIMSK1 = B00000010;
  OCR1A = 15;
  
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  
  SPI.begin();
  interrupts();
  
  accelemeter.begin(WIRE, LIS3DHTR_ADDRESS_UPDATED);
  delay(100);
  accelemeter.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
  bmp280.init();
  
  Pressure[0] = ' ';
  Pressure[1] = ' ';
  Pressure[2] = 'P';
  Pressure[3] = 'r';
  Pressure[4] = 'e';
  Pressure[5] = 's';
  Pressure[6] = 's';
  Pressure[7] = 'u';
  Pressure[8] = 'r';
  Pressure[9] = 'e';
  Pressure[10] = ':';
  
  Pressure[14] = 'k';
  Pressure[15] = 'P';
  Pressure[16] = 'a';
  Pressure[17] = ' ';
  Pressure[18] = ' ';
  Pressure[19] = ' ';
  Pressure[20] = ' ';
  Pressure[21] = ' ';
  Pressure[22] = ' ';
  Pressure[23] = ' ';
  Pressure[24] = '\0';
  
  fill_colour_wheel();
  clearfast();
}

void loop()
{  

  hScrollPotentio(0, "  GROVE BEGINNER KIT FOR ARDUINO       ", FONT8x16, 10, 1, 1);
  hScrollPotentio(0, "  WWW.SEEEDSTUDIO.COM       ", FONT8x16, 10, 1, 1);
  // Read the pressure value
  GetPressure();
  // Scrolling pressure value on the RGB led 16x20
  hScrollPotentio(0, Pressure, FONT8x16, 10, 1, 1);
  hScrollPotentio(0, "  Please SUBSCRIBE to TUENHIDIY YouTube channel. THANKS!!!       ", FONT8x16, 10, 1, 1);

}
 
void LED(int X, int Y, int R, int G, int B)
{
  X = constrain(X, 0, 19); 
  Y = constrain(Y, 0, 15);
  
  R = constrain(R, 0, 15);
  G = constrain(G, 0, 15); 
  B = constrain(B, 0, 15);

  int WhichByte = int(Y*3+ X/8);
  int WhichBit = 7-(X%8);

  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(green[BAM][WhichByte], 7 - WhichBit, bitRead(G, BAM));
    
    bitWrite(red[BAM][WhichByte], WhichBit, bitRead(R, BAM));

    bitWrite(blue[BAM][WhichByte], WhichBit, bitRead(B, BAM));
  }
}

ISR(TIMER1_COMPA_vect)
{  
PORTD |= ((1<<BLANK_PIN));

if(BAM_Counter==8)
BAM_Bit++;
else
if(BAM_Counter==24)
BAM_Bit++;
else
if(BAM_Counter==56)
BAM_Bit++;

BAM_Counter++;

// Anode scanning

SPI.transfer(anode[row][1]);    // Send out the anode level high byte
SPI.transfer(anode[row][0]);    // Send out the anode level low byte  

switch (BAM_Bit)
{
    case 0:              
        SPI.transfer(green[0][level+2]);
        SPI.transfer(green[0][level+1]);
        SPI.transfer(green[0][level+0]);
        
        SPI.transfer(red[0][level+2]);
        SPI.transfer(red[0][level+1]);
        SPI.transfer(red[0][level+0]);
                
        SPI.transfer(blue[0][level+2]);     
        SPI.transfer(blue[0][level+1]);
        SPI.transfer(blue[0][level+0]);
                                


      break;
    case 1:          
        SPI.transfer(green[1][level+2]);
        SPI.transfer(green[1][level+1]);
        SPI.transfer(green[1][level+0]);
        
        SPI.transfer(red[1][level+2]);
        SPI.transfer(red[1][level+1]);
        SPI.transfer(red[1][level+0]);
        
        SPI.transfer(blue[1][level+2]);     
        SPI.transfer(blue[1][level+1]);
        SPI.transfer(blue[1][level+0]);
                                            
      break;
    case 2:
    
        SPI.transfer(green[2][level+2]);
        SPI.transfer(green[2][level+1]);
        SPI.transfer(green[2][level+0]);
        
        SPI.transfer(red[2][level+2]);
        SPI.transfer(red[2][level+1]);
        SPI.transfer(red[2][level+0]);
                        
        SPI.transfer(blue[2][level+2]);     
        SPI.transfer(blue[2][level+1]);
        SPI.transfer(blue[2][level+0]);
                      
      break;
    case 3:
    
        SPI.transfer(green[3][level+2]);
        SPI.transfer(green[3][level+1]);
        SPI.transfer(green[3][level+0]);  
        
        SPI.transfer(red[3][level+2]);
        SPI.transfer(red[3][level+1]);
        SPI.transfer(red[3][level+0]);   
              
        SPI.transfer(blue[3][level+2]);     
        SPI.transfer(blue[3][level+1]);
        SPI.transfer(blue[3][level+0]);
                               
  if(BAM_Counter==120){
  BAM_Counter=0;
  BAM_Bit=0;
  }
  break;
}
  PORTD &= ~(1<<LATCH_PIN);
  PORTD |= 1<<LATCH_PIN;
  PORTD &= ~(1<<BLANK_PIN);
  row++;
  level = row * 3;
  if (row == 16) row = 0;
  if (level == 48) level = 0; 
  pinMode(BLANK_PIN, OUTPUT);
}

void clearfast ()
{
  memset(green, 0, sizeof(green[0][0]) * BAM_RESOLUTION * 48);
  memset(red, 0, sizeof(red[0][0]) * BAM_RESOLUTION * 48);
  memset(blue, 0, sizeof(blue[0][0]) * BAM_RESOLUTION * 48);
}


//*******************************************************COLORWHEEL*****************************************************//

//FAST SINE APPROX
float mySin(float x){
  float sinr = 0;
  uint8_t g = 0;

  while(x > myPI){
    x -= 2*myPI; 
    g = 1;
  }

  while(!g&(x < -myPI)){
    x += 2*myPI;
  }

  sinr = myDPI*x - myDPI2*x*myAbs(x);
  sinr = 0.225*(sinr*myAbs(sinr)-sinr)+sinr;

  return sinr;
}

//FAST COSINE APPROX
float myCos(float x){
  return mySin(x+myPI/2);
}

float myTan(float x){
  return mySin(x)/myCos(x);
}

//SQUARE ROOT APPROX
float mySqrt(float in){
  int16_t d = 0;
  int16_t in_ = in;
  float result = 2;
  
  for(d = 0; in_ > 0; in_ >>= 1){
    d++;
  }
  
  for(int16_t i = 0; i < d/2; i++){
    result = result*2;
  }
  
  for(int16_t i = 0; i < 3; i++){
    result = 0.5*(in/result + result);
  }
  
  return result;
}

//MAP NUMBERS TO NEW RANGE
float myMap(float in, float inMin, float inMax, float outMin, float outMax){
  float out;
  out = (in-inMin)/(inMax-inMin)*(outMax-outMin) + outMin;
  return out;
}

//ROUND A NUMBER
int16_t myRound(float in){
  int8_t s = in/myAbs(in);
  return (int16_t)(s*(myAbs(in) + 0.5));
}

//ABSOLUTE VALUE
float myAbs(float in){
  return (in)>0?(in):-(in);
} 

void fill_colour_wheel(void) 
{
  float red, green, blue;
  float c, s;
  int32_t phase = 0;
  int16_t I = 0;

  while (phase < COLOUR_WHEEL_LENGTH) 
  {
    s = (1 << BAM_RESOLUTION)*mySin(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));
    c = (1 << BAM_RESOLUTION)*myCos(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));

    red = (I == 0 ? 1 : 0)*s + (I == 1 ? 1 : 0)*c;
    green = (I == 1 ? 1 : 0)*s + (I == 2 ? 1 : 0)*c;
    blue = (I == 2 ? 1 : 0)*s + (I == 0 ? 1 : 0)*c;

    colourR[phase] = red;
    colourG[phase] = green;
    colourB[phase] = blue;

    if (++phase >= (1 + I)*COLOUR_WHEEL_LENGTH / 3) 
      I++;
  }
}

void get_colour(int16_t p, uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (p >= COLOUR_WHEEL_LENGTH)
    p -= COLOUR_WHEEL_LENGTH;

  *R = colourR[p];
  *G = colourG[p];
  *B = colourB[p];
}

void get_next_colour(uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (++ColPos >= COLOUR_WHEEL_LENGTH)
    ColPos -= COLOUR_WHEEL_LENGTH;

  *R = colourR[ColPos];
  *G = colourG[ColPos];
  *B = colourB[ColPos];
}

void increment_colour_pos(uint8_t i)
{
  colourPos += i;
  while (colourPos >= COLOUR_WHEEL_LENGTH)
  {
    colourPos -= COLOUR_WHEEL_LENGTH;
  }
}


byte getPixelChar(uint8_t x, uint8_t y, char ch, uint8_t font)
{
  if (font==FONT3x5)
  {
    if (x > 2) return 0;
    return bitRead(pgm_read_byte(&font3x5[ch-32][y]),2-x); 
  }  
  else if (font==FONT5x7)
  {
    if (x > 4) return 0;
    return bitRead(pgm_read_byte(&font5x7[ch-32][y]), 4-x); 
  } 
  else if (font==FONT8x8)
  {
    if (x > 7) return 0;
    return bitRead(pgm_read_byte(&font8x8[ch-32][y]),7-x); 
  }
  else if (font==FONT8x16)
  {
    if (x > 7) return 0;
    return bitRead(pgm_read_byte(&font8x16[ch-32][y]),7-x); 
  }
  
}

byte getPixelHString(uint16_t x, uint16_t y, char *p,uint8_t font)
{
  if (font==FONT3x5)
  {
    p=p+x/4;
    return getPixelChar(x%4,y,*p,FONT3x5);
  }
  
  else if (font==FONT5x7)
  {
    p=p+x/6;
    return getPixelChar(x%6,y,*p,FONT5x7);
  }
  else if (font==FONT8x8)
  {
    p=p+x/8;
    return getPixelChar(x%8,y,*p,FONT8x8);  
  }

  else if (font==FONT8x16)
  {
    p=p+x/9;
    return getPixelChar(x%9,y,*p,FONT8x16); 
  }
}

unsigned int lenString(char *p)
{
  unsigned int retVal=0;
  while(*p!='\0')
  { 
   retVal++;
   p++;
  }
  return retVal;
}


void hScrollPotentio(uint8_t y, char *mystring, uint8_t font, uint8_t delaytime, uint8_t times, uint8_t dir)
{
  //int offset =0;
  // FONT 5x7
  int offset;
  Color setcolor, For_color, Bk_color;
  if (font == FONT3x5)
  {
  while (times)
    {
    GetAcceleration();
    get_colour(colourPos , &Bk_color.red, &Bk_color.green, &Bk_color.blue);  
    for ((dir) ? offset=0 : offset=((lenString(mystring)-4)*4-1) ; (dir) ? offset <((lenString(mystring)-4)*4-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<20; xx++)
        {
        for (byte yy=0; yy<5; yy++)
            {            
              readPotentio();
              get_colour(OLD_POT + 4*yy + 4*xx, &For_color.red, &For_color.green, &For_color.blue);
              if (getPixelHString(xx+offset, yy, mystring, FONT3x5))
              {
                setcolor = For_color;                
              }
              else 
              {
                setcolor = Bk_color;
              }
                LED(xx,(yy+y),setcolor.red, setcolor.green, setcolor.blue);
            }
        }
        delay(delaytime); 
      }
    times--;
    }
  }

////////////////////////////////////////////////////////////////
  
  else if (font == FONT5x7)
  {
  while (times)
    {
    GetAcceleration();
    get_colour(colourPos , &Bk_color.red, &Bk_color.green, &Bk_color.blue);  
    for ((dir) ? offset=0 : offset=((lenString(mystring)-5)*6-1) ; (dir) ? offset <((lenString(mystring)-5)*6-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<20; xx++)
        {
        for (byte yy=0; yy<7; yy++)
            {            
              readPotentio();
              get_colour(OLD_POT + 4*yy + 4*xx, &For_color.red, &For_color.green, &For_color.blue);
              if (getPixelHString(xx+offset,yy,mystring,FONT5x7))
              {
                setcolor = For_color;                
              }
              else 
              {
                setcolor = Bk_color;
              }
                LED(xx,(yy+y), setcolor.red, setcolor.green, setcolor.blue);
            }
        }
        delay(delaytime);   
      }
    times--;
    }
  }
//////////////////////////////////////////// 

// FONT 8x8
  else if (font == FONT8x8)
    {
    while (times)
      {
      GetAcceleration();
      get_colour(colourPos , &Bk_color.red, &Bk_color.green, &Bk_color.blue);  
      for ((dir) ? offset=0 : offset=((lenString(mystring)-6)*8-1); (dir) ? offset <((lenString(mystring)-6)*8-1): offset >0; (dir) ? offset++ : offset--)
        {
        for (byte xx=0; xx<20; xx++)
          {
            for (byte yy=0; yy<8; yy++)
              {
                readPotentio();
                get_colour(OLD_POT + 4*yy + 4*xx, &For_color.red, &For_color.green, &For_color.blue);
                if (getPixelHString(xx+offset, yy, mystring, FONT8x8)) 
                  {
                  setcolor = For_color;
                  }
                else 
                {
                  setcolor = Bk_color;
                }
                  LED(xx,(yy+y), setcolor.red, setcolor.green, setcolor.blue);
              }          
            }
          delay(delaytime);
        }
      times--;
      }
    }

// FONT 8x16  
   else if (font == FONT8x16)
    {
    while (times)
    {
      GetAcceleration();
      get_colour(colourPos , &Bk_color.red, &Bk_color.green, &Bk_color.blue);  
    for ((dir) ? offset=0 : offset=((lenString(mystring)-6)*9-1); (dir) ? offset <((lenString(mystring)-6)*9-1) : offset >0; (dir) ? offset++ : offset--)
      {      
      for (byte xx=0; xx<20; xx++)
        {     
            for (byte yy=0; yy<16; yy++)
              {
                readPotentio();
                get_colour(OLD_POT + 4*yy + 4*xx, &For_color.red, &For_color.green, &For_color.blue);
                if (getPixelHString(xx+offset, yy, mystring, FONT8x16)) 
                  {
                    setcolor = For_color;
                  }
                else 
                  {
                    setcolor = Bk_color;
                  }
                LED(xx,(yy+y), setcolor.red, setcolor.green, setcolor.blue);
              }   
          }
          delay(delaytime); 
        }
        times--;
      } 
    }
 }

void readPotentio()
{
  POT = map(analogRead(POTPIN), 0, 1023, 0, 255);
  // Save the potentiometer value if its change is bigger than a presetedlimit value, eg: 10.  
  if (abs(POT - OLD_POT) > LIMIT_BAND_POT)
  {        
    OLD_POT = POT;           
  }
}

void GetAcceleration()
{
  ax = accelemeter.getAccelerationX();
  ay = accelemeter.getAccelerationY();
  az = accelemeter.getAccelerationZ();    
  
  // Save the values if their changes are bigger than the preseted limit value, eg: 0.1.
  if (abs(ax - prev_ax) > LIMIT_BAND_ACC)
    {   
      increment_colour_pos(colourPos + (uint16_t)abs(ax*16));
      prev_ax = ax;
    }    
  if (abs(ay - prev_ay) > LIMIT_BAND_ACC)
    {   
      increment_colour_pos(colourPos + (uint16_t)abs(ay*16));
      prev_ay = ay;
    }
  if (abs(az - prev_az) > LIMIT_BAND_ACC)
    {   
      increment_colour_pos(colourPos + (uint16_t)abs(az*16));
      prev_az = az;
    } 
}
void GetPressure()
{
  rawpressure = bmp280.getPressure();
  // Convert to kPa
  pressure = rawpressure/1000;
  
  Pressure[11] = ((pressure/ 100) % 10) + 48;
  Pressure[12] = ((pressure/10) %10) + 48;
  Pressure[13] = (pressure%10) + 48;
}
