/*************************************************************************
* File Name          : mbot_firmware.ino
* Author             : Ander, Mark Yan
* Updated            : Ander, Mark Yan
* Version            : V06.01.106
* Date               : 07/06/2016
* Description        : Firmware for Makeblock Electronic modules with Scratch.  
* License            : CC-BY-SA 3.0
* Copyright (C) 2013 - 2016 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
**************************************************************************/
#include <Wire.h>
#include <MeMCore.h>

Servo servos[8];  
MeDCMotor dc;
MeTemperature ts;
MeRGBLed led(0,30);
MeUltrasonicSensor us;
Me7SegmentDisplay seg;
MePort generalDevice;
MeLEDMatrix ledMx;
MeBuzzer buzzer;
MeIR ir;
MeGyro gyro;
MeJoystick joystick;
MeCompass Compass;
MeHumiture humiture;
MeFlameSensor FlameSensor;
MeGasSensor GasSensor;
MeTouchSensor touchSensor;
Me4Button buttonSensor;

/* Grid Follower Data */

int grid_x;
int grid_y;
int grid_heading;

int grid_travelDist;
int grid_turnAngle;

int grid_travelState;
int grid_turnState;

long unsigned int grid_timer;

/**********************/

typedef struct MeModule
{
    int device;
    int port;
    int slot;
    int pin;
    int index;
    float values[3];
} MeModule;

union{
    byte byteVal[4];
    float floatVal;
    long longVal;
}val;

union{
  byte byteVal[8];
  double doubleVal;
}valDouble;

union{
  byte byteVal[2];
  short shortVal;
}valShort;

#if defined(__AVR_ATmega32U4__) 
const int analogs[12] PROGMEM = {A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11};
#else
const int analogs[8] PROGMEM = {A0,A1,A2,A3,A4,A5,A6,A7};
#endif
String mVersion = "06.01.106";
boolean isAvailable = false;

int len = 52;
char buffer[52];
byte index = 0;
byte dataLen;
byte modulesLen=0;
boolean isStart = false;
char serialRead;
uint8_t command_index = 0;
#define VERSION 0
#define ULTRASONIC_SENSOR 1
#define TEMPERATURE_SENSOR 2
#define LIGHT_SENSOR 3
#define POTENTIONMETER 4
#define JOYSTICK 5
#define GYRO 6
#define SOUND_SENSOR 7
#define RGBLED 8
#define SEVSEG 9
#define MOTOR 10
#define SERVO 11
#define ENCODER 12
#define IR 13
#define IRREMOTE 14
#define PIRMOTION 15
#define INFRARED 16
#define LINEFOLLOWER 17
#define IRREMOTECODE 18
#define SHUTTER 20
#define LIMITSWITCH 21
#define BUTTON 22
#define HUMITURE 23
#define FLAMESENSOR 24
#define GASSENSOR 25
#define COMPASS 26
#define DIGITAL 30
#define ANALOG 31
#define PWM 32
#define SERVO_PIN 33
#define TONE 34
#define BUTTON_INNER 35
#define LEDMATRIX 41
#define TIMER 50
#define TOUCH_SENSOR 51

#define GRID_X 200
#define GRID_Y 201
#define GRID_HEADING 202
#define GRID_TRAVEL 203
#define GRID_TURN 204

#define GET 1
#define RUN 2
#define RESET 4
#define START 5
float angleServo = 90.0;
int servo_pins[8]={0,0,0,0,0,0,0,0};
unsigned char prevc=0;
boolean buttonPressed = false;
uint8_t keyPressed = KEY_NULL;

void readButtonInner(uint8_t pin, int8_t s)
{
  pin = pgm_read_byte(&analogs[pin]);
  pinMode(pin,INPUT);
  boolean currentPressed = !(analogRead(pin)>10);
      
  if(buttonPressed == currentPressed){
    return;
  }
  buttonPressed = currentPressed;
  writeHead();
  writeSerial(0x80);
  sendByte(currentPressed);
  writeEnd();
}

void setup(){
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  delay(300);
  digitalWrite(13,LOW);
  Serial.begin(115200);
  delay(500);
  buzzer.tone(500,50); 
  delay(50);
  buzzerOff();
  ir.begin();
  led.setpin(13);
  led.setColor(0,0,0);
  led.show();
  gyro.begin();
  Serial.print("Version: ");
  Serial.println(mVersion);
  ledMx.setBrightness(6);
  ledMx.setColorIndex(1);
}
int irDelay = 0;
int irIndex = 0;
char irRead = 0;
boolean irReady = false;
String irBuffer = "";
double lastTime = 0.0;
double currentTime = 0.0;
double lastIRTime = 0.0;

void loop(){
  readButtonInner(7,0);
  keyPressed = buttonSensor.pressed();
  currentTime = millis()/1000.0-lastTime;
  if(ir.decode())
  {
    irRead = ((ir.value>>8)>>8)&0xff;
    lastIRTime = millis()/1000.0;
    if(irRead==0xa||irRead==0xd){
      irIndex = 0;
      irReady = true;
    }else{
      irBuffer+=irRead; 
      irIndex++;
      if(irIndex>64){
        irIndex = 0;
        irBuffer = "";
      }
    }
    irDelay = 0;
  }else{
    irDelay++;
    if(irRead>0){
     if(irDelay>5000){
      irRead = 0;
      irDelay = 0;
     }
   }
  }
  readSerial();
  if(isAvailable){
    unsigned char c = serialRead&0xff;
    if(c==0x55&&isStart==false){
     if(prevc==0xff){
      index=1;
      isStart = true;
     }
    }else{
      prevc = c;
      if(isStart){
        if(index==2){
         dataLen = c; 
        }else if(index>2){
          dataLen--;
        }
        writeBuffer(index,c);
      }
    }
     index++;
     if(index>51){
      index=0; 
      isStart=false;
     }
     if(isStart&&dataLen==0&&index>3){ 
        isStart = false;
        parseData(); 
        index=0;
     }
  }

  GridFollow();
  
}
void buzzerOn(){
  buzzer.tone(500,1000); 
}
void buzzerOff(){
  buzzer.noTone(); 
}
unsigned char readBuffer(int index){
 return buffer[index]; 
}
void writeBuffer(int index,unsigned char c){
  buffer[index]=c;
}
void writeHead(){
  writeSerial(0xff);
  writeSerial(0x55);
}
void writeEnd(){
 Serial.println(); 
}
void writeSerial(unsigned char c){
 Serial.write(c);
}
void readSerial(){
  isAvailable = false;
  if(Serial.available()>0){
    isAvailable = true;
    serialRead = Serial.read();
  }
}
/*
ff 55 len idx action device port slot data a
0  1  2   3   4      5      6    7    8
*/
void parseData(){
  isStart = false;
  int idx = readBuffer(3);
  command_index = (uint8_t)idx;
  int action = readBuffer(4);
  int device = readBuffer(5);
  switch(action){
    case GET:{
        if(device != ULTRASONIC_SENSOR){
          writeHead();
          writeSerial(idx);
        }
        readSensor(device);
        writeEnd();
     }
     break;
     case RUN:{
       runModule(device);
       callOK();
     }
      break;
      case RESET:{
        //reset
        dc.reset(M1);
        dc.run(0);
        dc.reset(M2);
        dc.run(0);
        buzzerOff();
        callOK();
      }
     break;
     case START:{
        //start
        callOK();
      }
     break;
  }
}
void callOK(){
    writeSerial(0xff);
    writeSerial(0x55);
    writeEnd();
}
void sendByte(char c){
  writeSerial(1);
  writeSerial(c);
}
void sendString(String s){
  int l = s.length();
  writeSerial(4);
  writeSerial(l);
  for(int i=0;i<l;i++){
    writeSerial(s.charAt(i));
  }
}
//1 byte 2 float 3 short 4 len+string 5 double
void sendFloat(float value){ 
     writeSerial(2);
     val.floatVal = value;
     writeSerial(val.byteVal[0]);
     writeSerial(val.byteVal[1]);
     writeSerial(val.byteVal[2]);
     writeSerial(val.byteVal[3]);
}
void sendShort(double value){
     writeSerial(3);
     valShort.shortVal = value;
     writeSerial(valShort.byteVal[0]);
     writeSerial(valShort.byteVal[1]);
     writeSerial(valShort.byteVal[2]);
     writeSerial(valShort.byteVal[3]);
}
void sendDouble(double value){
     writeSerial(5);
     valDouble.doubleVal = value;
     writeSerial(valDouble.byteVal[0]);
     writeSerial(valDouble.byteVal[1]);
     writeSerial(valDouble.byteVal[2]);
     writeSerial(valDouble.byteVal[3]);
     writeSerial(valDouble.byteVal[4]);
     writeSerial(valDouble.byteVal[5]);
     writeSerial(valDouble.byteVal[6]);
     writeSerial(valDouble.byteVal[7]);
}
short readShort(int idx){
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx+1);
  return valShort.shortVal; 
}
float readFloat(int idx){
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx+1);
  val.byteVal[2] = readBuffer(idx+2);
  val.byteVal[3] = readBuffer(idx+3);
  return val.floatVal;
}
char _receiveStr[20] = {};
uint8_t _receiveUint8[16] = {};
char* readString(int idx,int len){
  for(int i=0;i<len;i++){
    _receiveStr[i]=readBuffer(idx+i);
  }
  _receiveStr[len] = '\0';
  return _receiveStr;
}
uint8_t* readUint8(int idx,int len){
  for(int i=0;i<len;i++){
    if(i>15){
      break;
    }
    _receiveUint8[i] = readBuffer(idx+i);
  }
  return _receiveUint8;
}

void doMove(int leftSpeed, int rightSpeed)
{
  dc.reset(M1);
  dc.run(leftSpeed * -1);
  dc.reset(M2);
  dc.run(rightSpeed);
}

void runModule(int device){
  //0xff 0x55 0x6 0x0 0x2 0x22 0x9 0x0 0x0 0xa 
  int port = readBuffer(6);
  int pin = port;
  switch(device){
   case MOTOR:{
     int speed = readShort(7);
     if(dc.getPort()!=port){
       dc.reset(port);
     }
     dc.run(speed);
   } 
    break;
    case JOYSTICK:{
      doMove(readShort(6) * -1, readShort(8));
      /*
     int leftSpeed = readShort(6);
     dc.reset(M1);
     dc.run(leftSpeed);
     int rightSpeed = readShort(8);
     dc.reset(M2);
     dc.run(rightSpeed);
     */
    }
    break;
   case RGBLED:{
     int slot = readBuffer(7);
     int idx = readBuffer(8);
     int r = readBuffer(9);
     int g = readBuffer(10);
     int b = readBuffer(11);
     led.reset(port,slot);
     if(idx>0)
     {
       led.setColorAt(idx-1,r,g,b); 
     }
     else
     {
       led.setColor(r,g,b); 
     }
     led.show();
   }
   break;
   case SERVO:{
     int slot = readBuffer(7);
     pin = slot==1?mePort[port].s1:mePort[port].s2;
     int v = readBuffer(8);
     Servo sv = servos[searchServoPin(pin)];
     if(v >= 0 && v <= 180)
     {
       if(!sv.attached())
       {
         sv.attach(pin);
       }
       sv.write(v);
     }
   }
   break;
   case SEVSEG:{
     if(seg.getPort()!=port){
       seg.reset(port);
     }
     float v = readFloat(7);
     seg.display(v);
   }
   break;
   case LEDMATRIX:{
     if(ledMx.getPort()!=port){
       ledMx.reset(port);
     }
     int action = readBuffer(7);
     if(action==1){
            int px = buffer[8];
            int py = buffer[9];
            int len = readBuffer(10);
            char *s = readString(11,len);
            ledMx.drawStr(px,py,s);
      }else if(action==2){
            int px = readBuffer(8);
            int py = readBuffer(9);
            uint8_t *ss = readUint8(10,16);
            ledMx.drawBitmap(px,py,16,ss);
      }else if(action==3){
            int point = readBuffer(8);
            int hours = readBuffer(9);
            int minutes = readBuffer(10);
            ledMx.showClock(hours,minutes,point);
     }else if(action == 4){
            ledMx.showNum(readFloat(8),3);
     }
   }
   break;
   case LIGHT_SENSOR:{
     if(generalDevice.getPort()!=port){
       generalDevice.reset(port);
     }
     int v = readBuffer(7);
     generalDevice.dWrite1(v);
   }
   break;
   case IR:{
     String Str_data;
     int len = readBuffer(2)-3;
     for(int i=0;i<len;i++){
       Str_data+=(char)readBuffer(6+i);
     }
     ir.sendString(Str_data);
     Str_data = "";
   }
   break;
   case SHUTTER:{
     if(generalDevice.getPort()!=port){
       generalDevice.reset(port);
     }
     int v = readBuffer(7);
     if(v<2){
       generalDevice.dWrite1(v);
     }else{
       generalDevice.dWrite2(v-2);
     }
   }
   break;
   case DIGITAL:{
     pinMode(pin,OUTPUT);
     int v = readBuffer(7);
     digitalWrite(pin,v);
   }
   break;
   case PWM:{
     pinMode(pin,OUTPUT);
     int v = readBuffer(7);
     analogWrite(pin,v);
   }
   break;
   case TONE:{
     int hz = readShort(6);
     int tone_time = readShort(8);
     if(hz>0){
       buzzer.tone(hz,tone_time);
     }else{
       buzzer.noTone(); 
     }
   }
   break;
   case SERVO_PIN:{
     int v = readBuffer(7);
     Servo sv = servos[searchServoPin(pin)]; 
     if(v >= 0 && v <= 180)
     {
       if(!sv.attached())
       {
         sv.attach(pin);
       }
       sv.write(v);
     }
   }
   break;
   case TIMER:{
    lastTime = millis()/1000.0; 
   }
   break;
   
    case GRID_X:{
      grid_x = readShort(6);    
    }
    break;
    case GRID_Y:{
      grid_y = readShort(6);    
    }
    break;   
    case GRID_HEADING:{
      grid_heading = readShort(6);
    }
    break;
    case GRID_TRAVEL:{
      grid_travelDist = readShort(6);
    }
    break;
    case GRID_TURN:{
      grid_turnAngle = readShort(6);      
    }
    break;
  }
}

int searchServoPin(int pin){
    for(int i=0;i<8;i++){
      if(servo_pins[i] == pin){
        return i;
      }
      if(servo_pins[i]==0){
        servo_pins[i] = pin;
        return i;
      }
    }
    return 0;
}

float readUltrasonicSensor(int port) {
  if(us.getPort()!=port){
    us.reset(port);
  }
  return (float)us.distanceCm();
}

void readSensor(int device){
  /**************************************************
      ff    55      len idx action device port slot data a
      0     1       2   3   4      5      6    7    8
      0xff  0x55   0x4 0x3 0x1    0x1    0x1  0xa 
  ***************************************************/
  float value=0.0;
  int port,slot,pin;
  port = readBuffer(6);
  pin = port;
  switch(device){
    case  ULTRASONIC_SENSOR:{

      value = readUltrasonicSensor(port);
      /*
      if(us.getPort()!=port){
        us.reset(port);
      }
      value = (float)us.distanceCm();
      */
      writeHead();
      writeSerial(command_index);
      sendFloat(value);
    }
    break;
    case  TEMPERATURE_SENSOR:{
      slot = readBuffer(7);
      if(ts.getPort()!=port||ts.getSlot()!=slot){
        ts.reset(port,slot);
      }
      value = ts.temperature();
      sendFloat(value);
    }
    break;
    case  LIGHT_SENSOR:
    case  SOUND_SENSOR:
    case  POTENTIONMETER:{
      if(generalDevice.getPort()!=port){
        generalDevice.reset(port);
        pinMode(generalDevice.pin2(),INPUT);
      }
      value = generalDevice.aRead2();
      sendFloat(value);
    }
    break;
    case  JOYSTICK:{
      slot = readBuffer(7);
      if(joystick.getPort() != port){
        joystick.reset(port);
      }
      value = joystick.read(slot);
      sendFloat(value);
    }
    break;
    case  IR:{
      if(irReady){
         sendString(irBuffer);
         irReady = false;
         irBuffer = "";
      }
    }
    break;
    case IRREMOTE:{
      unsigned char r = readBuffer(7);
      if(millis()/1000.0-lastIRTime>0.2){
        sendByte(0);
      }else{
        sendByte(irRead==r);
      }
      //irRead = 0;
      irIndex = 0;
    }
    break;
    case IRREMOTECODE:{
      if(irRead<0xff){
        sendByte(irRead);
      }
      irRead = 0;
      irIndex = 0;
    }
    break;
    case PIRMOTION:{
      if(generalDevice.getPort()!=port){
        generalDevice.reset(port);
        pinMode(generalDevice.pin2(),INPUT);
      }
      value = generalDevice.dRead2();
      sendFloat(value);
    }
    break;
    case LINEFOLLOWER:{
      if(generalDevice.getPort()!=port){
        generalDevice.reset(port);
          pinMode(generalDevice.pin1(),INPUT);
          pinMode(generalDevice.pin2(),INPUT);
      }
      value = generalDevice.dRead1()*2+generalDevice.dRead2();
      sendFloat(value);
    }
    break;
    case LIMITSWITCH:{
      slot = readBuffer(7);
      if(generalDevice.getPort()!=port||generalDevice.getSlot()!=slot){
        generalDevice.reset(port,slot);
      }
      if(slot==1){
        pinMode(generalDevice.pin1(),INPUT_PULLUP);
        value = !generalDevice.dRead1();
      }else{
        pinMode(generalDevice.pin2(),INPUT_PULLUP);
        value = !generalDevice.dRead2();
      }
      sendFloat(value);  
    }
    break;
    case BUTTON_INNER:{
      //pin = analogs[pin];
      pin = pgm_read_byte(&analogs[pin]);
      char s = readBuffer(7);
      pinMode(pin,INPUT);
      boolean currentPressed = !(analogRead(pin)>10);
      sendByte(s^(currentPressed?1:0));
      buttonPressed = currentPressed;
    }
    break;
    case COMPASS:{
      if(Compass.getPort()!=port){
        Compass.reset(port);
        Compass.setpin(Compass.pin1(),Compass.pin2());
      }
      sendFloat(Compass.getAngle());
    }
    break;
    case HUMITURE:{
      uint8_t index = readBuffer(7);
      if(humiture.getPort()!=port){
        humiture.reset(port);
      }
      uint8_t HumitureData;
      humiture.update();
      HumitureData = humiture.getValue(index);
      sendByte(HumitureData);
    }
    break;
    case FLAMESENSOR:{
      if(FlameSensor.getPort()!=port){
        FlameSensor.reset(port);
        FlameSensor.setpin(FlameSensor.pin2(),FlameSensor.pin1());
      }
      int16_t FlameData; 
      FlameData = FlameSensor.readAnalog();
      sendShort(FlameData);
    }
    break;
    case GASSENSOR:{
      if(GasSensor.getPort()!=port){
        GasSensor.reset(port);
        GasSensor.setpin(GasSensor.pin2(),GasSensor.pin1());
      }
      int16_t GasData; 
      GasData = GasSensor.readAnalog();
      sendShort(GasData);
    }
    break;
    case  GYRO:{
      int axis = readBuffer(7);
      gyro.update();
      value = gyro.getAngle(axis);
      sendFloat(value);
    }
    break;
    case  VERSION:{
      sendString(mVersion);
    }
    break;
    case  DIGITAL:{
      pinMode(pin,INPUT);
      sendFloat(digitalRead(pin));
    }
    break;
    case  ANALOG:{
      //pin = analogs[pin];
      pin = pgm_read_byte(&analogs[pin]);
      pinMode(pin,INPUT);
      sendFloat(analogRead(pin));
    }
    break;
    case TIMER:{
      sendFloat(currentTime);
    }
    break;
    case TOUCH_SENSOR:
    {
      if(touchSensor.getPort() != port){
        touchSensor.reset(port);
      }
      sendByte(touchSensor.touched());
    }
    break;
    case BUTTON:
    {
      if(buttonSensor.getPort() != port){
        buttonSensor.reset(port);
      }
      sendByte(keyPressed == readBuffer(7));
    }
    break;
    case GRID_X:{
      sendShort(grid_x);        
    }
    break;
    case GRID_Y:{
      sendShort(grid_y);
    }
    break;
    case GRID_HEADING:{
      sendShort(grid_heading);
    }
    break;
    case GRID_TRAVEL:{
      sendShort(grid_travelDist);
    }
    case GRID_TURN:{
      sendShort(grid_turnAngle);
    }
  }
}

void LineFollow(int lineFollower, int travelSpeed, int courseCorrectSpeed)
{
  switch (lineFollower)
  {
    case 0: // Both sensors on the line
      doMove(travelSpeed,travelSpeed);

    /*
      dc.reset(M1);
      dc.run(travelSpeed * -1);
      dc.reset(M2);
      dc.run(travelSpeed);
      */
      break;

    case 1: // Right sensor off the line - slow down left motor
      doMove(courseCorrectSpeed,travelSpeed);
      /*
      dc.reset(M1);
      dc.run(courseCorrectSpeed * -1);          
      dc.reset(M2);          
      dc.run(travelSpeed);
      */
      break;

    case 2: // Left sensor off the line - slow down right motor
      doMove(travelSpeed, courseCorrectSpeed);
      /*
      dc.reset(M1);
      dc.run(travelSpeed * -1);
      dc.reset(M2);          
      dc.run(courseCorrectSpeed);
      */
      break;
   }  
}

void GridFollow()
{
  int travelSpeed = 100;
  int courseCorrectSpeed = 50;
  int turnSpeed = 90;
  int gridSpace = 300;
  
  int lineFollower;
  float ultrasonic;

  // Assumes that:
  // - line follower is connected to port 2
  // - ultrasonic is connected to port 3
  // may want to make configurable?
  
  if(generalDevice.getPort()!=2){
    generalDevice.reset(2);
    pinMode(generalDevice.pin1(),INPUT);
    pinMode(generalDevice.pin2(),INPUT);
  }

  lineFollower = generalDevice.dRead1()*2+generalDevice.dRead2();
  if (grid_travelState > 0){
    ultrasonic = readUltrasonicSensor(3);
  }
  
  switch(grid_travelState)
  {
    case 0: // Idle
      if (grid_turnState == 0 && grid_travelDist > 0)
      {
        doMove(travelSpeed, travelSpeed);
        grid_travelState = 1;
      }
      break;
      
    case 1: // Look for line
      if (lineFollower != 3)
      {
        grid_travelState = 2;
      }
      break;
      
    case 2: // Follow line

      LineFollow(lineFollower, travelSpeed, courseCorrectSpeed);

      if (lineFollower == 3) // Both sensors off the line - found junction
      {
          doMove(travelSpeed, travelSpeed);

          grid_travelState = 3;
      }   
      break;
      
    case 3: // Cross intersection
      if (lineFollower != 3)
      {        
        grid_timer = millis();
        grid_travelState = 4;
      }       
      break;
      
    case 4: // Position over centre of intersection

      LineFollow(lineFollower, travelSpeed, courseCorrectSpeed);
      
      if (millis() - grid_timer > 250)
      {

        grid_travelDist = grid_travelDist - gridSpace;
        if (grid_travelDist <= 0 || ultrasonic < gridSpace / 2) 
        {
          grid_travelDist = 0;
        }
        else
        {
          grid_travelState = 2;
        }
        
      }
      break;
      
    default:
      grid_travelState = 0;      
  }
  if (grid_travelState !=0 && grid_travelDist == 0)
  {
    grid_travelState = 0;
    doMove(0, 0);
  }
  

  switch(grid_turnState)
  {
    case 0: // Idle
      if (grid_travelState == 0)
      {
        if (grid_turnAngle > 0)
        {
          grid_turnState = 1;
        }
        if (grid_turnAngle < 0)
        {
          grid_turnState = 3;
        }
      }
      break;
      
    case 1: // Turn right - Move off line
      doMove(turnSpeed, turnSpeed * -1);

      if (lineFollower == 3)
      {
        grid_turnState = 2;
      }
      break;

    case 2: // Find next line 
      if (lineFollower != 3)
      {
        grid_turnAngle = grid_turnAngle - 90;
        if (grid_turnAngle <= 0)
        {
          grid_turnAngle = 0;
        }
        else
        {
          grid_turnState = 1;
        }
      }
      break;
      
    case 3: // Turn left - Move off line
      doMove(turnSpeed * -1, turnSpeed);
      if (lineFollower == 3)
      {
        grid_turnState = 4;
      }
      break;

    case 4: // Find next line 
      if (lineFollower != 3)
      {
        grid_turnAngle = grid_turnAngle + 90;
        if (grid_turnAngle >= 0)
        {
          grid_turnAngle = 0;
        }
        else
        {
          grid_turnState = 3;
        }
      }
      break;
    default:
      grid_turnState = 0;   
  
  }

  if (grid_turnState !=0 && grid_turnAngle == 0)
  {
    grid_turnState = 0;
    doMove(0, 0);
  }
}

