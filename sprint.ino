#include <EEPROM.h>
#include <Servo.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2 
#define LED_PIN 13 
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}




// put next line after comment, if you have no HC-SR04 ultrasonic 
 #define HAVE_ULTRASONIC

// pin connections:
// servos:
//    D3   Right back 2 (RB2)
//    D5   Right back 1 (RB1)
//    D6   Right front 2 (RF2)
//    D7   Right front 1 (RF1)
//    D8   Left front 2 (LF2)
//    D9   Left front 1 (LF1)
//    D10  Left back 1 (LB1)
//    D11  Left back 2 (LB2)
// sirene:
//    +: D13
// touch:
//    1: A6
//    2: A7
// mp3 player:
//    RX:     A2
// ultrasonic HC-SR04
//    TRIG:   A0
//    ECHO:   A1
// gyro (MPU-6050)
//    SDA:    A4
//    SCL:    A5
// battery voltage:
//    Vbat:   A3

#define MP3_OUTPUT_PIN 16   // connect Rx pin of DFPlayer to A2

#define TRIG 14    // pin A0
#define ECHO 15    // pin A1

// left/right front/back 1=upper, 2=lower
// LF1, LB1, RF1, RB1, LF2, LB2, RF2, RB2

#define LF1 0
#define LB1 1
#define RF1 2
#define RB1 3
#define LF2 4
#define LB2 5
#define RF2 6
#define RB2 7

#define WARN_LED 13

#define zmenene 260
int natocenie = 0;

Servo legs[8];
Servo head;

uint8_t legv[8];
char pluskey[] = {'q', 'w', '3', '4', 'a', 'x', 'c', 'f'};
char minuskey[] = {'1', '2', 'e', 'r', 'z', 's', 'd', 'v'};
char key_step_plus = '+';
char key_step_minus = '-';
int8_t step_size;
int seq_length;
uint8_t del = 0;
uint8_t ultrasonic = 0;
uint8_t quiet = 0;
int pitch;
int roll;
uint8_t current_song = 1;
uint8_t volume;
uint8_t head_position = 90;


uint8_t usb_active;
uint8_t bt_active;

uint8_t in_edit_mode;
uint8_t debug_mode;
uint8_t debugged_step;

uint8_t initial[] = {90,90,90,90,90,90,90,90};



int usual_delay = 70;
int inp = 0;
int8_t laying = 0;

static uint8_t ignore_batteries = 0;
static volatile int touch;


void react_to_multitouch(uint8_t num_touches)
{
    switch (num_touches)
    {
      case 2: //double-touch
              automatic_move_head_left();
              Serial.println("hotov");
              break;
      case 3: //tripple-touch
              automatic_move_head_right();
              Serial.println("hotov");
              break;
      case 4: //quadruple-touch
              reset_position();
              break;
      case 5: //5-touches :-)
              forward();
              break;
      case 6: //6-touches :-)
              backward();
              break;
      case 7: //happy 7-touches
              
              break;
      case 8: //gothic 8 touches   
              
              break;    
      case 9: //epic 9 touches switch ultrasonic
              ultrasonic = 1 - ultrasonic;
              break;                
      case 10: //tenth finger moves the head
              //move_head();
              break;                
    }
}

void react_to_touch()
{
  if (quiet) toggle_quiet();
  mp3_play(current_song);
}

void react_to_tap()
{
  toggle_quiet();
}

void react_to_mic()
{
  //move_head();
  forward();
}
//----------------------------------- end of reactions code --------------------------------

void setup() 
{
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
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  delay(300);
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();



  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
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
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  
  usb_active = 0;
  bt_active = 0;
  in_edit_mode = 0;
  debug_mode = 0;
  volume = 20;
  seq_length = 0;
  
  legs[LB2].attach(11);
  legs[LB1].attach(10);
  legs[LF2].attach(8);
  legs[LF1].attach(9);
  legs[RB2].attach(3);
  legs[RB1].attach(5);
  legs[RF2].attach(6);
  legs[RF1].attach(7);
  head.attach(12);
  reset_position();

  pinMode(WARN_LED, OUTPUT);
  digitalWrite(WARN_LED, LOW);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(9600);
  init_tone2();

  // configure ADC
  ADMUX = 0b01000011;
  ADCSRA = 0b10010101;
  ADCSRB = 0;
  DIDR0 = 8;  // disable A3 digital

  while (Serial.available()) Serial.read();
  Serial.println(F("Hi! Press U for USB-powered run."));
  delay(2500);
  if (Serial.available())
  {
    if (Serial.read() == 'U') 
    {
      ignore_batteries = 1;
      Serial.println(F("USB-powered"));
    }
    usb_active = 1;
  }
  else Serial.println(F("Bat.powered"));
  Serial.println(F("MoKraRoSA. Press H for help"));
  
  step_size = 1;
  sound_greeting();
    
  mp3_set_volume(volume);

}

void sound_greeting()
{
  tone2(440, 50);
  delay(50);
  tone2(880, 100);
  delay(100);
  tone2(1760, 50);
  delay(50);
}

void position_90()
{
    for (int i = 0; i < 8; i++)
      legv[i] = 90;
       
    for (int i = 0; i < 8; i++)
      legs[i].write(legv[i]);
}

void reset_position()
{
    for (int i = 0; i < 8; i++)
      legv[i] = initial[i];
       
    for (int i = 0; i < 8; i++)
      legs[i].write(legv[i]);
      
    head.write(90);
}

// walking
const uint8_t len_fwdwalk = 13;
const uint8_t fwdwalk[] PROGMEM = {  82,74,113,136,136,118,62,79,4,
100,67,99,105,98,94,85,105,4,
100,67,165,105,98,94,127,105,4,
100,100,142,105,98,83,138,105,4,
100,86,142,135,98,108,138,56,4,
31,86,142,135,37,108,138,56,4,
31,86,141,88,37,108,138,85,4,
9,74,155,88,17,119,153,85,4,
9,74,155,175,17,119,153,142,4,
35,80,173,150,37,113,160,174,4,
52,80,175,171,44,113,106,165,4,
83,70,175,174,54,121,106,102,4,
83,16,175,174,54,37,106,102,4 };
                                     
void walk_from_memory(uint8_t dir, uint8_t *mem, int len)
{      
    int st[9];   
    uint8_t stop_walking = 0;
    natocenie = 0;
    do {
      char *m = mem;
      if (dir == 0) m += (len * 9 - 9);
      
      for (int i = 0; i < len; i++)
      {
        if (Serial.available()) { stop_walking = 1; break; }
        if (serial_available()) { stop_walking = 1; break; }
        read_touch();
        if (touch > zmenene) { stop_walking = 1; break; }
          
        for (int j = 0; j < 9; j++)
        {
          st[j] = (int)pgm_read_byte(m++);          
        }
        if (dir == 0) m -= 18;
      
        for (int k = 0; k < 100; k++)
        {
          for (int j = 0; j < 8; j++)        
             legs[j].write(legv[j] + ((int)(st[j] - legv[j]) * k) / 100);      
          delay(st[8]);
          if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

                  // display Euler angles in degrees
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                  mpu.dmpGetGravity(&gravity, &q);
                  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 
          }
        }
       
        for (int j = 0; j < 8; j++)        
          legv[j] = st[j];
                
        check_battery();
      }
      Serial.println(ypr[0] * 180 / M_PI);
      if (natocenie == 0)
      {
        if (ypr[0] * 180 / M_PI >= 15) {
         automatic_move_head_left();
         natocenie = 1;
         Serial.println(natocenie);
        }
        else if (ypr[0] * 180 / M_PI <= -15) {        
          automatic_move_head_right();
          natocenie = -1;
          Serial.println(natocenie);
        }
      }
      else if ((ypr[0] * 180 / M_PI >= -12) && (ypr[0] * 180 / M_PI <= 12)) {
        if (natocenie == 1)
        { 
           automatic_move_head_right();
        }
        else
        {
          automatic_move_head_left();
        }
        natocenie = 0;
        Serial.println(natocenie);
      }
    } while (!stop_walking);
    
    while (Serial.available()) Serial.read();
    while (serial_available()) serial_read(); 
    while (touch > zmenene  ) { read_touch();}
}

void forward()
{  
  walk_from_memory(1, fwdwalk, len_fwdwalk);
}

void backward()
{
  walk_from_memory(0, fwdwalk, len_fwdwalk);
}


const uint8_t position_down[] = { 5, 5, 175, 175, 175, 175, 5, 5 };
const uint8_t position_sit[] = { 90, 5, 90, 175, 90, 175, 90, 5 };

void position_from_memory(const uint8_t *m)
{  
  for (int k = 0; k < 100; k++)
  {
    for (int j = 0; j < 8; j++)        
       legs[j].write(legv[j] + ((int)(m[j] - legv[j]) * k) / 100);
    delay(3);
  }
  for (int j = 0; j < 8; j++)        
     legv[j] = m[j];  
}

void down_to_earth()
{  
  position_from_memory(position_down);
}

void sit_down()
{
  position_from_memory(position_sit);
}

char read_latest_char() 
{
  char c = 0;
  while (serial_available() > 0)
    c = serial_read();
  return c;
}

void toggle_quiet()
{
    if (quiet) mp3_set_volume(volume);
    else mp3_set_volume(0);   
    serial_print_flash(PSTR("music:"));
    serial_println_num(quiet);
    quiet = 1 - quiet;  
}

void both_modes(char c)
{
  if (c == '*')
  {
    reset_position();
    laying = 0;
  }
  else if (c == '9')
    position_90();
  else if (c == 'M')
  {
    toggle_quiet();
  }
  else if ((c == '<') || (c == '>'))
  {
    if (c == '<') { if (volume > 0) volume--; }
    else if (volume < 30) volume++;
    mp3_set_volume(volume);
    serial_print_flash(PSTR("vol: "));
    serial_println_num(volume);
  }
  else if (c == 'u')
  {
    ultrasonic = 1 - ultrasonic;
    serial_print_flash(PSTR("ultra:"));
    serial_println_num(ultrasonic);
  }
  else if ((c == 'h') || (c == 'H') || (c == '?')) print_usage();
  else if (c == 'B') serial_println_num(measure_bat());
  else if (c == 'P')  { read_touch(); Serial.println(touch); }
  else try_melodies(c);
}
 
void move_head_left()
{
  head.write(100);
  delay(100);
  head.write(90);
  read_touch();
  //Serial.println(touch);
}

void move_head_right()
{
  head.write(79);
  delay(100);
  head.write(90);
  read_touch();
}


void automatic_move_head_left()
{
  read_touch();
  while (touch > 82 && !Serial.available()){
    move_head_left();
    read_touch();
  }

  while (touch <= 80) {
    move_head_left();
    read_touch();
  }
  read_touch();
  delay(100);
  while (touch > 82 && !Serial.available()){
    move_head_left();
    read_touch();
  }
}

void automatic_move_head_right()
{
  read_touch();
    while (touch > 82 && !Serial.available()){
    move_head_right();
    read_touch();
  }
  
  while (touch <= 80) {
      move_head_right();
      read_touch();      
  }
  read_touch();
  
  delay(100);
  while (touch > 82 && !Serial.available()){    
    move_head_right();
    read_touch();
  }
}



void control_mode(char c)
{
  if (c == '1') forward();
  else if (c == '2') backward();
  else if (c == '5') down_to_earth(); 
  else if (c == '6') sit_down();
  else if (c == '7') move_head_left();
  else if (c == '0') automatic_move_head_right();
  else if (c == '8') move_head_right();
  else if (c == '9') automatic_move_head_left();
  else if (c == 'E') 
  {
    in_edit_mode = 1;
    serial_println_flash(PSTR("edit mode"));
  }
  else both_modes(c);
  delay(300);
}

void control_over_serial()
{
  char c = Serial.read();
  usb_active = 1;
  control_mode(c);
}

void try_melodies(char c)
{
  if (c == 'm')
  {
    mp3_play(current_song);
    delay(20);
    mp3_set_volume(volume);
  }
  else if ((c == 'j') || (c == 'n'))
  {
    if (c == 'j') if (current_song < 255) current_song++;
    if (c == 'n') if (current_song > 1) current_song--;
    serial_print_flash(PSTR("song "));
    serial_println_num(current_song);
  }
}

void read_touch()
{
  // read touch A6
  ADMUX = 0b01000110;  // select channel 6 and AVCC ref
  ADCSRA |= 64;  // start conversion
  while ((ADCSRA & 16) == 0); // wait for conversion to complete
  ADCSRA = 0b10010101; // clear the conversion complete signal for another time
  touch = ADCL;
  touch |= (ADCH << 8); 

  
}

void check_touch()
{
  read_touch();

  static uint32_t touch_touched_time = 0;
  static uint32_t touch_untouched_time = 0;
  static uint32_t touch_last_touch_length = 0;
  static uint8_t touch_multitouch = 0;

#define  DOUBLETOUCH_MAX_DELAY_TIME  500
#define  LONG_TOUCH_TAP_MIN_TIME     700
#define  TOUCH_SINGLE_MAX_TIME       300
  
  if (touch > zmenene)
  {
    if (touch_untouched_time)
    {
      touch_multitouch++;
      touch_touched_time = millis();
      touch_untouched_time = 0;
    }    
    else if (touch_touched_time == 0) touch_touched_time = millis();
  }
  else if (touch_touched_time)
  {          
      touch_untouched_time = millis();
      touch_last_touch_length = millis() - touch_touched_time;
      touch_touched_time = 0;
  }
  else if (touch_untouched_time)
  {
      if (millis() - touch_untouched_time > DOUBLETOUCH_MAX_DELAY_TIME)
      {
        touch_untouched_time = 0;          
        if (touch_multitouch)
        {
          touch_multitouch++;
          Serial.print(F("multi-touch "));
          Serial.println(touch_multitouch);

          for (int i = 0; i < touch_multitouch; i++)
          {
            tone2(440, 60); delay(90); 
          }
          react_to_multitouch(touch_multitouch);
          touch_multitouch = 0;
        }
        else if (touch_last_touch_length > LONG_TOUCH_TAP_MIN_TIME) // long tap
        {
          Serial.println(F("tap"));
          tone2(220, 150);
          react_to_tap();
        }
        else if (touch_last_touch_length < TOUCH_SINGLE_MAX_TIME) // short touch
        {
          Serial.println(F("touch"));
          tone2(880, 40);
          react_to_touch();
        }
        touch_last_touch_length = 0;
      }
  }
}

void check_mic()
{
  static int last_mic = 0;
  static int iteration = 0;
  
  ADMUX = 0b01000111;  // select channel 7 and AVCC ref
  ADCSRA |= 64;  // start conversion
  while ((ADCSRA & 16) == 0); // wait for conversion to complete
  ADCSRA = 0b10010101; // clear the conversion complete signal for another time
  int mic = ADCL;
  mic |= (ADCH << 8); 
  
  if (last_mic && abs(mic - last_mic) > 100)
     react_to_mic();
  last_mic = mic;
}

void loop() 
{

  
  check_mic();
  check_touch();

  
  
#ifdef HAVE_ULTRASONIC
  if ((measure_US() < 15) && ultrasonic)
  {
    // insert your code to react to obstacle in front of robot here
    backward();
  }
#endif

  if (Serial.available()) control_over_serial();
  
  pitch = 0;
  roll = 0;

  check_battery();
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  /*
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[2] * 180 / M_PI);
  */

  // blink LED to indicate activity
  //blinkState = !blinkState;
  //digitalWrite(LED_PIN, blinkState);
  }
}

void print_usage()
{
}

int measure_bat()
{
  ADMUX = (uint8_t) 0xc3; // 0b11000011;  // select channel 3 and 1.1V ref
  delay(5);
  ADCSRA |= 64;  // start conversion
  while ((ADCSRA & 16) == 0); // wait for conversion to complete
  ADCSRA = 0b10010101; // clear the conversion complete signal for another time
  
  ADCSRA |= 64;  // start conversion
  while ((ADCSRA & 16) == 0); // wait for conversion to complete
  ADCSRA = 0b10010101; // clear the conversion complete signal for another time
  
  int bat = ADCL;
  bat |= (ADCH << 8); 
  
  ADMUX = 0b01000011;  // put it back to AVCC ref
  delay(5);
  
  float volt = bat; 
  // volt * 1.1 * 242 / 22 / 1023  (22 KOhm out of 220+22=242 KOhm)
  return (int)(0.5 + 100.0 * volt * 0.01181640625);   
}

void check_battery()
{
  static long last_measurement = 0;
  static long measurement_count = 0;

  if (ignore_batteries) return;
  long tm = millis();
  if (tm - last_measurement > 500)
  {
    last_measurement = tm;
    if (measure_bat() < 620)  
    {
      measurement_count++;
      if (measurement_count < 5) return;
      delay(50);
      usb_active = 1;
      bt_active = 1;
      serial_println_flash(PSTR("!!!!!!!!!!!!!! Replace batteries !!!!!!!!!!!!!!!!!")); 
      mp3_set_volume(0);
      for (int i = 0; i < 8; i++)
        legs[i].detach();
      for (int i = 2; i < 20; i++)
        pinMode(i, INPUT);
      pinMode(WARN_LED, OUTPUT);
      while(1) SOS();
    }
    else measurement_count = 0;
  }
}

void SOS()
{
  for (uint8_t i = 0; i < 3; i++)
  {
    digitalWrite(WARN_LED, HIGH);
    tone2(1680, 100);
    delay(100);
    digitalWrite(WARN_LED, LOW);
    delay(50);
  }
  delay(300);
  for (uint8_t i = 0; i < 3; i++)
  {
    tone2(1680, 300);
    digitalWrite(WARN_LED, HIGH);
    delay(300);
    digitalWrite(WARN_LED, LOW);
    delay(50);
  }
  delay(300);
  for (uint8_t i = 0; i < 3; i++)
  {
    digitalWrite(WARN_LED, HIGH);
    tone2(1680, 100);
    delay(100);
    digitalWrite(WARN_LED, LOW);
    delay(50);
  }
  delay(500);
}

uint8_t serial_available()
{
  return 0;
}

int16_t serial_read()
{
  return -1;
}

int16_t serial_peek()
{
  if (usb_active) return Serial.peek();
  return -1;
}

void serial_write(uint8_t ch)
{
  if (usb_active)
    Serial.print((char)ch);
  if (bt_active)
  {  }
}

uint16_t serial_readln(uint8_t *ln, uint16_t max_length)
{
  uint16_t len;
  int16_t ch;
  do {
    ch = serial_read();
    if (ch == 13) continue;
  } while (ch == -1);

  do {
    if ((ch != 13) && (ch != 10) && (ch != -1))
    {
      *(ln++) = ch;
      max_length--;
      len++;
    }
    ch = serial_read();
  } while ((ch != 13) && max_length);
  *ln = 0;
  return len;
}

void serial_print_num(int32_t number)
{
  if (number < 0)
  {
    serial_write('-');
    number = -number;
  }
  int32_t rad = 1;
  while (number / rad) rad *= 10;
  if (number > 0) rad /= 10;
  while (rad)
  {
    serial_write((char)('0' + (number / rad)));
    number -= (number / rad) * rad;
    rad /= 10;
  }
}

void serial_print_char(char ch)
{
  serial_write(ch);
}

void serial_print(const char *str)
{
  while (*str) serial_write(*(str++));
}

void serial_println(const char *str)
{
  serial_print(str);
  serial_write(13);
  serial_write(10);
}

void serial_print_flash(const char *str)
{
  int ln = strlen_P(str);
  for (int i = 0; i < ln; i++)
    serial_write(pgm_read_byte(str + i));
}

void serial_println_flash(const char *str)
{
  serial_print_flash(str);
  serial_write(13);
  serial_write(10);
}

void serial_println_num(int32_t number)
{
  serial_print_num(number);
  serial_println();
}

void serial_println_char(char ch)
{
  serial_write(ch);
  serial_println();
}

void serial_println()
{
  serial_write(13);
  serial_write(10);
}


// volume 0-30
void mp3_set_volume(uint8_t volume)
{
 mp3_send_packet(0x06, volume);  
}

void mp3_play(uint8_t song_number)
{
 mp3_send_packet(0x03, song_number);  
}

void mp3_send_byte(uint8_t pin, uint8_t val)
{
  pinMode(MP3_OUTPUT_PIN, OUTPUT);
  float start_transmission = micros();
  float one_bit = 1000000 / 9600.0;
  float next_change = start_transmission + one_bit;
  digitalWrite(pin, LOW);
  while (micros() < next_change);
  
  for (int i = 2; i < 10; i++)
  {
    if (val & 1) digitalWrite(pin, HIGH);
    else digitalWrite(pin, LOW);
    next_change = start_transmission + one_bit * i;
    val >>= 1;
    while (micros() < next_change);
  }

  digitalWrite(pin, HIGH);
  next_change = micros() + 2 * one_bit;
  while (micros() < next_change);
  pinMode(MP3_OUTPUT_PIN, INPUT);
}

void mp3_send_packet(uint8_t cmd, uint16_t param)
{
  mp3_send_byte(MP3_OUTPUT_PIN, 0x7E);
  mp3_send_byte(MP3_OUTPUT_PIN, 0xFF);
  mp3_send_byte(MP3_OUTPUT_PIN, 0x06);
  mp3_send_byte(MP3_OUTPUT_PIN, cmd);
  mp3_send_byte(MP3_OUTPUT_PIN, 0x00);
  mp3_send_byte(MP3_OUTPUT_PIN, (uint8_t)(param >> 8));
  mp3_send_byte(MP3_OUTPUT_PIN, (uint8_t)(param & 0xFF));
  uint16_t chksm = 0xFF + 0x06 + cmd + (param >> 8) + (param & 0xFF);
  chksm = -chksm;
  mp3_send_byte(MP3_OUTPUT_PIN, (uint8_t)(chksm >> 8));
  mp3_send_byte(MP3_OUTPUT_PIN, (uint8_t)(chksm & 0xFF));
  mp3_send_byte(MP3_OUTPUT_PIN, 0xEF);
}

int measure_US()
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long counter = micros() + 10000;
  while ((digitalRead(ECHO) == 0) & (micros() < counter)) { }
  if (digitalRead(ECHO) == 0) return 255;
  unsigned long tm1 = micros();
  counter = tm1 + 10000; 
  while ((digitalRead(ECHO) == 1) && (micros() < counter)) { }
  if (digitalRead(ECHO) == 1) return 255;
  unsigned long tm2 = micros();
  return (tm2 - tm1) / 58;
}

//-------------------------------- the following code is for playing melodies using timer2 in the background
#define SIRENE_PORT  PORTB
#define SIRENE_DDR   DDRB
#define SIRENE_PIN   5

#define FIS3 2960
#define G3 3136

float octave_4[] = { 2093.00, 2217.46, 2349.32, 2489.02, 2637.02, 2793.83, 2959.96, 3135.96, 3322.44, 3520.00, 3729.31, 3951.07 };



static volatile uint8_t tone2_state;
static volatile uint8_t tone2_pause;
static volatile uint32_t tone2_len;

void init_tone2()
{
  tone2_pause = 0;
  TCCR2A = 2;
  TCCR2B = 0;
  TIMSK2 = 2;
  SIRENE_DDR |= (1 << SIRENE_PIN);
}

ISR(TIMER2_COMPA_vect)
{
  if (!tone2_pause)
  {
    if (tone2_state)
    {
      SIRENE_PORT |= (1 << SIRENE_PIN);
      tone2_state = 0;
    }
    else
    {
      SIRENE_PORT &= ~(1 << SIRENE_PIN);
      tone2_state = 1;
    }
  }
  if ((--tone2_len) == 0)
  {
    TCCR2B = 0;
    tone2_pause = 0;
    //next_note();
  }
}

void tone2(uint16_t freq, uint16_t duration)
{
  uint32_t period = ((uint32_t)1000000) / (uint32_t)freq;

  if (freq >= 977)  // prescaler 32
  {
    tone2_state = 0;
    tone2_len = ((uint32_t)duration * (uint32_t)1000) * 2 / period;
    if (tone2_len == 0) tone2_len++;
    TCNT2 = 0;
    OCR2A = (uint8_t) (250000 / (uint32_t)freq);
    TCCR2B = 3;
  }
  else if (freq >= 488) // prescaler 64
  {
    tone2_state = 0;
    tone2_len = ((uint32_t)duration * (uint32_t)1000) * 2 / period;
    if (tone2_len == 0) tone2_len++;
    TCNT2 = 0;
    OCR2A = (uint8_t) (125000 / (uint32_t)freq);
    TCCR2B = 4;
  }
  else if (freq >= 244) // prescaler 128
  {
    tone2_state = 0;
    tone2_len = ((uint32_t)duration * (uint32_t)1000) * 2 / period;
    if (tone2_len == 0) tone2_len++;
    TCNT2 = 0;
    OCR2A = (uint8_t) (62500 / (uint32_t)freq);
    TCCR2B = 5;
  }
  else if (freq >= 122) //prescaler 256
  {
    tone2_state = 0;
    tone2_len = ((uint32_t)duration * (uint32_t)1000) * 2 / period;
    if (tone2_len == 0) tone2_len++;
    TCNT2 = 0;
    OCR2A = (uint8_t) (31250 / (uint32_t)freq);
    TCCR2B = 6;
  }
  else if (freq >= 30) //prescaler 1024
  {
    tone2_state = 0;
    tone2_len = ((uint32_t)duration * (uint32_t)1000) * 2 / period;
    if (tone2_len == 0) tone2_len++;
 
   TCNT2 = 0;
    OCR2A = (uint8_t) (7813 / (uint32_t)freq);
    TCCR2B = 7;
  }
  else if (freq == 0)
  {
    tone2_pause = 1;
    tone2_state = 0;

    period = 1000000 / 500;
    tone2_len = ((uint32_t)duration * (uint32_t)1000) * 2 / period;
    TCNT2 = 0;
    OCR2A = (uint8_t) (125000 / (uint32_t)500);
    TCCR2B = 4;
  }
  else
  {
    TCCR2B = 0;
  }
}

void beep()
{
  tone2(698, 30);
}
