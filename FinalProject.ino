//Authors: Brysen Ross, DJ DeLucca, Rajat Sharma, Logan Wehr
//Purpose: CPE 301, Final Project
#include <Stepper.h>
#include <dht.h>

#include <RTClib.h>
#include <LiquidCrystal.h>

dht DHT;
RTC_DS3231 rtc;

#define RDA 0x80
#define TBE 0x20
#define DHT11_PIN 4
#define stepperSpeed 35

enum State {DISABLED, IDLE, RUNNING, ERROR};
volatile State currentState = DISABLED;

const int stepsPerRev = 750, mSpeed = 255, tempThreshold = 22, waterThreshold = 150, rs = 5, en = 6, d4 = 7, d5 = 8, d6 = 9, d7 = 10, IN1 = 46, IN2 = 48, IN3 = 50, IN4 = 52;
int stepperCounter = 0;
unsigned int waterLevel = 0;
volatile unsigned long previousMillis = 0;
const unsigned long lcdInterval = 60000;
volatile bool systemStart = false, systemStop = false, firstLCDUpdate = false;
bool motorRunning = false;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
Stepper myStepper = Stepper(stepsPerRev, IN1, IN3, IN2, IN4);

//UART Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

//Buttons Pointers (pins 22-29)
volatile unsigned char* port_a = (unsigned char*) 0x22;
volatile unsigned char* ddr_a  = (unsigned char*) 0x21;
volatile unsigned char* pin_a  = (unsigned char*) 0x20;

//Interupt Pointers (Pins 18-21, 38-41)
volatile unsigned char* ddr_d  = (unsigned char*) 0x2A;
volatile unsigned char* port_d = (unsigned char*) 0x2B;

//Motor Control Pointers (pins 0-3, 5)
volatile unsigned char* port_e  = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e   = (unsigned char*) 0x2D;

//LED Pointers (pins 42-49)
volatile unsigned char* ddr_l  = (unsigned char*) 0x10A;
volatile unsigned char* port_l = (unsigned char*) 0x10B;

//ADC Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

void setup(){
  U0Init(9600);

  //interupts setup (start and stop buttons)
  *ddr_d &= ~(1 << 2); //sets pin 19 as input (start button)
  *port_d |=  (1 << 2); // pin 19 pull-up enabled
  attachInterrupt(digitalPinToInterrupt(19), startISR, FALLING);
  *ddr_d &= ~(1 << 3); //sets pin 18 as input (stop button)
  *port_d |=  (1 << 3); // pin 18 pull-up enabled
  attachInterrupt(digitalPinToInterrupt(18), stopISR, FALLING);

  //stepper setup
  *ddr_a &= ~(1 << 5); //sets pin 27 as input 
  myStepper.setSpeed(stepperSpeed);

  //motor setup
  *ddr_e |= (1 << 5); //sets pin 3 as output

  //clock setup
  rtc.begin();

  //lcd setup
  lcd.begin(16,2);

  //ACD setup
  adc_init();

  //Buttons Setup
  *ddr_a &= ~(1 << 3); // sets pin 25 as input (Reset button)

  //LED setup
  *ddr_l |= (1 << 0); //pin 49 (red LED)
  *ddr_l |= (1 << 2); //pin 47 (yellow LED)
  *ddr_l |= (1 << 4); //pin 45 (green LED)
  *ddr_l |= (1 << 6); //pin 43 (blue LED)
}

void loop(){
  int chk = DHT.read11(DHT11_PIN);
  int temp = DHT.temperature;
  int humidity = DHT.humidity; 
  switch(currentState){
    case DISABLED:
      runDisabled();
      if(systemStart){
        systemStart = false;
        currentState = IDLE;
        printTimeAndStatus("State changed to IDLE");
      }
      break;
    case IDLE:
      runIdle(temp, humidity);
      if(systemStop){
        currentState = DISABLED;
        systemStop = false;
        printTimeAndStatus("State changed to DISABLED");
      }
      else if(waterLevel <= waterThreshold){
        currentState = ERROR;
        printTimeAndStatus("State changed to ERROR");
      }
      else if(temp > tempThreshold){
        currentState = RUNNING;
        printTimeAndStatus("State changed to RUNNING");
      }
      break;
    case RUNNING:
      runRunning(temp, humidity);
      if(systemStop){
        currentState = DISABLED;
        systemStop = false;
        printTimeAndStatus("State changed to DISABLED");
      }
      else if(waterLevel < waterThreshold){
        currentState = ERROR;
        printTimeAndStatus("State changed to ERROR");
      }
      else if(temp < tempThreshold){
        currentState = IDLE;
        printTimeAndStatus("State changed to IDLE");
      }
      break;
    case ERROR:
      runError();
      if(systemStop){
        currentState = DISABLED;
        break;
      }
      if((*pin_a & (1 << 3)) && (waterLevel > waterThreshold)){
        currentState = IDLE;
        printTimeAndStatus("State changed to IDLE");
      }
      break;
  }
}

//Author: DJ DeLucca
//Purpose: Handles stepper motor functionality when button is pressed
void runStepper(volatile unsigned char* pin){
  if(*pin & (1 << 5)){
    if(stepperCounter % 2 == 0){
      myStepper.step(stepsPerRev);
      printTimeAndStatus("Vent Opened");
    }else{
      myStepper.step(-stepsPerRev);
      printTimeAndStatus("Vent Closed");
    }
    stepperCounter++;
    delay(200);
  }
}

//Author: Brysen Ross
//Purpose: Turns DC motor on
void startMotor(uint8_t speed){
  analogWrite(3, speed);  
}

//Author: Brysen Ross
//Purpose: Turns DC motor off
void stopMotor(){
  analogWrite(3, 0);
}

//Author: Brysen Ross
//Purpose: Displays temperature and humidity to LCD display;
void runLCD(int temp, int humidity){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.print(" C     ");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print(" %    ");
}

//Author: DJ DeLucca
//Purpose: Displays error message to LCD display;
void errorLCD_Message(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Error");
}

//Author: Brysen Ross
//Purpose: Delays runLCD such that the display is updated once per minute
void updateLCDPerMinute(int temp, int humidity){
  unsigned long currentMillis = millis();
  if(!firstLCDUpdate){
    runLCD(temp, humidity);
    previousMillis = currentMillis;
    firstLCDUpdate = true;
    return;
  }
  if(currentMillis - previousMillis >= lcdInterval){
    previousMillis = currentMillis;
    runLCD(temp, humidity);
  }
}

//Author: Brysen Ross
//Purpose: Prints a timestamp with message to serial monitor
void printTimeAndStatus(const char* status){
  bool am = false;
  int hour, minute, second;
  DateTime now = rtc.now();
  for(int i = 0; status[i] != '\0'; i++){
    putChar(status[i]);
  }
  putChar(':');
  putChar(' ');
  if(now.hour() > 12){
    hour = now.hour() - 12;
    am = false;
  }else{
    hour = now.hour();
    am = true;
  }
  if(hour < 10){
    putChar('0');
    printUnsigned(hour);
  }else{
    printUnsigned(hour);
  }
  putChar(':');
  minute = now.minute();
  if(minute < 10){
    putChar('0');
    printUnsigned(now.minute());
  }else{
    printUnsigned(now.minute());
  }
  putChar(':');
  second = now.second();
  if(second < 10){
    putChar('0');
    printUnsigned(now.second());
  }else{
    printUnsigned(now.second());
  }
  putChar(' ');
  if(am){
    putChar('A');
  }else{
    putChar('P');
  }
  putChar('M');
  putChar(' ');
  printUnsigned(now.month());
  putChar('/');
  printUnsigned(now.day());
  putChar('/');
  printUnsigned(now.year());
  putChar(' ');
  putChar('\n');
}

//Author: Logan Wehr
//Purpose: Converts value to string and prints whole string using putChar
void printUnsigned(unsigned int num){
  char digits[5];
  int i = 0;
  do{
    digits[i++] = (num % 10) + '0';
    num /= 10;
  }while(num > 0);
  for(int j = i - 1; j >= 0; j--){
    putChar(digits[j]);
  }
}

//Author: Rajat Sharma
//Purpose: Initializes Analog to Digital Converter
void adc_init(){
  *my_ADCSRA |= (1 << 7);
  *my_ADCSRA &= ~(1 << 6);
  *my_ADCSRA &= ~(1 << 5);
  *my_ADCSRA &= ~0x07;
  *my_ADCSRB &= ~(1 << 3);
  *my_ADCSRB &= ~0x07;
  *my_ADMUX &= ~(1 << 7);
  *my_ADMUX |= (1 << 6);
  *my_ADMUX &= ~(1 << 5);
  *my_ADMUX &= ~0x1F;
}

//Author: Rajat Sharma
//Purpose: Reads data from ADC channel
unsigned int adc_read(unsigned char adc_channel_num){
  *my_ADMUX &= 0xE0;
  *my_ADCSRB &= ~(1 << 3);
  *my_ADMUX |= (adc_channel_num & 0x1F);
  *my_ADCSRA |= (1 << 6);
  while((*my_ADCSRA & 0x40) != 0);
  unsigned int val = *my_ADC_DATA;
  return val;
}

//Author: Rajat Sharma
//Purpose: Reads analog data
void runADC(){
  waterLevel = adc_read(0);
}

//Author: Logan Wehr
//Purpose: Equivalent of Serial.begin()
void U0Init(int U0baud){
  unsigned long FCPU = 16000000;
  unsigned int tbaud = (FCPU / 16 / U0baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

//Author: Logan Wehr
//Purpose: Equivalent of Serial.available()
unsigned char kbhit(){
  return *myUCSR0A & RDA;
}

//Author: Logan Wehr
//Purpose: Equivalent of Serial.read()
unsigned char getChar(){
  return *myUDR0;
}

//Author: Logan Wehr
//Purpose: Equivalent of Serial.write()
void putChar(unsigned char data){
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = data;
}

//Author: DJ DeLucca
//Purpose: Handles start button's interupt
void startISR(){
  systemStart = true;
  firstLCDUpdate = false;
}

//Author: DJ DeLucca
//Purpose: Handles stop button's interupt
void stopISR() {
  firstLCDUpdate = false;
  systemStop = true;
}

//Author: Rajat Sharma
//Purpose: Runs all ERROR state functions
void runError(){
  errorState_LEDs();
  stopMotor();
  runStepper(pin_a);
  errorLCD_Message();
  runADC();
  firstLCDUpdate = false;
  if(motorRunning){
    printTimeAndStatus("Motor turned OFF");
    motorRunning = false;
  }
}

//Author: Rajat Sharma
//Purpose: Runs all DISABLED state functions
void runDisabled(){
  lcd.clear();
  disabledState_LEDs();
  stopMotor();
  if(motorRunning){
    printTimeAndStatus("Motor turned OFF");
    motorRunning = false;
  }
}

//Author: DJ DeLucca
//Purpose: Runs all IDLE state functions
void runIdle(int temp, int humidity){
  idleState_LEDs();
  stopMotor();
  runStepper(pin_a);
  updateLCDPerMinute(temp, humidity);
  runADC();
  if(motorRunning){
    printTimeAndStatus("Motor turned OFF");
    motorRunning = false;
  }
}

//Author: Brysen Ross
//Purpose: Runs all RUNNING state functions
void runRunning(int temp, int humidity){
  startMotor(mSpeed);
  if(!motorRunning){
    motorRunning = true;
    printTimeAndStatus("Motor turned ON");
  }
  runningState_LEDs();
  runStepper(pin_a);
  updateLCDPerMinute(temp, humidity);
  runADC();
}

//Author: Logan Wehr
//Purpose: Turns on only the yellow LED
void disabledState_LEDs(){
  *port_l &= ~(1 << 0);  // Red OFF
  *port_l |=  (1 << 2);  // Yellow ON
  *port_l &= ~(1 << 4);  // Green OFF
  *port_l &= ~(1 << 6);  // Blue OFF
}

//Author: Logan Wehr
//Purpose: Turns on only the green LED
void idleState_LEDs(){
  *port_l &= ~(1 << 0);  // Red OFF
  *port_l &= ~(1 << 2); //Yellow OFF
  *port_l |= (1 << 4); //Green ON
  *port_l &= ~(1 << 6);  // Blue OFF
}

//Author: Logan Wehr
//Purpose: Turns on only the blue LED
void runningState_LEDs(){
  *port_l &= ~(1 << 0);  // Red OFF
  *port_l &= ~(1 << 2); //Yellow OFF
  *port_l &= ~(1 << 4);  // Green OFF
  *port_l |= (1 << 6); // Blue ON
}

//Author: Logan Wehr
//Purpose: Turns on only the red LED
void errorState_LEDs() {
  *port_l |= (1 << 0); //Red ON
  *port_l &= ~(1 << 2); //Yellow OFF
  *port_l &= ~(1 << 4); //Green OFF
  *port_l &= ~(1 << 6); //Blue OFF
}
