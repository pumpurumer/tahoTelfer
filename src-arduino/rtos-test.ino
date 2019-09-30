#define dimPin 9
#define zeroPin 2
#define tahoPin 3
#define pinSpeed1 6
#define pinSpeed2 7
#define pinSpeedSave 8
#define pinVira 4
#define pinMina 5
#define pinEncPlus A2
#define pinEncMinus A3

#define pinReleMotor A0

#include "CyberLib.h"
#include <PID_v1.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <stdio.h>
#include <Arduino_FreeRTOS.h>

static int serial_fputchar(const char ch, FILE *stream) { Serial.write(ch); return ch; }
static FILE *serial_stream = fdevopen(serial_fputchar, NULL);

double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=0.01, Ki=0.2, Kd=0.00005;
//double Kp=1, Ki=0.05, Kd=0.25;

int speedSettings[3];

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Bounce swSpeed1 = Bounce();
Bounce swSpeed2 = Bounce();
Bounce swSpeedSave = Bounce();
Bounce swVira = Bounce();
Bounce swMina = Bounce();

Bounce encIn1 = Bounce();
Bounce encIn2 = Bounce();


volatile int tic,tic_taho, Dimmer, lastPeriod, lastUpTime, lastDownTime, dimmState, lastTaho;

int motorRpm = 0;
int targetMotorRpm = 0;
volatile int tahoSteps = 0;


int curSwSpeedState = 0;
int curSwDirectionState =0;

int encoderFrame[4];
int encoderFramePosition = 0;

int lastDirection = 0;

int currentMemorySpeed = 0;

void setupMotorController()
{
  pinMode(dimPin, OUTPUT);
  digitalWrite(dimPin, 0);
  
  pinMode(zeroPin, INPUT);
  pinMode(tahoPin, INPUT);
  
  attachInterrupt(0, irs_detect_zero, FALLING);
  attachInterrupt(1, irs_detect_taho, FALLING);

  StartTimer1(irs_tm_40us, 40);
  
  Dimmer = 175;
  
  myPID.SetMode(AUTOMATIC);
  Serial.println("Start");
  
  targetMotorRpm = 0;
  motorRpm = 0;
}

void setup() {
  // put your setup code here, to run once:
  stdout = serial_stream;
  Serial.begin(115200);
  
  setupMotorController();

  pinMode(pinSpeed1, INPUT_PULLUP);
  pinMode(pinSpeed2, INPUT_PULLUP);
  pinMode(pinSpeedSave, INPUT_PULLUP);
  pinMode(pinVira, INPUT_PULLUP);
  pinMode(pinMina, INPUT_PULLUP);
  pinMode(pinEncPlus, INPUT_PULLUP);
  pinMode(pinEncMinus, INPUT_PULLUP);


  
  swSpeed1.attach(pinSpeed1);
  swSpeed2.attach(pinSpeed2);
  swSpeedSave.attach(pinSpeedSave);
  swVira.attach(pinVira);
  swMina.attach(pinMina);
  encIn1.attach(pinEncPlus);
  encIn2.attach(pinEncMinus);
  encIn1.interval(5);
  encIn2.interval(5);


  pinMode(pinReleMotor, OUTPUT);
  digitalWrite(pinReleMotor, 0);

  restoreSpeedSettings();


  xTaskCreate(
    TaskProcessMotor
    ,  "ProcessMotor"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );
    
  xTaskCreate(
    TaskProcessUserInput
    ,  "UserInput"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );
  
}

void loop() {
  // put your main code here, to run repeatedly:

}

void restoreSpeedSettings()
{
  if(EEPROM[0] == 255)
  {
    speedSettings[0] = 3000;
    speedSettings[1] = 300;
    speedSettings[2] = 1000;
    Serial.println("eeprom is empty.");
  }
  else
  {
    int buff = 0;
    
    EEPROM.get(sizeof(int) * 0, buff);
    speedSettings[0] = buff;

    EEPROM.get(sizeof(int) * 1, buff);
    speedSettings[1] = buff;

    EEPROM.get(sizeof(int) * 2, buff);
    speedSettings[2] = buff;
    Serial.println("eeprom readed");
  }
}



int getSwSpeedState(){
  swSpeed1.update();
  swSpeed2.update();
  
  return swSpeed1.read() * 1 + swSpeed2.read() * 2;
}

int getSwDirectionState(){
  swVira.update();
  swMina.update();
  
  return swVira.read() * 1 + swMina.read() * 2;
}


void processEncoder()
{
  int u1 = encIn1.update();
  int u2 = encIn2.update();
  
  
  
  int result = 0;
  
  if(u1 || u2){
    int v1 = encIn1.read();
    int v2 = encIn2.read();
    
    if(encoderFramePosition > 3){
      encoderFramePosition = 0;
    }
    encoderFrame[encoderFramePosition] = v1 * 1 + v2 * 2;
    encoderFramePosition++;

    if(v1 && v2)
    {
      // последний фрэйм, разбираемся с буфером
      switch(encoderFramePosition)
      {
        case 1:
          lastDirection = result = 0;
          break;
        case 2:
          result = lastDirection;
          break;
        case 3:
          result = lastDirection;
          break;
        case 4:
        {
          if(encoderFrame[0] == 2 && encoderFrame[2] == 1)
          {
            lastDirection = result = 1;
          }
          else
          {
            if(encoderFrame[0] == 1 && encoderFrame[2] == 2)
            {
              lastDirection = result = -1;
            }
            else
            {
              result = 0;
            }
          }
        } 
        break;
        default:
          result = 0;
        break;
      }
      
      encoderFramePosition = 0;
      onEncoderTick(result);
    }
  }
}


void onEncoderTick(int incr){
  switch(curSwSpeedState)
  {
    case 1:
    currentMemorySpeed+= 50 * incr;
    break;
    case 2:
    currentMemorySpeed+= 5 * incr;
    break;
    case 3:
    currentMemorySpeed+= 10* incr;
    break;
  }
  updateCurrentRpmValue(currentMemorySpeed);
  Serial.print("speed changed: ");
  Serial.println(currentMemorySpeed);
}



void updateCurrentRpmValue(int newValue){
  if(curSwDirectionState == 1 || curSwDirectionState == 2){
    targetMotorRpm = currentMemorySpeed;
  }
}

void setRpmDirection(int val){
  digitalWrite(pinReleMotor, val == 2);
}
int inStopProcessor = 0;

void processDirectionChange()
{
  int newSwDirectionState = getSwDirectionState();
  
  if(newSwDirectionState != curSwDirectionState)
  {
    switch(newSwDirectionState){
      case 1:
        Serial.println("direction: start plus");
        targetMotorRpm = 0;
        setPwm(180);
        vTaskDelay(50 / portTICK_PERIOD_MS );
        setRpmDirection(newSwDirectionState);
        vTaskDelay(50 / portTICK_PERIOD_MS );
        targetMotorRpm = currentMemorySpeed;
        Serial.println("direction: plus");
      break;
      case 2:
        Serial.println("direction: start minus");
        targetMotorRpm = 0;
        setPwm(180);
        vTaskDelay(50 / portTICK_PERIOD_MS );
        setRpmDirection(newSwDirectionState);
        vTaskDelay(50 / portTICK_PERIOD_MS );
        targetMotorRpm = currentMemorySpeed;
        Serial.println("direction: minus");
      break;
      case 3:
        Serial.println("direction: stoping");
        targetMotorRpm = 0;
        setPwm(180);
        
        printf("f1: up time %d down time %d taho %d dimm %d\n",lastUpTime, lastDownTime, lastTaho, Dimmer);
        
        if(motorRpm > 0){
          for(int i =0; i < 10; i++)
          {
            lastUpTime = 0;
            lastDownTime = 0;
            vTaskDelay(50 / portTICK_PERIOD_MS );
            printf("up time %d down time %d taho %d dimm %d\n",lastUpTime, lastDownTime, lastTaho, Dimmer);
            if(lastUpTime == 0)
            {
              printf("motor power down\n");
              break;
            }
          }
          
          setRpmDirection(curSwDirectionState == 2 ? 1 : 2);
          vTaskDelay(50 / portTICK_PERIOD_MS );
          
          getRpmMotor();

          inStopProcessor = 1;
          
          for(int i=0;i<10;i++)
          {
            getRpmMotor();
            int startRpm = motorRpm;
            printf("down motor from %d", motorRpm);
            
            if(motorRpm > 500)
            {
              setPwm(160);
            }
            else
            {
              printf("end down motor by %d\n", motorRpm);
              break;
            }
            vTaskDelay(100 / portTICK_PERIOD_MS );
            getRpmMotor();
            printf("to %d\n", motorRpm);
            if(startRpm < motorRpm)
            {
              printf("error down motor by %d -> %d\n", startRpm, motorRpm);
              break;
            }
          }
          
          setPwm(180);
          inStopProcessor = 0;
          
          //vTaskDelay(20 / portTICK_PERIOD_MS );
          //setPwm(170);
          //vTaskDelay(200 / portTICK_PERIOD_MS );
          //setPwm(180);
        }
        Serial.println("direction: stop");
      break;
    }
    
    curSwDirectionState = newSwDirectionState;
  }
}

void TaskProcessUserInput(void *pvParameters)
{
  (void) pvParameters;
  for (;;)
  {
    int newSwSpeedState = getSwSpeedState();
    
    if(newSwSpeedState != curSwSpeedState)
    {
      curSwSpeedState = newSwSpeedState;
      currentMemorySpeed = speedSettings[curSwSpeedState-1];
      Serial.print("sw speed: ");
      Serial.println(currentMemorySpeed);
      updateCurrentRpmValue(currentMemorySpeed);
    }
  
    processDirectionChange();
    
    if(swSpeedSave.update() && !swSpeedSave.read()){
       if(currentMemorySpeed > 0 && currentMemorySpeed < 20000)
       {
          if(speedSettings[curSwSpeedState-1] != currentMemorySpeed)
          {
            speedSettings[curSwSpeedState-1] = currentMemorySpeed;
            EEPROM.put(sizeof(int) * (curSwSpeedState-1), currentMemorySpeed);
            Serial.println("save speed settings");
          } 
       }
    }

    processEncoder();
  }
}


void TaskProcessMotor(void *pvParameters)
{
  (void) pvParameters;
  for (;;)
  {
    getRpmMotor();

    /*if(targetMotorRpm > 0)
    {*/
      Setpoint = targetMotorRpm;
      Input = motorRpm;
      
      myPID.Compute(); //  здесь происходят вычисления пид-регулятора - то есть pwmOut
    
      int target =  map(Output, 0, 255, 175, 1);
      if(targetMotorRpm > 0)
      {
        setPwm(target);
      }
      else
      {
        if(!inStopProcessor)
        {
          setPwm(180);
        }
      }
    /*}
    else
    {
      setPwm(180);
    }*/
  }
}

void getRpmMotor(){
   motorRpm = (1000000.0 / ((float)lastTaho * 40.0 * 8)) * 60;
}

void setPwm(int value){
    if(value > 175)
    {
      if(Dimmer != 175)
      {
        Dimmer = 175;
        digitalWrite(dimPin, 0);
        Serial.println("down tiristor");
      }
      return;
    }
    if(value < 1){
      Dimmer = 1;
      return;
    }
    Dimmer = value;
}


//----------------------ОБРАБОТЧИКИ ПРЕРЫВАНИЙ--------------------------
void irs_tm_40us() {       // прерывания таймера срабатывают каждые 40 мкс
  tic_taho++;
  tic++;                       // счетчик
  if (tic == Dimmer && Dimmer < 175 && Dimmer > 0){            // если настало время включать ток
    digitalWrite(dimPin, 1);   // врубить ток
    lastUpTime = tic;
    dimmState = 1;
  }
  
  if (tic == Dimmer + 10){
    digitalWrite(dimPin, 0);
    lastDownTime = tic;
    dimmState = 0;
  }

  if(tic_taho > 25000){
    lastTaho = 0;
    tic_taho = 0;    
    tahoSteps = 0;
  }
}

void  irs_detect_zero() {    // обработка внешнего прерывания на пересекание нуля снизу
  if(dimmState){
    lastDownTime = tic;
    digitalWrite(dimPin, 0);                  // вырубить ток
    dimmState = 0;
  }
  
  lastPeriod = tic;
  tic = 0;                                  // обнулить счетчик
  //ResumeTimer1();                           // перезапустить таймер
}


void irs_detect_taho(){
  tahoSteps++;
  //if(tahoSteps == 2){
    lastTaho = tic_taho;
    tic_taho = 0;
    tahoSteps = 0;
  //}
}












