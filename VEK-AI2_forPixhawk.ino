#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md(5, 4, 6, A0, 7, 8, 12, A1);

const uint16_t  PWM1ReadPin = 2;
const uint16_t  PWM2ReadPin = 3;

#define PWM_1_INPUT_MAX      (1920)
#define PWM_1_INPUT_MIN     (1120)
#define PWM_1_INPUT_MID     (1520)
#define PWM_1_OUTPUT_LIMIT    (100)
#define PWM_1_INPUT_DEADZONE_H  (PWM_1_INPUT_MID+20)
#define PWM_1_INPUT_DEADZONE_L  (PWM_1_INPUT_MID-20)

#define PWM_2_INPUT_MAX     (1920)
#define PWM_2_INPUT_MIN     (1120)
#define PWM_2_INPUT_MID     (1520)
#define PWM_2_INPUT_DEADZONE_H  (PWM_2_INPUT_MID+20)
#define PWM_2_INPUT_DEADZONE_L  (PWM_2_INPUT_MID-20)

#define UART_COMMAND_MAX    (2000)
#define UART_COMMAND_MID    (1500)
#define UART_COMMAND_MIN    (1000)

#define PWM_MOTOR_DUTY_RESOLUTION (400)

uint32_t  PWM1_pulseTimeH_start, PWM1_pulseTimeH_end;
uint32_t  PWM1_pulseTimeH;
bool      oldPWMIN1Pin_state, PWMIN1Pin_state;
bool      fCapturePWM1 = 0;
void capturePWM1()
{
  PWMIN1Pin_state = digitalRead(PWM1ReadPin);
  if ( PWMIN1Pin_state == 1 )
  {
    PWM1_pulseTimeH_start = micros();
  }
  else if ( PWMIN1Pin_state == 0)
  {
    PWM1_pulseTimeH_end = micros();
    PWM1_pulseTimeH = PWM1_pulseTimeH_end - PWM1_pulseTimeH_start;
    if(PWM1_pulseTimeH > PWM_1_INPUT_MAX)
    {
      PWM1_pulseTimeH = PWM_1_INPUT_MAX;
    }
    if(PWM1_pulseTimeH < PWM_1_INPUT_MIN)
    {
      PWM1_pulseTimeH = PWM_1_INPUT_MIN;
    }
    fCapturePWM1 = 1;
  }
}

uint32_t  PWM2_pulseTimeH_start, PWM2_pulseTimeH_end;
uint32_t  PWM2_pulseTimeH;
bool      oldPWMIN2Pin_state, PWMIN2Pin_state;
bool      fCapturePWM2 = 0;
void capturePWM2()
{
  PWMIN2Pin_state = digitalRead(PWM2ReadPin);
  if ( PWMIN2Pin_state == 1 )
  {
    PWM2_pulseTimeH_start = micros();
  }
  else if ( PWMIN2Pin_state == 0)
  {
    PWM2_pulseTimeH_end = micros();
    PWM2_pulseTimeH = PWM2_pulseTimeH_end - PWM2_pulseTimeH_start;
    if(PWM2_pulseTimeH > PWM_2_INPUT_MAX)
    {
      PWM2_pulseTimeH = PWM_2_INPUT_MAX;
    }
    if(PWM2_pulseTimeH < PWM_2_INPUT_MIN)
    {
      PWM2_pulseTimeH = PWM_2_INPUT_MIN;
    }
    fCapturePWM2 = 1;
  }
}
void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial2.println("M1 fault");
    while (1);
  }
  if (md.getM2Fault())
  {
    Serial2.println("M2 fault");
    while (1);
  }
}


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  pinMode(PWM1ReadPin, INPUT_PULLUP);
  pinMode(PWM2ReadPin, INPUT_PULLUP);
  md.init();
  delay(2000);  //受信機起動待ち
  attachInterrupt(digitalPinToInterrupt(PWM1ReadPin), capturePWM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM2ReadPin), capturePWM2, CHANGE);
  Serial2.println("Start...");
}

uint32_t  UARTtimer = 0;
uint32_t  PWMWatchTimer = 0;

int16_t calcDutyPWM1 = 0;
int16_t calcDutyPWM2 = 0;
int16_t calcSteeringLeft = 0,calcSteeringRight = 0;
int16_t calcMotorLeftDutyFromPropo=0,calcMotorRightDutyFromPropo=0;
uint32_t  oldPWM1_pulseTimeH_end=0, oldPWM2_pulseTimeH_end=0;
uint8_t cNoCapturePWM1 = 0,cNoCapturePWM2 = 0;
bool fNoCapturePWM1 = 0,fNoCapturePWM2 = 0;

int uartRcevData = 0;
uint8_t cUartRcevedPacket = 0;
bool fFoundUartHeader = 0;
uint16_t uartLeftMotorCommandVal = 1500;
uint16_t uartRightMotorCommandVal = 1500;
unsigned long uartRcevedTime_ms = 0;
bool fRcevedUartMotorCommand = 0;
bool fNoRecevUartCommand = 0;

int16_t calcMotorLeftDutyFromUart=0,calcMotorRightDutyFromUart=0;
int16_t setMotorLeftDuty=0,setMotorRightDuty=0;

void loop() {
  //J120からのUART受信処理
  if(Serial.available())
  {
    uartRcevData = Serial.read();
    uartRcevedTime_ms = millis();
    if( (uartRcevData == 0xff) && (fFoundUartHeader == 0) )
    {
      fFoundUartHeader = 1;
      cUartRcevedPacket = 0;
    }
    else if(fFoundUartHeader == 1)
    {
      switch(cUartRcevedPacket){
        case 0:
          uartLeftMotorCommandVal = uartRcevData<<8;
          break;
        case 1:
          uartLeftMotorCommandVal = uartLeftMotorCommandVal | uartRcevData;
          break;
        case 2:
          uartRightMotorCommandVal = uartRcevData<<8;
          break;
        case 3:
          uartRightMotorCommandVal = uartRightMotorCommandVal | uartRcevData;
          fFoundUartHeader = 0;
          if( ( (uartLeftMotorCommandVal >= 1000) && (uartLeftMotorCommandVal <= 2000) ) &&
              ( (uartRightMotorCommandVal >= 1000) && (uartRightMotorCommandVal <= 2000) ) )
          {
              fRcevedUartMotorCommand = 1;
              fNoRecevUartCommand = 0;
          }
          else
          {
            fRcevedUartMotorCommand = 0;
            uartLeftMotorCommandVal = 1500;
            uartRightMotorCommandVal = 1500;
          }
          break;
        default:
          cUartRcevedPacket = 0;
      }
      cUartRcevedPacket = (cUartRcevedPacket+1)%4;
    }
  }
  if(uartRcevedTime_ms + 1000 < millis())
  {
    fNoRecevUartCommand = 1;
    uartLeftMotorCommandVal = 1500;
    uartRightMotorCommandVal = 1500;
  }

  if(fRcevedUartMotorCommand == 1)
  {
    fRcevedUartMotorCommand = 0;
    if(uartLeftMotorCommandVal > UART_COMMAND_MID)
    {
      calcMotorLeftDutyFromUart =  ( (uartLeftMotorCommandVal - UART_COMMAND_MID) * (int32_t)(1*PWM_MOTOR_DUTY_RESOLUTION) )
                                    / ((int32_t)(UART_COMMAND_MAX - UART_COMMAND_MID));
    }
    else
    {
      calcMotorLeftDutyFromUart =  ( (UART_COMMAND_MID - uartLeftMotorCommandVal) * (int32_t)(-1*PWM_MOTOR_DUTY_RESOLUTION) )
                                    / ((int32_t)(UART_COMMAND_MID - UART_COMMAND_MIN));
    }
    if(uartRightMotorCommandVal > UART_COMMAND_MID)
    {
      calcMotorRightDutyFromUart =  ( (uartRightMotorCommandVal - UART_COMMAND_MID) * (int32_t)(-1*PWM_MOTOR_DUTY_RESOLUTION) )
                                    / ((int32_t)(UART_COMMAND_MAX - UART_COMMAND_MID));
    }
    else
    {
      calcMotorRightDutyFromUart =  ( (UART_COMMAND_MID - uartRightMotorCommandVal) * (int32_t)(1*PWM_MOTOR_DUTY_RESOLUTION) )
                                    / ((int32_t)(UART_COMMAND_MID - UART_COMMAND_MIN));
    }
  }

  //PWM無入力判定
  if( (PWMWatchTimer + 100) <= millis())
  {
    PWMWatchTimer = millis();
    if(oldPWM1_pulseTimeH_end == PWM1_pulseTimeH_end)
    {
      cNoCapturePWM1++;
    }
    else
    {
      cNoCapturePWM1=0;
    }
    oldPWM1_pulseTimeH_end = PWM1_pulseTimeH_end;
    
    if(oldPWM2_pulseTimeH_end == PWM2_pulseTimeH_end)
    {
      cNoCapturePWM2++;
    }
    else
    {
      cNoCapturePWM2=0;
    }
    oldPWM2_pulseTimeH_end = PWM2_pulseTimeH_end;
    
    if( (cNoCapturePWM1>=5) || (cNoCapturePWM2>=5) )
    {
      if(cNoCapturePWM1>=5) 
      {
        fNoCapturePWM1 = 1;
      }
      else
      {
        //Serial2.println("PWM2 No signal!");
        fNoCapturePWM2 = 1;
      }
    }
    else
    {
      fNoCapturePWM1 = 0;
      fNoCapturePWM2 = 0;
    }
  }

  //受信したパルス幅からモータPWMデューティ計算
  if (fCapturePWM1 == 1)
  {
    fCapturePWM1 = 0;
    if ( PWM_1_INPUT_DEADZONE_H <= PWM1_pulseTimeH)
    {
      calcDutyPWM1 =  ((PWM1_pulseTimeH - PWM_1_INPUT_DEADZONE_H) * (int32_t)100)
                      / ((int32_t)(PWM_1_INPUT_MAX - PWM_1_INPUT_DEADZONE_H));
    }
    else if ( PWM_1_INPUT_DEADZONE_L >= PWM1_pulseTimeH)
    {
      calcDutyPWM1 =  ((int32_t)( PWM_1_INPUT_DEADZONE_L - PWM1_pulseTimeH) * (int32_t) (-1 * 100) )
                      / ((int32_t)(PWM_1_INPUT_DEADZONE_L - PWM_1_INPUT_MIN));
    }
    else
    {
      calcDutyPWM1 = 0;
    }
  }
  if( fCapturePWM2 == 1)
  {
    fCapturePWM2 = 0;
    if ( PWM_2_INPUT_DEADZONE_H <= PWM2_pulseTimeH)
    {
      calcDutyPWM2 =  ((PWM2_pulseTimeH - PWM_2_INPUT_DEADZONE_H) * (int32_t)100)
                      / ((int32_t)(PWM_2_INPUT_MAX - PWM_2_INPUT_DEADZONE_H));
    }
    else if ( PWM_2_INPUT_DEADZONE_L >= PWM2_pulseTimeH)
    {
      calcDutyPWM2 =  ((int32_t)( PWM_2_INPUT_DEADZONE_L - PWM2_pulseTimeH) * (int32_t) (-1 * 100) )
                      / ((int32_t)(PWM_2_INPUT_DEADZONE_L - PWM_2_INPUT_MIN));
    }
    else
    {
      calcDutyPWM2 = 0;
    }
  }
 
  //モータ駆動ソース選択。UARTを優先する
  if( (fNoRecevUartCommand == 0) && ((uartLeftMotorCommandVal != UART_COMMAND_MID) || (uartRightMotorCommandVal != UART_COMMAND_MID)) )
  {
    setMotorLeftDuty = calcMotorLeftDutyFromUart;
    setMotorRightDuty = calcMotorRightDutyFromUart;
  }
  else
  {
    if( (fNoCapturePWM1 == 0) && (fNoCapturePWM2 == 0) )
    {
      setMotorLeftDuty = calcDutyPWM1*4;
      setMotorRightDuty = calcDutyPWM2*4;
    }
    //PWM無入力の場合モータ停止
    else
    {
      setMotorLeftDuty = 0;
      setMotorRightDuty = 0;
    }
  }
  md.setM1Speed(setMotorLeftDuty);
  md.setM2Speed(setMotorRightDuty);
  stopIfFault();

#if 1
  if ( (UARTtimer + 500) <= millis())
  {
    UARTtimer = millis();

    Serial2.print("PWM1:");
    Serial2.print(PWM1_pulseTimeH, DEC);
    Serial2.print(", ");
    Serial2.print("PWM2:");
    Serial2.print(PWM2_pulseTimeH, DEC);
    
    Serial2.print(", motorL:");
    Serial2.print(setMotorLeftDuty, DEC);
    Serial2.print(", motorR:");
    Serial2.print(setMotorRightDuty, DEC);
    Serial2.print(", ");
    Serial2.print(", uartL:");
    Serial2.print(uartLeftMotorCommandVal, DEC);
    Serial2.print(", ");
    Serial2.print(", uartR:");
    Serial2.print(uartRightMotorCommandVal, DEC);
    Serial2.println();

    Serial2.print("LeftCurrent: ");
    Serial2.print(md.getM1CurrentMilliamps());
    Serial2.print(", RightCurrent: ");
    Serial2.println(md.getM2CurrentMilliamps());
    Serial2.println();
  }
#endif
}

