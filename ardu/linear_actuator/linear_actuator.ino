#include <Arduino_FreeRTOS.h>
//#include <FreeRTOS_AVR.h>
#include <mcp2515.h>
#include <Servo.h>
#include <math.h>

struct can_frame canMsg;

MCP2515 mcp2515(10);

long xStart, xEnd, xDifference;

//xEnd = micros();
//xDifference = xEnd - xStart;
//xStart = micros();

union axis_data {
  char rawArray[12];
  uint16_t actuator[6];
};
union axis_data axisData;

unsigned int servo_0 = 50;
unsigned int servo_1 = 50;
unsigned int servo_2 = 50;
unsigned int servo_3 = 50;
unsigned int servo_4 = 50;
unsigned int servo_5 = 50;
unsigned int servo_counter = 0;
//ISR(PCINT0_vect) 
//{
//  //xDifference++;
//}
ISR(PCINT1_vect) 
{
  if(digitalRead(A0))
  {
    xStart = micros();
  }
  else
  {
    xEnd = micros();
    xDifference = xEnd - xStart;
  }
  //xDifference++;
}
//ISR(PCINT2_vect) 
//{
//  //xDifference++;
//}
ISR(TIMER2_OVF_vect)
{
  //xDifference++;
  servo_counter++;
  servo_counter &= 0b1111111;
    if (servo_counter<servo_0) PORTD |= 0b100;
    else PORTD &= 0b11111011;
    if (servo_counter<servo_1) PORTD |= 0b1000;
    else PORTD &= 0b11110111;
    if (servo_counter<servo_2) PORTD |= 0b10000;
    else PORTD &= 0b11101111;
    if (servo_counter<servo_3) PORTD |= 0b100000;
    else PORTD &= 0b11011111;
    if (servo_counter<servo_4) PORTD |= 0b1000000;
    else PORTD &= 0b10111111;
    if (servo_counter<servo_5) PORTD |= 0b10000000;
    else PORTD &= 0b01111111;
// xDifference = servo_counter;
}

void pcint()
{

  pinMode(A0, INPUT);

  //PCIFR = 0x04;
  PCICR = 0b00000010;
  //PCMSK0 = 0b00111111;
  PCMSK1 = 0b00000001;
  //PCMSK2 = 0b00111111;
  SREG = 0x80;

}

void ServoSignalGenerator()
{

  TCCR2A = 0b011; // Mode3 Fast PWM
  TCCR2B = 0b001; // clkT2S/64 (From prescaler) 4us Period
  TIMSK2 = 0x001; // Timer/Counter2 Overflow Interrupt Enable
  TCNT2  = 255;    // 16000000/64/(256-6)=1000Hz=1ms
  SREG   = 0x80;
  
  DDRD |= 0b11111100;

}

void TaskReadCanInit()
{
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS);
  mcp2515.setNormalMode();
}
void TaskReadCan(void *pvParameters)  // This is a task.
{

  (void) pvParameters;
  
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    
    while (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    
      if (0x0100==canMsg.can_id)
      {
    
        axisData.rawArray[0] = canMsg.data[0];
        axisData.rawArray[1] = canMsg.data[1];
        axisData.rawArray[2] = canMsg.data[2];
        axisData.rawArray[3] = canMsg.data[3];
        axisData.rawArray[4] = canMsg.data[4];
        axisData.rawArray[5] = canMsg.data[5];
        axisData.rawArray[6] = canMsg.data[6];
        axisData.rawArray[7] = canMsg.data[7];
    
      }
      
      else if (0x0101==canMsg.can_id)
      {
    
        axisData.rawArray[ 8] = canMsg.data[0];
        axisData.rawArray[ 9] = canMsg.data[1];
        axisData.rawArray[10] = canMsg.data[2];
        axisData.rawArray[11] = canMsg.data[3];
    
      }
    
    }
  
    xTaskDelayUntil( &xLastWakeTime, 1 );
  
  }

}

void TaskMonitor(void *pvParameters)  // This is a task.
{

  (void) pvParameters;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  for (;;)
  {

    Serial.print(canMsg.can_id, DEC), Serial.print(" "); 
    Serial.print(canMsg.can_dlc, DEC), Serial.print(" ");
    Serial.print(axisData.actuator[0]), Serial.print(" ");
    Serial.print(axisData.actuator[1]), Serial.print(" ");
    Serial.print(axisData.actuator[2]), Serial.print(" ");
    Serial.print(axisData.actuator[3]), Serial.print(" ");
    Serial.print(axisData.actuator[4]), Serial.print(" ");
    Serial.print(axisData.actuator[5]), Serial.print(" ");
    Serial.print(xDifference), Serial.print(" ");
    Serial.println(); 

    xTaskDelayUntil( &xLastWakeTime, 64 );

  }

}

void TaskServo(void *pvParameters)
{

  (void) pvParameters;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {

    //servo0.write(axisData.actuator[0]); 
    //servo1.write(axisData.actuator[1]); 
    //servo2.write(axisData.actuator[2]); 
    //servo3.write(axisData.actuator[3]); 
    //servo4.write(axisData.actuator[4]); 
    //servo5.write(axisData.actuator[5]); 

    servo_0=axisData.actuator[0]/4;
    servo_1=axisData.actuator[1]/4;
    servo_2=axisData.actuator[2]/4;
    servo_3=axisData.actuator[3]/4;
    servo_4=axisData.actuator[4]/4;
    servo_5=axisData.actuator[5]/4;

    //if(servo_0>100){
    //    servo_0 = 0;
    //    servo_1 = 0;
    //    servo_2 = 0;
    //    servo_3 = 0;
    //    servo_4 = 0;
    //    servo_5 = 0;
    //}

    xTaskDelayUntil( &xLastWakeTime, 1 );

  }

}

void setup() {

  pcint();
  ServoSignalGenerator();
  TaskReadCanInit();

  Serial.begin(115200);
  
  xTaskCreate(
    TaskServo
    ,  "sssss"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  
  xTaskCreate(
    TaskReadCan
    ,  "Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskMonitor
    ,  "AnalogRead"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );
      
  //vTaskStartScheduler(); //OS동작 시작


}

void loop() {}
