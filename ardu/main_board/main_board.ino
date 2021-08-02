#include <Arduino_FreeRTOS.h>
//#include <FreeRTOS_AVR.h>
#include <mcp2515.h>
#include <math.h>

#define axis_1_angle 3

void rt_mat( float *src, float *angle, float *pppp, float *xyz ){
    
    float &in_x = src[0];
    float &in_y = src[1];
    float &in_z = src[2];

    float &theta_x = angle[0];
    float &theta_y = angle[1];
    float &theta_z = angle[2];

    float buff_x_0 =  in_x;
    float buff_y_0 = (in_y*cos(theta_x)) - (in_z*sin(theta_x));
    float buff_z_0 = (in_y*sin(theta_x)) + (in_z*cos(theta_x));

    float buff_x_1 =  (buff_x_0*cos(theta_y)) + (buff_z_0*sin(theta_y));
    float buff_y_1 =   buff_y_0;
    float buff_z_1 = -(buff_x_0*sin(theta_y)) + (buff_z_0*cos(theta_y));

    float buff_x_2 = (buff_x_1*cos(theta_z)) - (buff_y_1*sin(theta_z));
    float buff_y_2 = (buff_x_1*sin(theta_z)) + (buff_y_1*cos(theta_z));
    float buff_z_2 =  buff_z_1;

    xyz[0] = buff_x_2;
    xyz[1] = buff_y_2;
    xyz[2] = buff_z_2;

}

void euler_distance( float *base, float *platform, float *point ){

  float a = platform[0] - base[0] ; 
  float b = platform[1] - base[1] ; 
  float c = platform[2] - base[2] ; 

  point[0] = (  sqrt( (a*a) + (b*b) + (c*c) )  ) ;

}

void math_maker( float *base_points, float *frame_points ){

  const float dfd[3] PROGMEM = {1,0,0};
  
  const float dfd_0[3] PROGMEM = {1,-0.1,1};
  const float dfd_1[3] PROGMEM = {1, 0.1,1};
  const float dfd_2[3] PROGMEM = {1,-0.1,1};
  const float dfd_3[3] PROGMEM = {1, 0.1,1};
  const float dfd_4[3] PROGMEM = {1,-0.1,1};
  const float dfd_5[3] PROGMEM = {1, 0.1,1};

  const float angle_0[3] PROGMEM = {0,0,( 30.0f/180.0f)*M_PI};
  const float angle_1[3] PROGMEM = {0,0,( 90.0f/180.0f)*M_PI};
  const float angle_2[3] PROGMEM = {0,0,(150.0f/180.0f)*M_PI};
  const float angle_3[3] PROGMEM = {0,0,(210.0f/180.0f)*M_PI};
  const float angle_4[3] PROGMEM = {0,0,(270.0f/180.0f)*M_PI};
  const float angle_5[3] PROGMEM = {0,0,(330.0f/180.0f)*M_PI};

  float transpose[3] = {0,0,0};

  rt_mat( dfd, angle_0, transpose, base_points    );
  rt_mat( dfd, angle_1, transpose, base_points+3  );
  rt_mat( dfd, angle_2, transpose, base_points+6  );
  rt_mat( dfd, angle_3, transpose, base_points+9  );
  rt_mat( dfd, angle_4, transpose, base_points+12 );
  rt_mat( dfd, angle_5, transpose, base_points+15 );

  rt_mat( dfd_0, angle_0, transpose, frame_points    );
  rt_mat( dfd_1, angle_1, transpose, frame_points+3  );
  rt_mat( dfd_2, angle_2, transpose, frame_points+6  );
  rt_mat( dfd_3, angle_3, transpose, frame_points+9  );
  rt_mat( dfd_4, angle_4, transpose, frame_points+12 );
  rt_mat( dfd_5, angle_5, transpose, frame_points+15 );

}

struct can_frame canMsg1;
struct can_frame canMsg2;

MCP2515 mcp2515(10);

unsigned int SerialParsingTime = 0 ;
unsigned int SerialNewCounter = 0 ;
unsigned int Converted = 0 ;

long xStart , xEnd , xDifference ;

//xEnd = micros();
//xDifference = xEnd - xStart;
//xStart = micros();

union pc_data {
  char rawArray[24];
  float anglarAndGforce[6];
};
union pc_data pcData;

union axis_data {
  char rawArray[12];
  uint16_t actuator[6];
};
union axis_data axisData;

//TIMER2 COMPA Timer/Counter2 compare match A
//TIMER2 COMPB Timer/Counter2 compare match B
//TIMER2 OVF Timer/Counter2 overflow
//TIMER1 CAPT Timer/Counter1 capture event
//TIMER1 COMPA Timer/Counter1 compare match A
//TIMER1 COMPB Timer/Counter1 compare match B
//TIMER1 OVF Timer/Counter1 overflow
//TIMER0 COMPA Timer/Counter0 compare match A
//TIMER0 COMPB Timer/Counter0 compare match B
//TIMER0 OVF

ISR(TIMER2_OVF_vect)
{

  static char a = 0;
  a += 1;
  a &= 0b111;

  if (a == 0)
  {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
  }

  digitalWrite(a+2, LOW);
  
  xDifference++;

}

void ActuatorPositionSignalGenerator()
{

  TCCR2A = 0b011; // Mode3 Fast PWM
  TCCR2B = 0b111; // clkT2S/64 (From prescaler) 4us Period
  TIMSK2 = 0x001; // Timer/Counter2 Overflow Interrupt Enable
  TCNT2  = 1;    // 16000000/64/(256-6)=1000Hz=1ms
  SREG   = 0x80;

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

}

void TaskBlinkInit()
{

  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS);
  mcp2515.setNormalMode();

  canMsg1.can_id  = 0x100 | 0x00;
  canMsg1.can_dlc = 8;
  canMsg2.can_id  = 0x100 | 0x01;
  canMsg2.can_dlc = 4;

}
void TaskBlink(void *pvParameters)
{
 
  (void) pvParameters;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while(1)
  {

    if(Converted)
    {

      Converted = 0;

      canMsg1.data[0] = axisData.rawArray[0];
      canMsg1.data[1] = axisData.rawArray[1];
      canMsg1.data[2] = axisData.rawArray[2];
      canMsg1.data[3] = axisData.rawArray[3];
      canMsg1.data[4] = axisData.rawArray[4];
      canMsg1.data[5] = axisData.rawArray[5];
      canMsg1.data[6] = axisData.rawArray[6];
      canMsg1.data[7] = axisData.rawArray[7];

      canMsg2.data[0] = axisData.rawArray[8];
      canMsg2.data[1] = axisData.rawArray[9];
      canMsg2.data[2] = axisData.rawArray[10];
      canMsg2.data[3] = axisData.rawArray[11];

      mcp2515.sendMessage(&canMsg1);
      mcp2515.sendMessage(&canMsg2);
    
    }
    
    xTaskDelayUntil( &xLastWakeTime, 2 );

  }

}

void TaskAnalogReadInit()
{
  
}
void TaskAnalogRead(void *pvParameters)
{

  (void) pvParameters;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  float base_points[18] = {  0,  };
  float frame_points[18] = {  0,  };
  float low_length[1] = {  0,  };

  math_maker( base_points, frame_points );

  euler_distance(  base_points, frame_points, low_length   );

  while(1)
  {

    if(SerialNewCounter)
    {
      
      SerialNewCounter = 0;
      
      float frame_points_transformed[3] = {  0,  };
      float actuator_length[6] = {  0,  };
      float angle[3] = {  
        pcData.anglarAndGforce[0],  
        pcData.anglarAndGforce[1],  
        pcData.anglarAndGforce[2]
      };
      float transpose[3] = {  
        pcData.anglarAndGforce[3],  
        pcData.anglarAndGforce[4],  
        pcData.anglarAndGforce[5]  
      };

      for (int a = 0; a <6; a++)
      {
        rt_mat( frame_points+(3*a), angle, transpose, frame_points_transformed    );
        euler_distance(  base_points   , frame_points_transformed, actuator_length   );
        axisData.actuator[a] = actuator_length[a]*100+0.001;
      }

      Converted++;

    }
    
    xTaskDelayUntil( &xLastWakeTime, 1 );

  }
  
}

void TaskMonitorInit()
{
  Serial.begin(115200);
  Serial.println("Example: Write to CAN");
}
void TaskMonitor(void *pvParameters)
{
  
  (void) pvParameters;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  int counter = 0;

  while(1)
  {

    //Serial.print(actuator_length[0]), Serial.print(" ");
    //Serial.print(actuator_length[1]), Serial.print(" ");
    //Serial.print(actuator_length[2]), Serial.print(" ");
    //Serial.print(actuator_length[3]), Serial.print(" ");
    //Serial.print(actuator_length[4]), Serial.print(" ");
    //Serial.print(actuator_length[5]), Serial.print(" ");

    //if(counter==0)Serial.print(pcData.anglarAndGforce[0]), Serial.print(" ");
    //else if(counter==1)Serial.print(pcData.anglarAndGforce[1]), Serial.print(" ");
    //else if(counter==2)Serial.print(pcData.anglarAndGforce[2]), Serial.print(" ");
    //else if(counter==3)Serial.print(pcData.anglarAndGforce[3]), Serial.print(" ");
    //else if(counter==4)Serial.print(pcData.anglarAndGforce[4]), Serial.print(" ");
    //else if(counter==5)Serial.print(pcData.anglarAndGforce[5]), Serial.print(" ");
    //else if(counter==6)Serial.print(SerialParsingTime), Serial.print(" "), SerialParsingTime = 0;
    //else if(counter==7)Serial.print(SerialNewCounter), Serial.print("  "), SerialNewCounter = 0;
    //else counter = 0b1111;
    Serial.print(xDifference);
    Serial.print("  ");
    Serial.println();

    //vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    
    xTaskDelayUntil( &xLastWakeTime, 66 );

  }

}

void TaskSerialControlInit()
{

}
void TaskSerialControl(void *pvParameters)
{

  (void) pvParameters;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  static unsigned char reset_v = 0;
  static unsigned char main_revice_size = 0;
  static unsigned char incomingByte;
  static unsigned char linit_counter = 0;
  
  while(1)
  {
    
    while (Serial.available())
    {
      
      linit_counter++;

      if (linit_counter>50) {
        linit_counter = 0;
        break;
      }
  
      incomingByte = Serial.read();
      
      if(reset_v!=4){

        if ((incomingByte=='0')&&(reset_v==0)) reset_v = 1;
        else if (  (incomingByte=='1')  &&  (reset_v==1)  ) reset_v = 2;
        else if (  (incomingByte=='2')  &&  (reset_v==2)  ) reset_v = 3;
        else if (  (incomingByte=='3')  &&  (reset_v==3)  ) reset_v = 4;
        else reset_v = 0;

      } else {

        pcData.rawArray[main_revice_size] = incomingByte;

        main_revice_size++;

        if(main_revice_size>23) 
        {
          reset_v = 5, main_revice_size = 0, SerialNewCounter++;
        }

      }

    }
    
    SerialParsingTime ++;

    xTaskDelayUntil( &xLastWakeTime, 1 );

  }

}

void setup() {


  ActuatorPositionSignalGenerator();
  TaskBlinkInit();
  TaskAnalogReadInit();
  TaskMonitorInit();
  TaskSerialControlInit();


  while (!Serial);
  

  xTaskCreate(
    TaskMonitor
    ,  "Blink"   // A name just for humans
    ,  80 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskAnalogRead
    ,  "Blink"   // A name just for humans
    ,  412 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskBlink
    ,  "Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskSerialControl
    ,  "AnalogRead"
    ,   128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );


  //vTaskStartScheduler(); //OS동작 시작


}

void loop() { }
