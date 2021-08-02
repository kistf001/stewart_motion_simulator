//#include <Servo.h> 
// 
//int servoPin = 9;
//
//Servo servo; 
//
//int angle = 0; // servo position in degrees 
//
//void setup() 
//{ 
//    servo.attach(servoPin); 
//} 
//
//
//void loop() 
//{ 
//  // scan from 0 to 180 degrees
//  for(angle = 0; angle < 180; angle++) 
//  { 
//    servo.write(angle); 
//    delay(15); 
//  } 
//  // now scan back from 180 to 0 degrees
//  for(angle = 180; angle > 0; angle--) 
//  { 
//    servo.write(angle); 
//    delay(15); 
//  } 
//} 



#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg1;
struct can_frame canMsg2;

MCP2515 mcp2515(10);

#include<math.h>

void setup() {
  
	while (!Serial);
	Serial.begin(115200);
	
	mcp2515.reset();
	mcp2515.setBitrate(CAN_125KBPS);
	mcp2515.setNormalMode();
	
	Serial.println("Example: Write to CAN");

	canMsg1.can_id  = 0x0100|0x00;
	canMsg1.can_dlc = 8;
	canMsg1.data[0] = 121;
	canMsg1.data[1] = 121<<8;
	canMsg1.data[2] = 121;
	canMsg1.data[3] = 121<<8;
	canMsg1.data[4] = 121;
	canMsg1.data[5] = 121<<8;
	canMsg1.data[6] = 121;
	canMsg1.data[7] = 121<<8;
	canMsg2.can_id  = 0x0100|0x01;
	canMsg2.can_dlc = 4;
	canMsg2.data[0] = 232;
	canMsg2.data[1] = 232<<8;
	canMsg2.data[2] = 232;
	canMsg2.data[3] = 232<<8;
}

void loop() {

	mcp2515.sendMessage(&canMsg1);
	mcp2515.sendMessage(&canMsg2);

	Serial.println("Messages sent");
	
	delay(1000);

}