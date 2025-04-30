#include <ESP32-TWAI-CAN.hpp>
#include <LiquidCrystal_I2C.h>
#include "Motor.h"


// Default for ESP32
#define CAN_TX		5
#define CAN_RX		4


float torque = 1;


Motor motor1(0x402);
CanFrame rxFrame;

/******************************************************/
void sendCANMessage(CanFrame* msg){
    ESP32Can.writeFrame(msg);
}

/******************************************************/
void receivedCANMessage(CanFrame* msg){
    ESP32Can.readFrame(msg);
}

void setup() {
    Serial.begin(115200);
    Serial.println("starting");
    while (!Serial) {};

/*-----------------------CAN SETUP---------------------------*/
    //Set pins
    pinMode(CAN_TX, OUTPUT);
    pinMode(CAN_RX, OUTPUT);
	ESP32Can.setPins(CAN_TX, CAN_RX);
	
    // set Rx/Tx queue
    ESP32Can.setRxQueueSize(5);
	ESP32Can.setTxQueueSize(5);

    ESP32Can.setSpeed(ESP32Can.convertSpeed(1000));//CAN Speed: 1 MHZ

    if(ESP32Can.begin()) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
    }
/*-----------------------MOTOR SETUP-----------------------*/
    CanFrame setup;
    motor1.zeroSet(&setup);
    sendCANMessage(&setup);
    motor1.enterMode(&setup);
    sendCANMessage(&setup);
}
bool setDirection = true;
void loop() {
    CanFrame loop;
    char rc;  
    if(!setDirection){
        //float p_des, float v_des, float kp, float kd, float t_ff
        motor1.pack_cmd(&loop,3.14F, 0.0F, 3.0F, 1.0F, 0.0F);
        sendCANMessage(&loop);
        setDirection = true;
        delay(100);
        
    }
   
     rc = Serial.read();
     delay(500);
     if(rc == 'e'){
         motor1.exitMode(&loop);
         sendCANMessage(&loop);
         Serial.println("Exit mode");
     }
     else if(rc == 's'){
         motor1.enterMode(&loop);
         sendCANMessage(&loop);
         Serial.println("Enter mode");
     }
     else if(rc == 'l'){
         motor1.zeroSet(&loop);
         sendCANMessage(&loop);
         Serial.println("Zero set");
     }
     else if(rc == 'a'){
         //float p_des, float v_des, float kp, float kd, float t_ff
         motor1.pack_cmd(&loop,1.0F, 1.0F, 1.0F, 1.0F, -((torque/0.67)-1));
         sendCANMessage(&loop);
         receivedCANMessage(&loop);
         motor1.unpack_reply(&loop);
         Serial.println(torque);
         torque += 0.5;
         Serial.println("Command send: a");
     }
     else if(rc == 'b'){
         motor1.unpack_reply(&loop);
         Serial.println(torque);
     }
     else if(rc == 'v'){
         torque = 0;
         Serial.println(torque);
     }
     delay(10);
    
}


