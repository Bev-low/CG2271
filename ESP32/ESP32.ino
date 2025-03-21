#include <ps5Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

#define UART_BAUD_RATE 115200
#define UART_TX_PIN 17  // ESP32 TX → Freedom Board RX
#define UART_RX_PIN 16 
unsigned long lastTimeStamp = 0;

/*

void notify()
{
  char messageString[200];
  sprintf(messageString, "%4d,%4d,%4d,%4d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
  ps5.LStickX(),
  ps5.LStickY(),
  ps5.RStickX(),
  ps5.RStickY(),
  ps5.Left(),
  ps5.Down(),
  ps5.Right(),
  ps5.Up(),
  ps5.Square(),
  ps5.Cross(),
  ps5.Circle(),
  ps5.Triangle(),
  ps5.L1(),
  ps5.R1(),
  ps5.L2(),
  ps5.R2(),  
  ps5.Share(),
  ps5.Options(),
  ps5.PSButton(),
  ps5.Touchpad(),
  ps5.Charging(),
  ps5.Audio(),
  ps5.Mic(),
  ps5.Battery());

  //Only needed to print the message properly on serial monitor. Else we dont need it.
  if (millis() - lastTimeStamp > 50)
  {
    Serial.println(messageString);
    lastTimeStamp = millis();
  }
}

*/

void onConnect()
{
  Serial.println("Connected!.");
}

void onDisConnect()
{
  Serial.println("Disconnected!.");    
}

void setup() 
{
  pinMode(34, OUTPUT);
  //pinMode(33, OUTPUT);
  //digitalWrite(34, HIGH);
  //delay(1000);
  Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN); 
  Serial.begin(UART_BAUD_RATE);

  //ps5.attach(notify);
  ps5.attachOnConnect(onConnect);
  ps5.attachOnDisconnect(onDisConnect);

  ps5.begin("88:03:4c:3d:ad:b7");  //your PS5 controller mac address 
  while (ps5.isConnected() == false) 
  { 
    Serial.println("PS5 controller not connected!");
    delay(300);
  }
  Serial.println("Ready.");

}

void loop() {
    if (ps5.isConnected()) {
        uint8_t command = getCommandFromPS5();
        Serial.print("Sending Command: ");
        Serial.println(command, BIN);
        Serial2.write(command);  // Send 8-bit data packet to Freedom Board
        delay(100);  // Small delay to prevent spamming
    }
}

uint8_t getCommandFromPS5() {

  int LeftStickX = ps5.LStickX();
  int LeftStickY = ps5.LStickY();


    //Serial.print("Left Stick X: "); Serial.print(LeftStickX);
    //Serial.print(" | Left Stick Y: "); Serial.println(LeftStickY);

    if (LeftStickY > 50) 
        digitalWrite(34, HIGH);
        return 0b00000001;  // Move Forward
    if (LeftStickY < -50) return 0b00000010;   // Move Backward
    if (LeftStickX < -50) return 0b00000011;  // Move Left
    if (LeftStickX > 50) return 0b00000100;   // Move Right
    if (ps5.Square()) return 0b10000000;         // Play Final Tune
    return 0b00000000;                           // Stop
}