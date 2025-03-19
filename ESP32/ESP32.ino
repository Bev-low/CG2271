#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

#define BAUD_RATE 115200
#define UART_TX_PIN 17  // ESP32 TX → Freedom Board RX
#define UART_RX_PIN 16  // Not used, but can be used for debugging

void setup() {
    Serial.begin(BAUD_RATE);
    Serial2.begin(BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);  // Initialize UART to send data to the Freedom Board

    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisconnect);
    PS4.begin();

    removePairedDevices();  // Helps with Bluetooth connection issues

    Serial.print("This device MAC is: ");
    printDeviceAddress();
    Serial.println("");
}

void loop() {
    if (PS4.isConnected()) {
        uint8_t command = getCommandFromPS4();
        Serial.print("Sending Command: ");
        Serial.println(command, BIN);
        Serial2.write(command);  // Send 8-bit data packet to Freedom Board
        delay(100);  // Small delay to prevent spamming
    }
}

// Convert PS4 input into movement commands
uint8_t getCommandFromPS4() {

    int lx = PS4.LStickX();
    int ly = PS4.LStickY();

    Serial.print("Left Stick X: "); Serial.print(lx);
    Serial.print(" | Left Stick Y: "); Serial.println(ly);

    if (PS4.LStickY() > 50) return 0b00000001;  // Move Forward
    if (PS4.LStickY() < -50) return 0b00000010;   // Move Backward
    if (PS4.LStickX() < -50) return 0b00000011;  // Move Left
    if (PS4.LStickX() > 50) return 0b00000100;   // Move Right
    if (PS4.Square()) return 0b10000000;         // Play Final Tune
    return 0b00000000;                           // Stop
}

// Function to remove paired Bluetooth devices
void removePairedDevices() {
    uint8_t pairedDeviceBtAddr[20][6];
    int count = esp_bt_gap_get_bond_device_num();

    if (count) {
        esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
        for (int i = 0; i < count; i++) {
            esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
        }
        Serial.println("Cleared paired Bluetooth devices.");
    }
}

// Print ESP32 Bluetooth MAC Address
void printDeviceAddress() {
    const uint8_t *point = esp_bt_dev_get_address();
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", (int)point[i]);
        if (i < 5) Serial.print(":");
    }
}

// PS4 Controller Connection Notification
void onConnect() {
    Serial.println("PS4 Controller Connected!");
}

// PS4 Controller Disconnection Notification
void onDisconnect() {
    Serial.println("PS4 Controller Disconnected!");
}
