#include "WS8610Receiver.h"

WS8610Receiver receiver = WS8610Receiver(2); // RF receiver connected to pin 2

void setup() {
    Serial.begin(115200);
    receiver.enableReceive();
}

void loop() {
    if (receiver.available()) {
        measure m = receiver.getReceivedValue();
        receiver.resetAvailable();

        Serial.print("Sensor #");
        Serial.print(m.sensorAddr);
        Serial.print(": ");
        Serial.print(m.units);
        Serial.print(".");
        Serial.print(m.decimals);    
        Serial.println((m.type == TEMPERATURE)? " °C" : " %rh");
    }
}
