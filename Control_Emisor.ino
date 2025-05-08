#include "BluetoothSerial.h"

// Configuración de pines 
const uint8_t PIN_A      = 17;
const uint8_t PIN_B      = 5;
const uint8_t PIN_UP     = 0;
const uint8_t PIN_DOWN   = 16;
const uint8_t PIN_LEFT   = 15;
const uint8_t PIN_RIGHT  = 4;

BluetoothSerial SerialBT;
unsigned long pressStart = 0;
const unsigned long disconnectTime = 3000; // 3 segundos para forzar desconexión

void setup() {
  Serial.begin(115200);
  
  // Pines con pull-up interno
  pinMode(PIN_A,      INPUT_PULLUP);
  pinMode(PIN_B,      INPUT_PULLUP);
  pinMode(PIN_UP,     INPUT_PULLUP);
  pinMode(PIN_DOWN,   INPUT_PULLUP);
  pinMode(PIN_LEFT,   INPUT_PULLUP);
  pinMode(PIN_RIGHT,  INPUT_PULLUP);
  
  // Iniciar Bluetooth en modo esclavo (isMaster=false) y sin PIN
  if (!SerialBT.begin("NES_Controller", /*isMaster=*/false)) {
    Serial.println("Error al iniciar Bluetooth!");
    while (1);
  }
  Serial.println("Control NES Bluetooth listo");
  Serial.println("Dispositivo: NES_Controller");
}

void loop() {
  // Desconexión forzada: botón A presionado >3s
  if (digitalRead(PIN_A) == LOW) {
    if (pressStart == 0) {
      pressStart = millis();
    }
    if (millis() - pressStart > disconnectTime && SerialBT.hasClient()) {
      Serial.println("Forzando desconexión del cliente...");
      SerialBT.disconnect();
      pressStart = 0;
    }
  } else {
    pressStart = 0;
  }

  // Leer estado botones en un byte
  uint8_t botones = 0;
   botones |= (!digitalRead(PIN_A))     << 0;
  botones |= (!digitalRead(PIN_B))      << 1;
  botones |= (!digitalRead(PIN_UP))     << 2;
  botones |= (!digitalRead(PIN_DOWN))   << 3;
  botones |= (!digitalRead(PIN_LEFT))   << 4;
  botones |= (!digitalRead(PIN_RIGHT))  << 6;
  
  // Debug por UART
  Serial.print("Estado botones: 0x"); Serial.print(botones, HEX); Serial.print(" -> ");
  Serial.print((botones & 0x01) ? "A "  : "-- ");
  Serial.print((botones & 0x02) ? "B "  : "-- ");
  Serial.print((botones & 0x10) ? "U "  : "-- ");
  Serial.print((botones & 0x20) ? "D "  : "-- ");
  Serial.print((botones & 0x40) ? "L "  : "-- ");
  Serial.print((botones & 0x80) ? "R "  : "-- ");
  Serial.println();
  
  // Enviar al cliente Bluetooth si está conectado
  if (SerialBT.hasClient()) {
    SerialBT.write(botones);
  }
  
  delay(20); // antirrebote
}
