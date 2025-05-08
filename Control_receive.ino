#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

void setup() {
  // USB-CDC para debug
  Serial.begin(115200);

  // Puerto UART físico que va a la STM32
  Serial2.begin(115200, SERIAL_8N1, /* RX2 pin*/16, /* TX2 pin*/17);

  Serial.println("ESP32 Receptor - Conexión Bluetooth");
  if (!SerialBT.begin("ESP32_Receptor", /*isMaster=*/true)) {
    Serial.println("Error al iniciar Bluetooth!");
    while (1);
  }
  delay(2000);
  Serial.print("Conectando a NES_Controller… ");
  if (SerialBT.connect("NES_Controller")) Serial.println("¡OK!");
  else                                Serial.println("FALLO");
}

void loop() {
  if (SerialBT.connected() && SerialBT.available()) {
    uint8_t data = SerialBT.read();
    String msg = "";
    if (data & 0x01) msg += "A,";
    if (data & 0x02) msg += "B,";
    if (data & 0x04) msg += "U,";
    if (data & 0x08) msg += "D,";
    if (data & 0x10) msg += "L,";
    if (data & 0x40) msg += "R,";
    if (msg.length() > 0) {
      msg.remove(msg.length() - 1);
      // ---> enviar por el UART físico al STM32
      Serial2.println(msg);

      // opcional: para debug también al USB:
      Serial.print("UART2->STM: ");
      Serial.println(msg);
    }
  }

if (!SerialBT.connected()) {
    Serial.println("Bluetooth caído: reintentando...");
    if (SerialBT.connect("NES_Controller")) Serial.println("Reconectado");
    delay(1000);
  }

  delay(20);
}
