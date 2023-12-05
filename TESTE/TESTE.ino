#include <Arduino.h>

// Instância das UARTs

 void pingTask(void *pvParameters){
    while(1){
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        Serial2.println("{\"type\":\"teste\"} ");
        Serial2.flush();
    }
    vTaskDelete(NULL);
}

void setup() {
  // Inicialização das UARTs
  Serial.begin(115200);    // RX TX
  delay(1000);
  Serial.println("Iniciando aluno...");
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX TX
  delay(1000);
  xTaskCreate(&pingTask, "pingTask", 2048, NULL, 5, NULL);
}

void loop() {

  // Verifica se há dados disponíveis na Serial2
  if (Serial.available() > 0) {
    // Lê o dado recebido na Serial2
    char data = Serial.read();
    
    // Envia o dado recebido de volta para a Serial
    Serial2.write(data);
  }
  // Verifica se há dados disponíveis na Serial2
  if (Serial2.available() > 0) {
    // Lê o dado recebido na Serial2
    char data = Serial2.read();
    
    // Envia o dado recebido de volta para a Serial
    Serial.write(data);
  }
}