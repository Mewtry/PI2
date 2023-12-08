# 1 "C:\\workspace\\PI2\\TESTE\\TESTE.ino"
# 2 "C:\\workspace\\PI2\\TESTE\\TESTE.ino" 2

// Instância das UARTs

 void pingTask(void *pvParameters){
    while(1){
        vTaskDelay(5000 / ( ( TickType_t ) 1000 / ( 
# 7 "C:\\workspace\\PI2\\TESTE\\TESTE.ino" 3 4
                         1000 
# 7 "C:\\workspace\\PI2\\TESTE\\TESTE.ino"
                         ) ));
        Serial2.println("{\"type\":\"teste\"} ");
        Serial2.flush();
    }
    vTaskDelete(
# 11 "C:\\workspace\\PI2\\TESTE\\TESTE.ino" 3 4
               __null
# 11 "C:\\workspace\\PI2\\TESTE\\TESTE.ino"
                   );
}

void setup() {
  // Inicialização das UARTs
  Serial.begin(115200); // RX TX
  delay(1000);
  Serial.println("Iniciando aluno...");
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX TX
  delay(1000);
  xTaskCreate(&pingTask, "pingTask", 2048, 
# 21 "C:\\workspace\\PI2\\TESTE\\TESTE.ino" 3 4
                                          __null
# 21 "C:\\workspace\\PI2\\TESTE\\TESTE.ino"
                                              , 5, 
# 21 "C:\\workspace\\PI2\\TESTE\\TESTE.ino" 3 4
                                                   __null
# 21 "C:\\workspace\\PI2\\TESTE\\TESTE.ino"
                                                       );
}

void loop() {

  // Verifica se há dados disponíveis na Serial
  if (Serial.available() > 0) {
    // Lê o dado recebido na Serial
    char data = Serial.read();

    // Envia o dado recebido de volta para a Serial
    Serial2.write(data);
  }
  delay(1000);
  // Verifica se há dados disponíveis na Serial2
  if (Serial2.available() > 0) {
    // Lê o dado recebido na Serial2
    char data = Serial2.read();

    // Envia o dado recebido de volta para a Serial
    Serial.write(data);
  }
}
