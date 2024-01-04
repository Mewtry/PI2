/**************************************************************************/
/**
 * @file    Ping.ino
 * @version 1.1
 * @author  Theo Pires
 * @date    26/08/2023
 * @see     www.linkedin.com/in/theo-pires-a34b33183/
 * 
 * @brief  Código para teste de comunicação entre o ESP32 e a esteira
 *         Você pode usar a task pingTask para enviar o ping ou então
 *         você mesmo enviar utilizando o seu prórprio terminal serial
 *          (não esqueça de mudar a velocidade da serial para 115200) 
*/
/**************************************************************************/

#include <Arduino.h>

// Task para enviar o ping para a esteira a cada 5 segundos
 void pingTask(void *pvParameters){               
    // Loop infinito   
    while(1){     
        // Delay de 5 segundos                                
        vTaskDelay(5000 / portTICK_PERIOD_MS);  
        // Envia o ping para a esteira  
        Serial2.println("{\"type\":\"teste\"} ");
        // Limpa o buffer da Serial2 
        Serial2.flush();                          
    }
    // Exclui a tarefa caso ela saia do loop
    vTaskDelete(NULL);                            
}

void setup() {
  // Inicialização da Serial
  Serial.begin(115200);
  // Aguarda inicialização da Serial
  delay(1000);
  Serial.println("Iniciando aluno...");
  // Inicialização da Serial2
  Serial2.begin(115200);
  // Aguarda inicialização da Serial2
  delay(1000);
  // Cria a tarefa para ficar enviando o ping para a esteira
  // xTaskCreate(&pingTask, "pingTask", 2048, NULL, 5, NULL);
}

void loop() {
  // Verifica se há dados disponíveis na Serial
  if (Serial.available() > 0) {
    // Lê o dado recebido na Serial
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