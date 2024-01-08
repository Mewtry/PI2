#include <ArduinoJson.h>

// Definindo a porta serial a ser utilizada
#define SERIAL_PORT Serial2

double timeStartWainting = 0;

void setup() {
    Serial.begin(115200);
    SERIAL_PORT.begin(115200);
}

void loop() {

    // Criando um JSON para o envio do teste
    StaticJsonDocument<50> json_OUT;
    json_OUT["type"] = "teste";

    // Enviando o JSON pela porta serial
    serializeJson(json_OUT, SERIAL_PORT);
    SERIAL_PORT.println();

    timeStartWainting = millis();
    // Aguardando resposta pela porta serial
    while ( ! SERIAL_PORT.available()) {
        if (millis() - timeStartWainting > 5000) {
            Serial.println("Timeout!");
            return;
        }
    }

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, SERIAL_PORT);

    if (error) {
        Serial.print("Erro ao ler JSON: ");
        Serial.println(error.c_str());
        return;
    }

    // Lendo valores do JSON recebido
    const char* jsonType = doc["type"];
    const char* jsonStatus = doc["status"];

    Serial.printf("%s: %s\n", jsonType, jsonStatus);
}
