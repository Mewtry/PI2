#include <Servo.h>
#include <SoftwareSerial.h>


// SoftwareSerial softSerial(3, 2); // RX - D3, TX - D2
SoftwareSerial softSerial(A3, A2);  // RX - D26 (A3), TX - D25 (A2)

int inputValue = 0;
int outputValue = 0;
bool sentidoEsteira = 0;

// Servo servo_5;

void setup()
{
  Serial.begin(9600);        // Comunicação com o computador
  softSerial.begin(9600);    // Comunicação com a ESP32 da Esteira
  //servo_5.attach(5, 500, 2500);
  pinMode(13, OUTPUT);// Estado da Esteira
  pinMode(12, INPUT); // Botão Posição R
  pinMode(11, INPUT); // Botão Posição G
  pinMode(10, INPUT); // Botão Posição B
  pinMode(9, INPUT);  // Botão Toggle Esteira

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT); // Blue
  pinMode(5, OUTPUT); // Red
  pinMode(6, OUTPUT); // Green
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  
  digitalWrite(2, LOW); 
  digitalWrite(3, LOW); 
  digitalWrite(4, LOW); // Blue
  digitalWrite(5, HIGH); // Red
  digitalWrite(6, LOW); // Green
  digitalWrite(7, LOW); 
  digitalWrite(8, LOW); 
}

void loop()
{
  if (softSerial.available() > 0) { 
    inputValue = softSerial.read();
    // Serial.print("Received from ESP32: ");
    // Serial.println(inputValue);
    // Serial.flush();
    if (inputValue == 82) {
      // servo_5.write(20);
      delay(50);
      softSerial.print("R ");
      softSerial.flush();
      Serial.println("Enviando: R");
      digitalWrite(4, LOW); // Blue
      digitalWrite(5, HIGH);// Red
      digitalWrite(6, LOW); // Green
    }
    else if (inputValue == 71) {
      // servo_5.write(90);
      delay(50);
      softSerial.print("G ");
      softSerial.flush();
      Serial.println("Enviando: G");
      digitalWrite(4, LOW); // Blue
      digitalWrite(5, LOW);// Red
      digitalWrite(6, HIGH); // Green
    }
    else if (inputValue == 66) {
      // servo_5.write(160);
      delay(50);
      softSerial.print("B ");
      softSerial.flush();
      Serial.println("Enviando: B");
      digitalWrite(4, HIGH); // Blue
      digitalWrite(5, LOW);// Red
      digitalWrite(6, LOW); // Green
    }
    else Serial.print((char)inputValue);
  }  
  if(Serial.available() > 0) { // Devolve na serial 2 o que receber na serial 0
    softSerial.write(Serial.read());
  }
  if (digitalRead(12) == LOW) { // Botão Posição R Pressionado
    while (digitalRead(12) == LOW) {
      delay(50); // Wait for 50 millisecond(s)
    }
    // servo_5.write(20);
    softSerial.print("R ");
    Serial.println("Enviando: R");
    digitalWrite(4, LOW); // Blue
    digitalWrite(5, HIGH);// Red
    digitalWrite(6, LOW); // Green
  }
  else if (digitalRead(11) == LOW) { // Botão Posição G Pressionado
    while (digitalRead(11) == LOW) {
      delay(50); // Wait for 50 millisecond(s)
    }
    // servo_5.write(90);
    softSerial.print("G ");
    softSerial.flush();
    Serial.println("Enviando: G");
    digitalWrite(4, LOW); // Blue
    digitalWrite(5, LOW);// Red
    digitalWrite(6, HIGH); // Green
  }
  else if (digitalRead(10) == LOW) { // Botão Posição B Pressionado
    while (digitalRead(10) == LOW) {
      delay(50); // Wait for 50 millisecond(s)
    }
    // servo_5.write(160);
    softSerial.print("B ");
    softSerial.flush();
    Serial.println("Enviando: B");
    digitalWrite(4, HIGH); // Blue
    digitalWrite(5, LOW);// Red
    digitalWrite(6, LOW); // Green
  }
  else if (digitalRead(9) == LOW) { // Botão Toggle Esteira Pressionado
    while (digitalRead(9) == LOW) {
      delay(50); // Wait for 1 millisecond(s)
    }
    if (digitalRead(13) == HIGH) {
      digitalWrite(13, LOW);
      digitalWrite(8, LOW);
      // softSerial.print("P ");
    } else {
      digitalWrite(13, HIGH);
      digitalWrite(8, HIGH);
    }
    softSerial.print("E ");
    Serial.println("Enviando: E");
  }
}