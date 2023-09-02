#include <AccelStepper.h>

// Define os pinos para o controle do motor de passo
#define STEP_PIN 22
#define DIR_PIN 23
#define STEPS_PER_REV 48
#define ACCEL 240
#define SPEED 96

// Cria uma instância do objeto AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

const char versao[] = "1.2.0";

int microStepMulti = 8;

void setup() {
  // Define o modo de operação do driver
  stepper.setMaxSpeed(SPEED*microStepMulti); // Velocidade máxima em passos por segundo
  stepper.setAcceleration(ACCEL*microStepMulti); // Aceleração em passos por segundo ao quadrado
  
  // Inicializa a comunicação serial
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {

    readSerial();
    
}

/*****************************************************************************************************************************/
/**
 * @brief Realiza a leitura de um byte do buffer de Serial
 * @return (char)32 - caso não tenha nada no buffer de entrada
*/
/*****************************************************************************************************************************/
char lerCharSerial() {
if (Serial.available() > 0) {
    return Serial.read();
}
  return ' ';
} // end lerCharSerial

/*****************************************************************************************************************************/
/**
 * @brief aguarda até que a porta serial esteja liberada ou então que tenha passado o tempo de timeout. O que 
 *        acontecer primeiro encerra a função.
 * @param timeout Indica o tempo em milisegundos a esperar alguma resposta de serial disponível.
*/
/*****************************************************************************************************************************/
void aguardaLeituraDisponivel(int timeout){
    unsigned long saida = millis() + timeout; // Salva o tempo de execução a liberar a função
    while(!Serial.available() && saida > millis());
    //escreveSerial("Saiu aguardaLeituraDisponivel"); // DEBUG
} // end aguardaLeituraDisponivel

/*****************************************************************************************************************************/
/**
 * @brief Rotina de leitura serial. Detecta se há algo em buffer de entrada e então executa a leitura do buffer, 
 *        concatenando em uma mensagem até que comece um novo comando ou então ele termine. Ao final de cada mensagem
 *        criada é chamada a função de tratamento de comandos.
 * @note  Tem um limete de bytes a serem lidos por mensagem de 50 caracteres
*/
/*****************************************************************************************************************************/
void readSerial() {
  String content = ""; // Armazenar o comando recebido
  char character;      // Armazenar temporariamente cada byte do comando recebido antes de concatená-lo em content
  character = lerCharSerial();
  if (character == ' ' || character == '\n' || character == '\r') {
    return;
  }
  int iLoop = 0;

  if (character == '@'){ 
    while (character != ' ' && character != '\n' && character != '\r' && iLoop < 10){
      iLoop++;
      content.concat(character);
      aguardaLeituraDisponivel(20);
      character = lerCharSerial();
      if(character == ' ' || character == '\n' || character == '\r') {
        trataComandoRecebido(content);
        content = "";
        iLoop = 0;
      }
    }
    return;
  }

  if (character == '$'){ // character que indica uma comunicação por Json
    while (character != ' ' && character != '\n' && character != '\r' && iLoop < 300){
      iLoop++;
      content.concat(character);
      aguardaLeituraDisponivel(20);
      character = lerCharSerial();
      if(character == '$' || character == ' ' || character == '\n' || character == '\r') {
        trataComandoRecebido(content);
        content = "";
        iLoop = 0;
      }
    }
    return;
  }

  // Enquanto não receber um caracter de fim de comando ou ultrapassar o limite de caracteres continua lendo bytes do comando
  while (character != ' ' && character != '\n' && character != '\r' && iLoop<50) {
    iLoop ++;    
    content.concat(character);
    aguardaLeituraDisponivel(50);
    character = lerCharSerial();
    if (character == ' ' || character == '+' || character == '-' || character == '\n' || character == '\r') {
      //escreveSerial("Comando recebido: " + content); // DEBUG
      trataComandoRecebido(content);
      content = ""; //Limpa a variável para um novo comando
    }
  }
  //escreveSerial("Saiu do loop readSerial"); // DEBUG
} // end readSerial

/*****************************************************************************************************************************/
/**
 * @brief trata a mensagem recebida e identifica qual função executar de acordo ao comando enviado.
 * @param comando String que contenha o comando a ser tratado
*/
/*****************************************************************************************************************************/
void trataComandoRecebido(String comando) {
    if (comando.startsWith("v")) { // Comando para informar versão do Firmware
        Serial.printf(versao);
        return;
    }

//   if(comando.startsWith("$")){ // É um Json
//     DeserializationError err = deserializeJson(json_IN, comando.substring(1));
//     if (err) {
//       // Serial.print(F("deserializeJson() failed with code "));
//       // Serial.println(err.f_str());
//       return;
//     }
//   }

    if (comando.startsWith("+")) { // Comando de abrir TAP
        // if (comando.startsWith("E", 1) || comando.startsWith("e", 1)) { // E -> TAP1
        //     // Mover esteira sentido horário
        //     stepper.moveTo(comando.substring(2).toInt());
        //     stepper.runToPosition();
        // } else if (comando.startsWith("M", 1) || comando.startsWith("m", 1)) { // D -> TAP2
        //     // Mover magazine sentido horário
        //     stepper.moveTo(comando.substring(2).toInt());
        //     stepper.runToPosition();
        // }
        int steps = comando.substring(1).toInt();
        while(stepper.isRunning()){};
        if (steps == 0){
          stepper.move((STEPS_PER_REV*microStepMulti)/3);
          stepper.runToPosition();
        }else{
          stepper.move(steps);
          stepper.runToPosition();
        }
    }
    else if (comando.startsWith("-")) { // Comando de abrir TAP
        // if (comando.startsWith("E", 1) || comando.startsWith("e", 1)) { // E -> TAP1
        //     // Mover esteira sentido anti-horário
        //     stepper.moveTo(comando.substring(2).toInt()*(-1));
        //     stepper.runToPosition();
        // } else if (comando.startsWith("M", 1) || comando.startsWith("m", 1)) { // D -> TAP2
        //     // Mover magazine sentido anti-horário
        //     stepper.moveTo(comando.substring(2).toInt()*(-1));
        //     stepper.runToPosition();
        // }
        int steps = comando.substring(1).toInt();
        while(stepper.isRunning()){};
        if (steps == 0){
          stepper.move((-STEPS_PER_REV*microStepMulti)/3);
          stepper.runToPosition();
        }else{
          stepper.move(-steps);
          stepper.runToPosition();
        }
    }
    else if (comando.startsWith("@")) {
        int separador = comando.lastIndexOf('@');  // Salva o índice do último '@' do comando
        int accel = comando.substring(1, separador).toInt();
        int speed = comando.substring(separador + 1).toInt();
        stepper.setAcceleration(accel*microStepMulti);
        stepper.setMaxSpeed(speed*microStepMulti);
        printf("\nNew Accel: %d\nNew Speed: %d\n", accel, speed);
    }
    else if (comando.startsWith("$")) {
        microStepMulti = comando.substring(1).toInt();
        Serial.print("Novo Multiplicador: "); Serial.println(microStepMulti);
        stepper.setAcceleration(ACCEL * microStepMulti);
        stepper.setMaxSpeed(SPEED * microStepMulti);
    }
}
