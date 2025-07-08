#include <Servo.h>
#include <String.h>
// Servos
Servo baseServo;   
Servo bracoServo;  
Servo pulsoServo;  
Servo garraServo;

#define FIM_DE_CURSO_PIN 7

String comando = "";
bool aguardandoResposta = false;

// Posições de referência
const int BASE_ZERO = 90;
const int BASE_ORGANICO = 0;
const int BASE_PLASTICO = 45;
const int BASE_METAL = 135;
const int BASE_VIDRO = 180;

const int BRACO = 80;

const int GARRA_ABERTA = 40;
const int GARRA_FECHADA = 0;

const int PULSO_BAIXO = 0;
const int PULSO_ALTO = 90;
int state;


//Funções de Movimento
// Movements
void moverServo(Servo& servo, int alvo, int passo = 1, int delay_ms = 60) {
  int atual = servo.read();  // Lê a posição atual real do servo

  alvo = constrain(alvo, 0, 180);


  if (atual < alvo) {
    for (int pos = atual; pos <= alvo; pos += passo) {
      servo.write(pos);
      delay(delay_ms);
    }
  } else {
    for (int pos = atual; pos >= alvo; pos -= passo) {
      servo.write(pos);
      delay(delay_ms);
    }
  }
}

void zero() {
  moverServo(garraServo, GARRA_ABERTA);
  moverServo(pulsoServo, PULSO_ALTO);
  moverServo(bracoServo, BRACO);
  moverServo(baseServo, BASE_ZERO);
  delay(200);
}

void fetch() {
  moverServo(pulsoServo, PULSO_BAIXO);
  moverServo(garraServo, GARRA_FECHADA);
  moverServo(pulsoServo, PULSO_ALTO);
}

void drop(String thrash) {
  int posicao;

  if (thrash == "metal") {
    posicao = BASE_METAL;
  } else if (thrash == "vidro") {
    posicao = BASE_VIDRO;
  } else if (thrash == "organico") {
    posicao = BASE_ORGANICO;
  } else if (thrash == "plastico") {
    posicao = BASE_PLASTICO;
  } else {
    Serial.println("Comando desconhecido, tente novamente");
    return;
  }

  moverServo(baseServo, posicao);
  moverServo(pulsoServo, PULSO_BAIXO);
  moverServo(garraServo, GARRA_ABERTA);
  moverServo(pulsoServo, PULSO_ALTO);
}


void executarComando(String material) {
  if (material == "lixo" || material == "") return;

  Serial.print(">> Detectado: ");
  Serial.println(material);
  zero();
  fetch();
  drop(material);
  zero();
}


void setup() {
  Serial.begin(9600);
  pinMode(FIM_DE_CURSO_PIN, INPUT_PULLUP);

  baseServo.attach(2);
  bracoServo.attach(5);
  pulsoServo.attach(4);
  garraServo.attach(3);
  state = 0;
  zero();
  Serial.println("Sistema iniciado. Monitorando sensor fim de curso...");
}

void loop() {
  if (!aguardandoResposta && digitalRead(FIM_DE_CURSO_PIN) == LOW) {
    Serial.println("foto");  // pede imagem ao Python
    aguardandoResposta = true;
    comando = "";
    delay(500);
  }

  if (aguardandoResposta && Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      comando.trim();
      executarComando(comando);
      aguardandoResposta = false;
      comando = "";
      Serial.println("zero"); // sinaliza ao Python que pode continuar
    } else {
      comando += c;
    }
  }
}

