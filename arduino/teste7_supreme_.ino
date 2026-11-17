
#include <SoftwareSerial.h>

// KARL-01 - Arduino Uno / L298N / 2 sensores IR / HC-05
// HC-05 TXD -> D2; D3 -> divisor resistivo -> HC-05 RXD
// VCC -> 5V; GND comum
SoftwareSerial bluetooth(2, 3);

const byte IN1 = 8, IN2 = 9, IN3 = 10, IN4 = 11;
const byte ENA = 5, ENB = 12;
const byte SENSOR_1 = 7, SENSOR_2 = 6;
const int NIVEL_PRETO = HIGH;
const int VELOCIDADE = 125;
const bool INVERTER_MOTOR_1 = true;
const bool INVERTER_MOTOR_2 = true;

const unsigned long DURACAO_TESTE = 5000;
const unsigned long TEMPO_LIMITE = 5000;
const unsigned long INTERVALO_MAX_FAIXAS = 2500;
const unsigned long IGNORAR_APOS_INICIO = 600;
const unsigned long DEBOUNCE_FAIXA = 25;
const unsigned long INTERVALO_SENSORES = 1500;

bool roboAtivo = false, testeAtivo = false;
byte tipoTeste = 0, faixasContadas = 0, numeroParada = 0;
bool pretoDuploAnterior = false, faixaConfirmada = false;
unsigned long inicioTeste = 0, ultimoHeartbeat = 0;
unsigned long inicioPercurso = 0, primeiraFaixaEm = 0, inicioPretoDuplo = 0;
unsigned long ultimaTelemetria = 0;

// Uma linha completa vai para o Monitor Serial e para o aplicativo.
void mensagem(const char* texto) {
  Serial.println(texto);
  bluetooth.println(texto);
}
void valor(const char* rotulo, int numero) {
  Serial.print(rotulo); Serial.println(numero);
  bluetooth.print(rotulo); bluetooth.println(numero);
}

void pararMotor1() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
void pararMotor2() { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
void pararMotores() { pararMotor1(); pararMotor2(); }

void moverMotor1(int velocidade) {
  velocidade = constrain(velocidade, 0, 255);
  if (velocidade == 0) { pararMotor1(); return; }
  if (INVERTER_MOTOR_1) {
    digitalWrite(IN1, HIGH);
    analogWrite(IN2, 255 - velocidade);
  } else {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, velocidade);
  }
}
void moverMotor2(int velocidade) {
  velocidade = constrain(velocidade, 0, 255);
  if (velocidade == 0) { pararMotor2(); return; }
  if (INVERTER_MOTOR_2) {
    digitalWrite(IN3, HIGH);
    analogWrite(IN4, 255 - velocidade);
  } else {
    digitalWrite(IN3, LOW);
    analogWrite(IN4, velocidade);
  }
}

void enviarSensores() {
  int s1 = digitalRead(SENSOR_1);
  int s2 = digitalRead(SENSOR_2);
  Serial.print("SENSORES:"); Serial.print(s1); Serial.print(','); Serial.println(s2);
  bluetooth.print("SENSORES:"); bluetooth.print(s1); bluetooth.print(','); bluetooth.println(s2);
}
void enviarEstado() {
  if (testeAtivo) mensagem("ESTADO:TESTE");
  else if (roboAtivo) mensagem("ESTADO:MOVIMENTO");
  else if (numeroParada % 2 == 1) mensagem("ESTADO:ESTACAO_B");
  else mensagem("ESTADO:PARADO");
  valor("PARADAS:", numeroParada);
}

void iniciarTeste(byte tipo) {
  roboAtivo = false;
  testeAtivo = true;
  tipoTeste = tipo;
  inicioTeste = millis();
  pararMotores();
  mensagem("TESTE:INICIADO");
}
void executarTeste() {
  if (millis() - inicioTeste >= DURACAO_TESTE) {
    pararMotores(); testeAtivo = false; tipoTeste = 0;
    mensagem("TESTE:FINALIZADO"); return;
  }
  moverMotor1(tipoTeste == 1 || tipoTeste == 3 ? VELOCIDADE : 0);
  moverMotor2(tipoTeste == 2 || tipoTeste == 3 ? VELOCIDADE : 0);
}

void processarComando(char comando) {
  if (comando == 'I' || comando == 'i') {
    pararMotores(); testeAtivo = false; roboAtivo = true;
    ultimoHeartbeat = millis(); inicioPercurso = millis();
    primeiraFaixaEm = 0; inicioPretoDuplo = 0;
    faixasContadas = 0; faixaConfirmada = false;
    pretoDuploAnterior = digitalRead(SENSOR_1) == NIVEL_PRETO &&
                           digitalRead(SENSOR_2) == NIVEL_PRETO;
    mensagem("EVENTO:MOVIMENTO_INICIADO");
  } else if (comando == 'P' || comando == 'p') {
    roboAtivo = false; testeAtivo = false; pararMotores();
    mensagem("EVENTO:PARADA_MANUAL");
  } else if (comando == 'H' || comando == 'h') {
    ultimoHeartbeat = millis();
  } else if (comando == '1') iniciarTeste(1);
  else if (comando == '2') iniciarTeste(2);
  else if (comando == 'T' || comando == 't') iniciarTeste(3);
  else if (comando == 'S' || comando == 's') enviarSensores();
  else if (comando == 'V' || comando == 'v') enviarEstado();
}

void verificarWatchdog() {
  if (roboAtivo && millis() - ultimoHeartbeat >= TEMPO_LIMITE) {
    roboAtivo = false; pararMotores();
    mensagem("EVENTO:PARADA_SEGURANCA");
  }
}

void verificarFaixas(bool preto1, bool preto2) {
  unsigned long agora = millis();
  bool pretoDuplo = preto1 && preto2;
  if (agora - inicioPercurso < IGNORAR_APOS_INICIO) {
    pretoDuploAnterior = pretoDuplo;
    inicioPretoDuplo = 0; faixaConfirmada = false;
    return;
  }
  if (faixasContadas == 1 &&
      agora - primeiraFaixaEm > INTERVALO_MAX_FAIXAS) {
    faixasContadas = 0;
    mensagem("AVISO:INTERVALO_FAIXAS_EXCEDIDO");
  }
  if (pretoDuplo) {
    if (!pretoDuploAnterior) {
      inicioPretoDuplo = agora;
      faixaConfirmada = false;
    }
    if (!faixaConfirmada &&
        agora - inicioPretoDuplo >= DEBOUNCE_FAIXA) {
      faixaConfirmada = true;
      faixasContadas++;
      valor("FAIXA:", faixasContadas);
      if (faixasContadas == 1) primeiraFaixaEm = agora;
      else if (faixasContadas == 2) {
        roboAtivo = false; pararMotores();
        faixasContadas = 0; numeroParada++;
        // Mensagens estruturadas: Flutter reconhece automaticamente.
        if (numeroParada % 2 == 1) mensagem("EVENTO:CHEGADA_B");
        else mensagem("EVENTO:CHEGADA_A");
      }
    }
  } else {
    inicioPretoDuplo = 0;
    faixaConfirmada = false;
  }
  pretoDuploAnterior = pretoDuplo;
}

void seguirLinha() {
  int s1 = digitalRead(SENSOR_1);
  int s2 = digitalRead(SENSOR_2);
  bool preto1 = s1 == NIVEL_PRETO;
  bool preto2 = s2 == NIVEL_PRETO;
  verificarFaixas(preto1, preto2);
  if (!roboAtivo) { pararMotores(); return; }
  if (!preto1 && !preto2) {
    moverMotor1(VELOCIDADE); moverMotor2(VELOCIDADE);
  } else if (preto1 && !preto2) {
    moverMotor1(0); moverMotor2(VELOCIDADE);
  } else if (!preto1 && preto2) {
    moverMotor1(VELOCIDADE); moverMotor2(0);
  } else {
    moverMotor1(VELOCIDADE); moverMotor2(VELOCIDADE);
  }
}

void setup() {
  // Deixa a ponte H desligada durante a inicializacao.
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  digitalWrite(ENA, LOW); digitalWrite(ENB, LOW);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pararMotores();
  pinMode(SENSOR_1, INPUT); pinMode(SENSOR_2, INPUT);
  Serial.begin(9600); bluetooth.begin(9600);
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH);
  pararMotores();
  mensagem("KARL-01:INICIADO");
}

void loop() {
  while (Serial.available()) processarComando(Serial.read());
  while (bluetooth.available()) processarComando(bluetooth.read());
  verificarWatchdog();
  if (testeAtivo) executarTeste();
  else if (roboAtivo) seguirLinha();
  else pararMotores();
  if (millis() - ultimaTelemetria >= INTERVALO_SENSORES) {
    ultimaTelemetria = millis();
    enviarSensores();
  }
}
