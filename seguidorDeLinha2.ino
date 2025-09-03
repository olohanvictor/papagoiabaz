// Pinos dos sensores
const int pinoSensorEsquerdo = A0;
const int pinoSensorCentro = A1;
const int pinoSensorDireito = A2;

// Pinos dos motores (PWM)
const int pinoMotorEsquerdo = 5;
const int pinoMotorDireito = 6;

const int velocidade = 150;

// PID
double Kp = 30.0;
double Ki = 0.0;
double Kd = 15.0;

double erro = 0.0;
double erroAnterior = 0.0;
double integral = 0.0;

// Threshold do sensor para detectar linha
const int threshold = 500;

void setup() {
  pinMode(pinoSensorEsquerdo, INPUT);
  pinMode(pinoSensorCentro, INPUT);
  pinMode(pinoSensorDireito, INPUT);

  pinMode(pinoMotorEsquerdo, OUTPUT);
  pinMode(pinoMotorDireito, OUTPUT);

  Serial.begin(9600);  // Útil para depurar os valores lidos
}

void loop() {
  // Leitura analógica dos sensores (0 a 1023)
  int leituraEsq = analogRead(pinoSensorEsquerdo);
  int leituraCentro = analogRead(pinoSensorCentro);
  int leituraDir = analogRead(pinoSensorDireito);

  // Determina se está sobre a linha (1) ou fora (0)
  int esquerda = leituraEsq < threshold ? 1 : 0;
  int centro = leituraCentro < threshold ? 1 : 0;
  int direita = leituraDir < threshold ? 1 : 0;

  int total = esquerda + centro + direita;

  if (total == 0) {
    pararMotores();
    return;
  }

  // Calcular posição
  double posicao = ((esquerda * -1.0) + (centro * 0.0) + (direita * 1.0)) / (double)total;

  // Calcular PID
  erro = posicao;
  integral += erro;
  double derivada = erro - erroAnterior;
  double correcao = (Kp * erro) + (Ki * integral) + (Kd * derivada);
  erroAnterior = erro;

  // Ajuste de velocidades com limite
  int velocidadeMotorEsquerdo = constrain(velocidade + correcao, 0, 255);
  int velocidadeMotorDireito = constrain(velocidade - correcao, 0, 255);

  analogWrite(pinoMotorEsquerdo, velocidadeMotorEsquerdo);
  analogWrite(pinoMotorDireito, velocidadeMotorDireito);

  // (Opcional) Mostrar valores no Serial Monitor
  Serial.print("LE: ");
  Serial.print(leituraEsq);
  Serial.print(" | CE: ");
  Serial.print(leituraCentro);
  Serial.print(" | LD: ");
  Serial.print(leituraDir);
  Serial.print(" | Pos: ");
  Serial.println(posicao);

  delay(10); // Pequeno delay para estabilidade
}

void pararMotores() {
  analogWrite(pinoMotorEsquerdo, 0);
  analogWrite(pinoMotorDireito, 0);
}
