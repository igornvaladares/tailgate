#include <avr/sleep.h>
#include <avr/interrupt.h>

// Pinos de controle da ponte H
const int R_PWM = 7;  // Pino PWM de controle da direção 1
const int L_PWM = 8;  // Pino PWM de controle da direção 2

const int R_ENA = 9;  // Pino de habilitação da direção R
const int L_ENA = 10;  // Pino de habilitação da direção L

// Pino do botão de controle externo
const int botao = 2;  // Pino de interrupção para wake-up

// Parâmetros de controle de movimento
const unsigned long tempoTotal = 8000;            // Tempo total de abertura/fechamento (8s)
const unsigned long tempoAceleracao = 2000;       // Tempo de aceleração/desaceleração (2s)
const unsigned long tempoMaximoMovimentacao = tempoTotal; // Tempo máximo "Padrão" de rerentencia
const int maxVelocidade = 255;                     // Velocidade máxima do motor (PWM)

// Parâmetros para Sensiblidade do sensor de corrente
const float correnteMaximaMotor =20;
const float limiteCorrenteAbertura = 5;
const float limiteCorrenteFechamento = 5; 
const float fatorDeConversaoCorrente = 5 / 1023 * correnteMaximaMotor; // 5 v

// Variáveis de controle de estado
unsigned long tempoInicio = 0;
unsigned long tempoMovimentacao = 0;
int velocidadeAtual = 0;                // Velocidade do motor (0 a 255)
bool motorEmMovimento = false;               // Estado do motor (ligado/desligado)
bool movimentoAbrindo = false;           // Abertura (true) ou fechamento (false)

void setup() {
  // Configurar pinos como saída
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  pinMode(R_ENA, OUTPUT);
  pinMode(L_ENA, OUTPUT);
  digitalWrite(R_ENA, HIGH);
  digitalWrite(L_ENA, HIGH);


  // Configurar o pino do botão como entrada com PULLUP interno
  pinMode(botao, INPUT_PULLUP);

  // Configurar interrupção para o botão (borda de descida)
  attachInterrupt(digitalPinToInterrupt(botao), wakeUp, FALLING);

  // Iniciar o motor desligado
  pararMotor();
  
  Serial.begin(9600);  // Para debug (opcional)
}

void loop() {
  // Verificar se o botão foi pressionado
  bool sinalAtivo = digitalRead(botao) == LOW;

  if (sinalAtivo && !motorEmMovimento) {
    // Iniciar movimento se o botão for pressionado e o motor estiver parado
    iniciarMovimento(tempoTotal);
  } 

  // Verificar se o tempo total de operação foi atingido
  if (motorEmMovimento && millis() - tempoInicio >= tempoTotal) {
    pararMotor();
    entrarModoSleep();
  }
}

void iniciarMovimento(unsigned long tempoMovimentacao) {
  motorEmMovimento = true;
  tempoInicio = millis();
  
  if (movimentoAbrindo) {
    abrirTailgate(tempoMovimentacao);
  } else {
    fecharTailgate(tempoMovimentacao);
  }
}

void reverterMovimento(unsigned long tempoMovimentacao) {
  movimentoAbrindo = !movimentoAbrindo;  // Inverte o sentido de abertura/fechamento
  if (movimentoAbrindo) {
    abrirTailgate(tempoMovimentacao);
  } else {
    fecharTailgate(tempoMovimentacao);
  }
}

void abrirTailgate(unsigned long tempoRestante) {
  controlarMotor(L_PWM,R_PWM, tempoRestante);  // Abrir 
}

void fecharTailgate(unsigned long tempoRestante) {
  controlarMotor(R_PWM,L_PWM, tempoRestante);  // Fechar 
}
void controlarMotor(int pinoSentido1,int pinoSentido2, unsigned long tempoTotal) {
    analogWrite(pinoSentido1, LOW);
    unsigned long tempoAteFimMovimento = tempoMaximoMovimentacao - (tempoMaximoMovimentacao - tempoTotal);

    unsigned long tempoDecorrido = 0;

    // Aceleração
    tempoDecorrido += acelerarMotor(pinoSentido2,tempoAteFimMovimento, tempoAceleracao);

    // Manter velocidade máxima
    tempoDecorrido += manterVelocidadeMaxima(pinoSentido2,tempoAteFimMovimento, tempoDecorrido);

    // Desaceleração
    desacelerarMotor(pinoSentido2,tempoAteFimMovimento, tempoDecorrido);
}

unsigned long acelerarMotor(int pinoSentido2,unsigned long tempoTotal, unsigned long tempoAceleracao) {
    unsigned long tempoDecorrido = 0;
    while (tempoDecorrido < tempoAceleracao && tempoDecorrido < tempoTotal) {
        if (verificarSinalExterno(tempoDecorrido)) return tempoDecorrido;  // Interrompe se sinal externo for detectado
        ajustarVelocidade(pinoSentido2,tempoDecorrido, 0, tempoAceleracao, maxVelocidade);
        delay(30);
        tempoDecorrido += 30;
    }
    return tempoDecorrido;  // Retorna o tempo total decorrido durante a aceleração
}

unsigned long manterVelocidadeMaxima(int pinoSentido2,unsigned long tempoTotal, unsigned long tempoDecorrido) {
    unsigned long tempoManter = tempoTotal - tempoDecorrido - tempoAceleracao;  // Cálculo do tempo restante
    if (tempoManter < 0) tempoManter = 0;  // Garante que o tempo não seja negativo

    while (tempoManter > 0) {
        if (verificarSinalExterno(tempoDecorrido)) return tempoDecorrido;  // Interrompe se sinal externo for detectado
        ajustarVelocidade(pinoSentido2,tempoDecorrido, 0, tempoAceleracao, maxVelocidade);  // Mantém a velocidade máxima
        delay(30);
        tempoManter -= 30;  // Reduz o tempo restante
        tempoDecorrido += 30;  // Atualiza o tempo decorrido
    }
    return tempoDecorrido;  // Retorna o tempo total decorrido após manter a velocidade máxima
}

void desacelerarMotor(int pinoSentido2,unsigned long tempoTotal, unsigned long tempoDecorrido) {
    unsigned long tempoDesaceleracao = min(tempoTotal - tempoDecorrido, tempoAceleracao); // Limitar a desaceleração a 2000 ms

    while (tempoDecorrido < tempoTotal) {
        if (verificarSinalExterno(tempoDecorrido)) return;  // Interrompe se sinal externo for detectado
        ajustarVelocidade(pinoSentido2,tempoTotal - tempoDecorrido, 0, tempoDesaceleracao, maxVelocidade);  // Desacelera o motor
        delay(30);
        tempoDecorrido += 30;  // Atualiza o tempo decorrido
    }

    pararMotor();  // Para o motor ao final do ciclo
}


void ajustarVelocidade(int pinoSentido2,int tempoAtual, int inicio, int fim, int velocidadeFinal) {
  velocidadeAtual = map(tempoAtual, inicio, fim, 0, velocidadeFinal);
  analogWrite(pinoSentido2, velocidadeAtual);
}

bool verificarSinalExterno(unsigned long tempoDecorrido) {
  // Verifica se o botão foi pressionado durante a operação
  if (digitalRead(botao) == LOW) {
    reverterMovimento(tempoDecorrido);
    return true;
  }
  return false;
}

void pararMotor() {
  analogWrite(R_PWM, LOW);
  analogWrite(L_PWM, LOW);
  motorEmMovimento = false;
}

void entrarModoSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();  // Coloca o Arduino em modo sleep até interrupção
}

void wakeUp() {
  sleep_disable();  // Desabilita o sleep quando acorda
}
