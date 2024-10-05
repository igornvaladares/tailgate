#include <avr/sleep.h>
#include <avr/interrupt.h>

// Pinos de controle da ponte H
const int IN1 = 7;  // Pino de controle da direção 1
const int IN2 = 8;  // Pino de controle da direção 2
const int ENA = 9;  // Pino de habilitação do PWM

// Pino do botão de controle externo
const int botao = 2;  // Pino de interrupção para wake-up

// Parâmetros de controle de movimento
const unsigned long tempoTotal = 8000;            // Tempo total de abertura/fechamento (8s)
const unsigned long tempoAceleracao = 2000;       // Tempo de aceleração/desaceleração (2s)
const unsigned long tempoMaximoMovimento = tempoTotal - 2 * tempoAceleracao; // Tempo máximo em velocidade constante
const int maxVelocidade = 255;                     // Velocidade máxima do motor (PWM)

// Variáveis de controle de estado
unsigned long tempoInicio = 0;
unsigned long tempoRestante = 0;
int velocidadeAtual = 0;                // Velocidade do motor (0 a 255)
bool estadoMotor = false;               // Estado do motor (ligado/desligado)
bool movimentoAbrindo = true;           // Abertura (true) ou fechamento (false)

void setup() {
  // Configurar pinos como saída
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

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

  if (sinalAtivo && !estadoMotor) {
    // Iniciar movimento se o botão for pressionado e o motor estiver parado
    iniciarMovimento();
  } else if (sinalAtivo && estadoMotor) {
    // Reverter movimento se o botão for pressionado enquanto o motor estiver ligado
    reverterMovimento();
  }

  // Verificar se o tempo total de operação foi atingido
  if (estadoMotor && millis() - tempoInicio >= tempoTotal) {
    pararMotor();
    entrarModoSleep();
  }
}

void iniciarMovimento() {
  estadoMotor = true;
  tempoInicio = millis();
  tempoRestante = tempoTotal;

  if (movimentoAbrindo) {
    abrirTailgate(tempoRestante);
  } else {
    fecharTailgate(tempoRestante);
  }
}

void reverterMovimento() {
  movimentoAbrindo = !movimentoAbrindo;  // Inverte o sentido de abertura/fechamento
  tempoRestante = tempoTotal - (millis() - tempoInicio);  // Calcula o tempo restante para reverter

  if (movimentoAbrindo) {
    abrirTailgate(tempoRestante);
  } else {
    fecharTailgate(tempoRestante);
  }
}

void abrirTailgate(unsigned long tempoRestante) {
  controlarMotor(IN1, IN2, tempoRestante);  // Abrir no sentido correto
}

void fecharTailgate(unsigned long tempoRestante) {
  controlarMotor(IN2, IN1, tempoRestante);  // Fechar no sentido reverso
}
void controlarMotor(int pinoSentido1, int pinoSentido2, unsigned long tempoTotal) {
    digitalWrite(pinoSentido1, HIGH);
    digitalWrite(pinoSentido2, LOW);

    unsigned long tempoDecorrido = 0;

    // Aceleração
    tempoDecorrido += acelerarMotor(tempoTotal, tempoAceleracao);

    // Manter velocidade máxima
    tempoDecorrido += manterVelocidadeMaxima(tempoTotal, tempoDecorrido);

    // Desaceleração
    desacelerarMotor(tempoTotal, tempoDecorrido);
}

unsigned long acelerarMotor(unsigned long tempoTotal, unsigned long tempoAceleracao) {
    unsigned long tempoDecorrido = 0;
    while (tempoDecorrido < tempoAceleracao && tempoDecorrido < tempoTotal) {
        if (verificarSinalExterno()) return tempoDecorrido;  // Interrompe se sinal externo for detectado
        ajustarVelocidade(tempoDecorrido, 0, tempoAceleracao, maxVelocidade);
        delay(30);
        tempoDecorrido += 30;
    }
    return tempoDecorrido;  // Retorna o tempo total decorrido durante a aceleração
}

unsigned long manterVelocidadeMaxima(unsigned long tempoTotal, unsigned long tempoDecorrido) {
    unsigned long tempoManter = tempoTotal - tempoDecorrido - tempoAceleracao;  // Cálculo do tempo restante
    if (tempoManter < 0) tempoManter = 0;  // Garante que o tempo não seja negativo

    while (tempoManter > 0) {
        if (verificarSinalExterno()) return tempoDecorrido;  // Interrompe se sinal externo for detectado
        ajustarVelocidade(tempoDecorrido, 0, tempoAceleracao, maxVelocidade);  // Mantém a velocidade máxima
        delay(30);
        tempoManter -= 30;  // Reduz o tempo restante
        tempoDecorrido += 30;  // Atualiza o tempo decorrido
    }
    return tempoDecorrido;  // Retorna o tempo total decorrido após manter a velocidade máxima
}

void desacelerarMotor(unsigned long tempoTotal, unsigned long tempoDecorrido) {
    unsigned long tempoDesaceleracao = min(tempoTotal - tempoDecorrido, tempoAceleracao); // Limitar a desaceleração a 2000 ms

    while (tempoDecorrido < tempoTotal) {
        if (verificarSinalExterno()) return;  // Interrompe se sinal externo for detectado
        ajustarVelocidade(tempoTotal - tempoDecorrido, 0, tempoDesaceleracao, maxVelocidade);  // Desacelera o motor
        delay(30);
        tempoDecorrido += 30;  // Atualiza o tempo decorrido
    }

    pararMotor();  // Para o motor ao final do ciclo
}


void ajustarVelocidade(int tempoAtual, int inicio, int fim, int velocidadeFinal) {
  velocidadeAtual = map(tempoAtual, inicio, fim, 0, velocidadeFinal);
  analogWrite(ENA, velocidadeAtual);
}

bool verificarSinalExterno() {
  // Verifica se o botão foi pressionado durante a operação
  if (digitalRead(botao) == LOW) {
    reverterMovimento();
    return true;
  }
  return false;
}

void pararMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  estadoMotor = false;
}

void entrarModoSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();  // Coloca o Arduino em modo sleep até interrupção
}

void wakeUp() {
  sleep_disable();  // Desabilita o sleep quando acorda
}
