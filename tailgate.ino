#include <avr/sleep.h>
#include <avr/interrupt.h>

// Pinos de controle da ponte H
const int R_PWM = 7;  // Pino PWM de controle da direção 1
const int L_PWM = 8;  // Pino PWM de controle da direção 2

//const int R_ENA = 9;  // Pino de habilitação da direção R
//const int L_ENA = 10;  // Pino de habilitação da direção L

const int PWM = 11;  // Ligar os pinos R_ENA e L_ENA no D11 do arduino

// Pino do botão de controle externo
const int botao = 2;  // Pino de interrupção para wake-up

// Pino do sensor corrente
const int correnteEsquerdo = A1;  // Pino corrente esquerda
const int correnteDireito = A2;  // Pino corrente direita

// Parâmetros de controle de movimento
const unsigned long tempoMaximoMovimentacao = 8000;     // Tempo máximo "Padrão" de rerentencia
const unsigned long tempoAceleracao = 2000;       // Tempo de aceleração/desaceleração (2s)
const unsigned long  tempoAteFimMovimento = tempoMaximoMovimentacao;  // Tempo total de abertura/fechamento (8s)
const unsigned long intervalo = 30;       // intervalo em ms para reduzir ou aumementar a velocidade 
// Parâmetros para Sensiblidade do sensor de corrente
//const float correnteMaximaMotor =20;
const float limiteCorrenteAbertura = 5;
const float limiteCorrenteFechamento = 5; 
//const float fatorDeConversaoCorrente = 5 / 1023 * correnteMaximaMotor; // 5 v

// Variáveis de controle de estado
unsigned long tempoInicio = 0;
int velocidadeAtual = 0;                // Velocidade do motor (0 a 255)
bool motorEmMovimento = false;               // Estado do motor (ligado/desligado)
bool movimentoAbrindo = false;           // Abertura (true) ou fechamento (false)



void setup() {
  // Configurar pinos como saída
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(PWM, OUTPUT);

  //pinMode(R_ENA, OUTPUT);
  //pinMode(L_ENA, OUTPUT);
  //digitalWrite(R_ENA, HIGH);
  //digitalWrite(L_ENA, HIGH);

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
    iniciarMovimento(tempoAteFimMovimento);
  } 

  // Verificar se o tempo total de operação foi atingido
 // if (motorEmMovimento && millis() - tempoInicio >= tempoAteFimMovimento) {
    pararMotor();
   
 // }
}

void iniciarMovimento(unsigned long tempoAteFimMovimento) {
  motorEmMovimento = true;
  tempoInicio = millis();
  
  if (movimentoAbrindo) {
    abrirTailgate(tempoAteFimMovimento);
  } else {
    fecharTailgate(tempoAteFimMovimento);
  }
}

void reverterMovimento(unsigned long tempoRestante) {

// Se houver multiplas reversos, o tempo de movimento vai se ajustando.
// tempoRestante tempo que restaria para completar o movimento se não fosse interrompido.
//  tempoAteFimMovimento   novo tempo que agora falta para completatar o movimento, por causa da inversão
 unsigned long tempoAteFimMovimento = (tempoMaximoMovimentacao -  tempoRestante); 


  //Desacelera antes de inverter  o movimento
  //(tempoAteFimMovimento - tempoAceleracao) -> Dentro do desacelerar vira tempoDecorrido para garantir o tempo de 2000 de desaceleração
  desacelerarMotor(tempoAteFimMovimento, tempoAteFimMovimento - tempoAceleracao);    
  
  movimentoAbrindo = !movimentoAbrindo;  // Inverte o sentido de abertura/fechamento
  if (tempoAteFimMovimento<=0) tempoAteFimMovimento = tempoMaximoMovimentacao;    // Garantir na primeira movimentação que o tempo será o correto
  
  iniciarMovimento(tempoAteFimMovimento);
  
}

void abrirTailgate(unsigned long tempoAteFimMovimento) {
  controlarMotor(L_PWM,R_PWM, tempoAteFimMovimento);  // Abrir 

}

void fecharTailgate(unsigned long tempoAteFimMovimento) {
  controlarMotor(R_PWM,L_PWM, tempoAteFimMovimento);  // Fechar 
}
void controlarMotor(int pinoSentido1,int pinoSentido2, unsigned long tempoAteFimMovimento) {
    //analogWrite(pinoSentido1, LOW);
    analogWrite(pinoSentido1, LOW);
    analogWrite(pinoSentido2, HIGH);
 
     unsigned long tempoDecorrido = 0;

    // Aceleração
    //tempoDecorrido += acelerarMotor(pinoSentido2,tempoAteFimMovimento, tempoAceleracao);
    tempoDecorrido += acelerarMotor(tempoAteFimMovimento, tempoAceleracao);

    // Manter velocidade máxima
    //tempoDecorrido += manterVelocidadeMaxima(pinoSentido2,tempoAteFimMovimento, tempoDecorrido);
    tempoDecorrido += manterVelocidadeMaxima(tempoAteFimMovimento, tempoDecorrido);

    // Desaceleração
    //desacelerarMotor(pinoSentido2,tempoAteFimMovimento, tempoDecorrido);
    desacelerarMotor(tempoAteFimMovimento, tempoDecorrido);
}

unsigned long acelerarMotor(unsigned long tempoAteFimMovimento, unsigned long tempoAceleracao) {
    unsigned long tempoDecorrido = 0;
    unsigned long tempoRestante = tempoAteFimMovimento;
    while (tempoDecorrido <= tempoAceleracao && tempoDecorrido < tempoAteFimMovimento && motorEmMovimento) {
        if (verificarSinalExternoOuObstaculo(tempoRestante)) return tempoDecorrido;  // Interrompe se sinal externo for detectado
        ajustarVelocidade(tempoDecorrido, 0, tempoAceleracao);
        delay(intervalo);
        tempoRestante-=intervalo;
        tempoDecorrido += intervalo;
    }
    return tempoDecorrido;  // Retorna o tempo total decorrido durante a aceleração
}

unsigned long manterVelocidadeMaxima(unsigned long tempoAteFimMovimento, unsigned long tempoDecorrido) {
    unsigned long tempoManter = tempoAteFimMovimento - tempoDecorrido - tempoAceleracao;  // Cálculo do tempo restante
    unsigned long tempoRestante = tempoAteFimMovimento;
    if (tempoManter < 0) tempoManter = 0;  // Garante que o tempo não seja negativo
    while (tempoManter > 0 && motorEmMovimento) {
        if (verificarSinalExternoOuObstaculo(tempoRestante)) return tempoDecorrido;  // Interrompe se sinal externo for detectado
        ajustarVelocidade(tempoDecorrido, 0, tempoAceleracao);  // Mantém a velocidade máxima
        delay(intervalo);
        tempoRestante-=intervalo;
        tempoManter -= intervalo;  // Reduz o tempo restante
        tempoDecorrido += intervalo;  // Atualiza o tempo decorrido
    }
    return tempoDecorrido;  // Retorna o tempo total decorrido após manter a velocidade máxima
}

void desacelerarMotor(unsigned long tempoAteFimMovimento, unsigned long tempoDecorrido) {
    unsigned long tempoDesaceleracao = min(tempoMaximoMovimentacao - tempoDecorrido, tempoAceleracao); // Limitar a desaceleração a 2000 ms ou tempo restante
    unsigned long tempoRestante = tempoAteFimMovimento;
    while (tempoDecorrido <= tempoAteFimMovimento && motorEmMovimento) {
        if (verificarSinalExternoOuObstaculo(tempoRestante)) return;  // Interrompe se sinal externo for detectado
        ajustarVelocidade(tempoAteFimMovimento - tempoDecorrido, 0, tempoDesaceleracao);  // Desacelera o motor
        delay(intervalo);
        tempoRestante-=intervalo;
        tempoDecorrido += intervalo;  // Atualiza o tempo decorrido
    }

    //pararMotor();  // Para o motor ao final do ciclo
}


void ajustarVelocidade(int tempoAteFimMovimento, int inicio, int fim) {
  velocidadeAtual = map(tempoAteFimMovimento, inicio, fim, 0, 255); //3000,0, 2000, 0  ,255
  //analogWrite(pinoSentido2, velocidadeAtual);
  if (motorEmMovimento) analogWrite(PWM, velocidadeAtual); // Pode completar o movimento de reverso pelo verificarSinalExterno() e parar
}

bool verificarSinalExternoOuObstaculo(unsigned long tempoRestante) {
  
  float limite = movimentoAbrindo ? limiteCorrenteAbertura: limiteCorrenteFechamento;
  if (digitalRead(botao) == LOW // Verifica se o botão foi pressionado durante a operação
  || analogRead(correnteEsquerdo)> limite // Verifica se a corente é maior q o limite
  || analogRead(correnteDireito)> limite) // Verifica se a corente é maior q o limite
  { 
    reverterMovimento(tempoRestante);
    return true;
  }
  return false;
  
}

void pararMotor() {
//  analogWrite(R_PWM, LOW);
//  analogWrite(L_PWM, LOW);
//  analogWrite(R_PWM, LOW);
//  analogWrite(L_PWM, LOW);
  analogWrite(R_PWM, LOW);
  analogWrite(L_PWM, LOW);
  analogWrite(PWM, 0);

  motorEmMovimento = false;
  entrarModoSleep();
}

void entrarModoSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();  // Coloca o Arduino em modo sleep até interrupção
}

void wakeUp() {
  sleep_disable();  // Desabilita o sleep quando acorda
}
