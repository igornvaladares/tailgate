#include <avr/sleep.h>
#include <avr/interrupt.h>

#define    encoder_C1   3                     //Conexão C1 do encoder
#define    encoder_C2   4                     //Conexão C2 do encoder
byte      Encoder_C1Last;
int       pulse_number;
boolean direction_m;


// Pinos de controle da ponte H
const int R_PWM = 7;  // Pino PWM de controle da direção 1
const int L_PWM = 8;  // Pino PWM de controle da direção 2

//const int R_ENA = 9;  // Pino de habilitação da direção R
const int SAIDAPULSO = 10;  // Pino saida pulsada

const int PWM = 11;  // Ligar os pinos R_ENA e L_ENA no D11 do arduino

// Pino do botão de controle externo
const int botao = 2;  // Pino de interrupção para wake-up

// Pino do sensor corrente
const int correnteEsquerdo = A1;  // Pino corrente esquerda
const int correnteDireito = A2;  // Pino corrente direita
const float sensibilidade = 0.1; // Sensibilidade (Quantidade de tensãp gerada por cada unidade de corrente- Cada Ampere de corrente, o sensor gerará 0.1V na saida)
// Parâmetros de controle de movimento
const unsigned long tempoMaximoMovimentacao = 7000;     // Tempo máximo "Padrão" de rerentencia
const unsigned long tempoAceleracaoPadrao = 2000;       // Tempo de aceleração/desaceleração (1.5s)
const unsigned long  tempoAteFimMovimento = tempoMaximoMovimentacao;  // Tempo total de abertura/fechamento (8s)
const unsigned long frequenciaVelocidade = 30;       // intervalo em ms para reduzir ou aumementar a velocidade 
// Parâmetros para Sensiblidade do sensor de corrente

//const float correnteMaximaMotor =20;
const float limiteCorrenteAbertura = 5;
const float limiteCorrenteFechamento = 5; 
//const float fatorDeConversaoCorrente = 5 / 1023 * correnteMaximaMotor; // 5 v

// Variáveis de controle de estado
int velocidadeAtual = 0;                // Velocidade do motor (0 a 255)
bool motorEmMovimento = false;               // Estado do motor (ligado/desligado)
bool movimentoAbrindo = false;           // Abertura (true) ou fechamento (false)

// Controle de beep
bool bipar = true;
const int intervaloBeep =500; // 1S
unsigned long tempoBeepAnterior=0;

// Controle de frequencia que calcula a velocidade do motor
const int intervaloPulso =500; // 1S
unsigned long tempoPulsoAnterior=0;
float freq=0;float freqAnt=0;
float desvioFrenquencia =100; // desvio para calcular a queda da valocidade 



void setup() {
   // Configurar pinos como saída
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(SAIDAPULSO, OUTPUT);
  pinMode(PWM, OUTPUT);
  Serial.begin(9600);  // Para debug (opcional)

  //pinMode(R_ENA, OUTPUT);
  //pinMode(L_ENA, OUTPUT);
  //digitalWrite(R_ENA, HIGH);
  //digitalWrite(L_ENA, HIGH);

  // Configurar o pino do botão como entrada com PULLUP interno
  pinMode(botao, INPUT_PULLUP);
  pinMode(encoder_C1, INPUT_PULLUP);
  
  // Configurar interrupção para o botão (borda de descida)
  attachInterrupt(digitalPinToInterrupt(botao), wakeUp, FALLING);

  
  attachInterrupt(digitalPinToInterrupt(encoder_C1), count_pulses, CHANGE);   //Interrupção externa 3 por mudança de estado

}

void loop() {
  // Verificar se o botão foi pressionado
  if (lerEventoExterno(botao) && !motorEmMovimento) {
    // Iniciar movimento se o botão for pressionado e o motor estiver parado
    iniciarMovimento(tempoAteFimMovimento);
    
  } 
    Serial.print("PULSO");
  Serial.println(pulse_number);
  pararMotor();
  entrarModoSleep();

}

void iniciarMovimento(unsigned long tempoAteFimMovimento) {
  delay(300); //  debouce 
  motorEmMovimento = true;
  if (movimentoAbrindo) {
    abrirTailgate(tempoAteFimMovimento);
  } else {
    fecharTailgate(tempoAteFimMovimento);
  }
}

void reverterMovimento(unsigned long tempoRestante) {

  
  pararMotor();
// Se houver multiplas reversos, o tempo de movimento vai se ajustando.
// tempoRestante tempo que restaria para completar o movimento se não fosse interrompido.
// tempoAteFimMovimento   novo tempo que agora falta para completatar o movimento, por causa da inversão
 unsigned long tempoAteFimMovimento = (tempoMaximoMovimentacao -  tempoRestante); 
  Serial.print("REVERTENDO por: ");
  Serial.print(tempoAteFimMovimento);
  Serial.print(" Restava : ");
  Serial.println(tempoRestante);

  movimentoAbrindo = !movimentoAbrindo;  // Inverte o sentido de abertura/fechamento
  if (tempoAteFimMovimento<=0) tempoAteFimMovimento = tempoMaximoMovimentacao;    // Garantir na primeira movimentação que o tempo será o correto
  
  iniciarMovimento(tempoAteFimMovimento);
  
}

void abrirTailgate(unsigned long tempoAteFimMovimento) {
  Serial.print("ABRINDO por: ");
  controlarMotor(L_PWM,R_PWM, tempoAteFimMovimento);  // Abrir 

}

void fecharTailgate(unsigned long tempoAteFimMovimento) {
   Serial.print("FECHANDO por: ");

  controlarMotor(R_PWM,L_PWM, tempoAteFimMovimento);  // Fechar 
}
void controlarMotor(int pinoSentido1,int pinoSentido2, unsigned long tempoAteFimMovimento) {
    //analogWrite(pinoSentido1, LOW);
    Serial.println(tempoAteFimMovimento);
    analogWrite(pinoSentido1, LOW);
    analogWrite(pinoSentido2, HIGH);
    unsigned long tempoDecorrido = 0;

    // Aceleração
    //tempoDecorrido += acelerarMotor(pinoSentido2,tempoAteFimMovimento, tempoAceleracao);
    tempoDecorrido += acelerarMotor(tempoAteFimMovimento);
  // Manter velocidade máxima
    //tempoDecorrido += manterVelocidadeMaxima(pinoSentido2,tempoAteFimMovimento, tempoDecorrido);
    tempoDecorrido = manterVelocidadeMaxima(tempoAteFimMovimento, tempoDecorrido);
    // Desaceleração
    //desacelerarMotor(pinoSentido2,tempoAteFimMovimento, tempoDecorrido);
    desacelerarMotor(tempoAteFimMovimento, tempoDecorrido);


}

unsigned long acelerarMotor(unsigned long tempoAteFimMovimento) {
    unsigned long tempoDecorrido = 0;
    unsigned long tempoAceleracao = min(tempoAceleracaoPadrao,tempoAteFimMovimento);
    Serial.print("VELOCIDADE ACE:");
    while ((tempoDecorrido <= tempoAceleracao) && motorEmMovimento) {
        beep();
        if (verificarSinalExternoOuObstaculo(tempoAteFimMovimento-tempoDecorrido)) pararMotor();  // Interrompe se sinal externo for detectado
        ajustarVelocidade(tempoDecorrido, tempoAceleracao);
        delay(frequenciaVelocidade);
        tempoDecorrido += frequenciaVelocidade;

    }
    Serial.print("ACELEROU por:");
    Serial.println(tempoDecorrido);
    return tempoDecorrido;  // Retorna o tempo total decorrido durante a aceleração
}

unsigned long manterVelocidadeMaxima(unsigned long tempoAteFimMovimento, unsigned long tempoDecorrido) {

    long tempoManter = tempoAteFimMovimento - tempoDecorrido - tempoAceleracaoPadrao;  // Cálculo do tempo restante
    if (tempoManter < 0) tempoManter = 0;  // Garante que o tempo não seja negativo
     Serial.print("MANTER por:");
    Serial.println(tempoManter);
    

    while ((tempoManter > 0) && motorEmMovimento) {
        beep();
        if (verificarSinalExternoOuObstaculo(tempoAteFimMovimento-tempoDecorrido)) pararMotor();  // Interrompe se sinal externo for detectado
        delay(frequenciaVelocidade);
        tempoManter -= frequenciaVelocidade;  // Reduz o tempo restante
        tempoDecorrido +=frequenciaVelocidade;  // Atualiza o tempo decorrido

    }
    return tempoDecorrido;  // Retorna o tempo total decorrido após manter a velocidade máxima
}

void desacelerarMotor(unsigned long tempoAteFimMovimento, unsigned long tempoDecorrido) {
  
    unsigned long tempoDesaceleracao = min(tempoAteFimMovimento - tempoDecorrido, tempoAceleracaoPadrao); // Limitar a desaceleração a 2000 ms ou tempo restante
    Serial.print("DESACELERANDO  por:");
    Serial.println(tempoDesaceleracao);

    Serial.print("VELOCIDADE DESACE:");
    while (tempoDecorrido <= tempoAteFimMovimento && motorEmMovimento) {
        beep();
        if (verificarSinalExternoOuObstaculo(tempoAteFimMovimento-tempoDecorrido)) pararMotor();  // Interrompe se sinal externo for detectado
        ajustarVelocidade(tempoAteFimMovimento - tempoDecorrido, tempoDesaceleracao);  // Desacelera o motor
        delay(frequenciaVelocidade);
        tempoDecorrido += frequenciaVelocidade;  // Atualiza o tempo decorrido

    }

    //pararMotor();  // Para o motor ao final do ciclo
}


void ajustarVelocidade(int tempoAteFimMovimento, int fim) {
  
  velocidadeAtual = map(tempoAteFimMovimento, 0, fim, 0, 255); //3000,0, 2000, 0  ,255
  //analogWrite(pinoSentido2, velocidadeAtual);
  if (motorEmMovimento) {
    Serial.print(velocidadeAtual);
    Serial.print("A|");
  }

  if (motorEmMovimento) analogWrite(PWM, velocidadeAtual); // Pode completar o movimento de reverso pelo verificarSinalExterno() e parar
}

bool verificarSinalExternoOuObstaculo(unsigned long tempoRestante) {
  
  float limite = movimentoAbrindo ? limiteCorrenteAbertura: limiteCorrenteFechamento;
   // VERIFICAR PULSO com millis
  if (lerEventoExterno(botao) // Verifica se o botão foi pressionado durante a operação
 // || lerCorrente(correnteEsquerdo)> limite // Verifica se a corente é maior q o limite
 // || lerCorrente(correnteDireito)> limite) // Verifica se a corente é maior q o limite
    || caiuFrequencia()
  )
  { 
    reverterMovimento(tempoRestante);
    return true;
  }
  return false;
  
}
bool caiuFrequencia(){

    return (freqAnt > (freq + desvioFrenquencia));

}
void frequenciaPulso(){

  if ((millis() - tempoPulsoAnterior) >= intervaloPulso){
    freq =   pulse_number / intervaloPulso;
    freqAnt = freq;
    pulse_number=0;
    tempoPulsoAnterior =millis();
  }
   
}
void beep(){

  if ((millis() - tempoBeepAnterior) >= intervaloBeep){
      digitalWrite(SAIDAPULSO, bipar);
      tempoBeepAnterior = millis();
      bipar = !bipar;
      if (bipar){
      //  Serial.println("BEEP");
      }
  }
   

}

void noBeep(){

       digitalWrite(SAIDAPULSO, LOW);

}
float lerCorrente(int pinCorrente){

    int leituraAnalogica =  analogRead(pinCorrente);
    float tensao = leituraAnalogica * (5 /1023.0); // 5V máxima voltagem do pino que corresponde a 1023.0
    return tensao / sensibilidade;

}

int lerEventoExterno(int pin_entrada) {

  int estado;
  if (digitalRead(pin_entrada) == LOW)
  {
    estado = digitalRead(pin_entrada);
    if (estado == LOW)
    {
      while (estado == LOW)
        estado = digitalRead(pin_entrada);
      return 1;
    }  
  }
  return 0;
 }

void pararMotor() {
  Serial.println("PAROU");
  noBeep();

//  analogWrite(R_PWM, LOW);
//  analogWrite(L_PWM, LOW);
//  analogWrite(R_PWM, LOW);
//  analogWrite(L_PWM, LOW);
  analogWrite(R_PWM, LOW);
  analogWrite(L_PWM, LOW);
  Serial.print("VELOCIDADE ATUAL:");
  ajustarVelocidade(0,0);


  motorEmMovimento = false;
}

void entrarModoSleep() {
  movimentoAbrindo=!movimentoAbrindo;
  Serial.println("DORMIU");

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();  // Coloca o Arduino em modo sleep até interrupção
  
}

void count_pulses()
{

  if (digitalRead(encoder_C1)){
      pulse_number++;           //incrementa número do pulso se direction limpa
  }       //Lê estado de encoder_C1 e armazena em Lstate
  
  
} //end count_pulses
void wakeUp() {
  sleep_disable();  // Desabilita o sleep quando acorda
}
