#include <avr/sleep.h>
#include <avr/interrupt.h>

#define    encoder_C1   3                     //Conexão C1 do encoder
#define    encoder_C2   4                     //Conexão C2 do encoder
byte      Encoder_C1Last;
volatile int pulsos;
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
const int correnteDireito = A0;  // Pino corrente direita
const float sensibilidade = 0.1; // Sensibilidade (Quantidade de tensãp gerada por cada unidade de corrente- Cada Ampere de corrente, o sensor gerará 0.1V na saida)
//const float correnteMaximaMotor =20;
const float limiteCorrenteAbertura = 25;
const float limiteCorrenteFechamento = 25; 
unsigned long tempoCorrenteAnterior =0;
//const float fatorDeConversaoCorrente = 5 / 1023 * correnteMaximaMotor; // 5 v


// Parâmetros de controle de movimento
const unsigned long tempoMaximoMovimentacao = 8000;     // Tempo máximo "Padrão" de rerentencia
const unsigned long tempoAceleracaoPadrao = 2000;       // Tempo de aceleração/desaceleração (2)
const unsigned long  tempoAteFimMovimento = tempoMaximoMovimentacao;  // Tempo total de abertura/fechamento (8s)
const unsigned long frequenciaVelocidade = 30;       // intervalo em ms para reduzir ou aumementar a velocidade 

// Variáveis de controle de estado
unsigned int velocidadeAtual = 0;                // Velocidade do motor (0 a 255)
const unsigned velocidadeLimiteReferencia = 255;
bool motorEmMovimento = false;               // Estado do motor (ligado/desligado)
bool movimentoAbrindo = false;           // Abertura (true) ou fechamento (false)

// Controle de beep
bool bipar = true;
const int intervaloBeep =500; // 1S
unsigned long tempoBeepAnterior=0;

// Controle de frequencia que calcula a velocidade do motor
const float intervaloPulso =500; // 1S
unsigned long tempoPulsoAnterior=0;
int pulsosAnt=0;

float DESVIO_FREQUENCIA =50; // desvio para calcular a queda da valocidade 10%
float menorVariacao =1000;


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

  tempoPulsoAnterior = millis();
}

void loop() {
  // Verificar se o botão foi pressionado
  if (lerEventoExterno(botao) && !motorEmMovimento) {
    // Iniciar movimento se o botão for pressionado e o motor estiver parado
    iniciarMovimento(tempoAteFimMovimento,velocidadeLimiteReferencia); // 255 velocidade máxima
    
  } 
  pararMotor();
  entrarModoSleep();

}

void iniciarMovimento(unsigned long tempoAteFimMovimento, int velocidadeLimite) {
  delay(300); //  debouce 
  motorEmMovimento = true;
  tempoCorrenteAnterior = millis(); // Evitar leitura inicial

  attachInterrupt(digitalPinToInterrupt(encoder_C1), count_pulses, FALLING);   //Interrupção externa 3 por mudança de estado
  attachInterrupt(digitalPinToInterrupt(botao), wakeUp, FALLING);

  if (movimentoAbrindo) {
    abrirTailgate(tempoAteFimMovimento,velocidadeLimite);
  } else {
    fecharTailgate(tempoAteFimMovimento,velocidadeLimite);
  }
}

void reverterMovimento(unsigned long tempoRestante) {

// Se houver multiplas reversos, o tempo de movimento vai se ajustando.
// tempoRestante tempo que restaria para completar o movimento se não fosse interrompido.
// tempoAteFimMovimento   novo tempo que agora falta para completatar o movimento, por causa da inversão
 unsigned long tempoAteFimMovimento = (tempoMaximoMovimentacao -  tempoRestante); 

  int velocidadeLimite = velocidadeLimiteReferencia;
  if (tempoAteFimMovimento <= tempoAceleracaoPadrao){// Está acelerando ou Desaletando
     velocidadeLimite = velocidadeAtual;   // Se acelerou até X na reversão deve ir até X também
  }//else tempoAteFimMovimento=tempoAteFimMovimento - tempoAceleracaoPadrao;

  
  
  pararMotor();

  Serial.print("REVERTENDO por: ");
  Serial.print(tempoAteFimMovimento);
  Serial.print(" Ate: ");
  Serial.println(velocidadeLimite);
//  Serial.print(" Restava : ");
//  Serial.println(tempoRestante);

  movimentoAbrindo = !movimentoAbrindo;  // Inverte o sentido de abertura/fechamento
  if (tempoAteFimMovimento<=0) tempoAteFimMovimento = tempoMaximoMovimentacao;    // Garantir na primeira movimentação que o tempo será o correto
  
  iniciarMovimento(tempoAteFimMovimento,velocidadeLimite);
  
}

void abrirTailgate(unsigned long tempoAteFimMovimento,int velocidadeLimite) {
  
  Serial.print("ABRINDO por: ");
  controlarMotor(L_PWM,R_PWM, tempoAteFimMovimento,velocidadeLimite);  // Abrir 

}

void fecharTailgate(unsigned long tempoAteFimMovimento, int velocidadeLimite) {
   Serial.print("FECHANDO por: ");
   controlarMotor(R_PWM,L_PWM, tempoAteFimMovimento,velocidadeLimite);  // Fechar 
}
void controlarMotor(int pinoSentido1,int pinoSentido2, unsigned long tempoAteFimMovimento, int velocidadeLimite) {

    digitalWrite(pinoSentido1, LOW);
    digitalWrite(pinoSentido2, HIGH);
    unsigned long tempoDecorrido = 0;

    // Aceleração
    tempoDecorrido += acelerarMotor(tempoAteFimMovimento,velocidadeLimite);
  // Manter velocidade máxima
    tempoDecorrido = manterVelocidadeMaxima(tempoAteFimMovimento, tempoDecorrido);
    // Desaceleração
    desacelerarMotor(tempoAteFimMovimento, tempoDecorrido,velocidadeLimite);


}

unsigned long acelerarMotor(unsigned long tempoAteFimMovimento, int velocidadeLimite) {
    unsigned long tempoDecorrido = 0;
    unsigned long tempoAceleracao = min(tempoAceleracaoPadrao,tempoAteFimMovimento);
   // Serial.print(" TEMPO ACE: ");
   // Serial.print(tempoAceleracao);
   // Serial.print(" TEMPO ATE FIM: ");
   // Serial.print(tempoAteFimMovimento);
    
    while ((tempoDecorrido <= tempoAceleracao) && motorEmMovimento) {
        beep();
        if (verificarSinalExterno(tempoAteFimMovimento-tempoDecorrido)
           || verificarObstaculo(tempoAteFimMovimento-tempoDecorrido))
                pararMotor();  // Interrompe se sinal externo for detectado
        ajustarVelocidade(tempoDecorrido, tempoAceleracao,velocidadeLimite);
        delay(frequenciaVelocidade);
        tempoDecorrido += frequenciaVelocidade;
    }
   // Serial.print("ACELEROU por:");
   // Serial.println(tempoDecorrido);
    return tempoDecorrido;  // Retorna o tempo total decorrido durante a aceleração
}

unsigned long manterVelocidadeMaxima(unsigned long tempoAteFimMovimento, unsigned long tempoDecorrido) {

    long tempoManter = tempoAteFimMovimento - tempoDecorrido - tempoAceleracaoPadrao;  // Cálculo do tempo restante
    if (tempoManter < 0) tempoManter = 0;  // Garante que o tempo não seja negativo
     Serial.println("");
     Serial.print("MANTER por:");
     Serial.println(tempoManter);
    
    while ((tempoManter > 0) && motorEmMovimento) {
        beep();
        if (verificarSinalExterno(tempoAteFimMovimento-tempoDecorrido) 
          ||  verificarObstaculo(tempoAteFimMovimento-tempoDecorrido))
         pararMotor();  // Interrompe se sinal externo for detectado
        delay(frequenciaVelocidade);
        tempoManter -= frequenciaVelocidade;  // Reduz o tempo restante
        tempoDecorrido +=frequenciaVelocidade;  // Atualiza o tempo decorrido

    }
    return tempoDecorrido;  // Retorna o tempo total decorrido após manter a velocidade máxima
}

void desacelerarMotor(unsigned long tempoAteFimMovimento, unsigned long tempoDecorrido, int velocidadeLimite) {
  
    unsigned long tempoDesaceleracao = min(tempoAteFimMovimento - tempoDecorrido, tempoAceleracaoPadrao); // Limitar a desaceleração a 2000 ms ou tempo restante
    Serial.print("DESACELERANDO  por:");
    Serial.println(tempoDesaceleracao);
    while (tempoDecorrido <= tempoAteFimMovimento && motorEmMovimento) {
        beep();
        if (verificarSinalExterno(tempoAteFimMovimento-tempoDecorrido) 
          || verificarObstaculo(tempoAteFimMovimento-tempoDecorrido))
            pararMotor();  // Interrompe se sinal externo for detectado
        ajustarVelocidade(tempoAteFimMovimento - tempoDecorrido, tempoDesaceleracao,velocidadeLimite);  // Desacelera o motor
        delay(frequenciaVelocidade);
        tempoDecorrido += frequenciaVelocidade;  // Atualiza o tempo decorrido
    }
}


void ajustarVelocidade(int tempoAteFimMovimento, int fim, int velocidadeLimite) {
  
  velocidadeAtual = map(tempoAteFimMovimento, 0, fim, 0, velocidadeLimite); //3000,0, 2000, 0  ,255

  if (motorEmMovimento) {
    Serial.println("");
    Serial.print(velocidadeAtual);
    Serial.print("A|");
   
  }
  if (motorEmMovimento)  analogWrite(PWM, velocidadeAtual); 
}

bool verificarSinalExterno(unsigned long tempoRestante) {

  if (lerEventoExterno(botao)){ 
    reverterMovimento(tempoRestante);
    return true;
  }
  return false;
  
}
bool verificarObstaculo(unsigned long tempoRestante) {
  // Cada movimento abrindo ou fechando terá seu limite máximo de correte
  float limite = movimentoAbrindo ? limiteCorrenteAbertura: limiteCorrenteFechamento;
    if (lerCorrente(correnteEsquerdo,correnteDireito) > limite  // Verifica se a corente é maior q o limite
        //temObstaculo()
        ){ 
      reverterMovimento(tempoRestante);
      return true;
    }
  return false;
  
}

bool temObstaculo(){
  bool caiu = false;
 

  if ((millis() - tempoPulsoAnterior) >= intervaloPulso){
    float variacao =  1 - (pulsosAnt/pulsos);
   
    if (variacao < 0){ // DEsacelerou
       // Serial.print("ATUAL:");
       // Serial.println(abs(variacao * 100));
        if (abs(variacao * 100) < menorVariacao)  {
            menorVariacao = abs(variacao * 100);
        }
        caiu =  abs(variacao * 100) > DESVIO_FREQUENCIA;
        if (caiu){
            Serial.print("OBSTACULO");
            detachInterrupt(digitalPinToInterrupt(encoder_C1));   //Interrupção externa 3 por mudança de estado
            detachInterrupt(digitalPinToInterrupt(botao));
        }
    }
    pulsosAnt = pulsos;
    pulsos=0;
    tempoPulsoAnterior =millis();
    }
    
    if (pulsosAnt==0){
        pulsosAnt = pulsos;
    } 

  return caiu;
}
void beep(){

  if ((millis() - tempoBeepAnterior) >= intervaloBeep){
      digitalWrite(SAIDAPULSO, bipar);
      tempoBeepAnterior = millis();
      bipar = !bipar;
      if (bipar){
       // Serial.println("BEEP");
      }
  }
   

}

void noBeep(){

   digitalWrite(SAIDAPULSO, LOW);

}

float lerCorrente(int pinCorrenteD,int pinCorrenteE){

  float tensao = 0;
  
  float tensaoA = leituraMediaCorrente(pinCorrenteD) * (5 /1023.0); // 5V máxima voltagem do pino que corresponde a 1023.0
  float tensaoB = leituraMediaCorrente(pinCorrenteE) * (5 /1023.0); // 5V máxima voltagem do pino que corresponde a 1023.0
  
  if ((millis() - tempoCorrenteAnterior) >= intervaloPulso){

     
     tensao = max(tensaoA,tensaoB);
    Serial.println("");
    Serial.print(" CORRENTE: ");
   Serial.print(tensao / sensibilidade);
     tempoCorrenteAnterior = millis();

  }

    return tensao / sensibilidade;

}

float leituraMediaCorrente(int pinCorrente){

  long soma=0;
   for(int i=1;i<=10;i++){
       soma = soma + analogRead(pinCorrente);
      
  }

  return soma/10;

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
  Serial.println("");
  Serial.println("PAROU");
  noBeep();
  digitalWrite(R_PWM, LOW);
  digitalWrite(L_PWM, LOW);
  ajustarVelocidade(0,0,0);
  motorEmMovimento = false;
  delay(intervaloPulso);
}

void entrarModoSleep() {
  movimentoAbrindo=!movimentoAbrindo;
 // Serial.println("DORMIU");

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();  // Coloca o Arduino em modo sleep até interrupção
  
}

void count_pulses()
{
   pulsos++; //incrementa número do pulso se direction limpa
   
} //end count_pulses
void wakeUp() {
  sleep_disable();  // Desabilita o sleep quando acorda
}
