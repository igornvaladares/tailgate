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
const float limiteCorrenteAbertura = 50;
const float limiteCorrenteFechamento = 49; 
long tempoCorrenteAnterior =0;
//const float fatorDeConversaoCorrente = 5 / 1023 * correnteMaximaMotor; // 5 v


// Parâmetros de controle de movimento
#define TEMPO_MAXIMO_MOVIMENTACAO  8000     // Tempo máximo "Padrão" de rerentencia
//#define TEMPO_MAXIMO_MOVIMENTACAO  9000     // Tempo máximo "Padrão" de rerentencia

#define TEMPO_ACELERACAO_PADRAO  1500       // Tempo de aceleração
#define TEMPO_DESACELERACAO_PADRAO  1800       // Tempo de desaceleração (2)
const long frequenciaVelocidade = 30;       // intervalo em ms para reduzir ou aumementar a velocidade 

// Variáveis de controle de estado
int forcaAtual = 0;                // Velocidade do motor (0 a 254)
#define FORCA_LIMITE_REFERENTE 200

bool motorEmMovimento = false;               // Estado do motor (ligado/desligado)
bool movimentoAbrindo = false;           // Abertura (true) ou fechamento (false)

// Controle de beep
bool bipar = true;
const int intervaloBeep =500; // 1S
long tempoBeepAnterior=0;

// Controle de frequencia que calcula a velocidade do motor
const float intervaloPulso =500; // 1S
long tempoPulsoAnterior=0;
int pulsosAnt=0;

#define DESVIO_FREQUENCIA 50; // desvio para calcular a queda da valocidade 10%
float menorVariacao =1000;


void setup() {
   // Configurar pinos como saída
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(SAIDAPULSO, OUTPUT);
  pinMode(PWM, OUTPUT);
  Serial.begin(9600);  // Para debug (opcional)

  // Configurar o pino do botão como entrada com PULLUP interno
  pinMode(botao, INPUT_PULLUP);
  pinMode(encoder_C1, INPUT_PULLUP);
  
  // Configurar interrupção para o botão (borda de descida)
  attachInterrupt(digitalPinToInterrupt(botao), wakeUp, FALLING);

  tempoPulsoAnterior = millis();
}


void iniciarMovimento(  float tempoAteFimMovimento =TEMPO_MAXIMO_MOVIMENTACAO, int forcaLimite=FORCA_LIMITE_REFERENTE, int reversoes=0) {
  delay(250); //  debouce 
  motorEmMovimento = true;
  tempoCorrenteAnterior = millis(); // Evitar leitura inicial

  
  attachInterrupt(digitalPinToInterrupt(encoder_C1), count_pulses, FALLING);   //Interrupção externa 3 por mudança de estado
  attachInterrupt(digitalPinToInterrupt(botao), wakeUp, FALLING);

  if (movimentoAbrindo) {
    abrirTailgate(tempoAteFimMovimento/2,forcaLimite,reversoes);// QUando diminui o tempoAteFimMovimento siginifica diminuir o tempo na força maxima
  } else {
    fecharTailgate(tempoAteFimMovimento,forcaLimite,reversoes);
  }

}
void loop() {
  // Verificar se o botão foi pressionado
  if (lerEventoExterno(botao) && !motorEmMovimento) {
    // Iniciar movimento se o botão for pressionado e o motor estiver parado
    iniciarMovimento(); // 255 velocidade máxima
    movimentoAbrindo=!movimentoAbrindo;

  } 
  pararMotor();
  entrarModoSleep();

}


void reverterMovimento(  float tempoRestante, int reversoes =0) {


   Serial.println("");
   Serial.print("REVERSOES: ");
  Serial.print(reversoes);

  if (reversoes>0){
      reiniciar();
  }
// Se houver multiplas reversos, o tempo de movimento vai se ajustando.
// tempoRestante tempo que restaria para completar o movimento se não fosse interrompido.
// tempoAteFimMovimento   novo tempo que agora falta para completatar o movimento, por causa da inversão
// Tempo de abertura é 50% do tempo e da força de fechamento
  float tempoAteFimMovimento;
  int forcaLimite = FORCA_LIMITE_REFERENTE;
  if (!movimentoAbrindo){
    
    tempoAteFimMovimento = (TEMPO_MAXIMO_MOVIMENTACAO -  tempoRestante) * 0.5; 
    forcaLimite = forcaAtual* 0.5;   // Se acelerou até X na reversão deve ir até X também

  }
  else tempoAteFimMovimento = TEMPO_MAXIMO_MOVIMENTACAO -  tempoRestante; 

  Serial.println("");
  Serial.print("TEMPO");
  Serial.print(TEMPO_MAXIMO_MOVIMENTACAO);
  Serial.print(":");
  Serial.print(tempoRestante);
  Serial.print(":");
  Serial.print(movimentoAbrindo);

  pararMotor();

  Serial.print("REVERTENDO por: ");
  Serial.print(tempoAteFimMovimento);
  Serial.print(" Ate: ");
  Serial.println(forcaLimite);
//  Serial.print(" Restava : ");
//  Serial.println(tempoRestante);

  if (tempoAteFimMovimento<=0) tempoAteFimMovimento = TEMPO_MAXIMO_MOVIMENTACAO;    // Garantir na primeira movimentação que o tempo será o correto
  
   movimentoAbrindo = !movimentoAbrindo;  // Inverte o sentido de abertura/fechamento

  iniciarMovimento(tempoAteFimMovimento,forcaLimite,++reversoes);
  
}
long acelerarMotor(float tempoAteFimMovimento, int forcaLimite, int reversoes,long tempoDecorrido = 0) {
      long tempoAceleracao = min(TEMPO_ACELERACAO_PADRAO,tempoAteFimMovimento);
   // Serial.print(" TEMPO ACE: ");
   // Serial.print(tempoAceleracao);
   // Serial.print(" TEMPO ATE FIM: ");
   // Serial.print(tempoAteFimMovimento);
    
    while ((tempoDecorrido <= tempoAceleracao) && motorEmMovimento) {
      ajustarVelocidade(tempoDecorrido, tempoAceleracao,forcaLimite);
      beep();
      if (verificarSinalExterno(tempoAteFimMovimento-tempoDecorrido)
          || verificarObstaculo(tempoAteFimMovimento-tempoDecorrido,reversoes))
              pararMotor();  // Interrompe se sinal externo for detectado
      
      delay(frequenciaVelocidade);
      tempoDecorrido += frequenciaVelocidade;
    }
    Serial.println("");
    Serial.print("ACELEROU por:");
    Serial.println(tempoDecorrido);
    return tempoDecorrido;  // Retorna o tempo total decorrido durante a aceleração
}

void controlarMotor(int pinoSentido1,int pinoSentido2,   float tempoAteFimMovimento, int forcaLimite, int reversoes,long tempoDecorrido = 0) {
 
    digitalWrite(pinoSentido1, LOW);
    digitalWrite(pinoSentido2, HIGH);

    // Aceleração
    tempoDecorrido = acelerarMotor(tempoAteFimMovimento,forcaLimite,reversoes,tempoDecorrido);
  // Manter velocidade máxima
    tempoDecorrido = manterVelocidadeMaxima(tempoAteFimMovimento, tempoDecorrido,reversoes);
    // Desaceleração
    desacelerarMotor(tempoAteFimMovimento, tempoDecorrido,forcaLimite);


}
void abrirTailgate(  float tempoAteFimMovimento,int forcaLimite,int reversoes) {
  
  Serial.print("ABRINDO por: ");
  long tempoDecorrido=0;
  if (reversoes==0){
     tempoDecorrido = arrancar(L_PWM,R_PWM, 1000,255); 
   } // Abrir 
  controlarMotor(L_PWM,R_PWM, tempoAteFimMovimento,forcaLimite,reversoes,tempoDecorrido);  // Abrir 
  freiar(R_PWM,L_PWM,tempoAteFimMovimento,10);  // Abrir 


}                                      

void fecharTailgate(  float tempoAteFimMovimento, int forcaLimite,int reversoes) {
   Serial.print("FECHANDO por: ");
   controlarMotor(R_PWM,L_PWM, tempoAteFimMovimento,forcaLimite,reversoes);  // Fechar 
}

 long arrancar(int pinoSentido1,int pinoSentido2,   float tempoAteFimMovimento, int forcaLimite) {
      digitalWrite(pinoSentido1, LOW);
      digitalWrite(pinoSentido2, HIGH);

      long tempoDecorrido = 0;
      long tempoAceleracao = tempoAteFimMovimento;
   // Serial.print(" TEMPO ACE: ");
   // Serial.print(tempoAceleracao);
   // Serial.print(" TEMPO ATE FIM: ");
   // Serial.print(tempoAteFimMovimento);
    
    while ((tempoDecorrido <= tempoAceleracao) && motorEmMovimento) {
        beep();
        ajustarVelocidade(tempoDecorrido, tempoAceleracao,forcaLimite);
        delay(frequenciaVelocidade);
        tempoDecorrido += frequenciaVelocidade;
    }
    Serial.println("");
    Serial.print("ARRAMCOU por:");
    Serial.println(tempoDecorrido);
    return tempoDecorrido;  // Retorna o tempo total decorrido durante a aceleração
}


  float manterVelocidadeMaxima(  float tempoAteFimMovimento,   float tempoDecorrido, int reversoes) {

    float tempoManter = tempoAteFimMovimento - tempoDecorrido - TEMPO_DESACELERACAO_PADRAO;  // Cálculo do tempo restante
    if (tempoManter < 0) tempoManter = 0;  // Garante que o tempo não seja negativo
    Serial.println("");
    Serial.print("MANTER por:");
    Serial.print(tempoManter);  
    while ((tempoManter > 0) && motorEmMovimento) {
        beep();
        if (verificarSinalExterno(tempoAteFimMovimento-tempoDecorrido) 
          ||  verificarObstaculo(tempoAteFimMovimento-tempoDecorrido,reversoes))
         pararMotor();  // Interrompe se sinal externo for detectado
        delay(frequenciaVelocidade);
        tempoManter -= frequenciaVelocidade;  // Reduz o tempo restante
        tempoDecorrido +=frequenciaVelocidade;  // Atualiza o tempo decorrido

    }
    //Serial.print(" MANTEVE por:");
    //Serial.println(tempoDecorrido-2000);
    return tempoDecorrido;  // Retorna o tempo total decorrido após manter a velocidade máxima
}

void desacelerarMotor(  float tempoAteFimMovimento,   float tempoDecorrido, int forcaLimite) {
  
    // Evitar que o motor pare bruscamente
    if (tempoAteFimMovimento < tempoDecorrido)
        tempoDecorrido = 0;
   
    float tempoDesaceleracao = min(tempoAteFimMovimento - tempoDecorrido, TEMPO_DESACELERACAO_PADRAO); // Limitar a desaceleração a 2000 ms ou tempo restante
    
    Serial.print("DESACELERANDO  por:");
    Serial.print(tempoDesaceleracao);
    Serial.print(":");
    Serial.print(tempoAteFimMovimento);
    Serial.print(":");
    Serial.println(tempoDecorrido);
    
    while (tempoDecorrido <= tempoAteFimMovimento && motorEmMovimento) {
        ajustarVelocidade(tempoAteFimMovimento - tempoDecorrido, tempoDesaceleracao,forcaLimite);  // Desacelera o motor
        beep();
        if (verificarSinalExterno(tempoAteFimMovimento-tempoDecorrido)) 
              pararMotor();  // Interrompe se sinal externo for detectado
        delay(frequenciaVelocidade);
        tempoDecorrido += frequenciaVelocidade;  // Atualiza o tempo decorrido
    }
}

void freiar(int pinoSentido1,int pinoSentido2, float tempoAteFimMovimento, int forcaLimite) {
  
    digitalWrite(pinoSentido1, LOW);
    digitalWrite(pinoSentido2, HIGH);
    // min 1500 
    // max 2500
    tempoAteFimMovimento = max(TEMPO_ACELERACAO_PADRAO,tempoAteFimMovimento-TEMPO_ACELERACAO_PADRAO);

    Serial.print("Freiar  por:");
    Serial.println(tempoAteFimMovimento);
    int tempoDecorrido =0;
    while (tempoDecorrido <= tempoAteFimMovimento && motorEmMovimento) {
        beep();
        ajustarVelocidade(255, 255,forcaLimite);  // Desacelera o motor
        delay(frequenciaVelocidade);
        tempoDecorrido += frequenciaVelocidade;  // Atualiza o tempo decorrido
    }
    pararMotor();
}

void ajustarVelocidade(float tempoAteFimMovimento, int fim, int forcaLimite) {
  
  forcaAtual = map(tempoAteFimMovimento, 0, fim, 0, forcaLimite) < 0 ? 0 : map(tempoAteFimMovimento, 0, fim, 0, forcaLimite) ; //3000,0, 2000, 0  ,255

  if (motorEmMovimento) {
    Serial.print(forcaAtual);
    Serial.print("A|");
   
  }
  if (motorEmMovimento)  analogWrite(PWM, forcaAtual); 
}

bool verificarSinalExterno(  float tempoRestante) {

  if (lerEventoExterno(botao)){ 
    reverterMovimento(tempoRestante);
    return true;
  }
  return false;
  
}
bool verificarObstaculo(  float tempoRestante, int reversoes) {
  // Cada movimento abrindo ou fechando terá seu limite máximo de correte
  // reversoes conta as reversões por obstáculo
  float limite = movimentoAbrindo ? limiteCorrenteAbertura: limiteCorrenteFechamento;
    if (lerCorrente(correnteEsquerdo,correnteDireito) > limite  // Verifica se a corente é maior q o limite
        //temObstaculo()
        ){ 
      reverterMovimento(tempoRestante,reversoes);
      return true;
    }
  return false;
  
}

bool temObstaculo(){
  bool obstaculo = false;
 
   if ((millis() - tempoPulsoAnterior) >= intervaloPulso){
    float variacao =  1 - (pulsosAnt/pulsos);
   
    if (variacao < 0){ // DEsacelerou
       // Serial.print("ATUAL:");
       // Serial.println(abs(variacao * 100));
        if (abs(variacao * 100) < menorVariacao)  {
            menorVariacao = abs(variacao * 100);
        }
        obstaculo =  abs(variacao * 100) > DESVIO_FREQUENCIA;
        if (obstaculo){
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

  return obstaculo;
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
     tempoCorrenteAnterior = millis();
  }
  return tensao / sensibilidade;

}

void pararMotor() {
  Serial.println("");
  Serial.println("PAROU");
  noBeep();
  //digitalWrite(R_PWM, LOW);
 // digitalWrite(L_PWM, LOW);
  ajustarVelocidade(0,0,0);
  motorEmMovimento = false;
  delay(intervaloPulso);
}

void entrarModoSleep() {
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

void reiniciar(){
   pararMotor();
   asm volatile ("   jmp 0");
}
