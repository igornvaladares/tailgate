#include <avr/sleep.h>
#include <EnableInterrupt.h>
#include <avr/power.h>


// Pino do botão de controle externo
const int pinAcaoExterna12V = 5;  // Pino de interrupção para wake-up
const int pinSensorMala = 2;  // Pino de leitura se mala aberta ou fechada
const int pinBotao = 4;  // Pino de leitura botao da mala

// Pinos de controle da ponte H
const int R_PWM = 12;  // Pino PWM de controle da direção 1
const int L_PWM = 13;  // Pino PWM de controle da direção 2
const int PWM = 11;  // Ligar os pinos R_ENA e L_ENA no D11 do arduino

const int pinBeep = 9;  // Pino saida beep

// Pino do sensor corrente
const int pinCorrenteEsquerdo = A1;  // Pino corrente esquerda
const int pinCorrenteDireito = A0;  // Pino corrente direita
const int pinTensao = A3;  // Pino tensao trabalho

const float sensibilidade = 0.1; // Sensibilidade (Quantidade de tensãp gerada por cada unidade de corrente- Cada Ampere de corrente, o sensor gerará 0.1V na saida)
const float limiteCorrenteAbertura = 51;
const float limiteCorrenteFechamento = 46; 
long tempoCorrenteAnterior =0;

// Parâmetros de controle de movimento
#define TEMPO_MAXIMO_MOVIMENTACAO 10500     // Tempo máximo "Padrão" de rerentencia
#define TEMPO_ACELERACAO_PADRAO  1500       // Tempo de aceleração
#define TEMPO_DESACELERACAO_PADRAO  2000       // Tempo de desaceleração (2)
const long frequenciaVelocidade = 30;       // intervalo em ms para reduzir ou aumementar a velocidade 

// Variáveis de controle de estado
int forcaAtual = 0;                // Velocidade do motor (0 a 254)
#define FORCA_LIMITE_REFERENTE 180

bool motorEmMovimento = false;               // Estado do motor (ligado/desligado)
bool abrir = false;           // Abertura (true) ou fechamento (false)
// Controle de frequencia que calcula a velocidade do motor
const float intervaloBeep =500; // 1S
long tempoBeepAnterior=0;
int clicksAnt=0;

void setup() {
  Serial.begin(9600);  // Para debug (opcional)
    // Configurar pinos como saída
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(PWM, OUTPUT);
  
  pinMode(pinBeep, OUTPUT);
  
  beep();
  delay(500);
  noBeep();
  // Configurar o pino do botão como entrada com PULLUP interno
  pinMode(pinAcaoExterna12V, INPUT_PULLUP);
  pinMode(pinBotao, INPUT_PULLUP);
  pinMode(pinSensorMala, INPUT_PULLUP);
 // Configurar interrupção para o botão (borda de descida)
  enableInterrupt(pinAcaoExterna12V, acaoExterna12V, RISING);  
  enableInterrupt(pinBotao, acaoExterna12V, RISING);  

  //Serial.print("Mala :");
  //Serial.print(digitalRead(pinSensorMala));
  tempoBeepAnterior = millis();
}


void iniciarMovimento(  float tempoAteFimMovimento =TEMPO_MAXIMO_MOVIMENTACAO, int forcaLimite=FORCA_LIMITE_REFERENTE, int reversoes=0) {
  delay(250); //  debouce 
  motorEmMovimento = true;
  tempoCorrenteAnterior = millis(); // Evitar leitura inicial
  calibrarSentidoMotor(abrir);  
  if (abrir) {
    abrirTailgate(tempoAteFimMovimento/2.0,forcaLimite,reversoes);// QUando diminui o tempoAteFimMovimento siginifica diminuir o tempo na força maxima
  } else {
    fecharTailgate(tempoAteFimMovimento,forcaLimite,reversoes);
  }

}
void loop() {
  //  Serial.print("Tensão :");
   //Serial.println(obterTensaoDeTrabalho());
    Serial.print("MALA :");
  Serial.println(digitalRead(pinSensorMala));

  // Verificar se o botão foi pressionado
  if (lerEventoExterno(pinAcaoExterna12V)){
    abrir = isFechado(); 
    // Iniciar movimento se o botão for pressionado e o motor estiver parado
    iniciarMovimento();
  } 

// BOTAO SÓ PARA FECHAR.
 if (lerEventoExterno(pinBotao) && isAberto()){
    abrir = isFechado(); 
    // Iniciar movimento se o botão for pressionado e o motor estiver parado
    iniciarMovimento();
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
  if (!abrir){
    if (tempoRestante>=1500.0) { 
      // Se o movimento de descida for menor igual a 6.5s (8000"TEMPO_MAXIMO" - 1500), o retorno deve ser beeeemm de leve.
        tempoAteFimMovimento = (TEMPO_MAXIMO_MOVIMENTACAO -  tempoRestante) * 0.15; 
        forcaLimite = forcaAtual* 0.15;   // Se acelerou até X na reversão deve ir até X também

    }else{
        tempoAteFimMovimento = (TEMPO_MAXIMO_MOVIMENTACAO -  tempoRestante) * 0.5; 
        forcaLimite = forcaAtual* 0.5;   // Se acelerou até X na reversão deve ir até X também
      }
  }
  else tempoAteFimMovimento = TEMPO_MAXIMO_MOVIMENTACAO -  tempoRestante; 

  pararMotor();

  Serial.print("REVERTENDO por: ");
  Serial.print(tempoAteFimMovimento);
  Serial.print(" Ate: ");
  Serial.println(forcaLimite);
  if (tempoAteFimMovimento<=0) tempoAteFimMovimento = TEMPO_MAXIMO_MOVIMENTACAO;    // Garantir na primeira movimentação que o tempo será o correto
  abrir = !abrir;  // Inverte o sentido de abertura/fechamento
  iniciarMovimento(tempoAteFimMovimento,forcaLimite,++reversoes);
  
}
void abrirTailgate(float tempoAteFimMovimento,int forcaLimite,int reversoes) {
   // Serial.print("ABRINDO por: ");
    long tempoDecorrido=0;
    long forcaArrancadaFinal = 50;
    long tempoVariavel= 600;
    if (reversoes==0){
       long tempoAceleracaoMaxima = 150; // Tempo até atingir a força máxima
       if (obterTensaoDeTrabalho()> 12.3){
          tempoVariavel=0;
          tempoAceleracaoMaxima = 1000;
          forcaArrancadaFinal=40;
       }
       tempoDecorrido = acelerarMotor(tempoAceleracaoMaxima,255,reversoes); // 1000 ou 600
       
    } 
    tempoDecorrido = tempoDecorrido - tempoVariavel + 1700; // dininuir o tempo do manter
    // Manter velocidade máxima
    //tempoDecorrido = 2700 (1250)
    tempoDecorrido = manterVelocidadeAtual(tempoAteFimMovimento, reversoes,tempoDecorrido);
    // Desacelerar
   // tempoDecorrido = 3250 ;
    tempoDecorrido = desacelerarMotor(tempoAteFimMovimento, tempoDecorrido,forcaLimite);
    //tempoAteFimMovimento = 5250
    //tempoDecorrido = 5250 
   
    // ARRAQNEU FINAL
    acelerarMotor(tempoAteFimMovimento,forcaArrancadaFinal,reversoes); //Acelera 1500 para velocidade 40
    tempoDecorrido = tempoDecorrido - 3000; // Tira 3s para manter 1s com velocidade de 40
    manterVelocidadeAtual(tempoAteFimMovimento, reversoes,tempoDecorrido);

    
}                                      

void fecharTailgate(  float tempoAteFimMovimento, int forcaLimite,int reversoes) {

    Serial.print("FECHANDO por: ");
     // Aceleração
    long tempoDecorrido =0;
    tempoDecorrido = acelerarMotor(tempoAteFimMovimento,forcaLimite,reversoes);
    // Manter velocidade máxima
    tempoDecorrido = manterVelocidadeAtual(tempoAteFimMovimento, reversoes,tempoDecorrido);
    // Desaceleração
    desacelerarMotor(tempoAteFimMovimento, tempoDecorrido,forcaLimite);

}


long acelerarMotor(float tempoAteFimMovimento, int forcaLimite, int reversoes) {
   long tempoAceleracao = min(TEMPO_ACELERACAO_PADRAO,tempoAteFimMovimento);
   long tempoDecorrido = 0;
    beep();
    while ((tempoDecorrido <= tempoAceleracao) && motorEmMovimento) {
      ajustarVelocidade(tempoDecorrido, tempoAceleracao,forcaLimite);
      
      if (verificarSinalExterno(tempoAteFimMovimento-tempoDecorrido))
             pararMotor();  // Interrompe se sinal externo for detectado
      
      delay(frequenciaVelocidade);
      tempoDecorrido += frequenciaVelocidade;
    }
    noBeep();
    Serial.println("");
    Serial.print("ACELEROU por:");
    Serial.println(tempoDecorrido);
    return tempoDecorrido;  // Retorna o tempo total decorrido durante a aceleração
}

float manterVelocidadeAtual(  float tempoAteFimMovimento,  int reversoes,float tempoDecorrido) {
    //tempoAteFimMovimento 5250 - 2700 - 2000
    //tempoManter = 500
    if (tempoDecorrido < 0 ) tempoDecorrido = 0;  // Garante que o tempo não seja negativo
    float tempoManter = tempoAteFimMovimento - tempoDecorrido - TEMPO_DESACELERACAO_PADRAO;  // Cálculo do tempo restante
    if (tempoManter < 0 ) tempoManter = 0;  // Garante que o tempo não seja negativo
   
    Serial.println("");
    Serial.print("MANTER por:");
    Serial.print(tempoManter);  
    Serial.print("DECORRIDO :");  
    Serial.print(tempoDecorrido);  
    Serial.print("ATE O FIM :");  
    Serial.print(tempoAteFimMovimento);  

   while ((tempoManter > 0) && motorEmMovimento) {
      if (verificarSinalExterno(tempoAteFimMovimento-tempoDecorrido) 
         ||  verificarObstaculo(tempoAteFimMovimento-tempoDecorrido,reversoes))
         pararMotor();  // Interrompe se sinal externo for detectado
        delay(frequenciaVelocidade);
        tempoManter -= frequenciaVelocidade;  // Reduz o tempo restante
        tempoDecorrido +=frequenciaVelocidade;  // Atualiza o tempo decorrido
       if (tempoDecorrido > 4000 && isFechado())
            return tempoDecorrido;
  
    }
    return tempoDecorrido;  // Retorna o tempo total decorrido após manter a velocidade máxima
}

long desacelerarMotor(  float tempoAteFimMovimento,   float tempoDecorrido, int forcaLimite) {
    //tempoAteFimMovimento = 5250 SUBIDA
    //tempoDecorrido = 3250
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
        if (verificarSinalExterno(tempoAteFimMovimento-tempoDecorrido)) 
              pararMotor();  // Interrompe se sinal externo for detectado
        delay(frequenciaVelocidade);
        tempoDecorrido += frequenciaVelocidade;  // Atualiza o tempo decorrido
        if (tempoDecorrido > 4000 && isFechado())
          return;
    }
    return tempoDecorrido ;
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

  if (lerEventoExterno(pinAcaoExterna12V)|| lerEventoExterno(pinBotao) ){ 
    reverterMovimento(tempoRestante);
    return true;
  }
  return false;
  
}
bool verificarObstaculo(  float tempoRestante, int reversoes) {
  // Cada movimento abrindo ou fechando terá seu limite máximo de correte
  // reversoes conta as reversões por obstáculo
  float limite = abrir ? limiteCorrenteAbertura: limiteCorrenteFechamento;
    if (lerCorrente(pinCorrenteEsquerdo,pinCorrenteDireito) > limite){  // Verifica se a corente é maior q o limite
       
      reverterMovimento(tempoRestante,reversoes);
      return true;
    }
  return false;
  
}


void beep(){

  digitalWrite(pinBeep, HIGH);
  
}

void noBeep(){

   digitalWrite(pinBeep, LOW);

}

float lerCorrente(int pinCorrenteD,int pinCorrenteE){

  float tensao = 0;
  float tensaoA = leituraMediaAnologica(pinCorrenteD) * (5 /1023.0); // 5V máxima voltagem do pino que corresponde a 1023.0
  float tensaoB = leituraMediaAnologica(pinCorrenteE) * (5 /1023.0); // 5V máxima voltagem do pino que corresponde a 1023.0
  
  if ((millis() - tempoCorrenteAnterior) >= intervaloBeep){
     tensao = max(tensaoA,tensaoB);
     tempoCorrenteAnterior = millis();
  }
  return tensao / sensibilidade;

}

void pararMotor() {
  Serial.println("");
  Serial.println("PAROU");
  noBeep();
  ajustarVelocidade(0,0,0);
  motorEmMovimento = false;
  delay(intervaloBeep);
}

void entrarModoSleep() {

  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_twi_disable();
  power_usart0_disable();
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();  // Coloca o Arduino em modo sleep até interrupção
}

void acaoExterna12V() {
  sleep_disable();  // Desabilita o sleep quando acorda
  power_all_enable();
}
float obterTensaoDeTrabalho(){
  
  
  float valorAnalogico = leituraMediaAnologica(analogRead(pinTensao));
  return (valorAnalogico *25)/1024;
 
 }


float contrololarTempoArranqueInicial(){

   float tensaoTrabalho =  obterTensaoDeTrabalho();

   return  tensaoTrabalho >= 12.2 ? 750 : 500;
  
}

 
void calibrarSentidoMotor(bool abrindo){

    digitalWrite(L_PWM, !abrindo);
    digitalWrite(R_PWM, abrindo);
    
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

bool isFechado(){

  return digitalRead(pinSensorMala) == HIGH;

}

bool isAberto(){

  return digitalRead(pinSensorMala) == LOW;

}

void reiniciar(){
   pararMotor();
   asm volatile ("   jmp 0");
}

float leituraMediaAnologica(int pinAnalogico){

  long soma=0;
   for(int i=1;i<=10;i++){
       soma = soma + analogRead(pinAnalogico);
    }
  return soma/10;

}
