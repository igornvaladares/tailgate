#include <avr/sleep.h>
#include <avr/interrupt.h>

volatile int clicks;
boolean direction_m;

const int botao = 2;  // Pino de interrupção para wake-up
const int sensorMala = 5;  // Pino de leitura se mala aberta ou fechada

// Pinos de controle da ponte H
const int R_PWM = 7;  // Pino PWM de controle da direção 1
const int L_PWM = 8;  // Pino PWM de controle da direção 2

const int SAIDAPULSO = 10;  // Pino saida pulsada
const int PWM = 11;  // Ligar os pinos R_ENA e L_ENA no D11 do arduino
const int _5V = 12;  // Pino 5V lógico
const int GND = 13;  // Pino GND lógico



// Pino do botão de controle externo

// Pino do sensor corrente
const int pinCorrenteEsquerdo = A1;  // Pino corrente esquerda
const int pinCorrenteDireito = A0;  // Pino corrente direita
const int pinTensao = A2;  // Pino tensao trabalho

const float sensibilidade = 0.1; // Sensibilidade (Quantidade de tensãp gerada por cada unidade de corrente- Cada Ampere de corrente, o sensor gerará 0.1V na saida)
//const float correnteMaximaMotor =20;
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
const float intervaloPulso =500; // 1S
long tempoPulsoAnterior=0;
int clicksAnt=0;

#define DESVIO_FREQUENCIA 50; // desvio para calcular a queda da valocidade 10%
float menorVariacao =1000;


void setup() {
  Serial.begin(9600);  // Para debug (opcional)
   Serial.print("Tensão :");
   Serial.println(obterTensaoDeTrabalho());
   // Configurar pinos como saída
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(SAIDAPULSO, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(_5V, OUTPUT);
  pinMode(GND, OUTPUT);
  
 
  digitalWrite(_5V, HIGH);
  digitalWrite(GND, LOW);

  beep();
  delay(500);
  noBeep();
  // Configurar o pino do botão como entrada com PULLUP interno
  pinMode(botao, INPUT);
  pinMode(sensorMala, INPUT_PULLUP);
 // Configurar interrupção para o botão (borda de descida)
  attachInterrupt(digitalPinToInterrupt(botao), acordar, RISING);

  tempoPulsoAnterior = millis();
}


void iniciarMovimento(  float tempoAteFimMovimento =TEMPO_MAXIMO_MOVIMENTACAO, int forcaLimite=FORCA_LIMITE_REFERENTE, int reversoes=0) {
  delay(250); //  debouce 
  motorEmMovimento = true;
  tempoCorrenteAnterior = millis(); // Evitar leitura inicial
  attachInterrupt(digitalPinToInterrupt(encoder_C1), clickBotao, FALLING);   //Interrupção externa 3 por mudança de estado
  attachInterrupt(digitalPinToInterrupt(botao), acordar, RISING);
  calibrarSentidoMotor(abrir);  
  if (abrir) {
    abrirTailgate(tempoAteFimMovimento/2.0,forcaLimite,reversoes);// QUando diminui o tempoAteFimMovimento siginifica diminuir o tempo na força maxima
  } else {
    fecharTailgate(tempoAteFimMovimento,forcaLimite,reversoes);
  }

}
void loop() {
  // Verificar se o botão foi pressionado
  if (lerEventoExterno(botao)){
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
    Serial.print("ABRINDO por: ");
    long tempoDecorrido=0;
    if (reversoes==0){
       long tempoAteFim = contrololarTempoArranqueInicial();       
       tempoDecorrido = acelerarMotor(tempoAteFim,255,reversoes);
    } 
    tempoDecorrido = tempoDecorrido + 1700; // dininuir o tempo do manter
    // Manter velocidade máxima
    //tempoDecorrido = 2850 (2500)
    tempoDecorrido = manterVelocidadeAtual(tempoAteFimMovimento, reversoes,tempoDecorrido);
    // Desacelerar
   // tempoDecorrido = 3250 ;
    tempoDecorrido = desacelerarMotor(tempoAteFimMovimento, tempoDecorrido,forcaLimite);
    //tempoAteFimMovimento = 5250
    //tempoDecorrido = 5250 
   
    // ARRAQNEU FINAL
    acelerarMotor(tempoAteFimMovimento,40,reversoes); //Acelera 1500 para velocidade 40
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
    //tempoAteFimMovimento 5250 - 2850 - 2000
    //tempoManter = 400
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

  if (lerEventoExterno(botao)){ 
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

  digitalWrite(SAIDAPULSO, HIGH);
  
}

void noBeep(){

   digitalWrite(SAIDAPULSO, LOW);

}

float lerCorrente(int pinCorrenteD,int pinCorrenteE){

  float tensao = 0;
  float tensaoA = leituraMediaAnologica(pinCorrenteD) * (5 /1023.0); // 5V máxima voltagem do pino que corresponde a 1023.0
  float tensaoB = leituraMediaAnologica(pinCorrenteE) * (5 /1023.0); // 5V máxima voltagem do pino que corresponde a 1023.0
  
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
  ajustarVelocidade(0,0,0);
  motorEmMovimento = false;
  delay(intervaloPulso);
}

void entrarModoSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();  // Coloca o Arduino em modo sleep até interrupção
}

void clickBotao()
{
   clicks++; //incrementa número do pulso se direction limpa
   
} //end clickBotao
void acordar() {
  sleep_disable();  // Desabilita o sleep quando acorda
}
float obterTensaoDeTrabalho(){
  
  
  float valorAnalogico = leituraMediaAnologica(analogRead(pinTensao));
  float tensaoDoPin =  (valorAnalogico /1023.0) * 5;
  return  tensaoDoPin * 5;
 
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
  if (digitalRead(pin_entrada) == HIGH)
  {
    estado = digitalRead(pin_entrada);
    if (estado == HIGH)
    {
      while (estado == HIGH)
        estado = digitalRead(pin_entrada);
      return 1;
    }  
  }
  return 0;
 }

bool isFechado(){

  return digitalRead(sensorMala) == HIGH;

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
void freiar(float tempoAteFimMovimento, int forcaLimite) {
    calibrarSentidoMotor(!abrir);
  
    tempoAteFimMovimento = max(TEMPO_ACELERACAO_PADRAO,tempoAteFimMovimento);

    Serial.print("Freiar  por:");
    Serial.println(tempoAteFimMovimento);
    int tempoDecorrido =0;
    while (tempoDecorrido <= tempoAteFimMovimento && motorEmMovimento) {
        
        ajustarVelocidade(255, 255,forcaLimite);  // Desacelera o motor
        delay(frequenciaVelocidade);
        tempoDecorrido += frequenciaVelocidade;  // Atualiza o tempo decorrido
    }
    pararMotor();
}
