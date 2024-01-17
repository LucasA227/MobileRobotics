#include <PololuMagneticEncoder.h>
#include "camp_pot_lib.h"
//Pinos da ponte H
#define PWMA 33
#define AIN2 25
#define AIN1 26
#define STBY 27
#define BIN1 14
#define BIN2 12
#define PWMB 13

#define pontos  2

struct Robo{

  //Características estruturais
  float eixo; //Define a distância entre os centros das rodas do robô 
  float raio; //Define o raio das rodas do robô
  float diametro; //Tamanho do robô

  //Posição direção atual do robo
  float Posicao[3]; //{PX,PY,W}: PX = Posição em x, PY = Posição em Y e W = Orientação

  //Velocidades calculadas atuais
  float Vlinear;
  float Vangular;

  //Velocidades medidas de cada roda
  float Vesquerda;
  float Vdireita;

  
};
//Características construtivas do robô
const float pi = radians(180);



//Parâmetros de controle
const float kp = 100;   //Ganho proporcional do controlador
const float ki = 1;     //Ganho integral do controlador
float erro[2] = {0,0};  //Vetor destinado a soma de erros das rodas esquerda (erro[0]) e direta (erro[1])
float dt = 0.1;         //Período de amostragem
int MTL = 0;            //Variável de controle da roda esquerda
int MTR = 0;            //Variável de controle da roda direita



//Inicialização do encoder
PololuMagneticEncoder encoders;

//variáveis controle de posição
float rho = 0;              //Erro de distância entre o robô e ponto objetivo
const float limitV = 0.1;   //limite de velocidade linear do robô
const float limitW = pi/2;  //limite de velocidade angular do robô

//Pontos objetivos
float Px[pontos] = {1, 1};  //coordenadas X  
float Py[pontos] = {1, 0};   //coordenadas y
float dx; // distancia entre robô e objetivo em X
float dy; // distancia entre robô e objetivo em y

int p = 0; // Variável auxiliar que determina o ponto objetivo que direcionará o robô

float vRefD = 0;    //Inicialização da Velocidade de referência da roda Direita
float vRefE = 0;    //Inicialização da Velocidade de referência da roda Esquerda
float prec = 0.025; //Determina a distância aceitável dos pontos objetivos


//Campos potenciais

//float FAtr[2] = {0,0};                     //Vetor utilizado para armazenar a força atrativa das direções x e y (FAtr[0] e FAtr[1] respectivamente)
//float FRep[2][2] = {{0,0},{0,0}};          //Vetor utilizado para armazenar a força repulsivas das direções x e y (FRep[i][0] e FRep[i][0] respectivamente para cada i obstáculo)
int nObs = 1;                              //Variável índice que indica a quantidade de obstáculos determinados
//float FTot[2] = {0,0};                     //Vetor utilizado para armazenar a força resultante das direções x e y (FTot[0] e FTot[1] respectivamente)
const float e0 = 0.15;                     //Determina a distância máxima de atuação dos campos potenciais (m)
float Obst[2][2] = {{0.5,0.5},{0,0}};    //Armazena as coordenadas dos obstáculos
const float katr = 10;                     //Constante atrativa do campo potencial
float kRep = 3;                            //Constante repulsiva do campo potencial


struct Robo robo1;  //Definição do robô



void setup() {

  //Inicialização dos pinos da Ponte H
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(STBY,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);

  //Determinação inicial dos sentidos de rotação das rodas
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,LOW);//esquerda
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,LOW);//direita
  digitalWrite(BIN2,HIGH);  

  //configuração do canal PWM de cada roda
  ledcAttachPin(PWMA, 1); 
  ledcSetup(0, 5000, 8); 

  ledcAttachPin(PWMB, 0); 
  ledcSetup(1, 5000, 8);

  //Inicialização dos encoders
  encoders.setupEncoders(35, 32, 34, 39);

  //Inicialização da comunicação Serial
  Serial.begin(9600);




  //Inicializando características do robô
  robo1.diametro = 0.13;  //(m)
  robo1.eixo = 0.09;      //(m)
  robo1.raio = 0.021;     //(m)
  robo1.Posicao[0] = 0;   //(m)
  robo1.Posicao[1] = 0;   //(m)
  robo1.Posicao[2] = 0;   //(m)
  delay(5000);
}

void loop() {

  //Cálculo inicial do erro de distância entre robô e ponto objetivo
  dx = Px[p] - robo1.Posicao[0];  //erro em x
  dy = Py[p] - robo1.Posicao[1];  //erro em y
  rho  = sqrt(dx*dx + dy*dy);

  while(rho > prec)
  { 
    /*Serial.print("MTL = ");
    Serial.print(MTL);
    Serial.print("; MTR = ");
    Serial.println(MTR);*/
    //Comunicação com matlab:
    Serial.println(robo1.Posicao[0]);
    Serial.println(robo1.Posicao[1]);

    //Determinação das velocidades individuais de cada roda (m):
    robo1.Vesquerda = converteVelocidade(encoders.getCountsAndResetEncoderLeft() , dt, robo1.raio);
    robo1.Vdireita = converteVelocidade(encoders.getCountsAndResetEncoderRight(), dt, robo1.raio);

    //Cálcula a posição atual do robô:
    calculaPosicao(robo1.Vesquerda, robo1.Vdireita, robo1.eixo, robo1.Posicao, dt);

    //Realiza o controle de posição por campos potenciais:
    rho = controlePosicao(robo1.Posicao, Px[p], Py[p], Obst, nObs, e0, katr, kRep, limitV, limitW, dt, &vRefE, &vRefD, robo1.eixo);

    //Realiza o control de velocidade:
    controleVelocidade(abs(robo1.Vesquerda), vRefE, abs(robo1.Vdireita), vRefD, ki, kp, erro, &MTL, &MTR);
    ledcWrite(0, abs(MTL));  //Atualiza o PWM de controle da roda esquerda
    ledcWrite(1, abs(MTR));  //Atualiza o PWM de controle da roda direita
    delay(int(1000*dt));
  }
  if (p < pontos-1)
  {
    p+=1;
  }
  ledcWrite(0, 0);  //esquerdo
  ledcWrite(1, 0);  //direito
  delay(2000);

  
}
