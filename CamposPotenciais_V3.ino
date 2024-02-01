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

#define pontos  1

#define ECHO 22
#define TRIGGER 23

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
float dt = 0.1;  
const float kpe = 142.911459665792;   //Ganho proporcional do controlador
const float kie = 1186.71898724183*dt;     //Ganho integral do controlador
const float kpd = 166.645143888503;   //Ganho proporcional do controlador
const float kid = 1050.88238282574*dt;     //Ganho integral do controlador
float erro[2] = {0,0};  //Vetor destinado a soma de erros das rodas esquerda (erro[0]) e direta (erro[1])
int MTL = 0;            //Variável de controle da roda esquerda
int MTR = 0;            //Variável de controle da roda direita
int aux;


//Inicialização do encoder
PololuMagneticEncoder encoders;

//variáveis controle de posição
float rho = 0;              //Erro de distância entre o robô e ponto objetivo
const float limitV = 0.1;   //limite de velocidade linear do robô
const float limitW = pi/2;  //limite de velocidade angular do robô

//Pontos objetivos
float Px[pontos] = {1};  //coordenadas X  
float Py[pontos] = {0};   //coordenadas y
float dx; // distancia entre robô e objetivo em X
float dy; // distancia entre robô e objetivo em y

int p = 0; // Variável auxiliar que determina o ponto objetivo que direcionará o robô

float vRefD = 0;    //Inicialização da Velocidade de referência da roda Direita
float vRefE = 0;    //Inicialização da Velocidade de referência da roda Esquerda
float prec = 0.025; //Determina a distância aceitável dos pontos objetivos


//Campos potenciais

int nObs = 0;                              //Variável índice que indica a quantidade de obstáculos determinados
//float FTot[2] = {0,0};                     //Vetor utilizado para armazenar a força resultante das direções x e y (FTot[0] e FTot[1] respectivamente)
const float e0 = 0.25;                     //Determina a distância máxima de atuação dos campos potenciais (m)
float Obst[2][2] = {{0,0},{0,0}};    //Armazena as coordenadas dos obstáculos
const float katr = 1;                     //Constante atrativa do campo potencial
float kRep = 0.05;                            //Constante repulsiva do campo potencial


float distSensor;
int cont = 0;

struct Robo robo1;  //Definição do robô

float getDistancia(void)
{
    digitalWrite(TRIGGER,HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER,LOW);
    float duration = float(pulseIn(ECHO, HIGH));
    return duration*0.017/100;
}

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

  pinMode(ECHO,INPUT);
  pinMode(TRIGGER,OUTPUT);
  digitalWrite(TRIGGER,LOW);


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
    aux = millis(); //Atualiza aux com o tempo atual para a próxima iteração do loop
     //loop de espera que garante que o código aguarde um tempo específico (dt) antes de continuar

    

    //Comunicação com matlab:
    Serial.println(robo1.Posicao[0]);
    Serial.println(robo1.Posicao[1]);


    //Determinação das velocidades individuais de cada roda (m):
    robo1.Vesquerda = converteVelocidade(encoders.getCountsAndResetEncoderLeft() , dt, robo1.raio);
    robo1.Vdireita = converteVelocidade(encoders.getCountsAndResetEncoderRight(), dt, robo1.raio);

    //Cálcula a posição atual do robô:
    calculaPosicao(robo1.Vesquerda, robo1.Vdireita, robo1.eixo, robo1.Posicao, dt);

    //Realiza o controle de posição por campos potenciais:

    distSensor = getDistancia();
    if(distSensor < 0.40 && cont == 0)
    { 
      Obst[nObs][0] = robo1.Posicao[0]+distSensor*cos(robo1.Posicao[2]);
      Obst[nObs][1] = robo1.Posicao[1]+distSensor*sin(robo1.Posicao[2]);
      nObs++;
      cont++;
      Serial.println("b");
      Serial.println(Obst[0][0]);
      Serial.println(Obst[0][1]);
    }
    else
    {
      Serial.println("a");
    }

    rho = controlePosicao(robo1.Posicao, Px[p], Py[p], Obst, nObs, e0, katr, kRep, limitV, limitW, dt, &vRefE, &vRefD, robo1.eixo);

    
    //Realiza o control de velocidade:
    controleVelocidade(abs(robo1.Vesquerda), vRefE, abs(robo1.Vdireita), vRefD, kie, kpe, kid, kpd, erro, &MTL, &MTR);
    ledcWrite(0, abs(MTL));  //Atualiza o PWM de controle da roda esquerda
    ledcWrite(1, abs(MTR));  //Atualiza o PWM de controle da roda direita
    //delay(int(1000*dt));
    while (millis() - aux < dt * 1000) {}
  }
  if (p < pontos-1)
  {
    p+=1;
  }
  ledcWrite(0, 0);  //esquerdo
  ledcWrite(1, 0);  //direito
  delay(2000);

  
}
