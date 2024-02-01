#ifndef camp_pot_lib.h
#define camp_pot_lib.h



int limitaMT(int MT)
{
  if (MT>255)
  {
    MT = 255;
  }
  if(MT < 0)
  {
    MT = 0;
  }
  return MT;
}

float converteVelocidade(int encoderRead, float tempo, float raio) //Converte a leitura do robô em velocidade (m/s): (encoderRead=Leitura do encoder, tempo = tempo entre as leituras (s), raio = raio das rodas (m))
{
  return (encoderRead*2*radians(180)*raio/900/tempo);
}

float ajustaAngulo(float a) //Ajusta o ângulo para que esteja entre 0 e 360º: (ângulo a ser ajustado)
{
  a = fmod(a, 2*radians(180));
  while(a > radians(180))
  {
    a = a - 2*radians(180);
  }
  return a;
}

void calculaPosicao(float VL, float VR, float eixo, float *P, float dt) //Calcula a posição atual do robô: (Velocidade da roda esquerda, Velocidade da roda direita, comprimento do eixo do robô, ponteiro para o vetor de posição e orientação do robô, período de amostragem)
{
  float vLinear = (VL + VR)*0.5; //converte as velocidades individuais de cada roda em velocidade linear
  float vAngular = (VR - VL)/eixo; // converte as velocidades individuais de cada roda em velocidade angular


  P[0] += vLinear*cos(P[3])*dt; //calcula posição em X
  P[1] += vLinear*sin(P[3])*dt; //calcula posição em Y
  P[3] += vAngular*dt;          //Define nova orientação
  P[3] = ajustaAngulo(P[3]);
}


void somaForcas(float FAtr[], float FRep[][2], float nObs, float* FTot) //Realiza o somatório de todas as forças do campo potencial sobre o robô
{
  FTot[0] = FAtr[0];  //A força atrativa deve estar sempre presente
  FTot[1] = FAtr[1];
  for(int i = 0; i<nObs; i++)
  {
    FTot[0] += FRep[i][0]; //Averá força repulsiva apenas próximo dos obstáculos
    FTot[1] += FRep[i][1];
  }
}

float controlePosicao(float P[], float Px, float Py, float Obst[][2], int nObs, float e0, float krho, float kRep, float limitV, float limitW, float dt, float *vRefE, float *vRefD, float eixo)
{
  //Cálculo do erro:
  float dx = Px - P[0];
  float dy = Py - P[1];
  float rho  = sqrt(dx*dx + dy*dy);
  //Ângulo entre robô e ponto objetivo:
  float gam = ajustaAngulo(atan2(dy,dx));

  
  float FAtr[2];        //Vetor utilizado para armazenar a força atrativa das direções x e y (FAtr[0] e FAtr[1] respectivamente)
  float FRep[2][2] = {};     //Vetor utilizado para armazenar a força repulsivas das direções x e y (FRep[i][0] e FRep[i][0] respectivamente para cada i obstáculo)
  float FTot[2];        //Vetor utilizado para armazenar a força resultante das direções x e y (FTot[0] e FTot[1] respectivamente)
  float distObs;        //Distância entre o robô e o obstáculo
  float vLin, wRef;     //Velocidades linear e angular de referência
  //alpha = ajustaAngulo(gam - w);

  //vLin = min(krho*rho, limitV);

  /*if(abs(alpha) > pi/2)
  {
    vLin = -vLin;
    alpha = ajustaAngulo(alpha+pi);
  }*/

  
  FAtr[0] = krho*rho*cos(gam);  //Cálculos das forças atrativas na direção X
  FAtr[1] = krho*rho*sin(gam);  //Cálculos das forças atrativas na direção Y


  //Cálcula as forças repulsivas pra cada obstáculo
  for (int i = 0; i < nObs; i++)
  {
    FRep[i][0] = 0; //Se estiver distante do obstáculo, a força repulsiva será 0
    FRep[i][1] = 0;
    distObs = sqrt(pow(P[0]-Obst[i][0],2)+pow(P[1]-Obst[i][1],2)); //Determina a distância entre robô e obstáculo
    if(distObs <= e0 && nObs>0) //Cálcula as forças repulsivas se estiver próximo do obstáculo
    {
      FRep[i][0] = (kRep/pow(distObs,3))*(1/e0-1/distObs)*(Obst[i][0]-P[0]);
      FRep[i][1] = (kRep/pow(distObs,3))*(1/e0-1/distObs)*(Obst[i][1]-P[1]);
    }
  }

  Serial.println(FAtr[0]);
  Serial.println(FAtr[1]);
  Serial.println(FRep[0][0]);
  Serial.println(FRep[0][1]);

  somaForcas(FAtr, FRep, nObs, FTot); //Soma todas as forças

  vLin = min(sqrt(FTot[0]*FTot[0]+FTot[1]*FTot[1])/dt,limitV); //Determina a velocidade linear de referência

  wRef = atan2(FTot[1],FTot[0]);  //Determina a velocidade angular de referência
  wRef = ajustaAngulo(wRef - P[3]);


  if(wRef != 0)
  {
    wRef = (wRef/abs(wRef))*min(abs(wRef),limitW); //limita a velocidade angular ao ser máximo
  }
  wRef = ajustaAngulo(wRef);
  
  *vRefD = (2*vLin + wRef*eixo)/(2); //Converte as velocidades de referência angular e linear em velocidade da roda direita
  *vRefE = (2*vLin - wRef*eixo)/(2); //Converte as velocidades de referência angular e linear em velocidade da roda esquerda


  //verificaSentido();

  return rho;
}

void controleVelocidade(float VelLeft, float VRefLeft, float VelRight, float VRefRight, float kie, float kpe, float kid, float kpd,  float *erro, int *MTL, int *MTR)
{
  float erroL = VRefLeft - VelLeft;   //Determina erro da roda esquerda
  float erroR = VRefRight - VelRight; //Determina erro da roda direita
  erro[0] += erroL; //acumula erro da roda esquerda
  erro[1] += erroR; //acumula erro da roda direita

  *MTL = limitaMT(*MTL + int(kpe*erroL + kie*erro[0])); //Controle PI da roda esquerda
  *MTR = limitaMT(*MTR + int(kpd*erroR + kid*erro[1])); //Controle PI da roda direita
  
  
  

}

#endif