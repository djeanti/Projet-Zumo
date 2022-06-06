/*This demo requires a Zumo 32U4 Front Sensor Array to be
connected, and jumpers on the front sensor array must be
installed in order to connect pin 4 to DN4 and pin 20 to DN2. */

#include <Wire.h>
#include <Zumo32U4.h>
#include "gyro_use.h"


// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const uint16_t maxSpeed = 400;

//Zumo32U4Buzzer //buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonB buttonB;
Zumo32U4Encoders encoders;

Zumo32U4IMU imu;

int16_t lastError = 0;

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

void turncalibrate(short nbangle45){
  // sens trigo   1 sens trigo  0 inverse
    const uint16_t calibrationSpeed = 160;
   motors.setSpeeds(calibrationSpeed*(nbangle45>0 ? -1 :1) ,calibrationSpeed *(nbangle45>0 ? 1 :-1));
  turnSensorReset();
  while((nbangle45>=0 ? (int32_t)turnAngle < (turnAngle45 *nbangle45 ) : (int32_t)turnAngle >= (turnAngle45 *nbangle45 ))){
                  lineSensors.calibrate();
                  turnSensorUpdate();
  }
  motors.setSpeeds(0, 0);
  turnSensorUpdate();
}

void calibrateSensors2()
{   char tab[20];
  const uint16_t calibrationSpeed = 160;
  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(100);
  // Turn to the left 90 degrees.
  lineSensors.calibrate();
  turncalibrate(2);
  turncalibrate(-4);
  turncalibrate(2);
}

volatile char vgo,vstop,vcont,id,vcalline;
//*****************************************************
// STEUP initialisations
//****************************************************
void setup()
{
  Serial1.begin(38400);
  
  Wire.begin();
  while(vgo==0){
    serialEvent1();
  }
//  buttonB.waitForButton();
  delay(800);
  Serial1.println("Bonjour");
//  //buzzer.play("L16 cdegreg4");
  lineSensors.initFiveSensors();
  if(!imu.init()){
        ledRed(1);
        while(1);
  }
  imu.enableDefault();
  imu.configureForTurnSensing();
  
  turnSensorSetup();
  // pas de calibrage capteur sol pour le moment
  if(vcalline==1){
        calibrateSensors2(); 
  }

  // Play music and wait for it to finish before we start driving.
  //buzzer.play("L16 cdegreg4");
  //while(buzzer.isPlaying());
 // encoders.init();
 encoders.getCountsAndResetRight();
 encoders.getCountsAndResetLeft();
}

char etat;
unsigned char sens_stat;
int positionl;

void PID(){
  // Our "error" is how far we are away from the center of the
  // line, which corresponds to position 2000 done il loop function.
  int16_t error = positionl;
  int16_t speedDifference = error / 4 + 6 * (error - lastError);
  lastError = error;

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

  // Constrain our motor speeds to be between 0 and maxSpeed.
  // One motor will always be turning at maxSpeed, and the other
  // will be at maxSpeed-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you
  // might want to allow the motor speed to go negative so that
  // it can spin in reverse.
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);

  motors.setSpeeds(leftSpeed, rightSpeed);
}



unsigned char  Analyse(int *p){
  unsigned char i;
  unsigned char ret=0;
    for(i=0;i<5;i++){
         ret=ret<<1;
         ret=ret+(p[i]>(lineSensors.calibratedMinimumOn[i]*2+lineSensors.calibratedMaximumOn[i])/3 ? 1:0);
    } 
    return ret;      
}

struct trajzumo {
      long gpos;
      long dpos;
      unsigned int vitesse; };

const struct trajzumo wpoint[10] PROGMEM={{300,300,150},{-300,-300,150},{0,0,0}};
unsigned char indw=0;
unsigned char etaw=0;
long distdroit,distgauche,mdist;
short leftSpeedw= 150, rightSpeedw=-150;
#define ROT90  633
#define AVANCER_PEU  120
#define ROT_PEU  50

long tb[9][2] ={{2580,2580},{ROT90/2,-ROT90/2},{-ROT90/2,+ROT90/2},{ROT90,-ROT90},{-ROT90,+ROT90},{AVANCER_PEU,AVANCER_PEU},{-AVANCER_PEU,-AVANCER_PEU},{ROT_PEU,-ROT_PEU},{-ROT_PEU,ROT_PEU}};
#define DIST90   633  //((long)604*10600/10000)
char auto_traj(){
    long bias;
    char buff[60];
    long llt=0,lltold=0;
    distdroit = distdroit + encoders.getCountsAndResetRight();
    distgauche = distgauche + encoders.getCountsAndResetLeft();
    serialEvent1();
    switch(etaw){
          case 0 :
                   turnSensorReset();
                   leftSpeedw = 250 *( tb[id][0]>0 ? 1 : -1);
                   rightSpeedw = 250 *( tb[id][1]>0 ? 1 : -1);
                   motors.setSpeeds(leftSpeedw,rightSpeedw);
                   lltold=millis();
                   etaw=1;
                  break;
          case 1 :
                  if((millis()-lltold)>=40){ // période d'affichage 40ms
                        bias = distgauche-distdroit;
                        lltold=millis();
                        sprintf(buff,"%6ld  %6ld   %5ld",distgauche,distdroit,bias);
                        Serial1.println(buff);
                  }
                  turnSensorUpdate();
                 // //mdist=(distdroit+distgauche)/2;
                  if( vstop==1) {
                        etaw=2;
                        vstop=0;
                        motors.setSpeeds(-5, -5); //permet d'aider au freinnage du robot sans qu'il ne recule en arrière
                        lltold=millis();                         
                  }
                  if((abs(distgauche)>=abs(tb[id][0]))&& (abs(distdroit)>=abs(tb[id][1]))){
                        if((tb[id][0]>0) && (tb[id][1]>0)){
                          motors.setSpeeds(-5, -5); //permet d'aider au freinnage du robot sans qu'il ne recule en arrière
                        }
                        else{
                          motors.setSpeeds(5, 5); //permet d'aider au freinnage du robot sans qu'il ne recule en arrière
                        }
                        lltold=millis();
                        etaw=2; // on va vers l'état 2 où le robot est sensé être arrêté
                  }
                  break;      
          case 2 :
                  turnSensorUpdate();
                  if((millis()-lltold)>1000){ // On attends 1 seconde pour etre sur que le robot est bien arrêté
                        motors.setSpeeds(0, 0); // on met à 0 les ordres moteurs
                        sprintf(buff," FIN : %6ld  %6ld   %5ld",distgauche,distdroit,(long)(((float)turnAngle/turnAngle45)*45));
                        Serial1.println(buff); //on affiche la dernière valeur moteur à l'arret.
                        etaw=3;//on ne fait plus rien
                  }
          case 3 :
                  if (vcont==1){
                        vcont=0;
                        distdroit=0;
                        distgauche=0;
                        etaw=0;
                  }
                  break;
                    
         default :
                  break;
    }
     return 0;
}

void serialEvent1() {
  if (Serial1.available()) {
    char inChar = (char)Serial1.read();
    switch(inChar) {
        case 'g' : vgo=1;
                   Serial1.println("GO !");
                  break;
        case 's' : vstop=1;
                  Serial1.println("STOP !");
                  break;
        case 'c' : vcont = 1;
                  Serial1.println("On continu !");
                  break;
        case '0' :id=inChar-'0';
                   vcont=1;
                   Serial1.println("choix segment Ok !");
                   break;
        case '1' :id=inChar-'0';
                   vcont=1;
                   Serial1.println("choix segment Ok !");
                   break;

        case '2' :id=inChar-'0';
                   vcont=1;
                   Serial1.println("choix segment Ok !");
                   break;

        case '3' :id=inChar-'0';
                   vcont=1;
                   Serial1.println("choix segment Ok !");
                   break;

        case '4' : id=inChar-'0';
                   vcont=1;
                   Serial1.println("choix segment Ok !");
                   break;
        case 'F' :id=5;
                   vcont=1;
                   break;
        case 'L' :id=8;
                   vcont=1;
                   break;
        case 'R' :id=7;
                   vcont=1;
                   break;
        case 'B' :id=6;
                   vcont=1;
                   break;
        case 'm' : // prévoir avant de lancer go  si on veut faire un calibrage ligne
                    // lorsqu'on lancera g c'est à dire g il faura un calibrage ligne avant de commencer
                  vcalline=1;
                  Serial1.println("calibrage ligne à faire Ok !");
                  break;
        default :
                break;
    }
  }

}


unsigned long ll,dll,lastll;
char flag10ms;
void autom1(){
    positionl = lineSensors.readLine(lineSensorValues)-2000;
    sens_stat=Analyse(lineSensorValues);
    ll=micros();
    if((ll-lastll)>=10000){
        flag10ms=1;                   
        lastll=ll;     
    }
    switch(etat){
        case 0 :
                if(((sens_stat&0x0E)!=0) && ((sens_stat&0x11)==0)){
                        if(flag10ms==1){
                              PID();
                              flag10ms=0;     
                        }
                }else{
                      if(sens_stat==0){
                        motors.setSpeeds(0, 0);
                        etat=1;
                      }else{
                        if(sens_stat==0x1F){
                          motors.setSpeeds(0, 0);
                          etat=2;
                        }
                      }
                }
                break;
        case 1 : // pas de route
               motors.setSpeeds(-160, 160);
               turnSensorReset();
               etat=2;
               break;
        case 2:// terrain combat
               break;
               if(((int32_t)turnAngle < turnAngle45 * 2) && ((sens_stat&0x0E)==0)) {
                          turnSensorUpdate();
               }else{
                          motors.setSpeeds(0,0);
                          turnSensorUpdate();
                          if((sens_stat&0x0E)!=0) {
                                  etat=0;
                          }else{
                                  etat=4;
                          }
               }
               break;            

       case 4 : // pas de route
               motors.setSpeeds(160, -160);
               turnSensorReset();
               etat=5;
               break;
       case 5 :
               if(((int32_t)turnAngle >= -turnAngle45 * 4) && ((sens_stat&0x0E)==0)) {
                        turnSensorUpdate();
              }else{
                        motors.setSpeeds(0,0);
                        turnSensorUpdate();
                        if((sens_stat&0x0E)!=0) {
                                etat=0;
                        }else{
                                //buzzer.play("L16 cdegreg4");
                                etat=6;
                        }
              }
              break;         
     case 6 : /// je me bloque volontairement ne plus rien faire
            break;                  
     default :
            break;
    }
}


void loop(){
    auto_traj();
}