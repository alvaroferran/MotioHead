#include "BNO055.h"
#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial bt(4, 3); // RX, TX

#define A 0X28  //I2C address selection pin LOW
#define B 0x29  //                          HIGH
BNO055 sensor(A);



void setup(){
    Wire.begin();
    Serial.begin(19200);
    bt.begin(9600);
    // bt.print("AT+NAMEMotioGlove\r\n");
    sensor.init();
}



double map(double vx, double v1, double v2, double n1, double n2){
    // v1 start of range, v2 end of range, vx the starting number between the range
    double percentage = (vx-v1)/(v2-v1);
    // n1 start of new range, n2 end of new range
    return (n2-n1)*percentage+n1;
}



int checkEnable(double uD, int dir){
    int uDMin=30, uDMax=70;
    static int en, enOld=0, lookedUp=0, lookedUpOld=0;
    //If stopped and looked up
    if(dir==0 && uD>uDMin && uD<uDMax) lookedUp=1;
    else lookedUp=0;
    //If was disabled and looking up flank detected, enable
    if (enOld==0 && lookedUpOld==0 && lookedUp==1)      en=1;
    else if (enOld==1 && lookedUpOld==0 && lookedUp==1) en=0;
    enOld=en;
    lookedUpOld=lookedUp;
    return en;
}



double normalizeUpDown(double uD){
    int uDMin=80, uDMax=100;
    uD=map((uD-uDMin)/(uDMax-uDMin),0,1,1,0);
    if(uD>1 && uD<2)    uD=1; //above threshold
    if(uD<0 && uD>-2)   uD=0; //below threshold
    if(uD<=-2 || uD >=2)uD=0; //speed 0 in non-defined positions
    return uD;
}



int selectDirection(double tw){
    int twThresh=10;
    static int dirOld=0, twOld=0,dir=0;
    // Discretize twist
    if (tw<-twThresh)     tw=-1; //Left
    else if (tw>twThresh) tw=1;  //Right
    else tw=0;
    // Flank detector and direction selection
    if (twOld==0 && tw==-1) {                  //If turned left and before were
        if(dirOld==0)  dir=1;                       //stopped, now move forwards
        else if(dirOld==-1 || dirOld==1) dir=0;    //moving, now stop
    }
    if (twOld==0 && tw==1) {              //If turned right and before were
        if(dirOld==0)  dir=-1;                     //stopped, now move backwards
        else if(dirOld==-1 || dirOld==1) dir=0;    //moving, now stop
    }
    twOld=tw;
    dirOld=dir;
    return dir;
}



double setSpeed(int dir, double uD){
    double speed, speedFwd=1, speedBwd=-0.5;
    if (dir==0) speed=0;
    if (dir==-1)speed=speedBwd;
    if (dir==1) speed=speedFwd*uD;
    return speed;
}



void loop(){
    static double upDown, leftRight,twist;
    static int direction=0;
    double speed;
    int enable;

    sensor.readEul();   //Read angles
    upDown=sensor.euler.z;
    leftRight=sensor.euler.x;
    twist=sensor.euler.y;

    enable=checkEnable(upDown, direction);

    if(enable==1){
        upDown=normalizeUpDown(upDown);
        direction=selectDirection(twist);
        speed=setSpeed(direction, upDown);

        Serial.println("Speed:  " + String(speed));
        bt.println("Speed:  " + String(speed));
    }
    delay(100);
}
