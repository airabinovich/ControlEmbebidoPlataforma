#include <Servo.h>
#include <Arduino.h>
#include "servoStewart.h"

void servoStewart::writeToServos(float *motorAng){
  int CantServos= sizeof(servos)/sizeof(platformServos);
  for (int k=0;CantServos;k++){
     if(servos[k].quad == SECOND){
       motorAng[k]=180-motorAng[k]; //ajusta el angulo para aquellos motores que operen sobre el segundo cuadrante.
     }   
     /*Limite en los angulos máximos y minimos permitidos para cada motor de acuerdo al cuadrante en el que operan*/    
     if(servos[k].quad == SECOND && (motorAng[k]<=minAngSegundoCuadrante)){motorAng[k]=minAngSegundoCuadrante;}
     else if(servos[k].quad == SECOND && (motorAng[k]>=maxAngSegundoCuadrante)){motorAng[k]=maxAngSegundoCuadrante;}
     else if(servos[k].quad == FIRST && (motorAng[k]<=minAngPrimerCuadrante)){motorAng[k]=minAngPrimerCuadrante;}
     else if(servos[k].quad == FIRST && (motorAng[k]>=maxAngPrimerCuadrante)){motorAng[k]=maxAngPrimerCuadrante;}
     servos[k].servo.write(motorAng[k]); //escritura de angulo el motor.
   }
}

void servoStewart::servosSetupConfigurations(){
   servos[0].quad=SECOND;  
   servos[1].quad=FIRST;  
   servos[2].quad=SECOND;  
   servos[3].quad=FIRST;  
   servos[4].quad=SECOND;  
   servos[5].quad=SECOND;  
}

void servoStewart::attachServos(int initPin){
     for(int j=0;j<6;j++){
       pinMode(j+initPin, OUTPUT); 
       servos[j].servo.attach(j+initPin);
     }
}
