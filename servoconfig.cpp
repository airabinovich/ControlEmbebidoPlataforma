#include <Servo.h>
#include "servoconfig.h"
void  writeToServos(float *motorAng, platformServos *servos){
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


