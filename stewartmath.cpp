#include "stewartmath.h"
#include <math.h>

void xyz_yprToServoAng(float x,float y,float z, float yaw, float pitch, float roll,float *motorAngles){
                float terminoA,terminoB,terminoC,terminoD,terminoE,terminoF;
                float terminos[N_SERVOS][3];
                float alpha= yaw*(PI/180),
                      beta= pitch*(PI/180),
                      gamma= roll*(PI/180);    
                terminoA= cos(alpha)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma);
                terminoB= cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma);
                terminoC= sin(alpha)*cos(beta);
                terminoD= cos(alpha)*cos(beta);
                terminoE= sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
                terminoF= sin(alpha)*sin(gamma)+cos(alpha)*sin(beta)*cos(gamma);
                terminos[0][0]= (B2x * terminoA + B2y * terminoB + x - A2x);
                terminos[0][1]= (B2x * terminoC + B2y * terminoD + y - A2y);
                terminos[0][2]= (B2x * terminoE + B2y * terminoF + z);
                terminos[1][0]= (B3x * terminoA + B3y * terminoB + x - A3x);
                terminos[1][1]= (B3x * terminoC + B3y * terminoD + y - A3y);
                terminos[1][2]= (B3x * terminoE + B3y * terminoF + z);
                terminos[2][0]= (B4x * terminoA + B4y * terminoB + x - A4x);
                terminos[2][1]= (B4x * terminoC + B4y * terminoD + y - A4y);
                terminos[2][2]= (B4x * terminoE + B4y * terminoF + z);
                terminos[3][0]= (B5x * terminoA + B5y * terminoB + x - A5x);
                terminos[3][1]= (B5x * terminoC + B5y * terminoD + y - A5y);
                terminos[3][2]= (B5x * terminoE + B5y * terminoF + z);
                terminos[4][0]= (B6x * terminoA + B6y * terminoB + x - A6x);
                terminos[4][1]= (B6x * terminoC + B6y * terminoD + y - A6y);
                terminos[4][2]= (B6x * terminoE + B6y * terminoF + z);
                terminos[5][0]= (B1x * terminoA + B1y * terminoB + x - A1x);
                terminos[5][1]= (B1x * terminoC + B1y * terminoD + y - A1y);
                terminos[5][2]= (B1x * terminoE + B1y * terminoF + z);
                
                float feetLengths[N_SERVOS];             
                for (int k=0;k<N_SERVOS;k++){
                  feetLengths[N_SERVOS-k-1] = sqrt ( pow(terminos[k][0],2) + pow(terminos[k][1],2) + pow(terminos[k][2],2) );           
                }
                
                for (int k=0;k<N_SERVOS;k++){
                  motorAngles[k] = 90- (acos((B*B + feetLengths[k]*feetLengths[k] - C*C) / (2*B*feetLengths[k])))* (180/PI);
                  if(isnan(motorAngles[k])){motorAngles[k]=90;}
                }
}

