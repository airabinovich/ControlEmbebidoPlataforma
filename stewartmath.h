

#define PI 3.14159265
#define N_SERVOS 6

/*Definición de los puntos (x,y) en el espacio de las junturas 
  de los brazos con cada motor(variables A) y de cada brazo 
  con la plataforma (variables B). Magnitud medida en milimetros. */
#define A1x 8.66
#define A1y 68.21 
#define A2x 45.16 
#define A2y 5 
#define A3x 230.5	
#define A3y 5    
#define A4x 267	
#define	A4y 68.21  
#define A5x 174.33 
#define A5y 233.71
#define A6x 101.33 
#define A6y 233.71
#define B1x -81.5
#define B1y -38.4 
#define B2x -74 
#define B2y -51.3
#define B3x 74
#define B3y -51.3
#define B4x 81.5
#define B4y -38.4 
#define B5x 7.5
#define B5y 89.8
#define B6x -7.5
#define B6y 89.8

/*Definición de la longitud de cada brazo del mecanismo. Magnitud medida en milimetros.*/
#define B 54  //longitud corta de las patas
#define C 246 //long larga

/*Metodo que devuelve el valor en angulos(deg) 
  que debe adoptar cada uno de los seis motores para que la plataforma
  establezca su posicion en el punto x,y,z,yaw,pitch,roll. Realiza las operaciones de cinematica inversa*/
void xyz_yprToServoAng(float x,float y,float z, float yaw, float pitch, float roll,float *motorAngles);

