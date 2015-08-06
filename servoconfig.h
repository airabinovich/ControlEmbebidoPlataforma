#define minAngPrimerCuadrante  10
#define maxAngPrimerCuadrante  90
#define minAngSegundoCuadrante 90
#define maxAngSegundoCuadrante 170
/*Enumeracion de los cuadrantes en que puede operar un motor*/
enum Quadrant{FIRST,SECOND};

/*Estructura de servo de Plataforma. servo es un objeto Servo definido en la biblioteca Servo.h.
  quad represenda el cuadrante en donde debe operar el servomotor en la plataforma*/
struct platformServos{
  Servo servo;
  Quadrant quad;
};

void  writeToServos(float *motorAng,platformServos *servos);
void servosSetupConfigurations();
