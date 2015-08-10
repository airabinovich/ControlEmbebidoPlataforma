#include <Servo.h>
/*Enumeracion de los cuadrantes en que puede operar un motor*/
enum Quadrant{FIRST,SECOND};

#define minAngPrimerCuadrante  10
#define maxAngPrimerCuadrante  90
#define minAngSegundoCuadrante 90
#define maxAngSegundoCuadrante 170

struct platformServos{
  Servo servo;
  Quadrant quad;
};



class servoStewart{
  public:
    void  writeToServos(float *motorAng);
    void servosSetupConfigurations();
    void attachServos(int initPin);
  private:
    platformServos servos [6];
};


