unsigned long previousT  {0}; 
unsigned long currentT {0}; 
float deltaT {0};
float kp {1} ;
float ki {1} ; 
float errorI {0};
float kd {1} ; 
float errorD {0}; 
float targetVelocity {0}; 
float currentVelocity {0}; 
float errorVelocity  {0};
float plantInput {0}; 

float PIDController (float errorVelocity , float errorI , float errorD, float kp , float ki, float kd) ; 

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    currentT = micros();
    deltaT = (static_cast<float>(currentT - previousT))/1.0e6 ; 
    errorVelocity = targetVelocity - currentVelocity ;  
    errorI = errorI +  errorVelocity * deltaT ; 
    errorD =  (targetVelocity - currentVelocity) / deltaT ; 
    plantInput = PIDController (errorVelocity,errorI,errorD,kp,ki,kd) ; 
    previousT = currentT ;   
}

float PIDController (float errorVelocity , float errorI , float errorD, float kp , float ki, float kd)
{
    plantInput = kp * errorVelocity + ki * errorI + kd * errorD ;
    return plantInput ; 

}