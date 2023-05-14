/*
 *   This program is to get the speed of the wheels using 
 *   Speedometer sensor , on top of that to control speed using PID controllers
 *   --------------------------------------   
 *   Microcontroller | Arduino 
 *   --------------------------------------
 *   Sensor type     | Speedometer 
 *   Connection      | Digital Pin 2 
 *   Sensor specs    | 
 *   Sensor Freq     |
 *   Max Speed       |
 *  --------------------------------------
 *  Mechanism 
 *   Wheel Radius    |
 *  --------------------------------------
*/

#define speedometer 2  
#define speedometerFreq 100000  
#define maxSpeed 10  
#define wheelRadius 0.145  

unsigned long previousT  {0}; 
unsigned long currentT {0}; 

float deltaT {0.0};
float deltaTmints {0.0};
float kp {1.0} ;
float ki {1.0} ; 
float errorI {0.0};
float kd {1.0} ; 
float errorD {0.0}; 
float targetVelocity {0.0}; 
float currentVelocity {0.0}; 
float errorVelocity  {0.0};
float plantInput {0.0}; 
float angularVelocityMeasured {0.0};

int revolutions {0} ; 
int rpm {0}; 
int pulsOnTime {0};
int noOfPulses {0}; 


float PIDController (float errorVelocity , float errorI , float errorD, float kp , float ki, float kd) ; 

void setup()
{  
    pinMode(speedometer, INPUT);

}

void loop()
{   /*
        This code measure delta time , this happened by mesuring current time and 
        pervious time. 
    */
    currentT = micros();

    // This block of code is kept, in case work with the intrupt ISR

    // attachInterrupt(digitalPinToInterrupt(speedometer), isr, RISING);
    // detachInterrupt(speedometer);
    //time units in seconds  
    
    deltaT = (static_cast<float>(currentT - previousT))/1.0e6 ; 
    
    // velocity units m/s 
    // if (revolutions > 0) 
    // {
    //     rpm = (max(1, revolutions)* 60)/deltaT;
    // } 

    /*
     * using pulseIn function is to calculate the duration time for the T(ON), 
     * from that we could determine the No of pulses 
     * using cross multiplcation the speed should be optained 
    */
    pulsOnTime = pulseIn(speedometer, HIGH);
    noOfPulses = speedometerFreq * pulsOnTime ; 
    angularVelocityMeasured =  noOfPulses * maxSpeed / speedometerFreq ; 
    currentVelocity = angularVelocityMeasured * wheelRadius ; 
    
    /*
     * PID controllers 
    */
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
// void isr ()
// {
//     revolutions++ ; 
// }