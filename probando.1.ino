//#include <PololuWheelEncoders.h>

#include <avr/interrupt.h>


//float Kp, Ki, Kd, last_error, Integral;

float Kp = 2;
float Ki = 0.01;
float Kd = 0.01;
float last_error = 0;
float Integral = 0;

float Kp1 = 1;
float Ki1 = 0.01;
float Kd1 = 0.01;
float last_error1 = 2500;
float Integral1 = 0;

float PID_Controller(float IN, float SET, float dt, float last_error, float Integral);
float PID_Controller1(float IN, float SET, float dt, float last_error, float Integral);

const int MOTOR = 11;
const int MOTOR2 = 10;

 int set = 2500;
 int vel = 0;
 int vel2 = 0;
 int pwm = 0;
 int pwm2 = 0;
 
 int die11 = 0;
 int die12 = 0;
 int die21 = 0;
 int die22 = 0;
 
 float vel11 = 0;
 float vel12 = 0;
 float vel21 = 0;
 float vel22 = 0;
 
 int CONTADOR = 0;
 
 
 
 volatile uint8_t portchistory = 0x00;     // default is high because the pull-up



void setup() {
 pinMode(MOTOR,OUTPUT);
 pinMode(MOTOR2,OUTPUT);  
 pinMode(6,OUTPUT);
 pinMode(5,OUTPUT);
 
 pinMode(A0,INPUT);
 pinMode(A1,INPUT);
 pinMode(A2,INPUT);
 pinMode(A3,INPUT);
 
 digitalWrite(6,HIGH);
 digitalWrite(5,LOW);
 digitalWrite(4,HIGH);
 digitalWrite(3,LOW);
 pinMode(13,OUTPUT);



 Serial.begin(9600);
 delay(1000);
 Serial.print("test ardubot...");
 Serial.println();
 
 
 
 
 //////////////////////////////////////////////
	PCICR |= (1 << PCIE1);     // set PCIE0 to enable PCMSK0 scan
	PCMSK1 |= (1 << PCINT8) + (1 << PCINT9) + (1 << PCINT10) + (1 << PCINT11);   // set PCINT0 to trigger an interrupt on state change 
 /////////////////////////////////

/*
        OCR1A = 0x3D08;
    //OCR1A = 0x3D08/2;
    
    TCCR1B |= (1 << WGM12);
    // Mode 4, CTC on OCR1A

    TIMSK1 |= (1 << OCIE1A);
    //Set interrupt on compare match

    TCCR1B |= (1 << CS12) | (1 << CS10);
    // set prescaler to 1024 and start the timer
*/
        //OCR0A = 0xAF;
        OCR1A = 65000;
        TIMSK1 |= _BV(OCIE1A);
	
	
	sei();
	// enable interrupts
//////////////////////////////////////////
 
 
 
}

//long t1,t2;

void loop() {
  
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial.println(inByte,DEC);
    if (inByte == 119){
      set = set + 50;
    }
    else if (inByte == 115){
      set = set - 50;
    }
    
  }
  

Serial.print(vel11);
Serial.print('\t');
Serial.print('\t');
Serial.print(vel12);

Serial.print('\t');
Serial.print('\t');
Serial.print(set);

Serial.print('\t');
Serial.print('\t');
Serial.println(pwm);
if (vel11 > 30){
  digitalWrite(13, HIGH);
}
else if (vel11 < 30){
  digitalWrite(13,LOW);
}

pwm = PID_Controller(vel11, set, 0.5, last_error, Integral);
analogWrite(MOTOR,pwm);

pwm2 = PID_Controller1(vel22, set, 0.5, last_error1, Integral1);
analogWrite(MOTOR2,pwm2);

}



SIGNAL (TIMER1_COMPA_vect)
{
    // action to be done every 1 sec
    if(CONTADOR == 500){
    vel11 = die11*5;
    vel12 = die12*5;
    die11 = 0;
    die12 = 0;
    
    
    vel21 = die21*5;
    vel22 = die22*5;
    die21 = 0;
    die22 = 0;
    
    
    CONTADOR = 0;
    //Serial.println(CONTADOR);
    }
    CONTADOR++;
    
}




ISR (PCINT1_vect)
{
    uint8_t changedbits;


    changedbits = PINC ^ portchistory;
    portchistory = PINC;

    
    if(changedbits & (1 << PINC0))
    {
        /* PCINT0 changed */
        die11 = die11 + 1;
        //Serial.println(die11);
    }
    
    if(changedbits & (1 << PINC1))
    {
        /* PCINT1 changed */
        die12 = die12 + 1;
    }

    if(changedbits & (1 << PINC2))
    {
        /* PCINT2 changed */
        die21 = die21 + 1;
    }
    
    if(changedbits & (1 << PINC3))
    {
        /* PCINT2 changed */
        die22 = die22 + 1;
    }

}

float PID_Controller(float IN, float SET, float dt, float last_error, float Integral) {

	float OUT;
        float error;
        float Derivative;

	error = SET - IN;
	Integral = Integral + error;
	Derivative = error - last_error;

	OUT = ( SET + Kp*error + Ki*Integral*dt + Kd*Derivative/dt)*255/12000;  //*255/(SET*Kp)

	if (OUT > 255)
		OUT = 255;
	else if (OUT < 0)
		OUT = 0;

	return OUT;
}





float PID_Controller1(float IN, float SET, float dt, float last_error, float Integral) {

	float OUT;
        float error;
        float Derivative;

	error = SET - IN;
	Integral1 = Integral1 + error;
	Derivative = error - last_error1;

	OUT = (SET + Kp1*error + Ki1*Integral*dt + Kd1*Derivative/dt)*255/12000;  //*255/(SET*Kp)

	if (OUT > 255)
		OUT = 255;
	else if (OUT < 0)
		OUT = 0;

	return OUT;
}
