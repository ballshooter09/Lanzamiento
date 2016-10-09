//#include <PololuWheelEncoders.h>

#include <avr/interrupt.h>

//Motor LeftMotor(5,3);
//Motor RightMotor(9,6);
//Encoder RightEncoder(2,3);
//Encoder LeftEncoder(7,8);
//PololuWheelEncoders encoder;
const int MOTOR = 11;
const int MOTOR2 = 10;
// Metro tim1(100,1);

//char val1, val2;
//char s;
 int set = 10;
 int vel = 0;
 int vel2 = 0;
 int pwm = 0;
 int pwm2 = 0;
 
 int die11 = 0;
 int die12 = 0;
 int die21 = 0;
 int die22 = 0;
 
 int vel11 = 0;
 int vel12 = 0;
 int vel21 = 0;
 int vel22 = 0;
 
 int CONTADOR = 0;
 
 
 //int vel11 = 0;
 
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
 //digitalWrite(13,LOW);

//  RightEncoder.start();
//  LeftEncoder.start();

 //  encoderInit();
 
 //encoder.init(A2,A3,A0,A1);
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
  
Serial.print(die11);
Serial.print('\t');
Serial.print(die12);
Serial.print('\t');
Serial.print(vel11);
Serial.print('\t');
Serial.println(vel12);
if (vel11 > 30){
  digitalWrite(13, HIGH);
}
else if (vel11 < 30){
  digitalWrite(13,LOW);
}

}

/*
ISR (TIMER1_COMPA_vect)
{
    // action to be done every 1 sec
    vel11 = die11/24;
    vel12 = die12/24;
}
*/
SIGNAL (TIMER1_COMPA_vect)
{
    // action to be done every 1 sec
    if(CONTADOR == 500){
    vel11 = die11/12;
    vel12 = die12/12;
    die11 = 0;
    die12 = 0;
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


