// Berceanu Cristian

#define F_CPU 16000000 //pentru 16Mhz
#include <avr/io.h>
#include <util/delay.h>


#define setHigh(port,pin) port|=(1<<pin)
#define setLow(port,pin) port&=~(1<<pin)
#define toggle(port,pin) port^=(1<<pin)
#define check(port,pin) ((port&(1<<pin))!=0)




#define dt 50
#define maxIntegral dt*5*6

#define minZeta 110
#define maxZeta 255

float kp=1.5;
float kd=0;
float ki=0;

#define phi 50 // -145<=phi<=145

#define theta 50 // 0<=theta<=100

//eps_max = 127

#define eps_a0 0
#define eps_a1 2
#define eps_a2 4
#define eps_a3 6

#define eps_a01 1 
#define eps_a12 3
#define eps_a23 5

#define eps_a4 7 //cand iese de pe traseu


//#define TRASEU_CU_INTRERUPERI




int8_t sign_int8_t(int8_t);

//compatibil cu ATmega16 si ATmega32
void init_PWM() // folosim OC1A si OC1B de pe portul D
{
        TCCR1A |=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM10); //mode 5 Fast-PWM 8-bit, non-inverting pentru amandoua
        TCCR1B |=(1<<WGM12)|(1<<CS10); //no prescaler, avem nevoie de frecventa cat mai mare
        OCR1A=127;
        OCR1B=127;
}

/*
* PD0 - *  (RXD)
* PD1 - *  (TXD)
* PD2 - IN3
* PD3 - IN4
* PD4 - EnableA
* PD5 - EnableB
* PD6 - IN1
* PD7 - IN2
* VCC - VCC
* GND - GND
*/

void setRightMotor(uint8_t speed, uint8_t direction) //OC1B pe PD4
{
        OCR1B = speed;
        // PD0 si PD1 pentru directia motorului din stanga
        if(direction==0)
        {
                setHigh(PORTD,2);
                setLow(PORTD,3);
        }
        else
        {
                setLow(PORTD,2);
                setHigh(PORTD,3);
        }
        
}

void setLeftMotor(uint8_t speed,uint8_t direction) //OC1A pe PD5
{
        OCR1A = speed;
        // PD2 si PD3 pentru directia motorului din dreapta
        if(direction==0)
        {
                setHigh(PORTD,7);
                setLow(PORTD,6);
        }
        else
        {
                setLow(PORTD,7);
                setHigh(PORTD,6);
        }
}

int8_t readInput()
{
        uint8_t x=PINA;
        int8_t error=0;
		static int8_t previous_error=0;
        switch(x)
        {
			#ifndef TRASEU_CU_INTRERUPERI
				case 0x00 : error=sign_int8_t(previous_error)*eps_a4; break; // merge doar daca nu sunt intreruperi pe traseu
			#endif
			#ifdef TRASEU_CU_INTRERUPERI
				case 0x00 : error=previous_error; break;
			#endif
                // 0000 0001
                case 0x01 : error=eps_a3; break;
                
                // 0000 0010
                case 0x02 : error=eps_a2; break;
                
                // 0000 0011
                case 0x03 : error=eps_a23; break;
                
                // 0000 0100
                case 0x04 : error=eps_a1; break;
                
                // 0000 0110
                case 0x06 : error=eps_a12; break;
                
                // 0000 1000
                case 0x08 : error=eps_a0; break;
                
                // 0000 1100
                case 0x0c : error=eps_a01; break;
                
                // 0001 0000
                case 0x10 : error=-eps_a0; break;
                
                // 0001 1000
                case 0x18 : error=0; break;
                
                // 0010 0000
                case 0x20 : error=-eps_a1; break;
                
                // 0011 0000
                case 0x30 : error=-eps_a01; break;
                
                // 0100 0000
                case 0x40 : error=-eps_a2; break;
                
                // 0110 0000
                case 0x60 : error=-eps_a12; break;
                
                // 1000 0000
                case 0x80 : error=-eps_a3; break;
                
                // 1100 1000
                case 0xc0 : error=-eps_a23;        break;        
                
                // xxxx xxxx
                default : error=0; 
                
        }
		
		previous_error=error;
		
        return error;
}

int8_t sign_float(float x)
{
	if(x>=0)
		return 1;
	else
		return -1;
}

int8_t sign_int8_t(int8_t x)
{
	if(x>=0)
		return 1;
	else
		return -1;
}

float abs_float(float x)
{
	if(x>=0)
		return x;
	else
		return -x;
}

int16_t abs_int16_t(int16_t x)
{
	if(x>=0)
		return x;
	else
		return -x;
}

float limitIntegral(float x)
{
		//TODO: de modificat
	    if(abs_float(x)>maxIntegral)
                return sign_float(x)*maxIntegral;
        else
                return x;
}

int16_t limit(float x) //returneaza ceva intre -145 si 145
{
	if(abs_float(x)>=145)
	{
		return sign_float(x)*145;
	}
	else
	{
		return round(x);
	}
}


float y(int8_t e) //y=output u=input
{
        float derivative,output;
        static float pe=0,integral=0;
        integral=limitIntegral(integral+e*dt);

        derivative=(e-pe)/dt;
        output = kp*e + ki*integral + kd*derivative;
        
        pe=e;
        return output;
}

void interpret_y(float y)
{
		int16_t motorL,motorR;
		
		if(y>0)
		{
			motorL=limit(phi+y);
			motorR=limit(phi-(y*theta)/100);
		}
		else if(y<0)
		{
			motorR=limit(phi+y);
			motorL=limit(phi-(y*theta)/100);
		}
		else
		{
			motorR=phi;
			motorL=phi;
		}

		if(motorR>=0)
		{
			setRightMotor(abs_int16_t(motorR)+minZeta,0);
		}
		else
		{
			setRightMotor(abs_int16_t(motorR)+minZeta,1);
		}
		if(motorL>=0)
		{
			setLeftMotor(abs_int16_t(motorL)+minZeta,0);
		}
		else
		{
			setLeftMotor(abs_int16_t(motorL)+minZeta,1);
		}
}

int main(void)
{

    DDRD=0xff; //output, motoare 
    DDRA=0x00; // citire senzori
    DDRB=0x00; //Presupunem PB0, PB1, PB2 si PB3 intrari pentru dip switch si buton de pornire
    PORTA=0x00;
    PORTB=0x00;
    PORTD=0x00;
    init_PWM(); //initializare PWM
    _delay_ms(2000);
    while(1)
    {
                interpret_y(y(readInput()));
                _delay_ms(dt); //delay de 50ms
    }
        return 0;
}