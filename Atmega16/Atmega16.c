/*
 * Atmega16.c
 *
 * Created: 7/16/2013 3:13:00 PM
 *  Author: Cristi
 * Atmega16
 */ 

#define F_CPU 20000000 //pentru 20Mhz
#include <avr/io.h>
#include <util/delay.h>



#define setHigh(port,pin) port|=(1<<pin)
#define setLow(port,pin) port&=~(1<<pin)
#define toggle(port,pin) port^=(1<<pin)
#define check(port,pin) ((port&(1<<pin))!=0)

//kp=1000 => kp=10 pentru alpha=100
int kp=2345;
int kd=2610;
int ki=675; //crestem ki cand avem nevoie de variatii bruste pentru y()
//int delay_ms=5; // Un MCU Atmel face 5 operatii elementare in virgula fixa pe nanosecunda pentru 20Mhz, deci am putea pune si un delay de oridinul microsecundelor (daca ar exista functie de delay in microsecunde)
const int dt=5; // functia _delay_ms() cere ca delay-ul sa fie o constanta
const int alpha=100; // a>=1 , preferabil 10 la o putere si crescut proportional cu delay-ul, default: alpha=1  -  cu cat alpha creste, cu atat este atenuat outputul functiei y (si gain-ul pentru motor, adica beta)
int theta=25; // 0<=theta<=100    -  cand duty cycle-ul unui motor creste cu beta, duty-cycle-ul celuilalt motor este scazut cu gamma = theta/100 * beta
int phi=170; // 0<=phi<=255   -   default motor duty cycle ,  se va modifica 100 cu 255 pentru MCU(uC)

int counter=0;
int booster_enabled=0;

#define maxCount 30

#define minPhi 110

#define epsilonMax 343
#define maxIntegral 5*dt*epsilonMax



#define eps_a0 0
#define eps_a1 4
#define eps_a2 49
#define eps_a3 343

#define eps_a01 2
#define eps_a12 14
#define eps_a23 130



struct settings{
	uint16_t kp; //valoare maxima pentru uint16_t = 65535
	uint16_t kd;
	uint16_t ki;
	uint8_t theta;
	uint8_t phi;
};

typedef struct settings SETTINGS;


void setPinHigh(volatile uint8_t* PORT,uint8_t pin)
{
	*PORT|=(1<<pin);
}

void setPinLow(volatile uint8_t* PORT,uint8_t pin)
{
	*PORT&=~(1<<pin);
}

void togglePin(volatile uint8_t* PORT,uint8_t pin)
{
	*PORT^=(1<<pin);
}


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

// 1 - enable, 0 - disable 
void booster(uint8_t en_dis)
{
	static int init_phi=phi;
	static int init_theta=theta;
	if(en_dis!=0)
	{
		booster_enabled=1;
		//override phi
		phi = 240;
		//override theta
		theta = 5; //sau 10
	}
	else
	{
		booster_enabled=0;
		phi=init_phi;
		theta=init_theta;
	}		
}

void setRightMotor(uint8_t speed, uint8_t direction) //OC1B pe PD4
{
	OCR1B = speed;
	// PD0 si PD1 pentru directia motorului din stanga
	if(direction==0)
	{
		setPinHigh(&PORTD,3);
		setPinLow(&PORTD,2);
	}
	else
	{
		setPinLow(&PORTD,3);
		setPinHigh(&PORTD,2);
	}
	
}

void setLeftMotor(uint8_t speed,uint8_t direction) //OC1A pe PD5
{
	OCR1A = speed;
	// PD2 si PD3 pentru directia motorului din dreapta
	if(direction==0)
	{
		setPinHigh(&PORTD,6);
		setPinLow(&PORTD,7);
	}
	else
	{
		setPinLow(&PORTD,6);
		setPinHigh(&PORTD,7);
	}
}

int64_t readInput()
{
	uint8_t x=PINA;
	int64_t error=0;
	//prelucram intrarea
	//int64_t y=check(x,0)*10+check(x,1)*8+check(x,2)*6+check(x,5)*(-6)+check(x,6)*(-8)+check(x,7)*(-10);
	switch(x)
	{
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
		case 0xc0 : error=-eps_a23;	break;	
		
		// xxxx xxxx
		default : error=0; 
		
	}
	return error;
}


int64_t sign(int64_t x)
{
	if(x>=0)
		return 1;
	else
		return -1;
}

int64_t limitIntegral(int64_t x)
{
	if(abs(x)>maxIntegral)
		return sign(x)*maxIntegral;
	else
		return x;
}

// y este functia de output pentru PID
int64_t y(int64_t e) //y=output u=input
{
	int64_t derivative,output;
	static int64_t pe=0,integral=0;
	integral=limitIntegral(integral+e*dt);
	
	//limitam integrala
	/*
	if(integral>maxIntegral)
	{
		integral=maxIntegral;
	}
	if(integral<-maxIntegral)
	{
		integral=-maxIntegral;
	}
	*/
	derivative=(e-pe)/dt;
	output = kp*e + ki*integral + kd*derivative;
	
	//activate booster if error is 0 for maxCount times (dt)
	if(e==0)
	{
			if(counter<maxCount)
			{
				counter++;
			}
			else
			{
				if(booster_enabled==0)
					booster(1);
			}
	}
	else
	{
		if(booster_enabled!=0)
			booster(0);
		counter=0;
	}
	
	
	pe=e;
	return output;
}

/*
int64_t y(int64_t u) //y=output u=input
{
	int64_t e,derivative,output;
	static int64_t pe=0,integral=0;
	e=u-pe; //e = eroare , pe = eroare anterioara
	integral=integral+e*dt;
	derivative=(e-pe)/dt;
	output = kp*e + ki*integral + kd*derivative;
	pe=e;
	return output;
}*/
/*
* beta=y(u(t))/alpha
* gamma=theta/100*beta;
*
*/

uint8_t limit(int64_t x)
{  
	if(x>255)
		return 255;
	else
	{
		if(x<minPhi)
		   return minPhi;
		else 
		   return x;
	}
}
void interpret_y(int64_t y)
{
	int64_t beta,gamma;
	beta=y/alpha;
	gamma=theta*beta/100;
	if(y>0) // beta>0 gamma>0
	{
		setLeftMotor(limit(phi+abs(beta)),0);
		setRightMotor(limit(phi-abs(gamma)),0);
		//M1=phi+beta;
		//M2=phi-gamma;
	}
	if(y<0) // beta<0 gamma<0
	{
		setLeftMotor(limit(phi-abs(gamma)),0);
		setRightMotor(limit(phi+abs(beta)),0);
		//M1=phi+gamma;
		//M2=phi-beta;
	}
	/*
	if(y>0) // beta>0 gamma>0
	{
		setLeftMotor(limit(phi+beta),0);
		setRightMotor(limit(phi-gamma),0);
		//M1=phi+beta;
		//M2=phi-gamma;
	}
	if(y<0) // beta<0 gamma<0
	{
		setLeftMotor(limit(phi+gamma),0);
		setRightMotor(limit(phi-beta),0);
		//M1=phi+gamma;
		//M2=phi-beta;
	}
	*/
	if(y==0) // beta=0 gamma=0
	{
		setLeftMotor(phi,0);
		setRightMotor(phi,0);
	}
}

void DipSwitch()
{
	//{kp,kd,ki,theta,phi}
		//Practic, kp=100 in program insamna kp=1 la TSA;
		//Aflam un Kp pentru care iesirea sistemului oscileaza cu amplitudine constanta, memoram si perioada de oscilatie T, aplicam metoda Zigler-Nichols (agresiva dar stabila in bucla inchisa)
/* testing
		SETTINGS array[16]={  //64 biti * 16 = 1024 biti = 128 bytes
			{200,0,0,25,180},		// 0 0 0 0
			{400,0,0,25,180},			// 0 0 0 1
			{600,0,0,25,180},			// 0 0 1 0
			{800,0,0,25,180},			// 0 0 1 1
			{1000,0,0,25,180},			// 0 1 0 0
			{2000,0,0,25,180},			// 0 1 0 1
			{3000,0,0,25,180},			// 0 1 1 0
			{4000,0,0,25,180},			// 0 1 1 1
			{5000,0,0,25,180},			// 1 0 0 0
			{7500,0,0,25,180},			// 1 0 0 1
			{10000,0,0,25,180},			// 1 0 1 0
			{20000,0,0,25,180},			// 1 0 1 1
			{30000,0,0,25,180},			// 1 1 0 0
			{40000,0,0,25,180},			// 1 1 0 1
			{5000,0,0,25,180},			// 1 1 1 0
			{60000,0,0,25,180}			// 1 1 1 1
		};
*/
		SETTINGS array[16]={  //64 biti * 16 = 1024 biti = 128 bytes
			{4200,2750,750,25,180},		// 0 0 0 0
			{27500,19669,5824,25,180},			// 0 0 0 1
			{12478,9858,7062,25,180},			// 0 0 1 0
			{25464,6205,8245,25,180},			// 0 0 1 1
			{24281,17327,10061,25,180},			// 0 1 0 0
			{15604,4402,253,25,180},			// 0 1 0 1
			{1292,12882,7521,25,180},			// 0 1 1 0
			{10894,9203,3911,25,180},			// 0 1 1 1
			{14300,8750,4000,25,180},			// 1 0 0 0
			{15827,5936,2491,25,180},			// 1 0 0 1
			{12538,2369,10012,25,180},			// 1 0 1 0
			{2322,10833,7773,25,180},			// 1 0 1 1
			{10894,9203,3911,25,180},			// 1 1 0 0
			{19716,20218,9995,25,180},			// 1 1 0 1
			{5827,5936,2491,25,180},			// 1 1 1 0
			{2322,10866,4871,25,180}			// 1 1 1 1
	};
	uint8_t x;
	x=check(PINB,1)*8+check(PINB,2)*4+check(PINB,3)*2+check(PINB,4)*1;
	//modificam variabilele globale
	kp=array[x].kp;
	kd=array[x].kd;
	ki=array[x].ki;
	theta=array[x].theta;
	phi=array[x].phi;
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
	PORTD=0b00000001; //RXD aprins , TXD stins
	while(1)
	{
		if(!check(PINB,0)) // push button
			break;
	}
	//DipSwitch(); //citire dip switch si modificare variabile globale (kp,kd,ki,theta si phi)
	PORTD=0b10111010;  // //RXD stins , TXD aprins , PWM pornit (50%) - directie standard (inainte)
    while(1)
    {
		interpret_y(y(readInput()));
		_delay_ms(dt); //delay de 5 milisecunde
    }
	return 0;
}