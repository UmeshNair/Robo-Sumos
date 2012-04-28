#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#define PI 3.14159265
unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
unsigned char receive_data=0;   // used to save Receiving data
unsigned char dummy=0;
unsigned char dummy1=0;
unsigned int x=0;
//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
 PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

//Function to configure INT1 (PORTD 3) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRD  = DDRD & 0xF7;  //Set the direction of the PORTD 3 pin as input
 PORTD = PORTD | 0x08; //Enable internal pull-up for PORTD 3 pin
}

//Function to configure INT0 (PORTD 2) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRD  = DDRD & 0xFB;  //Set the direction of the PORTD 2 pin as input
 PORTD = PORTD | 0x04; //Enable internal pull-up for PORTD 2 pin
}

//Function to initialize ports
void port_init()
{
 motion_pin_config();          //robot motion pins config
 left_encoder_pin_config();    //left encoder pin config
 right_encoder_pin_config();   //right encoder pin config	
}

void left_position_encoder_interrupt_init (void) //Interrupt 1 enable
{
 cli(); //Clears the global interrupt
 MCUCR = MCUCR | 0x08; // INT1 is set to trigger with falling edge
 GICR = GICR | 0x80;   // Enable Interrupt INT1 for left position encoder
 sei(); // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 0 enable
{
 cli(); //Clears the global interrupt
 MCUCR = MCUCR | 0x02; // INT0 is set to trigger with falling edge
 GICR = GICR | 0x40;   // Enable Interrupt INT5 for right position encoder
 sei(); // Enables the global interrupt 
}

//ISR for right position encoder
ISR(INT0_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}

//ISR for left position encoder
ISR(INT1_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}

void uart0_init(void)
{
 UCSRB = 0x00; //disable while setting baud rate
 UCSRA = 0x00;
 UCSRC = 0x86;
 UBRRL = 0x2F; //set baud rate lo  //67 is for 16MHz 9600 baudrate
 UBRRH = 0x00; //set baud rate hi
 UCSRB = 0x98; 
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortBRestore = PORTB; 		// reading the PORT original status
 PortBRestore &= 0xF0; 		// making lower direction nibbel to 0
 PortBRestore |= Direction; // adding lower nibbel for forward command and restoring the PORTB status
 PORTB = PortBRestore; 		// executing the command
}

void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop (void)
{
  motion_set(0x00);
}


//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 12.85; // division by resolution to get shaft count 
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 12.92; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
 return;
}

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}
void left_mm(unsigned int DistanceInMM)
{
 left();
 linear_distance_mm(DistanceInMM);
}

void right_mm(unsigned int DistanceInMM)
{
 right();
 linear_distance_mm(DistanceInMM);
}


void left_degrees(unsigned int Degrees) 
{
// 28 pulses for 360 degrees rotation 12.92 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
// 28 pulses for 360 degrees rotation 12.92 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
 // 56 pulses for 360 degrees rotation 12.85 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
 // 56 pulses for 360 degrees rotation 12.85 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
 // 56 pulses for 360 degrees rotation 12.85 degrees per count
 soft_left_2(); //Turn reverse soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
 // 56 pulses for 360 degrees rotation 12.85 degrees per count
 soft_right_2();  //Turn reverse soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

//Function to initialize all the devices
void init_devices()
{
 cli(); //Clears the global interrupt
 port_init();  //Initializes all the ports
 uart0_init();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 sei();   // Enables the global interrupt 
}


void uart0_clr(void)
{
 UCSRB = 0x08; 
}



SIGNAL(SIG_UART_RECV) 
{
 receive_data = UDR;			
 
 UDR = receive_data+1;           // Echo the received data plus 1
} 

//Main Function

int main(void)
{
 init_devices();
 while(1)
{
if(receive_data=='l'||receive_data=='r')
 {
    uart0_clr();
	dummy=receive_data ;
	receive_data = '0';
	uart0_init();
 }
 if(receive_data=='f')        
 {
    uart0_clr();
	receive_data = '0';
	int n = (rand() % 5)+5;
	forward_mm(n*100);
	uart0_init();
 }
if(((receive_data== 'a'||receive_data=='b')&&receive_data!= '0')&&(dummy=='l'))
 {
    uart0_clr();
	dummy1=receive_data ;
	receive_data = '0';
	uart0_init();
 }
if(((receive_data== 'a'||receive_data=='b')&&receive_data!= '0')&&(dummy=='r'))
 {
    uart0_clr();
	dummy1=receive_data ;
	receive_data = '0';
	uart0_init();
 }

if((receive_data!= 'a'&&receive_data!= '0')&&(dummy1=='a')&&(dummy=='l'))
 {
	if(receive_data=='9')
	{
	uart0_clr();
	left_degrees(90);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='8')
	{
	uart0_clr();
	left_degrees(80);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='7')
	{
	uart0_clr();
	left_degrees(70);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='6')
	{
	uart0_clr();
	left_degrees(60);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='5')
	{
	uart0_clr();
	left_degrees(50);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='4')
	{
	uart0_clr();
	left_degrees(40);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='3')
	{
	uart0_clr();
	left_degrees(30);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='2')
	{
	uart0_clr();
	left_degrees(20);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='1')
	{
	uart0_clr();
	left_degrees(13);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	
 }
 if((receive_data!= 'b'&&receive_data!= '0')&&(dummy1=='b')&&(dummy=='l'))
 {
	if(receive_data=='1')
	{
	uart0_clr();
	left_degrees(100);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='2')
	{
	uart0_clr();
	left_degrees(200);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='3')
	{
	uart0_clr();
	left_degrees(300);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
  }

if((receive_data!= 'a'&&receive_data!= '0')&&(dummy1=='a')&&(dummy=='r'))
 {
	if(receive_data=='9')
	{
	uart0_clr();
	right_degrees(90);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='8')
	{
	uart0_clr();
	right_degrees(80);      
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='7')
	{
	uart0_clr();
	right_degrees(70);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='6')
	{
	uart0_clr();
	right_degrees(60);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='5')
	{
	uart0_clr();
	right_degrees(50);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='4')
	{
	uart0_clr();
	right_degrees(40);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='3')
	{
	uart0_clr();
	right_degrees(30);      
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='2')
	{
	uart0_clr();
	right_degrees(20);      
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='1')
	{
	uart0_clr();
	right_degrees(13);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	
 }
 if((receive_data!= 'b'&&receive_data!= '0')&&(dummy1=='b')&&(dummy=='r'))
 {
	if(receive_data=='1')
	{
	uart0_clr();
	right_degrees(100);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='2')
	{
	uart0_clr();
	right_degrees(200);       
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
	if(receive_data=='3')
	{
	uart0_clr();
	right_degrees(300);
	receive_data = '0';
	dummy1='0';
	uart0_init();
    }
  }

}
}
