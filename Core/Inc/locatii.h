/*
 * locatii.h
 *
 *  Created on: Nov 25, 2023
 *      Author: stoica
 */

#ifndef INC_LOCATII_H_
#define INC_LOCATII_H_



unsigned long timp_sett_min;
unsigned long timp_sett_max;


int buttonState_min;
int buttonState_max;

int lastButtonState_min;
int lastButtonState_max;
 int nr_poi_sateliti;

int poi ;
double curent_lat;
double curent_lng;
double save_lat[10];
double save_lng[10];

int  ch1_H;
int  ch3_H ;
int servo_val;

int sateliti;
float altitudine;
//int heading;
int viteza ;
unsigned long Distance_To_Home;                                    // variable for storing the distance to destination
float currentHeading;
float desiredBearing;

float HeadingError;
float controlSignal; // pt pid
int ch3_H;
int ch4_B;

//float kp = 15.5;
//int bias;


void poi_sett( int ch)
{

if(ch > 1200){  timp_sett_min = HAL_GetTick();}
if(ch < 1800){  timp_sett_max = HAL_GetTick();}

if(HAL_GetTick()-timp_sett_min> 100)
{
     buttonState_min = 1;
}

if(HAL_GetTick()-timp_sett_max> 100)
{
   buttonState_max = 1;
}

if(ch < 1800 && ch > 1200){
buttonState_min=0;
buttonState_max=0;
}


  if (buttonState_min != lastButtonState_min) {

    // change the state of the led when someone pressed the button
    if (buttonState_min == 1) {
         poi--;
      if(poi<=0) poi=0;
    }
    // remember the current state of the button
    lastButtonState_min = buttonState_min;
  }
  if (buttonState_max != lastButtonState_max) {

    // change the state of the led when someone pressed the button
    if (buttonState_max == 1) {
         poi++;
      if(poi>=10) poi=10;
    }
    // remember the current state of the button
    lastButtonState_max = buttonState_max;
  }
}

unsigned long timp_salvare;
//int mod=10;
unsigned long timp_mod;
int mod = 10;


void salvare_punct( int ch)
{

if(ch > 1200){  timp_salvare = HAL_GetTick();}

if(HAL_GetTick()-timp_salvare >500)
{
  mod=0;
  save_lat[poi]= curent_lat;
  save_lng[poi]=curent_lng;
}
if(mod !=0){  timp_mod=HAL_GetTick();}

if(HAL_GetTick()-timp_mod > 4000)
{
  mod=10;
}

}


int pornire_autopilot;

unsigned long t_change;
int nr_c;
//	0			1	2		3		4		5			6	7		8			9
// salvare , home , reset , wait , manual , return , ready, calib , error , to point ,
int mod_pilot;
void mod_operare ( int ch7)
{
	if(ch7 < 1300){ mod=4;mod_pilot=1; pornire_autopilot=0;}   // manual
	else if(ch7 < 1700 && ch7 > 1300) {mod=9; mod_pilot=2;}
	else if (ch7>1700)
	{
		mod_pilot=3;
		if(HAL_GetTick()-t_change >2000)
		{
			t_change=HAL_GetTick();
			nr_c++;
			if(nr_c==2){nr_c=0;}
		}


		if(nr_c==0) {mod=5;}
		else if (nr_c==1) {mod=1;}
	}

}

unsigned long timp_tr;

void trimite_catre_punct( int ch)
{

if(ch < 1800){  timp_tr = HAL_GetTick();}

if(HAL_GetTick()-timp_tr >500)
{
// porneste autopilotul

	if(GPS.satelites <=3)
	{
		mod=8;   //eroare
	}
	else
	{
		mod=3; // asteapta
		pornire_autopilot=1;
	}


}

}


#endif /* INC_LOCATII_H_ */
