/**
  Code Tube-Pitot
  @par J-C Dansereau aka Chinaski aka GELGIF drop-out
  @version presente: v0.7
  @2018-03-31
*/
/**
Carte sd:
CD : pas connecter
DO: MISO : 12
GND:
SCK : clock: 13
VCC: 3.3
DI: MOSI:11
CS: 10
 */
#include <SPI.h>
#include <math.h>
#include "Adafruit_MAX31855.h"
#include <SD.h>
#include <stdlib.h> 
#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5
#define LARGEUR_FILTRE 10
#define STRING_WIDTH 4
#define STRING_PRECISION 2

File sensorDataFile; // objet file 
float time1 = 0.0;
String dataString = ""; 
int csPin = 10; // cs chip sd
float V = 0.0; // Vitesse
int freq_ech = 4;
char Temps[10];
char Vitesse[10];
char Temperature[10];
char P_DYN[10];
char P_STAT[10];
char MACH[10];
char Vit_son[10];
// initialisation des sensors
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
int analogPin0 = A0; //Pression Dynamique (MPXHZ6250A)
int analogPin1 = A1; //Pression Statique (MPXA6115A)
int filtre_Pdynamique[LARGEUR_FILTRE];
unsigned int index_Pdynamique=0;
int filtre_Pstatique[LARGEUR_FILTRE];
unsigned int index_Pstatique=0;

float gamma = 1.401; // air CONSTANTE, gamma=k=cp/cv
float R = 0.287; // air CONSTANTE kJ/(kg*K)
                                                                         // Si on essayait 287 (TC)
int get_filtre_moyenne_int(int filtre[], unsigned int largeur_filtre){
  int somme = 0;
  for (unsigned int i = 0; i < largeur_filtre; i++) {
    somme += filtre[i];
  }
  return somme/largeur_filtre;
}

void setup() {
  // communication avec le port serial
  Serial.begin(9600);
  pinMode(csPin,OUTPUT); // csPin en mode output
  while (!Serial) delay(1);
  Serial.println("Initialisation des sensors.");
  Serial.println("Initialisation du thermocouple.");
  //  stabilisation des sensors
  if (!SD.begin(csPin)) { // Initialize SD card
    Serial.println("Erreur dans initialisation de la carte"); 
    return;
  }
  Serial.println("Carte initialisee");
  File sensorDataFile = SD.open("data.csv",FILE_WRITE);
  if (sensorDataFile)
  {
    sensorDataFile.println("Temps(s) , Vitesse (m/s) , Temperature (deg. C), Pression Dyn. (kPa), Pression Stat. (kPa), Mach, Vitesse du son(m/s)");
    sensorDataFile.close();
  }
}

void loop() {
   double T0 = thermocouple.readCelsius(); //T0 en celcius
   if (isnan(T0)) {
     T0=0.0;
     Serial.println("Probleme avec le thermocouple!");
   } else {
     //Serial.print("Temperature_celcius = "); 
     //Serial.println(T0);
   }
  filtre_Pdynamique [index_Pdynamique] = map(analogRead(analogPin0),0,1023,0,5000); //Output Dynamique millivolt
  index_Pdynamique ++;
  if (index_Pdynamique >= LARGEUR_FILTRE) {
    index_Pdynamique = 0;
  }
  filtre_Pstatique [index_Pstatique] = map(analogRead(analogPin1),0,1023,0,5000); //Output statique millivolt
  index_Pstatique ++;
  if (index_Pstatique >= LARGEUR_FILTRE) {
    index_Pstatique = 0;
  }
  int sensorMilliVolts0 = get_filtre_moyenne_int(filtre_Pdynamique, LARGEUR_FILTRE);
  delay(1);
  int sensorMilliVolts1 = get_filtre_moyenne_int(filtre_Pstatique, LARGEUR_FILTRE); //Output Statique millivot
  delay(1);
  
  float P_dynamique = (sensorMilliVolts0/1000.0 +0.2)/0.02; //kPa
  float P_statique =(sensorMilliVolts1/1000.0+0.475)/0.045; // kPa
                                                      // C'est quoi ça (TC)
  // Calcul de vitesse premiere iteration
  // notes: T0 = thermocouple au top, T1 = var. temporaire,
  // T0 = température d'arrêt = température du thermocouple
  float T1=pow((P_statique/ P_dynamique),((gamma-1)/gamma));
  float T = T1*(T0+273.0); // en Kelvin
  float C =sqrt(R*1000*gamma*T);// m/s ,R en *1000 car kJ/(kg*K) à J/(kg*K)
  float Ma=sqrt((2*(((T0+273.0)/T) -1))/(gamma-1));// le nombre de Mach, unitless
  float delta_P=P_dynamique-P_statique; //kPa
  // calcul de rho gaz parfait: P=rho*RT  : rho =P/RT
  float rho = (P_statique/(R*T)); // kg/m3
  
  if (Ma<=0.3 && delta_P>0.0)
  {
    // Regime incompressible
    V=sqrt(2*(delta_P*1000)/rho);
  }
  else if(Ma>0.3 && delta_P>0.0)
  {
    // Regime Compressible
    V=C*Ma;
  }
  else
  {
    V=sqrt(2*(delta_P*1000)/rho);
    // si Delta_P<0.0 , on a un de probleme
  }
  time1 = millis()/1000.0;
  delay(1000/freq_ech); // 
  dtostrf(time1, STRING_WIDTH, STRING_PRECISION, Temps);
  dtostrf(V, STRING_WIDTH, STRING_PRECISION, Vitesse);
  dtostrf(T0, STRING_WIDTH, STRING_PRECISION,Temperature);
  dtostrf(P_dynamique, STRING_WIDTH, STRING_PRECISION, P_DYN);
  dtostrf(P_statique, STRING_WIDTH, STRING_PRECISION, P_STAT);
  dtostrf(Ma, STRING_WIDTH, STRING_PRECISION, MACH);
  dtostrf(C, STRING_WIDTH, STRING_PRECISION, Vit_son);
  
  dataString = String(time1) + "," + String(Vitesse) + "," + String(Temperature)+ "," + String(P_DYN)+ "," + String(P_STAT)+ "," + String(MACH)+ "," + String(Vit_son);
  saveData();
   
  //Serial.println("Pression dynamique =");
  //Serial.println(P_dynamique);
  //Serial.println("Pression statique =");
  //Serial.println(P_dynamique);
  //Serial.println(P_statique);
  Serial.println(T0);
}
void saveData(){
  if(SD.exists("data.csv")){ // check the card is still there
    // now append new data file
    sensorDataFile = SD.open("data.csv", FILE_WRITE);
    if (sensorDataFile){
      sensorDataFile.println(dataString);
      sensorDataFile.close(); // close the file
    }
  }
  else{
    Serial.println("Error writing to file !");
  }
}
 
