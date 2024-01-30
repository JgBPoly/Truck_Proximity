//Librairie EEPROM
#include <EEPROM.h>

//Définit la taille de la mémoire EEPROM à utiliser 
#define EEPROM_SIZE 4095


String MAC_addr_1 = "00-1F-3A-D5-6F-14";
String MAC_addr_1_EEPROM = "00-00-00-00-00-00";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //Initialisation de l'EEPROM à la taille définie
  EEPROM.begin(EEPROM_SIZE);

  //EEPROM.put(0,MAC_addr_1);
  
  //EEPROM.commit();
  //Solution alternative
  //EEPROM.write(1,MAC_addr_1);//Modification que si nécessaire pour diminuer l'accès en écriture
}

void loop() {
  // put your main code here, to run repeatedly:
  //MAC_addr_1_EEPROM = EEPROM.read(a);
  EEPROM.get(0, MAC_addr_1_EEPROM);
  
  Serial.print(MAC_addr_1_EEPROM);
  Serial.println();

  
}
