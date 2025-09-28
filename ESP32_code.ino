#include <WiFi.h>
#include <PubSubClient.h>
#include "SoftwareSerial.h"
#include <PZEM004Tv30.h>
#include "Wire.h"
#include "DHT.h"
#include "max6675.h"
#include <stdio.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 4);

int thermoDO = 19;
int thermoCS = 23;
int thermoCLK = 5;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

#define DHTPIN 15

#define DHTTYPE DHT22   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);
#define PIN_PIR 36

int digitalPin =25; // pour le relais1
int var_led = 14; // GPIO pour le demarrage à dstance

int nbPresencesTotal = 0;
int nbPresencesJour = 0;
unsigned long tempsPrecedent = 0;

#define WIFISSID "TECNO" // votre SSID WiFi 
#define PASSWORD "ange2023" //  votre mot de passe WiFi 
#define TOKEN "BBUS-zxgD46rv6W2pTZpIKiWQO2laEN6pU9" // votre jeton Ubidots ici 
#define MQTT_CLIENT_NAME "esp32" // Nom du client MQTT

#define DEVICE_LABEL "esp32"
#define VARIABLE_LABEL1  "Temperature" // Assing the variable label
#define VARIABLE_LABEL2  "Humidity"    // Assing the variable label
#define VARIABLE_LABEL5  "tension"     // Assing the variable label
#define VARIABLE_LABEL3  "energy"      // Assing the variable label
#define VARIABLE_LABEL4  "current"
#define VARIABLE_LABEL6  "frequency"
#define VARIABLE_LABEL8  "puissance"
#define VARIABLE_LABEL7  "pf"
#define VARIABLE_LABEL_SUBSCRIBE  "var_led"
#define VARIABLE_LABEL_PRESENCE_TOTAL "total_presence"
#define VARIABLE_LABEL10 "daily_energy" // Nouvelle variable pour l'énergie quotidienne
#define VARIABLE_LABEL_HOURLY_ENERGY "hourly_energy"
#define VARIABLE_LABEL12  "tc" // Assing the variable label

// Brokers MQTT pour Ubidots
char mqttBroker[] = "industrial.api.ubidots.com";

WiFiClient ubidots;
PubSubClient client(ubidots);

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17

#define PZEM_SERIAL Serial2

PZEM004Tv30 pzem(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN);

char payload[2000];
char topic1[150];
char topic2[150];
char topic3[100];
char topic4[150];
char topic5[150];
char topic6[150];
char topic7[150];
char topicSubscribe[150];
char topic8[150];
char topic9[150];
char topic10[150];
char topic11[150];
char topic12[150];

char str_Temp[60];
char str_humidity[60];
char str_tension[60];
char str_current[60];
char str_power[60];
char str_energy[20];
char str_frequency[60];
char str_pf[60];
char str_PIN[60];
char str_daily_energy[20]; // KKKKKKKKKKK
char str_hourly_energy[20];
char str_tc[60];

// WiFi and NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

float previousEnergy = 0.0;
float dailyEnergy = 0.0;
float hourlyEnergy = 0.0;
unsigned long lastUpdateTime = 0;

void callback(char *topic, byte *payload, unsigned int length)
{
char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  Serial.write(payload, length);
  Serial.println(topic);

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char value[20];
  
  
  for (int i = 0; i < 20; i++) {
    value[i] = '\0';
  }
  
  for (int i=0;i<length;i++) {
    char c = (char)payload[i];
    value[i] = payload[i];
    Serial.print(c);
  }
  Serial.println();

  float f_value = atof(value);
  if (f_value == 0) {
    digitalWrite(var_led, LOW);
  } else {
    digitalWrite(var_led, HIGH);
  }  
}
  
void reconnect()
{
    while (!client.connected())
    {
        Serial.println("Tentative de connexion MQTT...");
        if (client.connect(MQTT_CLIENT_NAME, TOKEN, ""))
        {
            Serial.println("Connecté à MQTT");
            sprintf(topicSubscribe, "/v1.6/devices/%s/%s/lv", DEVICE_LABEL, VARIABLE_LABEL_SUBSCRIBE);
            client.subscribe(topicSubscribe);
        }
        else
        {
            Serial.print("Échec de la connexion MQTT, rc=");
            Serial.print(client.state());
            Serial.println(" Réessayez dans 2 secondes");
            delay(2000);
        }
    }
}

void setup()
{

   WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Désactiver le brownout detector
    Serial.begin(9600);
  
    WiFi.begin(WIFISSID, PASSWORD);

      Serial.println(F("DHTxx test!"));

  dht.begin();

    pinMode(var_led, OUTPUT);
    pinMode(PIN_PIR, INPUT);
    pinMode(digitalPin, OUTPUT);
    digitalWrite(digitalPin, LOW);

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }

    Serial.println("");
    Serial.println("WiFi connecté");
    Serial.println("Adresse IP: ");
    Serial.println(WiFi.localIP());

    client.setServer(mqttBroker, 1883);
    client.setCallback(callback);

    sprintf(topicSubscribe, "/v1.6/devices/%s/%s/lv", DEVICE_LABEL, VARIABLE_LABEL_SUBSCRIBE);
    client.subscribe(topicSubscribe);

  // Initialize NTP client
    timeClient.begin();
    timeClient.setTimeOffset(3600); // Adjust for your time zone (3600s = 1h for CET)



}

void loop()
{
    if (!client.connected())
    {
        reconnect();   
    }

  delay(2000);
  lcd.init();//Initialisation de l'écran LCD

  lcd.backlight();//Allumage du rétroéclairage

 // Mise à jour de l'heure NTP
    timeClient.update();

 // basic readout test, just print the current temp
  
  float h = dht.readHumidity();
  // Lecture de la temperature en Celsius (par defaut)
  float t = dht.readTemperature();
  // Lecture de la niveau de chaleur  en Farenheit (si Fahrenheit = true)
  float f = dht.readTemperature(true);

  // Verifie si une lecture a échouer......./(isnan pour dire si aucune donnée)
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Echec de lecture du capteur DHT sensor!"));
    return;
  }
  // Calcul de l'index de chaleur en Fahrenheit (par default)
  float hif = dht.computeHeatIndex(f, h);
  // Calcul de l'index de chaleur en Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

 float tc = thermocouple.readCelsius();
    float tcf = thermocouple.readFahrenheit();

  Serial.print("Temperature_TC(°c) = "); 
  Serial.println(thermocouple.readCelsius());
  Serial.print("Temperature_TC(F)= ");
  Serial.println(thermocouple.readFahrenheit());
  

    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" °C ");
    Serial.println();

delay(2000);

 
    if (t>= 42.0) {
// Température compris entre 8°C à 10°C, activer le relais
digitalWrite(digitalPin, LOW);// Démarrer le compresseur

} else { // sinon

digitalWrite(digitalPin, LOW);//Arreter le compresseur

}

    float voltage = pzem.voltage();
    float tension = voltage;
    if (voltage != NAN)
    
    {
        Serial.print("Voltage: ");
        Serial.print(voltage);
        Serial.println("V");
    }
    else
    {
        Serial.println("Error reading voltage");
    }
    float current = pzem.current();
    if (current != NAN)
    {
        Serial.print("Current: ");
        Serial.print(current);
        Serial.println("A");
    }
    else
    {
        Serial.println("Error reading current");
    }
    float power = pzem.power();
    if (current != NAN)
    {
        Serial.print("Power: ");
        Serial.print(power);
        Serial.println("W");
    }
    else
    {
        Serial.println("Error reading power");
    }
    float energy = pzem.energy();
    if (current != NAN)
    {
        Serial.print("Energy: ");
        Serial.print(energy, 3);
        Serial.println("kWh");
    }
    else
    {
        Serial.println("Error reading energy");
    }
 
    float frequency = pzem.frequency();
    if (current != NAN)
    {
        Serial.print("Frequency: ");
        Serial.print(frequency, 1);
        Serial.println("Hz");
    }
    else
    {
        Serial.println("Error reading frequency");
    }
    float pf = pzem.pf();
    if (current != NAN)
    {
        Serial.print("PF: ");
        Serial.println(pf);
    }
    else
    {
        Serial.println("Error reading power factor");
    }
    Serial.println();

    while (Serial1.available())
    {
        Serial1.read();
    }

 //Reset daily energy consumption at midnight
 
unsigned long currentMillis = millis();
    if (currentMillis - lastUpdateTime >= 60000) {
        lastUpdateTime = currentMillis;

        int currentHour = timeClient.getHours();
        int currentMinute = timeClient.getMinutes();
        if (currentHour == 0 && currentMinute == 0) {
            dailyEnergy = 0;
        }

        if (currentMinute == 0) {
            hourlyEnergy = 0;
        }

        dailyEnergy += (energy - previousEnergy);
        hourlyEnergy += (energy - previousEnergy);
        previousEnergy = energy;
    }
        Serial.print("Energy_jour: ");
        Serial.print(dailyEnergy, 3);
        Serial.println("kWh");

        Serial.print("Energy_Heure: ");
        Serial.print(hourlyEnergy, 3);
        Serial.println("kWh");

if (digitalRead(PIN_PIR) == HIGH) {
    nbPresencesJour++;
    nbPresencesTotal++;
    Serial.print("Présence détectée ! Nombre total : ");
    Serial.print(nbPresencesTotal);
    Serial.print(" - Nombre aujourd'hui : ");
    Serial.println(nbPresencesJour);
  }
  
  unsigned long tempsActuel = millis();
  if (tempsActuel - tempsPrecedent >= 86400000) { // 86400000 millisecondes = 24 heures
    nbPresencesJour = 0;
    tempsPrecedent = tempsActuel;
  }

delay(2000);
  
  lcd.clear();//On efface l'écran

  // Le texte à afficher
  String texte = "chambre froide";

  // Calcul de l'indice de départ pour centrer le texte
  int longueurTexte = texte.length();
  int colonneDebut = (20 - longueurTexte) / 2;

  // Placer le curseur au centre de la première ligne
  lcd.setCursor(colonneDebut, 0);

  // Afficher le texte
  lcd.print(texte);
    
  lcd.setCursor(0, 1);//On place le curseur à l'origine (coin supérieur gauche)

  lcd.print("Temperature:");
  
  lcd.print(thermocouple.readCelsius());

  lcd.print(" ");
  lcd.write(223); //Pour avoir le symbole°
  lcd.print("C");

lcd.setCursor(0, 2);//On passe à la ligne suivante
lcd.print("Humidity:");
lcd.print(h);
lcd.print(" %");

lcd.setCursor(0, 3);//On passe à la ligne suivante

  lcd.print("Energy:");
  
   lcd.print(energy,3);
   
   lcd.print("kWh");

     delay(2000);

    dtostrf(t, 4, 2, str_Temp);
    
    dtostrf(h, 4, 2, str_humidity);

    dtostrf(tension, 4, 2, str_tension);
    dtostrf(current, 4, 2, str_current);
    dtostrf(power, 4, 2, str_power);
    dtostrf(energy, 6, 4, str_energy);
    dtostrf(frequency, 4, 2, str_frequency);
    dtostrf(pf, 4, 2, str_pf);
    dtostrf(dailyEnergy, 6, 4, str_daily_energy);
    dtostrf(hourlyEnergy, 6, 4, str_hourly_energy);
    dtostrf(tc, 4, 2, str_tc); 


    sprintf(topic1, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(payload, "%s", "");
    sprintf(payload, "{\"%s\":", VARIABLE_LABEL1);
    sprintf(payload, "%s {\"value\": %s}}", payload, str_Temp);

    client.publish(topic1, payload);
    delay(180000);

    sprintf(topic2, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(payload, "%s", "");
    sprintf(payload, "{\"%s\":", VARIABLE_LABEL2);
    sprintf(payload, "%s {\"value\": %s}}", payload, str_humidity);

    client.publish(topic2, payload);
    delay(180000);

    sprintf(topic5, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(payload, "%s", "");
    sprintf(payload, "{\"%s\":", VARIABLE_LABEL5);
    sprintf(payload, "%s{\"value\": %s}}", payload, str_tension);

    client.publish(topic5, payload);

    sprintf(topic4, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(payload, "%s", "");
    sprintf(payload, "{\"%s\":", VARIABLE_LABEL4);
    sprintf(payload, "%s {\"value\": %s}}", payload, str_current);
  
    client.publish(topic4, payload);
    delay(180000);

    sprintf(topic3, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(payload, "%s", "");
    sprintf(payload, "{\"%s\":", VARIABLE_LABEL3);
    sprintf(payload, "%s {\"value\": %s}}", payload, str_energy);

    client.publish(topic3, payload);
    delay(180000);

    sprintf(topic6, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(payload, "%s", "");
    sprintf(payload, "{\"%s\":", VARIABLE_LABEL6);
    sprintf(payload, "%s {\"value\": %s}}", payload, str_frequency); 
  
    client.publish(topic6, payload);
    delay(180000);

    sprintf(topic8, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(payload, "%s", "");
    sprintf(payload, "{\"%s\":", VARIABLE_LABEL8);
    sprintf(payload, "%s {\"value\": %s}}", payload, str_power);
   
    client.publish(topic8, payload);

  sprintf(topic7, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", "");
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL7);
  sprintf(payload, "%s {\"value\": %s}}", payload, str_pf);
  client.publish(topic7, payload);
  delay(180000);
  
  sprintf(topic9, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", "");
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL_PRESENCE_TOTAL);
  sprintf(payload, "%s {\"value\": %s}}", payload, str_PIN);
  client.publish(topic9, payload);
  delay(180000);

  sprintf(topic10, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", "");
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL10);
  sprintf(payload, "%s {\"value\": %s}}", payload, str_daily_energy);
  client.publish(topic10, payload);
  delay(180000);

       sprintf(topic11, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
        sprintf(payload, "%s", "");
        sprintf(payload, "{\"%s\":", VARIABLE_LABEL_HOURLY_ENERGY);
        sprintf(payload, "%s {\"value\": %s}}", payload, str_hourly_energy);
        client.publish(topic11, payload);
       delay(180000);


    sprintf(topic12, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(payload, "%s", "");
    sprintf(payload, "{\"%s\":", VARIABLE_LABEL12);
    sprintf(payload, "%s {\"value\": %s}}", payload, str_tc);
    
    client.publish(topic12, payload);
    delay(180000);

 delay (2000);
   client.loop();
}
