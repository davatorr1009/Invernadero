#include <DHT.h>
#include <Arduino.h>
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include <Keypad.h>
#include <ESP32Servo.h>
#include "UbidotsEsp32Mqtt.h"

const char *UBIDOTS_TOKEN = "BBUS-sMYmaOkapJYhZFttPUdypw4ALFAVsc";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "NETLIFE-WTORRES";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "0000110300065";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "INVERNADERO";   // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL1 = "Temperatura"; // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL2 = "Humedad"; // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL3 = "releBomba"; // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL4 = "releTemperatura"; // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL5 = "Acceso"; // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL6 = "releHumedad"; // Put here your Variable label to which data  will be published

Ubidots ubidots(UBIDOTS_TOKEN);

#define DHTPIN 2 // Pin de datos del DHT22 conectado a PD2
#define DHTTYPE DHT22

LiquidCrystal_PCF8574 lcd(0x27); // Dirección I2C 0x27
const int releTemperatura = 16;
const int releHumedad = 17;
const int releEv1 = 5;
const int releBomba = 18;

DHT dht(DHTPIN, DHTTYPE);
const float tempMinima = 23.00;
const float tempMaxima = 23.50;
const float humMinima = 55.0;
const float humMaxima = 60.0;

// Keypad
const byte FILAS = 4;
const byte COLUMNAS = 4;
char teclas[FILAS][COLUMNAS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte pinesFilas[FILAS] = {14, 27, 26, 25};
byte pinesColumnas[COLUMNAS] = {33, 32, 23, 19};

Keypad teclado = Keypad(makeKeymap(teclas), pinesFilas, pinesColumnas, FILAS, COLUMNAS);
const char contrasenaCorrecta[] = "1111";
char contrasenaIngresada[5]; 
int indiceContrasena = 0; 
bool accesoConcedido = false;

Servo servoMotor;
const int buzzerPin = 15;

unsigned long previousMillis = 0;
const long interval = 2000; // Intervalo para la lectura del sensor (2 segundos)

void ingreso(){
  // Leer el teclado
  char teclaPresionada = teclado.getKey();
  if (teclaPresionada) {
    contrasenaIngresada[indiceContrasena] = teclaPresionada;
    indiceContrasena++;
    Serial.print("*");
    lcd.setCursor(0, 2);
    lcd.print("    Contrasena: ");
    for (int i = 0; i < indiceContrasena; i++) {
      lcd.print("*");
    }
    lcd.print("    ");
    if (indiceContrasena == 4) {
      contrasenaIngresada[4] = '\0';
      lcd.setCursor(0, 3);
      if (strcmp(contrasenaIngresada, contrasenaCorrecta) == 0) {
        Serial.println("ACCESO: CONCEDIDO");
        accesoConcedido=true;
        lcd.print("    Acceso: Concedido");
        if (accesoConcedido==true){
          digitalWrite(buzzerPin, HIGH);
          delay(2000);
          digitalWrite(buzzerPin, LOW);
          servoMotor.write(135);}
      } else {
        Serial.println("ACCESO: DENEGADO");
        lcd.print("    Acceso: Denegado ");
        accesoConcedido=false;
        if (accesoConcedido==false){
          digitalWrite(buzzerPin, HIGH);
          delay(500);
          digitalWrite(buzzerPin, LOW);
          servoMotor.write(45);
          delay(1500);}
      }
      delay(2000);
      lcd.setCursor(0, 2);
      lcd.print("                     ");
      lcd.setCursor(0, 3);
      lcd.print("                     ");
      indiceContrasena = 0;
      memset(contrasenaIngresada, 0, sizeof(contrasenaIngresada));
    }
  }
}

void leersensor(){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println(F("Error al leer el sensor DHT!"));
      lcd.setCursor(0, 0);
      lcd.print("Error en lectura");
    } else {
      Serial.print("Evaluando relés: t=");
      Serial.print(t);
      Serial.print(", h=");
      Serial.println(h);

      if (t < tempMinima) {
        digitalWrite(releTemperatura, HIGH);
      } else if (t > tempMaxima) {
        digitalWrite(releTemperatura, LOW);
      }
      Serial.print("Estado relé temperatura: ");
      Serial.println(digitalRead(releTemperatura)); 

      if (h < humMinima) {
        digitalWrite(releHumedad, LOW);
      } else if (h > humMaxima) {
        digitalWrite(releHumedad, HIGH);
      }
      Serial.print("Estado relé humedad: ");
      Serial.println(digitalRead(releHumedad));

      lcd.setCursor(0, 0);
      lcd.print("Tem: ");
      lcd.print(t);
      if (digitalRead(releTemperatura) == LOW) {
          lcd.print(" C ON ");
      } else {
          lcd.print(" C OFF");
      }

      lcd.setCursor(0, 1);
      lcd.print("Hum: ");
      lcd.print(h);
      if (digitalRead(releHumedad) == LOW) {
          lcd.print(" %");
          lcd.print(" ON ");
      } else {
          lcd.print(" %");
          lcd.print(" OFF");
    }
  }
  }
}

void callback(char *topic, byte *payload, unsigned int length){
  Serial.print("Message arrived[");
  Serial.print(topic);
  Serial.print("]");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void TareaIngreso(void *pvParameters) {
    for (;;) {
        ingreso();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void TareaLeerSensor(void *pvParameters) {
    for (;;) {
        leersensor();
        vTaskDelay(pdMS_TO_TICKS(interval));
    }
}

void TareaRiego(void *pvParameters) {
    pinMode(releBomba, OUTPUT);
    pinMode(releEv1, OUTPUT);

    for (;;) {
        digitalWrite(releBomba, LOW);
        digitalWrite(releEv1, LOW);
        
        lcd.setCursor(0, 2);
        lcd.print("    Riego Iniciado...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        lcd.setCursor(0, 2);
        lcd.print("                      ");

        vTaskDelay(pdMS_TO_TICKS(10000));

        digitalWrite(releBomba, HIGH);
        digitalWrite(releEv1, HIGH);
        
        lcd.setCursor(0, 2);
        lcd.print("    Riego Finalizado...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        lcd.setCursor(0, 2);
        lcd.print("                       ");
        
        vTaskDelay(pdMS_TO_TICKS(20000));
    }
}

void TareaEnviarUbidots(void *pvParameters) {
    for (;;) {
        if (!ubidots.connected()) {
            Serial.println("Intentando reconectar a Ubidots...");
            ubidots.reconnect();
        }

        float temp = dht.readTemperature();
        float hum = dht.readHumidity();
        bool releBombaEstado = digitalRead(releBomba) == LOW;
        bool releTempEstado = digitalRead(releTemperatura) == LOW;
        bool releHumedadEstado = digitalRead(releHumedad)==LOW;
        

        if (!isnan(temp) && !isnan(hum)) {
            Serial.println("Enviando datos a Ubidots...");
            Serial.print("Temperatura: "); Serial.println(temp);
            Serial.print("Humedad: "); Serial.println(hum);
            Serial.print("Estado Bomba: "); Serial.println(releBombaEstado);
            Serial.print("Estado Refrigeración: "); Serial.println(releTempEstado);
            Serial.print("Estado EV: "); Serial.println(releHumedadEstado);
            Serial.print("Acceso: "); Serial.println(accesoConcedido);

            ubidots.add(VARIABLE_LABEL1, temp);
            ubidots.add(VARIABLE_LABEL2, hum);
            ubidots.add(VARIABLE_LABEL3, releBombaEstado);
            ubidots.add(VARIABLE_LABEL4, releTempEstado);
            ubidots.add(VARIABLE_LABEL5, accesoConcedido);
            ubidots.add(VARIABLE_LABEL6, releHumedadEstado);
            ubidots.publish(DEVICE_LABEL);
        } else {
            Serial.println("Lectura de sensores fallida, no se enviaron datos.");
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Enviar datos cada 10 segundos
    }
}


void setup() {
  Wire.begin(21, 22);
  lcd.begin(16, 4);
  lcd.setBacklight(255);
  lcd.clear();
  dht.begin();
  Serial.begin(115200);
  servoMotor.attach(13);
  servoMotor.write(45);
  pinMode(buzzerPin, OUTPUT);
  pinMode(releTemperatura, OUTPUT);
  pinMode(releHumedad, OUTPUT);

  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();

  xTaskCreate(TareaLeerSensor, "TareaLeerSensor", 2048, NULL, 1, NULL);
  xTaskCreate(TareaIngreso, "TareaIngreso", 2048, NULL, 1, NULL);
  xTaskCreate(TareaRiego, "TareaRiego", 2048, NULL, 1, NULL);
  xTaskCreate(TareaEnviarUbidots, "TareaEnviarUbidots", 4096, NULL, 1, NULL);
}

void loop() {
  vTaskDelete(NULL);
}
