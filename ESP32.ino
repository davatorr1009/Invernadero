//Código para ESP32 update de 23/06/2024

#include <DHT.h>
#include <Arduino.h>
#include <LiquidCrystal_PCF8574.h>

#include <Wire.h>

#define DHTPIN 2 // Pin de datos del DHT22 conectado a PD2
#define DHTTYPE DHT22
#define RXD2 3 // GPIO03 del ESP32 DevkitV4 (RX2)
#define TXD2 1 // GPIO01 del ESP32 DevkitV4 (TX2)

unsigned long tiempoMensaje = 0; // Variable para almacenar el tiempo de inicio del mensaje
bool mostrarMensaje = false;    // Bandera para indicar si se debe mostrar el mensaje


LiquidCrystal_PCF8574 lcd(0x27); // Dirección I2C 0x27, comprobe que si es el puerto para el LCD

const int releTemperatura = 14;
const int releHumedad = 12;
const int concebido = 16; // Pin digital del ESP32 para recibir la señal de A2
const int denegado = 17; // Pin digital del ESP32 para recibir la señal de A3


DHT dht(DHTPIN, DHTTYPE);
const float tempMinima = 20.0;
const float tempMaxima = 24.0;
const float humMinima = 40.0;
const float humMaxima = 60.0;

void setup() {
  Wire.begin(21, 22); // Pines SDA y SCL para ESP32 DEVKIT V4
  lcd.begin(16, 2); // Inicializa el LCD (16 columnas x 2 filas)
  lcd.setBacklight(255); // Enciende la retroiluminación al máximo
  lcd.clear(); // Limpia la pantalla
  dht.begin();
  Serial.begin(9600);
  pinMode(releTemperatura, OUTPUT);
  pinMode(releHumedad, OUTPUT);
  Serial2.begin(9600,SERIAL_8N1, RXD2, TXD2);
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (Serial2.available()) {
  char estado = Serial2.read();
  Serial.print("Recibido: "); // Mensaje de depuración
  
  if (estado == 'A') {
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print(estado);
    lcd.print("Acceso Concedido");
    mostrarMensaje = true;
    tiempoMensaje = millis();
    
    
  } else if (estado == 'D') {
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print(estado);
    lcd.print("Acceso Denegado");
    mostrarMensaje = true;
    tiempoMensaje = millis();
    
  }
}
  // Mostrar mensaje por 3 segundos
  if (mostrarMensaje && (millis() - tiempoMensaje >= 3900)) {
    mostrarMensaje = false;
  }

  // Verificar si la lectura fue exitosa
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Error al leer el sensor DHT!"));
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("Error en lectura");
    delay(1000);
    lcd.clear();
    return;}
  // Imprimir valores antes de evaluar condiciones
  Serial.print("Evaluando relés: t=");
  Serial.print(t);
  Serial.print(", h=");
  Serial.println(h);

  // Control del relé de temperatura
  if (t < tempMinima) {
  digitalWrite(releTemperatura, LOW);
  } if (t > tempMaxima) {
  digitalWrite(releTemperatura, HIGH);
  }

  // Control del relé de humedad
  if (h < humMinima) {
  digitalWrite(releHumedad, LOW);
  } if (h > humMaxima) {
  digitalWrite(releHumedad, HIGH);
  }

   if (!mostrarMensaje) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(t);
    lcd.print(" C");

    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(h);
    lcd.print(" %");

  }
  delay(2000);
}
