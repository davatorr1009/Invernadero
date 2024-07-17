#include <DHT.h>
#include <Arduino.h>
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include <Keypad.h>
#include <ESP32Servo.h>

#define DHTPIN 2 // Pin de datos del DHT22 conectado a PD2
#define DHTTYPE DHT22

LiquidCrystal_PCF8574 lcd(0x27); // Dirección I2C 0x27, comprobe que si es el puerto para el LCD
const int releTemperatura = 16;
const int releHumedad = 17;
const int releEv1 = 5;
const int releBomba = 18;

DHT dht(DHTPIN, DHTTYPE);
const float tempMinima = 24.0;
const float tempMaxima = 25.0;
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
const char contrasenaCorrecta[] = "1111"; // Contraseña de 4 dígitos
char contrasenaIngresada[5]; // Arreglo para almacenar la contraseña ingresada (un espacio extra para el \0)
int indiceContrasena = 0; // Índice para llevar el seguimiento de la contraseña ingresada
bool accesoConcedido = false;

Servo servoMotor;  // Crear un objeto Servo
const int buzzerPin = 15; // Pin al que está conectado el buzzer

unsigned long previousMillis = 0;
const long interval = 2000; // Intervalo para la lectura del sensor (2 segundos)

void ingreso(){
// Leer el teclado
  char teclaPresionada = teclado.getKey();
  if (teclaPresionada) {
    contrasenaIngresada[indiceContrasena] = teclaPresionada;
    indiceContrasena++;
    Serial.print("*"); // Imprimir un asterisco para ocultar la contraseña
    
    lcd.setCursor(0, 2); // Mover a la tercera fila
    lcd.print("    Contrasena: ");
    for (int i = 0; i < indiceContrasena; i++) {
      lcd.print("*");
    }
    lcd.print("    "); // Limpiar cualquier carácter sobrante

    if (indiceContrasena == 4) { // Si se ingresaron 4 dígitos
      contrasenaIngresada[4] = '\0'; // Terminar la cadena con null
      lcd.setCursor(0, 3); // Mover a la cuarta fila
      if (strcmp(contrasenaIngresada, contrasenaCorrecta) == 0) {
        Serial.println("ACCESO: CONCEDIDO");
        lcd.print("    Acceso: Concedido");
        digitalWrite(buzzerPin, HIGH); // Encender el buzzer (tono continuo)
        delay(2000);
        digitalWrite(buzzerPin, LOW); // Apagar el buzzer
        servoMotor.write(135);
      } else {
        Serial.println("ACCESO: DENEGADO"); 
        lcd.print("    Acceso: Denegado ");
        digitalWrite(buzzerPin, HIGH); // Encender el buzzer (tono continuo)
        delay(500);
        digitalWrite(buzzerPin, LOW); // Apagar el buzzer
        servoMotor.write(45);
        delay(1500);
      }
      delay(2000);  // Mostrar mensaje por 2 segundos
      lcd.setCursor(0, 2);
      lcd.print("                     "); // Limpiar línea de asteriscos
      lcd.setCursor(0, 3);
      lcd.print("                     "); // Limpiar línea de acceso
      indiceContrasena = 0; 
      memset(contrasenaIngresada, 0, sizeof(contrasenaIngresada)); // Limpiar el arreglo
    }
  }
}
void leersensor(){
 unsigned long currentMillis = millis();
  
  

  // Leer el sensor DHT cada intervalo de tiempo
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
        digitalWrite(releTemperatura, LOW);
      } else if (t > tempMaxima) {
        digitalWrite(releTemperatura, HIGH);
      }
      Serial.print("Estado relé temperatura: ");
      Serial.println(digitalRead(releTemperatura)); // Imprimir estado del relé

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
      if (digitalRead(releTemperatura) == LOW) { // Comprobar el estado del pin del relé
          lcd.print(" C ON ");
      } else {
          lcd.print(" C OFF");
      }

      lcd.setCursor(0, 1);
      lcd.print("Hum: ");
      lcd.print(h);
      if (digitalRead(releHumedad) == LOW) { // Comprobar el estado del pin del relé
          lcd.print(" %");
          lcd.print(" ON ");
      } else {
          lcd.print(" %");
          lcd.print(" OFF");
    }
}
}}

void TareaIngreso(void *pvParameters) {
    for (;;) {
        ingreso();  // Tu función existente para manejar la contraseña
        vTaskDelay(pdMS_TO_TICKS(100)); // Pequeño retraso para no saturar el bucle
    }
}
void TareaLeerSensor(void *pvParameters) {
    for (;;) {
        leersensor();  // Tu función existente para leer el sensor y actualizar el LCD
        vTaskDelay(pdMS_TO_TICKS(interval)); // Esperar el intervalo definido
    }
}
void TareaRiego(void *pvParameters) {
    // Inicializar pines de relés como salidas
    pinMode(releBomba, OUTPUT);
    pinMode(releEv1, OUTPUT);

    for (;;) {
        // Encender relés
        digitalWrite(releBomba, LOW);
        digitalWrite(releEv1, LOW);

        // Enviar mensaje de depuración (opcional)
        
        lcd.setCursor(0,2);
        lcd.print("    Riego Iniciado...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        lcd.setCursor(0,2);
        lcd.print("                      ");

        // Esperar el tiempo de riego (ajusta según tus necesidades)
        vTaskDelay(pdMS_TO_TICKS(4000)); // 5 minutos (300000 ms)

        // Apagar relés
        digitalWrite(releBomba, HIGH);
        digitalWrite(releEv1, HIGH);
        
        lcd.setCursor(0,2);
        lcd.print("    Riego Finalizado...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        lcd.setCursor(0,2);
        lcd.print("                       ");
        // Esperar una semana
        vTaskDelay(pdMS_TO_TICKS(20000)); // 7 días (604800000 ms)
    }
}

void setup() {
  Wire.begin(21, 22); // Pines SDA y SCL para ESP32 DEVKIT V4
  lcd.begin(16, 4); // Inicializa el LCD (16 columnas x 4 filas)
  lcd.setBacklight(255); // Enciende la retroiluminación al máximo
  lcd.clear(); // Limpia la pantalla
  dht.begin();
  Serial.begin(115200);
  servoMotor.attach(13);  // Conectar el servo al pin 13
  servoMotor.write(45);    // Posición inicial del servo (0 grados)
  pinMode(buzzerPin, OUTPUT);
  pinMode(releTemperatura, OUTPUT);
  pinMode(releHumedad, OUTPUT);
  xTaskCreate(TareaLeerSensor, "TareaLeerSensor", 2048, NULL, 1, NULL);
  xTaskCreate(TareaIngreso, "TareaIngreso", 2048, NULL, 1, NULL);
  xTaskCreate(TareaRiego, "TareaRiego", 2048, NULL, 1, NULL);
}

void loop() {
  vTaskDelete(NULL); // Elimina la tarea en loop()
 }
