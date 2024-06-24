#include <Keypad.h>
#include "HID.h"
#include <Servo.h>

const byte FILAS = 4;
const byte COLUMNAS = 4;
char teclas[FILAS][COLUMNAS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte pinesFilas[FILAS] = {9, 8, 7, 6};
byte pinesColumnas[COLUMNAS] = {5, 4, 3, 2};

Keypad teclado = Keypad( makeKeymap(teclas), pinesFilas, pinesColumnas, FILAS, COLUMNAS );

const char contrasenaCorrecta[] = "1111"; // Contraseña de 4 dígitos
char contrasenaIngresada[5];  // Arreglo para almacenar la contraseña ingresada (un espacio extra para el \0)
int indiceContrasena = 0;      // Índice para llevar el seguimiento de la contraseña ingresada

Servo miServo;  // Creamos un objeto Servo
int pinServo = 10; // Pin al que está conectado el servo (elige un pin PWM)

const int buzzerPin = 11; // Pin al que está conectado el buzzer

const int TRIGGER_PIN = 13; 
const int ECHO_PIN = 12;

//const int concebido = A2; // Pin analógico A2 como salida digital
//const int denegado = A3; // Pin analógico A3 como salida digital

bool accesoConcedido = false;

void setup() {
  Serial.begin(115200);
  miServo.attach(pinServo);
  miServo.write(45);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(buzzerPin, OUTPUT); // Configurar el pin del buzzer como salida
  //pinMode(concebido, OUTPUT);
  //pinMode(denegado, OUTPUT);
}

void loop() {
  char teclaPresionada = teclado.getKey();
  long duracion, distancia;

  // Medir la distancia con el sensor ultrasónico
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  duracion = pulseIn(ECHO_PIN, HIGH);
  distancia = (duracion * 0.0343) / 2; // Calcular distancia en cm
  // Medir la distancia con el sensor ultrasónico
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  duracion = pulseIn(ECHO_PIN, HIGH);
  distancia = (duracion * 0.0343) / 2; // Calcular distancia en cm
 

  if (teclaPresionada) {
    contrasenaIngresada[indiceContrasena] = teclaPresionada;
    indiceContrasena++;
    Serial.print("*"); // Imprimir un asterisco para ocultar la contraseña
    

    if (indiceContrasena == 4) { // Si se ingresaron 4 dígitos
      contrasenaIngresada[4] = '\0'; // Terminar la cadena con null
      
      
      

      // Aquí iría la comparación de la contraseña (próximo paso)
      if (strcmp(contrasenaIngresada, contrasenaCorrecta) == 0 && distancia <= 40 && distancia>0) {
        Serial.println("ACCESO:CONCEDIDO");
        Serial.println("Enviando 'A'..."); // Mensaje de depuración
        String mensaje = "Concedido";
        Serial.println(mensaje);
        // Aquí podrías agregar código para abrir una puerta, encender un LED, etc.
        digitalWrite(buzzerPin, HIGH);// Encender el buzzer (tono continuo)
        //digitalWrite(concebido, HIGH); 
        //Serial.println(digitalRead(concebido));
        //Serial.println(digitalRead(denegado));
        delay(2000);
        digitalWrite(buzzerPin, LOW);// Apagar el buzzer
        //digitalWrite(concebido, LOW);  
        miServo.write(135);  // Movemos el servo a 110 grados
        
        
        }
       
        
       else {
        Serial.println("ACCESO:DENEGADO"); 
        Serial.println("Enviando 'D'..."  ); // Mensaje de depuración
        String mensaje = "Denegado";
        Serial.println(mensaje);
        //Serial.write('D');
        //accesoConcedido = false;
        
        miServo.write(45);
        digitalWrite(buzzerPin, HIGH); // Encender el buzzer (tono continuo)
        //digitalWrite(denegado, HIGH);
        //Serial.println(digitalRead(concebido));
        //Serial.println(digitalRead(denegado)); 
        delay(500);
        digitalWrite(buzzerPin, LOW);// Apagar el buzzer
        delay(1500);
        
        //digitalWrite(denegado, LOW);  
        
        // Aquí podrías agregar código para indicar un error, como un sonido de alarma.
      }

      indiceContrasena = 0; 
      memset(contrasenaIngresada, 0, sizeof(contrasenaIngresada)); // Limpiar el arreglo
}}
  }
