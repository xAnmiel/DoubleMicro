  #include "BluetoothSerial.h"
  #include <Wire.h>
  BluetoothSerial SerialBT;
  // Constantes y variables para sensores
  const int pinIRIzquierdo = 4; 
  const int pinIRDerecho = 2;
  const int pinIRFrontal = 15;

  const int pinSensorRanurado1 = 16;
  const int pinSensorRanurado2 = 17;

  const int enA = 5;
  const int in1 = 18;
  const int in2 = 19;
  const int enB = 21;
  const int in3 = 22;
  const int in4 = 23;

  int valorIRIzquierdo;
  int valorIRDerecho;
  int valorIRFrontal;
  int valorRanurado1;
  int valorRanurado2;
  int fase = 1;

  void leerSensores();
  void enviarDatos();
  void recibirComandos();
  void ejecutarMovimientos(char comando);

  volatile long wheelCountLeft = 0;
  volatile long wheelCountRight = 0;

  void leftWheelInterrupt() {
    wheelCountLeft++;
  }

  void rightWheelInterrupt() {
    wheelCountRight++;
  }


  void setup() {
    Serial.begin(9600);
    SerialBT.begin("Micromouse");

    pinMode(pinSensorRanurado1, INPUT_PULLUP);
    pinMode(pinSensorRanurado2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(pinSensorRanurado1), leftWheelInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinSensorRanurado2), rightWheelInterrupt, CHANGE);
    
    pinMode(pinIRIzquierdo, INPUT);
    pinMode(pinIRDerecho, INPUT);
    pinMode(pinIRFrontal, INPUT);
    pinMode(pinSensorRanurado1, INPUT);
    pinMode(pinSensorRanurado2, INPUT);

    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enA, 255);
    analogWrite(enB, 255);
    
    Serial.println("Dispositivo Bluetooth iniciado");
  }

  void loop() {
    leerSensores();
    enviarDatos();
    recibirComandos();
  }

  void leerSensores() {
    // Leer el sensor infrarrojo izquierdo
    valorIRIzquierdo = analogRead(pinIRIzquierdo);
    // Leer el sensor infrarrojo derecho
    valorIRDerecho = analogRead(pinIRDerecho);
    // Leer el sensor infrarrojo frontal
    valorIRFrontal = analogRead(pinIRFrontal);
    // Leer el primer sensor ranurado (odometría)
    valorRanurado1 = digitalRead(pinSensorRanurado1);
    // Leer el segundo sensor ranurado (odometría)
    valorRanurado2 = digitalRead(pinSensorRanurado2);
  }

  void enviarDatos() {
    // Envía el valor del sensor IR izquierdo
    SerialBT.print("IRL,");
    SerialBT.print(valorIRIzquierdo);
    SerialBT.print(",");
    // Envía el valor del sensor IR derecho
    SerialBT.print("IRD,");
    SerialBT.print(valorIRDerecho);
    SerialBT.print(",");
    // Envía el valor del sensor IR frontal
    SerialBT.print("IRF,");
    SerialBT.print(valorIRFrontal);
    SerialBT.print(",");
    // Envía el valor del primer sensor ranurado
    SerialBT.print("SR1,");
    SerialBT.print(valorRanurado1);
    SerialBT.print(",");
    // Envía el valor del segundo sensor ranurado
    SerialBT.print("SR2,");
    SerialBT.print(valorRanurado2);
    // Indica el final de la línea de datos
    SerialBT.println();
    // Envía el valor de wheelCountRight
    SerialBT.print("wheelCountRight,");
    SerialBT.print(wheelCountRight);
    SerialBT.print(",");
    // Envía el valor de wheelCountLeft
    SerialBT.print("wheelCountLeft,");
    SerialBT.print(wheelCountLeft);
    // Indica el final de la línea de datos
    SerialBT.println();
  }

  void recibirComandos() {
    if (SerialBT.available()) {
      char comando = SerialBT.read();
      if (comando == 'S') { // Cambio de fase
        fase = 2;
      } else {
        ejecutarMovimientos(comando);
      }
    }
  }

  void ejecutarMovimientos(char comando) {
    // Si estamos en la fase 1, permitimos los movimientos
    if (fase == 1) {
      // Detener los motores antes de cambiar de dirección
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);

      switch (comando) {
        case 'A': // Avanzar
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          break;
        case 'R': // Retroceder
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
          break;
        case 'L': // Girar a la izquierda
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          break;
        case 'D': // Girar a la derecha
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
          break;
        default:
          // Si se recibe un comando no válido, no hacer nada
          break;
      }
    }
  }