/**********************************************************************************
*   Movimiento MIA
*     Control de motor trifasico con variador de frecuencia Powtran PI130
*   Autor: Dario Capucchio
*   Fecha: 15-02-2023
*   Hardware: Raspberry Pi Pico
* 
**********************************************************************************/
/* ============= LIBRERIAS ============= */
#include <Arduino.h>
/* ============= DEFINICIONES ============= */
#define BUTTON1 12
#define BUTTON2 13
#define LED1  14
#define LED2  15
#define RS485_EN 2

#define SLAVE_ADDR 0x01   // Direccion del esclavo
#define CMD_READ 0x03     // Comando de lectura
#define CMD_WRITE 0x06    // Comando de escritura

// FUNCIONES
void motorRUN(void);
void motorSTOP(void);
void motorSETfreq(void);

// VARIABLES DE FLUJO DE PROGRAMA
char dato;
char comando;
//byte trama[10];

/* ============= SETUP CORE 0 ============= */
void setup() {
  Serial.begin(115200);   // Comunicacion serie con la PC
  Serial1.begin(9600);    // Comunicacion serie con modulo RS485

  pinMode(BUTTON1,INPUT_PULLUP);
  pinMode(BUTTON2,INPUT_PULLUP);
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(RS485_EN,OUTPUT);
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,HIGH);    // Motor parado
  digitalWrite(RS485_EN,LOW); // Recivo RS485
  //digitalWrite(RS485_EN,HIGH); // Envio RS485

  comando = '0';

  delay(1000);
}

/* ============= LOOP CORE 0 ============= */
void loop() {
  if (Serial1.available() > 0){   // Reviso la comunicacion con el variador
    dato = Serial1.read();
    Serial.print(dato, HEX);
  }
  if (Serial.available() > 0){    // Reviso la comunicacion con la PC
    comando = Serial.read();
  }
  switch (comando)
  {
  case 'R':                     // RUN
    digitalWrite(LED1,HIGH);
    digitalWrite(LED2,LOW);
    motorRUN();
    Serial.println("Motor - RUN");
    comando = '0';
    break;
  case 'S':                     // STOP
    digitalWrite(LED1,LOW);
    digitalWrite(LED2,HIGH);
    motorSTOP();
    Serial.println("Motor - STOP");
    comando = '0';
    break;
  case 'F':                   // F SET 50%
    motorSETfreq();
    Serial.println("Motor - set F 50%");
    comando = '0';
    break;
  case '0':                   // DELAY
    Serial.print("-");
    Serial.flush();
    delay(200);
    break;
  default:                    // OTRO CARACTER
    Serial.println("Comando incorrecto :(");
    comando = '0';
    break;
  }
}
/* ============= FUNCIONES ============= */
void motorRUN(void){
  char trama[8];
  trama[0] = SLAVE_ADDR; // 0x01
  trama[1] = CMD_WRITE;  // 0x06
  trama[2] = 0x20;
  trama[3] = 0x00;
  trama[4] = 0x00;
  trama[5] = 0x01;
  trama[6] = 0x43;
  trama[7] = 0xCA;

  while (Serial1.available() > 0);
  digitalWrite(RS485_EN,HIGH);
  delayMicroseconds(365);
  int i;
  for(i=0; i < 8; i++)
  {
  Serial1.write(trama[i]);
  }
  //Serial1.flush();
  delayMicroseconds(2100);
  //Serial1.flush();
  digitalWrite(RS485_EN,LOW);
  while (Serial1.available() > 0);
  //delayMicroseconds(2000);
}
void motorSTOP(void){
  char trama[8];
  trama[0] = SLAVE_ADDR; // 0x01
  trama[1] = CMD_WRITE;  // 0x06
  trama[2] = 0x20;
  trama[3] = 0x00;
  trama[4] = 0x00;
  trama[5] = 0x06;
  trama[6] = 0x02;
  trama[7] = 0x08;

  while (Serial1.available() > 0);
  digitalWrite(RS485_EN,HIGH);
  delayMicroseconds(365);
  int i;
  for(i=0; i < 8; i++)
  {
  Serial1.write(trama[i]);
  }
  //Serial1.flush();
  delayMicroseconds(2100);
  //Serial1.flush();
  digitalWrite(RS485_EN,LOW);
  while (Serial1.available() > 0);
  //delayMicroseconds(2000);
}
void motorSETfreq(void){
  char trama[8];
  trama[0] = SLAVE_ADDR; // 0x01
  trama[1] = CMD_WRITE;  // 0x06
  trama[2] = 0x10;
  trama[3] = 0x00;
  trama[4] = 0x13;
  trama[5] = 0x88;
  trama[6] = 0x80;
  trama[7] = 0x5C;

  while (Serial1.read() != -1);
  digitalWrite(RS485_EN,HIGH);
  delayMicroseconds(365);
  int i;
  for(i=0; i < 8; i++)
  {
  Serial1.write(trama[i]);
  }
  //Serial1.flush();
  delayMicroseconds(2100);
  //Serial1.flush();
  digitalWrite(RS485_EN,LOW);
  while (Serial1.read() != -1);
  //delayMicroseconds(2000);
}