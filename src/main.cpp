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
#define LED1_PIN 14
#define LED2_PIN 15
#define RS485_EN 2

#define SLAVE_ADDR 0x01           // Direccion del esclavo (variador)
#define CMD_READ 0x03             // Comando de lectura
#define CMD_WRITE 0x06            // Comando de escritura
#define COMMAND_WORD_ADDR 0x2000  // Direccion para comandos al variador
#define RUN_VALUE 0x0001          // Valor para el comando RUN
#define STOP_VALUE 0x0006         // Valor para el comando STOP
#define SPEED_PARAM_ADDR 0x1000   // Direccion para setear la velocidad

// FUNCIONES
int ModbusEscribirRegistro(char dir, uint16_t registro, uint16_t valor);
uint16_t crc16_update(uint16_t crc, uint8_t a);

// VARIABLES DE FLUJO DE PROGRAMA
char dato;
char comando;
// byte trama[10];

/* ============= SETUP CORE 0 ============= */
void setup()
{
  Serial.begin(115200); // Comunicacion serie con la PC
  Serial1.begin(9600);  // Comunicacion serie con modulo RS485

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(RS485_EN, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);    // Motor parado
  digitalWrite(RS485_EN, LOW); // Recivo RS485
  // digitalWrite(RS485_EN,HIGH); // Envio RS485

  comando = '0';

  delay(1000);
}

/* ============= LOOP CORE 0 ============= */
void loop()
{ 
  if (Serial.available() > 0)
  { // Reviso la comunicacion con la PC
    comando = Serial.read();
    switch (comando)
    {
    case 'R':           // RUN
      digitalWrite(LED1_PIN, HIGH);
      Serial.print("Motor RUN - check: ");
      Serial.println(ModbusEscribirRegistro(SLAVE_ADDR,COMMAND_WORD_ADDR,RUN_VALUE));
      break;
    case 'S':           // STOP
      digitalWrite(LED1_PIN, LOW);
      Serial.print("Motor STOP - check: ");
      Serial.println(ModbusEscribirRegistro(SLAVE_ADDR,COMMAND_WORD_ADDR,STOP_VALUE));
      break;
    case 'F':           // F SET 50%
      Serial.println("Motor SET F 50% - check:");
      Serial.println(ModbusEscribirRegistro(SLAVE_ADDR,SPEED_PARAM_ADDR,0x1388));
      break;
    default:            // OTRO CARACTER
      Serial.println("Comando incorrecto :(");
      break;
    }
    //Serial.flush();
  }
  Serial.print(".");    // Estoy vivo
  delay(200);
  // Toggle led
  (digitalRead(LED2_PIN)) ? digitalWrite(LED2_PIN, LOW) : digitalWrite(LED2_PIN, HIGH);
}
/* ============= FUNCIONES ============= */
int ModbusEscribirRegistro(char dir, uint16_t registro, uint16_t valor) {
  char trama[8];
  int i = 0;
  int check_count = 0;
  uint16_t crc = 0xFFFF;
  trama[0] = dir;         // 0x01 - Direccion del esclavo
  trama[1] = CMD_WRITE;   // 0x06 - Escribir Registro
  trama[2] = highByte(registro);
  trama[3] = lowByte(registro);
  trama[4] = highByte(valor);
  trama[5] = lowByte(valor);
  for (i = 0; i < 6; i++)
  {
    crc = crc16_update(crc, trama[i]);
  }
  trama[6] = lowByte(crc);
  trama[7] = highByte(crc);

  digitalWrite(RS485_EN, HIGH);
  //delayMicroseconds(365);
  
  i = 0;
  for (i = 0; i < 8; i++)
  {
    Serial1.write(trama[i]);
  }

  //Serial1.flush();
  delayMicroseconds(2500);  // 3 bytes

  digitalWrite(RS485_EN, LOW);

  delayMicroseconds(6700);  // 8 bytes

  // Chequeo del dato
  i = 0;
  for (i = 0; i < 8; i++)
  {
    (Serial1.read() == trama[i]) ? check_count++ : i=8;
  }
  return check_count;
}
/* FUNCION PARA CALCULAR EL CRC */
uint16_t crc16_update(uint16_t crc, uint8_t a) {
	int i;
	crc ^= (uint16_t)a;
	for (i = 0; i < 8; ++i) {
		if (crc & 1)
			crc = (crc >> 1) ^ 0xA001;
		else
			crc = (crc >> 1);
	}
	return crc;
}