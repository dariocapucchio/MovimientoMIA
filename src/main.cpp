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
#include <EthernetENC.h>
#include <PubSubClient.h>
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
#define FOWARD_VALUE      0x0001  // Valor para el comando FOWARD
#define REVERSE_VALUE     0x0002  // Valor para el comando REVERSE
#define FOWARD_JOG_VALUE  0x0003  // Valor para el comando FOWARD JOG
#define REVERSE_JOG_VALUE 0x0004  // Valor para el comando REVERSE JOG
#define FREE_STOP_VALUE   0x0005  // Valor para el comando FREE STOP
#define STOP_VALUE        0x0006  // Valor para el comando STOP
#define JOG_STOP_VALUE    0x0008  // Valor para el comando JOG STOP
#define SPEED_PARAM_ADDR  0x1000  // Direccion para setear la velocidad

#define CLIENT_ID "MIA"

// FUNCIONES
int ModbusEscribirRegistro(char dir, uint16_t registro, uint16_t valor);
uint16_t crc16_update(uint16_t crc, uint8_t a);
void callback(char* topic, byte* payload, unsigned int length);  // Funcion para la recepcion via MQTT
void reconnect(void);

// DEFINICIONES PARA LA CONEXION ETHERNET CON ENC28J60
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE}; // Dirección MAC del módulo Ethernet
IPAddress server(163, 10, 43, 68);                 // IP del broker MQTT

EthernetClient client;
PubSubClient mqttClient(client);

// VARIABLES DE FLUJO DE PROGRAMA
char dato;
char comando;
bool flag_mqtt;
uint16_t speed;
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

  Serial.print("Apreta el boton 1 o me quedo aca ");
  while (digitalRead(BUTTON1) != LOW)
  {
    Serial.print("?");
    delay(500);
  }

  // Inicio modulo Ethernet
  Serial.print("Inicializando modulo Ethernet.... ");
  Serial.println(Ethernet.begin(mac));
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  { // Verifico la comunicacion SPI con el modulo
    Serial.println("No hay conexión con el módulo de Ethernet :(");
  }
  if (Ethernet.linkStatus() == LinkOFF)
  { // Verifico si esta el cable de Ethernet conectado
    Serial.println("No está conectado el cable de Ethernet!");
  }
  Serial.print("IP local: "); // Verifico la direccion de ip
  Serial.println(Ethernet.localIP());

  Serial.print("connecting to host...");
  while (!client.connect(server, 1883))
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" connected!");

  mqttClient.setServer(server, 1883);  // Servidor MQTT
  mqttClient.setCallback(callback);    // Callback para la recepcion via MQTT
  Serial.print("Conexion al broker MQTT = ");
  Serial.println(mqttClient.connect(CLIENT_ID));  // Conexion al broker MQTT
  if (!client.connected()) {
    reconnect();
  }
  
  mqttClient.subscribe("movimiento/motor1");

  mqttClient.loop();

  comando = '0';
  flag_mqtt = false;
  speed = 50;

  delay(1000);
}

/* ============= LOOP CORE 0 ============= */
void loop()
{
  if (Serial.available() > 0 || flag_mqtt == true)
  { // Reviso la comunicacion con la PC
    if (flag_mqtt == false){
      comando = Serial.read();
    } else {
      flag_mqtt = false;
    }
    switch (comando)
    {
    case 'R': // FOWARD
      digitalWrite(LED1_PIN, HIGH);
      Serial.print("Motor FOWARD - check: ");
      Serial.println(ModbusEscribirRegistro(SLAVE_ADDR, COMMAND_WORD_ADDR, FOWARD_VALUE));
      break;
    case 'r': // REVERSE
      digitalWrite(LED1_PIN, HIGH);
      Serial.print("Motor REVERSE - check: ");
      Serial.println(ModbusEscribirRegistro(SLAVE_ADDR, COMMAND_WORD_ADDR, REVERSE_VALUE));
      break;
    case 'J': // FOWARD JOG
      digitalWrite(LED1_PIN, HIGH);
      Serial.print("Motor FOWARD JOG - check: ");
      Serial.println(ModbusEscribirRegistro(SLAVE_ADDR, COMMAND_WORD_ADDR, FOWARD_JOG_VALUE));
      break;
    case 'j': // REVERSE JOG
      digitalWrite(LED1_PIN, HIGH);
      Serial.print("Motor REVERSE JOG - check: ");
      Serial.println(ModbusEscribirRegistro(SLAVE_ADDR, COMMAND_WORD_ADDR, REVERSE_JOG_VALUE));
      break;
    case 's': // FREE STOP (Emergencia - enclavamiento)
      digitalWrite(LED1_PIN, LOW);
      Serial.print("Motor FREE STOP - check: ");
      Serial.println(ModbusEscribirRegistro(SLAVE_ADDR, COMMAND_WORD_ADDR, FREE_STOP_VALUE));
      break;
    case 'S': // STOP (Rodado libre)
      digitalWrite(LED1_PIN, LOW);
      Serial.print("Motor STOP - check: ");
      Serial.println(ModbusEscribirRegistro(SLAVE_ADDR, COMMAND_WORD_ADDR, STOP_VALUE));
      break;
    case 'x': // STOP JOG
      digitalWrite(LED1_PIN, LOW);
      Serial.print("Motor STOP JOG - check: ");
      Serial.println(ModbusEscribirRegistro(SLAVE_ADDR, COMMAND_WORD_ADDR, JOG_STOP_VALUE));
      break;
    case 'F': // F SET SPEED
      Serial.println("Motor SET SPEED  - check:");
      Serial.println(ModbusEscribirRegistro(SLAVE_ADDR, SPEED_PARAM_ADDR, speed));
      break;
    default: // OTRO CARACTER
      Serial.println("Comando incorrecto :(");
      break;
    }
    // Serial.flush();
  }
  if (!mqttClient.connected()) {
    Serial.println("reconectando...");
    reconnect();
  }
  //Serial.print("."); // Estoy vivo
  delay(200);
  mqttClient.loop();
  // Toggle led
  (digitalRead(LED2_PIN)) ? digitalWrite(LED2_PIN, LOW) : digitalWrite(LED2_PIN, HIGH);
}
/* ============= FUNCIONES ============= */
int ModbusEscribirRegistro(char dir, uint16_t registro, uint16_t valor)
{
  char trama[8];
  int i = 0;
  int check_count = 0;
  uint16_t crc = 0xFFFF;
  trama[0] = dir;       // 0x01 - Direccion del esclavo
  trama[1] = CMD_WRITE; // 0x06 - Escribir Registro
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
  // delayMicroseconds(365);

  i = 0;
  for (i = 0; i < 8; i++)
  {
    Serial1.write(trama[i]);
  }

  // Serial1.flush();
  delayMicroseconds(2500); // 3 bytes

  digitalWrite(RS485_EN, LOW);

  delayMicroseconds(6700); // 8 bytes

  // Chequeo del dato
  i = 0;
  for (i = 0; i < 8; i++)
  {
    (Serial1.read() == trama[i]) ? check_count++ : i = 8;
  }
  return check_count;
}
/* FUNCION PARA CALCULAR EL CRC */
uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= (uint16_t)a;
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }
  return crc;
}
void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(CLIENT_ID)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
/* CALLBACK MQTT */
void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Mensaje en topico [");
  Serial.print(topic);
  Serial.print("] : ");
  String topic_str(topic);  // Topico a String
  String string_aux;        // Payload a String
  for (unsigned int i = 1; i < length; i++) {
    string_aux += (char)payload[i];
  }
  Serial.print((char)payload[0]);
  comando = (char)payload[0];
  flag_mqtt = true;
  if (comando == 'F') {
    speed = string_aux.toInt() * 100;
    Serial.print(speed);
  }
  Serial.println();
}