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
#define PWM_PIN 3
#define DELAY_MQTT 1500  // Tiempo de espera entre publicaciones en ms

#define SLAVE_ADDR    0x01          // Direccion del esclavo (variador)
#define CMD_READ      0x03          // Comando de lectura
#define CMD_WRITE     0x06          // Comando de escritura
#define COMMAND_WORD_ADDR   0x2000  // Direccion para comandos al variador
#define FOWARD_VALUE        0x0001  // Valor para el comando FOWARD
#define REVERSE_VALUE       0x0002  // Valor para el comando REVERSE
#define FOWARD_JOG_VALUE    0x0003  // Valor para el comando FOWARD JOG
#define REVERSE_JOG_VALUE   0x0004  // Valor para el comando REVERSE JOG
#define FREE_STOP_VALUE     0x0005  // Valor para el comando FREE STOP
#define STOP_VALUE          0x0006  // Valor para el comando STOP
#define JOG_STOP_VALUE      0x0008  // Valor para el comando JOG STOP
#define SPEED_PARAM_ADDR    0x1000  // Direccion para setear la velocidad
#define RUNNING_SPEED_ADDR  0x1007  // Direccion con la velocidad actual

#define CLIENT_ID "MIA_movimiento"  // ID del cliente mqtt

// FUNCIONES
int ModbusEscribirRegistro(char dir, uint16_t registro, uint16_t valor);
int ModbusLeerRegistro(char dir, uint16_t registro);
uint16_t crc16_update(uint16_t crc, uint8_t a);
void callback(char* topic, byte* payload, unsigned int length);  // Funcion para la recepcion via MQTT
void reconnect(void);
void enviar_mqtt(const char* topic, int number);

// DEFINICIONES PARA LA CONEXION ETHERNET CON ENC28J60
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE}; // Dirección MAC del módulo Ethernet
IPAddress server(163, 10, 43, 68);                 // IP del broker MQTT

EthernetClient client;
PubSubClient mqttClient(client);
long previousMillis;  // Variable para contar el tiempo entre publicaciones

// VARIABLES DE FLUJO DE PROGRAMA
char comando;             // Caracter con la accion de comando
bool flag_mqtt;           // Flag para reconocer la recepcion por mqtt
uint16_t speed;           // Velocidad (set point) al variador
int inst_speed_hz = 0;    // Velocidad instantanea en Hz
uint8_t pwm = 0;

/* ========================== SETUP CORE 0 ========================== */
void setup()
{
  Serial.begin(115200); // Comunicacion serie con la PC - USB
  Serial1.begin(38400);  // Comunicacion serie con modulo RS485

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(RS485_EN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);    // Motor parado
  digitalWrite(RS485_EN, LOW); // Recivo RS485
  // digitalWrite(RS485_EN,HIGH); // Envio RS485

  // Espero a que se aprete el boton 1 para poder verificar por puerto 
  // serie la conexion al broker mqtt. Despues se comenta
  while (digitalRead(BUTTON1) != LOW)
  {
    Serial.println("Apreta el boton 1 o me quedo aca ");
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
  // Suscripciones
  mqttClient.subscribe("movimiento/motor1");
  mqttClient.loop();

  // Inicio variables
  comando = '0';
  flag_mqtt = false;
  speed = 5000;
  previousMillis = millis();

  delay(1000);
}

/* ========================== LOOP CORE 0 ========================== */
void loop()
{
  if (Serial.available() > 0 || flag_mqtt == true)
  { // Reviso la comunicacion con la PC o la recepcion por mqtt
    // Obtengo el comando
    if (flag_mqtt == false){
      comando = Serial.read();
    } else {
      flag_mqtt = false;
    }
    // Evaluo el comando
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
  }
  
  if (!mqttClient.connected())    // Reviso comunicacion con el broker mqtt
  {
    Serial.println("reconectando...");
    reconnect();
  }

  //Serial.print("."); // Estoy vivo

  if (millis() - previousMillis > DELAY_MQTT) {  // Envio todo al broker cada DELAY_MQTT
    inst_speed_hz = ModbusLeerRegistro(SLAVE_ADDR, RUNNING_SPEED_ADDR);
    enviar_mqtt("movimiento/motor1_speed", inst_speed_hz);
    previousMillis = millis();
  }

  (pwm < 255) ? pwm++ : pwm = 0;
  analogWrite(PWM_PIN, pwm);

  delay(200);
  mqttClient.loop();
  // Toggle led - Alive test
  (digitalRead(LED2_PIN)) ? digitalWrite(LED2_PIN, LOW) : digitalWrite(LED2_PIN, HIGH);
}
/* ========================== FUNCIONES ========================== */
/* LEER UN REGISTRO MODBUS RTU */
int ModbusLeerRegistro(char dir, uint16_t registro)
{
  char trama[8], trama_recep[3];
  trama_recep[0] = 0x00;
  trama_recep[1] = 0x00;
  trama_recep[2] = 0x00;
  int i = 0;
  int resultado = 0;
  //uint16_t resultado = 0x0000;
  uint16_t crc = 0xFFFF;
  trama[0] = dir;       // 0x01 - Direccion del esclavo
  trama[1] = CMD_READ;  // 0x03 - Leer Registro
  trama[2] = highByte(registro);    // Direccion
  trama[3] = lowByte(registro);     // Direccion
  trama[4] = 0x00;                  // Cantidad de registros
  trama[5] = 0x01;                  // a leer
  for (i = 0; i < 6; i++)
  {
    crc = crc16_update(crc, trama[i]);
  }
  trama[6] = lowByte(crc);
  trama[7] = highByte(crc);

  // Envio la trama
  digitalWrite(RS485_EN, HIGH);
  i = 0;
  for (i = 0; i < 8; i++)
  {
    Serial1.write(trama[i]);
  }
  Serial1.flush();
  delayMicroseconds(2500); // 3 bytes
  digitalWrite(RS485_EN, LOW);
  delayMicroseconds(6700); // 8 bytes

  // Termina el envio de la trama

  while(Serial1.read() != 0x0B);    // Espero a que termine la trama de envio
  if (Serial1.read() == -1)         // Verifico que llego algo
  {                                 // Si no llego nada salgo, error de conexion
    Serial.println("RS485 no connection :(");
    return -1;
  }
  while(Serial1.read() != 0x03);
  trama_recep[0] = Serial1.read();  // Guardo lo que sigue
  trama_recep[1] = Serial1.read();
  trama_recep[2] = Serial1.read();

  // Esto recive todo, pa chequear nomas. Despues se comenta
  /*trama_recep[1] = Serial1.read();
  while(trama_recep[1] != 0x44)
  {
    Serial.print((int)trama_recep[1]);
    Serial.print(":");
    trama_recep[1] = Serial1.read();
  }
  Serial.println("XX");*/  // 0x44
  // Fin del chequeo

  // calculo el resultado y vuelvo
  resultado = 256 * (uint8_t)trama_recep[1] + (uint8_t)trama_recep[2];
  return resultado;
}
/* ESCRIBIR UN REGISTRO MODBUS RTU */
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

  // Envio la trama
  digitalWrite(RS485_EN, HIGH);
  i = 0;
  for (i = 0; i < 8; i++)
  {
    Serial1.write(trama[i]);
  }
  Serial1.flush();
  delayMicroseconds(2500); // 3 bytes
  digitalWrite(RS485_EN, LOW);
  delayMicroseconds(6700); // 8 bytes

  // Termina el envio de la trama

  while(Serial1.read() != trama[7]);    // Espero a que llege el CRC enviado
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
/* RECONEXION MQTT */
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
/* PUBLICO UN DATO POR MQTT */
void enviar_mqtt(const char* topic, int number)
{
  // Paso el dato a string
  char dato_mqtt[6];
  uint8_t decena, unidad, decima, centesima;
  decena = number / 1000;
  unidad = (number - decena * 1000) / 100;
  decima = (number - (decena * 1000) - (unidad * 100)) / 10;
  centesima = number - (decena * 1000) - (unidad * 100) - (decima * 10);
  dato_mqtt[0] = decena + '0';
  dato_mqtt[1] = unidad + '0';
  dato_mqtt[2] = '.';
  dato_mqtt[3] = decima + '0';
  dato_mqtt[4] = centesima + '0';
  dato_mqtt[5] = '\0';
  //Serial.println(dato_mqtt);
  mqttClient.publish(topic, dato_mqtt);   // Publico
}
/* ==================================================== */
// EOF