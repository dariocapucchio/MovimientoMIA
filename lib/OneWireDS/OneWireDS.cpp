#include <Arduino.h>
#include "OneWireDS.h"

/**
 * @brief Inicio de la comunicacion one wire
 * 
 * @param pin : pin de conexion de los datos (DQ)
 * @return 1 si se detecta un sensor conecatado, 0 si no
*/
uint8_t owBegin(uint8_t pin)
{
  uint8_t respuesta;
  //uint8_t time_out = 20;
  pinMode(pin, OUTPUT);           // Tomo control del canal
  digitalWrite(pin, HIGH);        // Pongo el nivel alto
  delayMicroseconds(50);          // Espero
  digitalWrite(pin, LOW);         // Pulso de nivel bajo
  delayMicroseconds(480);         // por 480us
  digitalWrite(pin, HIGH);        // Pulso de nivel alto
                                  // Respuesta del sensor
  pinMode(pin, INPUT);            // Le doy control del canal al sensor
  /*while (digitalRead(pin))        // Espera a que el sensor ponga el nivel bajo
  {
     delayMicroseconds(10);
     if (--time_out == 0) return 0;   // Si no responde salgo
  }*/ 
  delayMicroseconds(70);          // Espero a que el sensor ponga el nivel bajo
  if(digitalRead(pin)){           // Si sigue en 1
    respuesta = 0;                // Salgo y retorno un cero
  } else {                        // Si se puso en cero
    delayMicroseconds(410);       // Espero a que el sensor ponga el nivel alto
    (digitalRead(pin)) ? respuesta = 1 : respuesta = 0;   // retorno el valor de nivel
  }
  return respuesta;
}

/**
 * @brief Envia un bit por one wire
 * 
 * @param dato : dato (bit) a enviar
 * @param pin : pin de la comunicacion one wire
*/
void owWriteBit(uint8_t dato, uint8_t pin)
{
  pinMode(pin, OUTPUT);           // Tomo control del canal
  digitalWrite(pin, LOW);       // Pongo el nivel bajo
  delayMicroseconds(3);
  if (dato & 1)
  {                               // Envio un uno
    digitalWrite(pin, HIGH);      // Pulso de nivel alto
    delayMicroseconds(55);
  } else {                        // Envio un cero
    delayMicroseconds(60);
    digitalWrite(pin, HIGH);      // Pulso de nivel alto
    //delayMicroseconds(5);
  }
}

/**
 * @brief Lee un bit por one wire
 * 
 * @param pin : pin de la comunicacion one wire
 * @return 
*/
uint8_t owReadBit(uint8_t pin)
{
  uint8_t respuesta;
  pinMode(pin, OUTPUT);         // Tomo control del canal
  digitalWrite(pin, LOW);       // Pongo el nivel bajo
  delayMicroseconds(3);
  pinMode(pin, INPUT);          // Le doy control del canal al sensor
  delayMicroseconds(3);
  respuesta = digitalRead(pin); // Guardo el valor del bus
  delayMicroseconds(45);
  return respuesta;
}


bool ow_read(uint8_t pin){
  bool respuesta;
  pinMode(pin, OUTPUT);         // Tomo control del canal
  digitalWrite(pin, LOW);       // Pongo el nivel bajo
  delayMicroseconds(3);
  pinMode(pin, INPUT);          // Le doy control del canal al sensor
  delayMicroseconds(3);
  respuesta = digitalRead(pin); // Guardo el valor del bus
  delayMicroseconds(45);
  return respuesta;
}

//
//
//
//
void owWriteByte(uint8_t dato, uint8_t pin)
{
  /*uint8_t mask;
  for(mask = 0x01; mask; mask <<= 1)
  {
    owWriteBit((mask & dato),pin);
  }*/
  for (uint8_t i = 0; i < 8; i++)
  {
    owWriteBit(dato >> i,pin);
    //delayMicroseconds(60);
  }
}

//
//
//
//
void owWriteBytes(const uint8_t *buf, uint16_t count, uint8_t pin)
{
  for (uint16_t i = 0; i < count; i++)
  {
    owWriteByte(buf[i],pin);
  }
}

//
//
//
//
uint8_t owReadByte(uint8_t pin)
{
  /*uint8_t mask;
  uint8_t resultado = 0;
  for (mask = 0x01; mask; mask <<= 1)
  {
    if(owReadBit(pin)) resultado |= mask;
  }
  return resultado;*/
  uint8_t answer = 0x00;
	int i;
	for (i = 0; i < 8; i++) {
		answer = answer >> 1;// shift over to make room for the next bit
		if (ow_read(pin))
			answer = (uint8_t) (answer | 0x80);// if the data port is high, make this bit a 1
	}
	return answer;
}

//
//
//
//
void owReadBytes(uint8_t *buf, uint16_t count, uint8_t pin)
{
  for (uint16_t i = 0; i < count; i++)
  {
    buf[i] = owReadByte(pin);
  }
}

/**
 * @brief Calculo del CRC byte a byte
 * 
 * @param crc : crc del byte anterior. Tiene que empezar en 0x00
 * @param byte : byte para el calculo del crc
 * @return byte con el resultado del crc
*/
uint8_t crc_byte(uint8_t crc, uint8_t byte) {
	int j;
	for (j = 0; j < 8; j++) {
		if ((byte & 0x01) ^ (crc & 0x01)) {
			// DATA ^ LSB CRC = 1
			crc = crc >> 1;
			// Set the MSB to 1
			crc = (uint8_t) (crc | 0x80);
			// Check bit 3
			if (crc & 0x04) {
				crc = (uint8_t) (crc & 0xFB);// Bit 3 is set, so clear it
			} else {
				crc = (uint8_t) (crc | 0x04);// Bit 3 is clear, so set it
			}
			// Check bit 4
			if (crc & 0x08) {
				crc = (uint8_t) (crc & 0xF7);// Bit 4 is set, so clear it
			} else {
				crc = (uint8_t) (crc | 0x08);// Bit 4 is clear, so set it
			}
		} else {
			// DATA ^ LSB CRC = 0
			crc = crc >> 1;
			// clear MSB
			crc = (uint8_t) (crc & 0x7F);
			// No need to check bits, with DATA ^ LSB CRC = 0, they will remain unchanged
		}
		byte = byte >> 1;
	}
	return crc;
}
/* ============================================================================= */
// EOF