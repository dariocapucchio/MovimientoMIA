#include <Arduino.h>

// DS18B20
#define TEMP_12_BIT 0x7F    // Valor para configurar la conversion a 12 bit
#define RES_12_BIT 0.0625   // Resolucion de la temperatura en ºC para 12 bit
#define CONVERSION_TIMEOUT 750  // Maximo tiempo para la conversion en ms

// Posiciones de memoria
#define TEMP_LSB 0          // Posicion del LSB en la memoria
#define TEMP_HSB 1          // Posicion del HSB en la memoria

// Comandos ROM
#define SEARCH_ROM    0xF0  // Pedir la ADDR cuando hay varios dispositivos
#define READ_ROM      0x33  // Pide la ADDR cuando solo hay un dispositivo
#define MATCH_ROM     0x55  // Seleccionar dispositivo
#define SKIP_ROM      0xCC  // Eviar comando a todos los dispositivos
#define ALARM_ROM     0xEC  // Consulta condicion de alarma

// Comandos de funcion
#define CONVERSION    0x44  // Iniciar conversion de temperatura
#define WRITE_SCRATCH 0x4E  // Escribir la memoria scratchpad
#define READ_SCRATCH  0xBE  // Leer la memoria scrtchpad
#define COPY_SCRATCH  0x48  // Copia scratchpad a la eeprom
#define REALL_EEPROM  0xB8  // Copia la eeprom ala scratchpad
#define READ_POWER    0xB4  // Verifica si hay alimentacion parasita

// Funciones
uint8_t owBegin(uint8_t pin);
void owWriteBit(uint8_t dato, uint8_t pin);
uint8_t owReadBit(uint8_t pin);
void owWriteByte(uint8_t dato, uint8_t pin);
void owWriteBytes(const uint8_t *buf, uint16_t count, uint8_t pin);
uint8_t owReadByte(uint8_t pin);
void owReadBytes(uint8_t *buf, uint16_t count, uint8_t pin);
bool ow_read(uint8_t pin);
/* ============================================================================= */
// EOF