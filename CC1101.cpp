/*
 * Author: Klusjesman, modified bij supersjimmie for Arduino/ESP8266
 */

#include "CC1101.h"

// default constructor
CC1101::CC1101()
{
#ifdef ESP32
	// Configure the SPI bus once. CC1101 is the sole device on this bus,
	// so we never call beginTransaction/endTransaction — those hold a
	// portENTER_CRITICAL (disables all IRQs incl. FreeRTOS tick) across
	// the entire transaction window, which starves the tick ISR during
	// spi_waitMiso() and triggers the interrupt WDT on single-core ESP32-C3.
	SPI.begin(CC1101_SCK_PIN, CC1101_MISO_PIN, CC1101_MOSI_PIN, -1);
	SPI.setFrequency(4000000);
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	pinMode(CC1101_CSN_PIN, OUTPUT);
	digitalWrite(CC1101_CSN_PIN, HIGH);
#else
	SPI.begin();
#ifdef ESP8266
	pinMode(SS, OUTPUT);
#endif
#endif
} //CC1101

// default destructor
CC1101::~CC1101()
{
} //~CC1101

/***********************/
// SPI helper functions select() and deselect()
inline void CC1101::select(void) {
#ifdef ESP32
	digitalWrite(CC1101_CSN_PIN, LOW);
#else
	digitalWrite(SS, LOW);
#endif
}

inline void CC1101::deselect(void) {
#ifdef ESP32
	digitalWrite(CC1101_CSN_PIN, HIGH);
#else
	digitalWrite(SS, HIGH);
#endif
}

void CC1101::spi_waitMiso()
{
#ifdef ESP32
	// 200 µs busy-wait — covers CC1101 chip-ready max (150 µs per datasheet).
	// Does NOT disable interrupts, so the FreeRTOS tick can fire normally.
	// vTaskDelay() was removed: it prevented the idle task from running during
	// WiFi scanning, causing the interrupt watchdog to fire.
	uint32_t start = micros();
	while (digitalRead(CC1101_MISO_PIN) == HIGH) {
		if (micros() - start > 200) break;
	}
#else
	while(digitalRead(MISO) == HIGH) yield();
#endif
}

void CC1101::init()
{
	reset();
}

void CC1101::reset()
{
	deselect();
	delayMicroseconds(5);
	select();
	delayMicroseconds(10);
	deselect();
	delayMicroseconds(45);
	select();

	spi_waitMiso();
	SPI.transfer(CC1101_SRES);
	delay(10);
	spi_waitMiso();
	deselect();
}

uint8_t CC1101::writeCommand(uint8_t command) 
{
	uint8_t result;
	
	select();
	spi_waitMiso();
	result = SPI.transfer(command);
	deselect();
	
	return result;
}

void CC1101::writeRegister(uint8_t address, uint8_t data) 
{
	select();
	spi_waitMiso();
	SPI.transfer(address);
	SPI.transfer(data);
	deselect();
}

uint8_t CC1101::readRegister(uint8_t address)
{
	uint8_t val;
  
	select();
	spi_waitMiso();
	SPI.transfer(address);
	val = SPI.transfer(0);
	deselect();
  
	return val;
}

uint8_t CC1101::readRegisterMedian3(uint8_t address)
{
  uint8_t val, val1, val2, val3;

  select();
  spi_waitMiso();
  SPI.transfer(address);
  val1 = SPI.transfer(0);
  SPI.transfer(address);
  val2 = SPI.transfer(0);
  SPI.transfer(address);
  val3 = SPI.transfer(0);
  deselect();
  // reverse sort (largest in val1) because this is te expected order for TX_BUFFER
  if (val3 > val2) {val = val3; val3 = val2; val2 = val; } //Swap(val3,val2)
  if (val2 > val1) {val = val2; val2 = val1, val1 = val; } //Swap(val2,val1)
  if (val3 > val2) {val = val3; val3 = val2, val2 = val; } //Swap(val3,val2)
  
  return val2;
}

/* Known SPI/26MHz synchronization bug (see CC1101 errata)
This issue affects the following registers: SPI status byte (fields STATE and FIFO_BYTES_AVAILABLE), 
FREQEST or RSSI while the receiver is active, MARCSTATE at any time other than an IDLE radio state, 
RXBYTES when receiving or TXBYTES when transmitting, and WORTIME1/WORTIME0 at any time.*/
//uint8_t CC1101::readRegisterWithSyncProblem(uint8_t address, uint8_t registerType)
uint8_t /* ICACHE_RAM_ATTR */ CC1101::readRegisterWithSyncProblem(uint8_t address, uint8_t registerType)
{
	uint8_t value1, value2;
#ifdef ESP32
	uint8_t tries = 10; // prevent infinite loop if CC1101 is unresponsive
#endif
	
	value1 = readRegister(address | registerType);
	
	//if two consecutive reads gives us the same result then we know we are ok
	do 
	{
		value2 = value1;
		value1 = readRegister(address | registerType);
	} 
#ifdef ESP32
	while (value1 != value2 && --tries > 0);
#else
	while (value1 != value2);
#endif
	
	return value1;
}

//registerType = CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER
uint8_t CC1101::readRegister(uint8_t address, uint8_t registerType)
{
	switch (address)
	{
		case CC1101_FREQEST:
		case CC1101_MARCSTATE:
		case CC1101_RXBYTES:
		case CC1101_TXBYTES:
		case CC1101_WORTIME1:
		case CC1101_WORTIME0:	
			return readRegisterWithSyncProblem(address, registerType);	
			
		default:
			return readRegister(address | registerType);
	}
}

void CC1101::writeBurstRegister(uint8_t address, uint8_t* data, uint8_t length)
{
	uint8_t i;

	select();
	spi_waitMiso();
	SPI.transfer(address | CC1101_WRITE_BURST);
	for (i = 0; i < length; i++) {
		SPI.transfer(data[i]);
	}
	deselect();
}

void CC1101::readBurstRegister(uint8_t* buffer, uint8_t address, uint8_t length)
{
	uint8_t i;
	
	select();
	spi_waitMiso();
	SPI.transfer(address | CC1101_READ_BURST);
	
	for (i = 0; i < length; i++) {
		buffer[i] = SPI.transfer(0x00);
	}
	
	deselect();
}

//wait for fixed length in rx fifo
uint8_t CC1101::receiveData(CC1101Packet* packet, uint8_t length)
{
	uint8_t rxBytes = readRegisterWithSyncProblem(CC1101_RXBYTES, CC1101_STATUS_REGISTER);
	rxBytes = rxBytes & CC1101_BITS_RX_BYTES_IN_FIFO;
	
	//check for rx fifo overflow
	if ((readRegisterWithSyncProblem(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & CC1101_BITS_MARCSTATE) == CC1101_MARCSTATE_RXFIFO_OVERFLOW)
	{
		writeCommand(CC1101_SIDLE);	//idle
		writeCommand(CC1101_SFRX); //flush RX buffer
		writeCommand(CC1101_SRX); //switch to RX state	
	}
	else if (rxBytes == length)
	{
		readBurstRegister(packet->data, CC1101_RXFIFO, rxBytes);

		//continue RX
		writeCommand(CC1101_SIDLE);	//idle		
		writeCommand(CC1101_SFRX); //flush RX buffer
		writeCommand(CC1101_SRX); //switch to RX state	
		
		packet->length = rxBytes;				
	}
	else
	{
		//empty fifo
		packet->length = 0;
		writeCommand(CC1101_SIDLE); //idle    
		writeCommand(CC1101_SFRX); //flush RX buffer
		writeCommand(CC1101_SRX); //switch to RX state
#ifdef ESP32
		// On ESP32/FreeRTOS the idle task only runs when all other tasks block.
		// Without this delay the main loop polls at full speed and the idle task
		// never runs, starving the interrupt watchdog (IWDT fires at 300 ms).
		// delay(1) = vTaskDelay(1 tick) — suspends this task so idle can run.
		delay(1);
#endif
	}

	return packet->length;
}

//This function is able to send packets bigger then the FIFO size.
void CC1101::sendData(CC1101Packet *packet)
{
	uint8_t index = 0;
	uint8_t txStatus, MarcState;
	uint8_t length;
	
	writeCommand(CC1101_SIDLE);		//idle

	txStatus = readRegisterWithSyncProblem(CC1101_TXBYTES, CC1101_STATUS_REGISTER);
		
	//clear TX fifo if needed
	if (txStatus & CC1101_BITS_TX_FIFO_UNDERFLOW)
	{
		writeCommand(CC1101_SIDLE);	//idle
		writeCommand(CC1101_SFTX);	//flush TX buffer
	}	
	
	writeCommand(CC1101_SIDLE);		//idle	
	
	//determine how many bytes to send
	length = (packet->length <= CC1101_DATA_LEN ? packet->length : CC1101_DATA_LEN);
	
	writeBurstRegister(CC1101_TXFIFO, packet->data, length);

	writeCommand(CC1101_SIDLE);
	//start sending packet
	writeCommand(CC1101_STX);		

	//continue sending when packet is bigger than 64 bytes
	if (packet->length > CC1101_DATA_LEN)
	{
		index += length;
		
		//loop until all bytes are transmitted
		while (index < packet->length)
		{
			//check if there is free space in the fifo
			while ((txStatus = (readRegisterMedian3(CC1101_TXBYTES | CC1101_STATUS_REGISTER) & CC1101_BITS_RX_BYTES_IN_FIFO)) > (CC1101_DATA_LEN - 2)) {
#ifdef ESP32
				delay(1); // vTaskDelay lets idle task run, resetting the IWDT
#endif
			}
			
			//calculate how many bytes we can send
			length = (CC1101_DATA_LEN - txStatus);
			length = ((packet->length - index) < length ? (packet->length - index) : length);
			
			//send some more bytes
			for (int i=0; i<length; i++)
				writeRegister(CC1101_TXFIFO, packet->data[index+i]);
			
			index += length;			
		}
	}

	//wait until transmission is finished (TXOFF_MODE is expected to be set to 0/IDLE or TXFIFO_UNDERFLOW)
	do
	{
		MarcState = (readRegisterWithSyncProblem(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & CC1101_BITS_MARCSTATE);
//		if (MarcState == CC1101_MARCSTATE_TXFIFO_UNDERFLOW) Serial.print(F("TXFIFO_UNDERFLOW occured in sendData() \n"));
#ifdef ESP32
		delay(1); // vTaskDelay lets idle task run, resetting the IWDT
#endif
	}
  	while((MarcState != CC1101_MARCSTATE_IDLE) && (MarcState != CC1101_MARCSTATE_TXFIFO_UNDERFLOW));
}
