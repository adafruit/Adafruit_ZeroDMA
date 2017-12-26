// DMA-based I2C buffer write for M0 boards. This is a transmit-only as written operation.
// The example here is using an I2C verison of the 64x48 pixel SSD1306 OLED.
// I2C operation requires a acknowledge from the slave device, therfore
// the buffer data is only transmitted if the device is connected.
// The START condition, address byte, and a (device specific) control byte are sent manually, 
// followed by a DMA buffer write. The DMA buffer contains the pixels to be displayed on the OLED
// screen. After the DMA operation is completed, a manual STOP condition is sent to release
// the bus and finish the transaction.



#include <Wire.h>
#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"

Adafruit_ZeroDMA myDMA;
ZeroDMAstatus    stat; // DMA status codes returned by some functions


//These are the registers for the SSD1306 OLED							   
#define I2C_ADDRESS 0x3C	//This is the default SSD1306 I2C device address. 
#define I2C_WRITE 0x00
#define I2C_READ 0x01
#define DISPLAY_OFF 0xAE
#define DISPLAY_ON 0xAF
#define CONTROL_BYTE_COMMAND 0x00
#define CONTROL_BYTE_DATA 0x40
#define SET_DISPLAY_CLOCK_DIV 0xD5
#define SET_MULTIPLEX_RATIO 0xA8
#define SET_DISPLAY_OFFSET 0xD3
#define SET_DISPLAY_START_LINE 0x40
#define SET_CHARGE_PUMP 0x8D
#define SET_NORMAL_DISPLAY 0xA6
#define SET_DISPLAY_ON_RESUME_RAM 0xA4
#define SET_INVERSE_DISPLAY 0xA6
#define SET_SEG_REMAPPED_FALSE 0xA0
#define SET_SEG_REMAPPED_TRUE 0xA1
#define SET_COM_SCAN_REMAPPED_TRUE 0xC8
#define SET_COM_SCAN_REMAPPED_FALSE 0xC0
#define SET_CONTRAST 0x81
#define SET_PRE_CHARGE_PERIOD 0xD9
#define SET_VCOMH_DESELECT_LEVEL 0xDB
#define SET_MEMORY_ADDRESS_MODE 0x20
#define SET_COM_PINS	0xDA
#define HORIZONTAL_MODE 0x00
#define VERTICAL_MODE 0x01
#define PAGE_ADDRESSING_MODE 0x02
//These commands are for horizontal/vertical addressing modes only
#define SET_COLUMN_START_END_ADDRESS 0x21
#define SET_PAGE_START_END_ADDRESS 0x22
//These commands are for page addressing mode only
#define SET_PAGE_ADDRESS_PAGE_MODE 0xB0
#define SET_LN_COLUMN_ADDRESS_PAGE_MODE_MASK  0x0F
#define SET_HN_COLUMN_ADDRESS_PAGE_MODE  0x10

//64x48 pixels, 8 pixels per byte, total 64x48/8=384 bytes. 
#define DATA_LENGTH 384  

// The memory we'll be issuing to I2C:
uint8_t source_memory[DATA_LENGTH];

volatile bool transfer_is_done = false; // Done yet?

//debug switch used, for user selected ouput
#define SERIAL_OBJECT SerialUSB
//#define SERIAL_OBJECT Serial

// Callback for end-of-DMA-transfer
void dma_callback(Adafruit_ZeroDMA *dma) {
	transfer_is_done = true;
}

void setup() {
	uint32_t t;
	pinMode(LED_BUILTIN, OUTPUT);   // Onboard LED can be used for precise
	digitalWrite(LED_BUILTIN, LOW); // benchmarking with an oscilloscope


	SERIAL_OBJECT.begin(115200);
	while (!SERIAL_OBJECT);                 // Wait for SerialUSB monitor before continuing

	SERIAL_OBJECT.println("DMA test: I2C data out");
	
	Wire.begin();
	
	initOLED();


	SERIAL_OBJECT.println("Configuring DMA trigger");
	
	//default I2C for arduino zero is on SERCOM3
	myDMA.setTrigger(SERCOM3_DMAC_ID_TX);
	myDMA.setAction(DMA_TRIGGER_ACTON_BEAT); 

	SERIAL_OBJECT.print("Allocating DMA channel...");
	stat = myDMA.allocate();
	myDMA.printStatus(stat);

	SERIAL_OBJECT.println("Setting up transfer");

	myDMA.addDescriptor(
		source_memory,                    // move data from here
		(void *)(&SERCOM3->I2CM.DATA.reg),// to here (M0)
		DATA_LENGTH,                      // this many...
		DMA_BEAT_SIZE_BYTE,               // bytes/hword/words
		true,                             // increment source addr?
		false);                           // increment dest addr?

	SERIAL_OBJECT.println("Adding callback");
	// register_callback() can optionally take a second argument
	// (callback type), default is DMA_CALLBACK_TRANSFER_DONE
	myDMA.setCallback(dma_callback);


	// Fill the source buffer with intended pattern
	for (uint32_t i = 0; i < DATA_LENGTH; i++) source_memory[i] = i%64 ;


	SERIAL_OBJECT.println("Starting transfer job");
	t = micros();
	digitalWrite(LED_BUILTIN, HIGH);

	/*
	SERCOM3->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_SMEN;		//Smart mode enable, not required using adafruit zero DMA library
	SERCOM3->I2CM.ADDR.reg |= SERCOM_I2CM_ADDR_LENEN;		//Automatic transfer length enable, not required using adafruit zero DMA library
	SERCOM3->I2CM.ADDR.reg |= SERCOM_I2CM_ADDR_LEN(255);	//Transaction lenth, 0 to 255, not required using adafruit zero DMA library
	*/

	//Starting a I2C transaction:
	SERCOM3->I2CM.ADDR.reg |= SERCOM_I2CM_ADDR_ADDR(I2C_ADDRESS<<1 | I2C_WRITE);  //START condition and address(7 bit address + 1 r/w bit) is sent upon register write. 
	
	while (!(SERCOM3->I2CS.STATUS.reg & SERCOM_I2CM_STATUS_CLKHOLD)); //Wait until previous transmittion completed 

	SERCOM3->I2CM.DATA.reg = (CONTROL_BYTE_DATA);	//Send a control byte. This byte is to tell SSD1036 that following bytes are screen data bytes. Accessing DATA register auto-triggers read or write operation

	while (!(SERCOM3->I2CS.STATUS.reg & SERCOM_I2CM_STATUS_CLKHOLD)); //Wait until previous transmittion completed 
	
	// Because we've configured a peripheral trigger (I2C), there's
	// no need to manually trigger transfer, it starts up on its own.
	stat = myDMA.startJob();

	
	// Your code could do other things here while I2C write happens!
	while (!transfer_is_done); // Chill until DMA transfer completes

	digitalWrite(LED_BUILTIN, LOW);
	t = micros() - t; // Elapsed time

	while (! (SERCOM3->I2CS.STATUS.reg & SERCOM_I2CM_STATUS_CLKHOLD)); //Wait until last byte transmittion completed

	SERCOM3->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(0x3); //Generate a STOP condition . We are done with this transaction
	//Ending the I2C transaction

	myDMA.printStatus(stat); // Results of start_transfer_job()

	SERIAL_OBJECT.print("Done! ");
	SERIAL_OBJECT.print(t);
	SERIAL_OBJECT.println(" microseconds");
}

void loop() { }


void initOLED()
{
	//initialization sequency for SSD1306, 64x48 pixels
	sendCommand(DISPLAY_OFF);

	sendCommand(SET_DISPLAY_CLOCK_DIV);
	sendCommand(0x80);//default value

	sendCommand(SET_MULTIPLEX_RATIO);
	sendCommand(0x2F);

	sendCommand(SET_DISPLAY_OFFSET);
	sendCommand(0x00);//default value

	sendCommand(SET_DISPLAY_START_LINE | 0x00); 

	sendCommand(SET_CHARGE_PUMP); //enable charge pump 
	sendCommand(0x14); //Vcc generated by internal DC/DC circuit

	sendCommand(SET_NORMAL_DISPLAY); //in case inverse display is used

	sendCommand(SET_DISPLAY_ON_RESUME_RAM);

	sendCommand(SET_SEG_REMAPPED_TRUE); //column address 127 is mapped to SEG0, this changes the column direction

	sendCommand(SET_COM_SCAN_REMAPPED_TRUE);//This changes the page display direction

	sendCommand(SET_COM_PINS);
	sendCommand(0x12);

	sendCommand(SET_CONTRAST);
	sendCommand(255);

	sendCommand(SET_PRE_CHARGE_PERIOD);
	sendCommand(0xF1);

	sendCommand(SET_VCOMH_DESELECT_LEVEL);	
	sendCommand(0x40);

	//Use horizontal mode. 64x48  memory is located near the upper center of the full 128x64 memory space
	sendCommand(SET_MEMORY_ADDRESS_MODE);
	sendCommand(HORIZONTAL_MODE);
	sendCommand(SET_PAGE_START_END_ADDRESS);
	sendCommand(0x00);
	sendCommand(0x05);
	sendCommand(SET_COLUMN_START_END_ADDRESS);
	sendCommand(0x20);
	sendCommand(0x5F);
	
	fillHorizontal(0x00);//clear display ram using normal, non-DMA method

	sendCommand(DISPLAY_ON);
	
	SERIAL_OBJECT.println("OLED initialized");
}

void sendCommand(uint8_t command)
{
	//send a command to SSD1306
	Wire.beginTransmission(I2C_ADDRESS);
	Wire.write(CONTROL_BYTE_COMMAND);
	Wire.write(command);
	Wire.endTransmission();
}

void fillHorizontal(uint8_t value)
{
	for (uint8_t page = 0; page < 6; page++)
	{
		//send data in batches of 32, due to the buffer size in the wire library is 64
		for (uint8_t batches = 0; batches < 2; batches++)
		{
			Wire.beginTransmission(I2C_ADDRESS);
			Wire.write(CONTROL_BYTE_DATA);
			for (uint8_t column = 0; column < 32; column++)
			{
				Wire.write(value);
			}
			Wire.endTransmission();
		}
	}
}
