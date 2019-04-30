/**
 * 
 * SAOGenie
 * Shitty add-on hacking tool
 * Rob Rehr
 * @mediumrehr
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <asf.h>
#include <stdio.h>
#include <string.h>

void i2c_read_request_callback(struct i2c_slave_module *const module);
void i2c_slave_read_complete_callback(struct i2c_slave_module *const module);
void i2c_write_request_callback(struct i2c_slave_module *const module);
void i2c_slave_write_complete_callback(struct i2c_slave_module *const module);
void usart_read_callback(struct usart_module *const usart_module);
void usart_write_callback(struct usart_module *const usart_module);
void i2c_write_complete_callback(struct i2c_master_module *const module);

void configure_i2c_slave(void);
void configure_i2c_slave_callbacks(void);
void configure_i2c_master(void);
void configure_i2c_master_callbacks(void);
void configure_usart(void);
void configure_usart_callbacks(void);

void set_genie_mode(uint8_t new_mode);

// i2c packets
#define PACKET_BUFFER_SIZE 50
static uint8_t dest_addr = 0;
static struct i2c_slave_packet packet_in;
struct i2c_master_packet wr_packet_out;
struct i2c_master_packet rd_packet_out;

#define DATA_LENGTH 50
static uint8_t write_buffer_in[DATA_LENGTH];
static uint8_t read_buffer_in [DATA_LENGTH];

static uint8_t write_buffer_out[DATA_LENGTH];
static uint8_t read_buffer_out[DATA_LENGTH];

// SAO Genie mode definitions
#define GENIE_MODE_OFF 			0
#define GENIE_MODE_PASSTHROUGH 	1
#define GENIE_MODE_REDIRECT		2 // not implememnted
#define GENIE_MODE_INJECT		3 // not implemented

uint8_t genie_mode = GENIE_MODE_OFF;

// Init device instance
struct i2c_slave_module i2c_slave_instance;
struct i2c_master_module i2c_master_instance;
struct usart_module usart_instance;

#define MAX_RX_BUFFER_LENGTH   1
volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];

void i2c_read_request_callback(struct i2c_slave_module *const module) {
	// Init i2c packet
	packet_in.data_length = DATA_LENGTH;
	packet_in.data        = write_buffer_in;

	// Write buffer to master
	i2c_slave_write_packet_job(module, &packet_in);
}

void i2c_write_request_callback(struct i2c_slave_module *const module) {	
	// init i2c packet
	packet_in.data_length = DATA_LENGTH;
	packet_in.data        = read_buffer_in;
	
	// store intended destination for packet
	dest_addr = module->hw->I2CS.DATA.reg >> 1;

	// read buffer from master
	i2c_slave_read_packet_job(module, &packet_in);
}

void i2c_slave_write_complete_callback(struct i2c_slave_module *const module) {
	uint8_t temp[] = "test";
	usart_write_buffer_wait(&usart_instance, temp, sizeof(temp));
}

void i2c_slave_read_complete_callback(struct i2c_slave_module *const module) {
	// make sure SAO Genie mode is not set to off
	if (genie_mode == GENIE_MODE_PASSTHROUGH) {
		// turn i2c LED on to show activity
		port_pin_set_output_level(LEDI2C_PIN, true);
		
		// copy serial data locally to pass along
		uint8_t genie_dest_addr = dest_addr;
		uint8_t bytes_received = module->buffer_received;
		
		// copy destination and data to output packet for master to send
		wr_packet_out.address	  = genie_dest_addr;
		wr_packet_out.data_length = bytes_received;
		wr_packet_out.data        = &read_buffer_in[0];

		// send output packet out master
		i2c_master_write_packet_wait(&i2c_master_instance, &wr_packet_out);

		// print i2c direction and addresses
		char dirStr[29] = {0};
		snprintf(dirStr, sizeof(dirStr), "| Host > Genie > 0x%02X       ", genie_dest_addr);
		usart_write_buffer_wait(&usart_instance, dirStr, sizeof(dirStr));

		// print i2c data captured to uart
		usart_write_buffer_wait(&usart_instance, "| ", sizeof("| "));
		uint8_t i = 0;
		while(read_buffer_in[i] != 0) {
			uint8_t tempStr[4] = {0};
			snprintf(tempStr, sizeof(tempStr), "%02X ", read_buffer_in[i]);
			usart_write_buffer_wait(&usart_instance, tempStr, sizeof(tempStr));
			i++;
		}
		usart_write_buffer_wait(&usart_instance, "\n", sizeof("\n"));
		
		// toggle LED to show activity
		port_pin_set_output_level(LEDI2C_PIN, false);
	}
}

void i2c_write_complete_callback(struct i2c_master_module *const module) {
	// initiate new packet read
	i2c_master_read_packet_job(&i2c_master_instance,&rd_packet_out);
}

void usart_read_callback(struct usart_module *const usart_module) {
	// no user input currently
	// major to do item
}

void usart_write_callback(struct usart_module *const usart_module) {
	
}

void configure_i2c_slave(void) {
	// initialize config for i2c slave
	struct i2c_slave_config config_i2c_slave;
	i2c_slave_get_config_defaults(&config_i2c_slave);

	// set slave to respond to any address between 0x00 and 0xFF
	config_i2c_slave.address	  = 0xFF; // upper i2c address range
	config_i2c_slave.address_mask = 0x00; // lower i2c address range
	config_i2c_slave.address_mode = I2C_SLAVE_ADDRESS_MODE_RANGE;
	config_i2c_slave.pinmux_pad0  = PINMUX_PA22C_SERCOM3_PAD0;
	config_i2c_slave.pinmux_pad1  = PINMUX_PA23C_SERCOM3_PAD1;

	// initialize and enable i2c slave with config
	while(i2c_slave_init(&i2c_slave_instance, SERCOM3, &config_i2c_slave) != STATUS_OK);
	i2c_slave_enable(&i2c_slave_instance);
}

void configure_i2c_slave_callbacks(void) {
	// Register and enable i2c slave callback functions
	i2c_slave_register_callback(&i2c_slave_instance, i2c_read_request_callback,
		I2C_SLAVE_CALLBACK_READ_REQUEST);
	i2c_slave_register_callback(&i2c_slave_instance, i2c_slave_read_complete_callback,
		I2C_SLAVE_CALLBACK_READ_COMPLETE);
	i2c_slave_register_callback(&i2c_slave_instance, i2c_write_request_callback,
		I2C_SLAVE_CALLBACK_WRITE_REQUEST);
	i2c_slave_register_callback(&i2c_slave_instance, i2c_slave_write_complete_callback,
		I2C_SLAVE_CALLBACK_WRITE_COMPLETE);

	i2c_slave_enable_callback(&i2c_slave_instance, I2C_SLAVE_CALLBACK_READ_REQUEST);
	i2c_slave_enable_callback(&i2c_slave_instance, I2C_SLAVE_CALLBACK_READ_COMPLETE);
	i2c_slave_enable_callback(&i2c_slave_instance, I2C_SLAVE_CALLBACK_WRITE_REQUEST);
	i2c_slave_enable_callback(&i2c_slave_instance, I2C_SLAVE_CALLBACK_WRITE_COMPLETE);
}

void configure_usart(void) {
	// initialize config for uart
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	// set uart baudrate and uart pins
	config_usart.baudrate    = 9600;
	config_usart.mux_setting = EXT1_UART_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EXT1_UART_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EXT1_UART_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EXT1_UART_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EXT1_UART_SERCOM_PINMUX_PAD3;

	// initialize and enable uart with config
	while (usart_init(&usart_instance, EXT1_UART_MODULE, &config_usart) != STATUS_OK);
	usart_enable(&usart_instance);
}

void configure_usart_callbacks(void) {
	// register and enable uart callback functions
	usart_register_callback(&usart_instance,
		usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance,
		usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);

	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}

void configure_i2c_master(void) {
	// initialize config for i2c master
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);

	// set buffer timerout and pins
	config_i2c_master.buffer_timeout = 65535;
	config_i2c_master.pinmux_pad0    = PINMUX_PA16C_SERCOM1_PAD0;
	config_i2c_master.pinmux_pad1    = PINMUX_PA17C_SERCOM1_PAD1;

	// initialize and enable i2c master with config
	while(i2c_master_init(&i2c_master_instance, SERCOM1, &config_i2c_master) != STATUS_OK);
	i2c_master_enable(&i2c_master_instance);
}

void configure_i2c_master_callbacks(void)
{
	// Register and enable i2c master callback functions
	i2c_master_register_callback(&i2c_master_instance, \
		i2c_write_complete_callback, I2C_MASTER_CALLBACK_WRITE_COMPLETE);

	i2c_master_enable_callback(&i2c_master_instance, I2C_MASTER_CALLBACK_WRITE_COMPLETE);
}

static void config_led(void) {
	// initialize config for pins
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	// set direction, apply config, and set output to off
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LEDI2C_PIN, &pin_conf);
	port_pin_set_output_level(LEDI2C_PIN, false);
}

void set_genie_mode(uint8_t new_mode) {
	// switch and print new SAO Genie mode
	switch(new_mode) {
		case GENIE_MODE_PASSTHROUGH:
		; // print new mode
		uint8_t printStr1[] = "|                           | [*] mode set to: passthrough\n";
		usart_write_buffer_wait(&usart_instance, printStr1, sizeof(printStr1));
		break;

		case GENIE_MODE_REDIRECT:
		; // print new mode
		uint8_t printStr3[] = "|                           | [*] mode set to: redirect\n";
		usart_write_buffer_wait(&usart_instance, printStr3, sizeof(printStr3));
		break;

		case GENIE_MODE_INJECT:
		; // print new mode
		uint8_t printStr2[] = "|                           | [*] mode set to: inject\n";
		usart_write_buffer_wait(&usart_instance, printStr2, sizeof(printStr2));
		break;

		default:
		; // default/off
		uint8_t printStr0[] = "|                           | [*] mode set to: off\n";
		usart_write_buffer_wait(&usart_instance, printStr0, sizeof(printStr0));
		break;
	}

	genie_mode = new_mode;
}

int main(void) {

	system_init();

	configure_i2c_slave();
	configure_i2c_slave_callbacks();
	
	configure_i2c_master();
	configure_i2c_master_callbacks();
	
	configure_usart();
	configure_usart_callbacks();
	
	config_led();

	system_interrupt_enable_global();

	uint8_t aboutStr[] = "\n********************\n   SAO GENIE v0.1\n     mediumrehr\n********************\n\n";
	usart_write_buffer_wait(&usart_instance, aboutStr, sizeof(aboutStr));

	uint8_t headerStr[] = "/---------------------------|-------------------------------------------------\\\n|         Direction         |  Data                                           |\n|---------------------------|-------------------------------------------------|\n";
	usart_write_buffer_wait(&usart_instance, headerStr, sizeof(headerStr));

	set_genie_mode(GENIE_MODE_PASSTHROUGH);

	while (true) {
		// loop and wait for uart input
		usart_read_buffer_job(&usart_instance, (uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
	}
}
