// slaves
// can be controlled by serial, input, or I2C
// do not mix any of them at the same time, as the result may be undefined
// every received byte overwrite the previous one, and thus only the last validated byte i
// results in an action
#include <Wire.h>

// I2C setup
unsigned char i2c_addr = 0x10;
#define I2C_ADDR_PORT              2  // reserved input to define address
#define I2C_ADDR_PORT_COUNT 	     4  // input port start at I2C_ADDR_PORT to I2C_ADDR_PORT+I2C_ADDR_PORT_COUNT

// serial setup
#define SERIAL_SPEED            9600  // remote by serial

// command setup
#define INPUT_SWITCH_DELAY_MS   200   // minimum HIGH elapsed time before button is handled 
#define POWER_LOAD_PIN            0   // input to measure power value

// input command
#define BTN_POWER_SWITCH          8   // switch to power on/off AC
int power_button_high_start = 0;      // timing when the button was pushed

// light feedback
#define LED_POWER_ON              9   // lighted on power on
#define LED_POWER_OFF            10   // lighted on power off

// command received by input (serial, i2c or button)
enum command_t
{
  CMD_NONE                  = 0x00,
  CMD_POWER_ON,                       // switch power on
  CMD_POWER_OFF,                      // switch power off
  CMD_I2C_ADDR,                       // request I2C address
  CMD_POWER_LOAD,                     // request power load
  CMD_POWER_STATE,                    // request power state (on or off)
  CMD_UNKNOWN                         // from this point command is unknown
};
// shall be present just after the command
#define COMMAND_VALIDATE 0xDD

// source on the command
enum source_t
{
  SRC_NONE          = 0x00,
  SRC_SERIAL,
  SRC_I2C,
  SRC_INPUT
};

// serial command consist of 1 byte and is validated by SERIAL_COMMAND_END
char last_received_byte         = CMD_NONE;
source_t last_received_source   = SRC_NONE;
volatile bool command_validated = false;

// POWER LOAD measure
#define POWER_LOAD_PICK_TIME        100 // measure power load every POWER_LOAD_PICK_TIME ms
#define POWER_LOAD_AVG_PICK_COUNT    10 // create an average value every POWER_LOAD_AVG_PICK_COUNT measure
int  power_load_sum 	= 0;  // sum of power value measured since first
int  power_load_count = 0;  // number of power value measured since first
char power_load_avg   = 0;  // average power value
int  power_load_last  = 0;  // last power value measured
 
// waiting command
volatile command_t pending_command = CMD_NONE;
volatile source_t pending_source   = SRC_NONE;

void setup()
{  
  // read i2c adress from digital dedicated PIN
  for (unsigned char i = 0; i < I2C_ADDR_PORT_COUNT; ++i)
  {
    pinMode(I2C_ADDR_PORT + i, INPUT);
    i2c_addr |= (digitalRead(I2C_ADDR_PORT + i) << i);
  }
  
  // setup i2c
  Wire.begin(i2c_addr);
  Wire.onReceive(i2c_onReceive);
  Wire.onRequest(i2c_onRequest);
  
  // setup serial port
  Serial.begin(SERIAL_SPEED, SERIAL_8N1);
  
  // setup in/output
  pinMode(BTN_POWER_SWITCH, INPUT);
  pinMode(LED_POWER_ON, OUTPUT);
  pinMode(LED_POWER_OFF, OUTPUT);
}

void loop()
{
  // read inputs
  serial_read();
  input_read();
  
  // do what you need if needed 
  handle_command();
  
  // update power measure
  update_power_load();
}

void ac_power(bool on)
{
  // switch the relay to the request position
  
  // light the corresponding led
  digitalWrite(LED_POWER_ON,  on ? HIGH : LOW);
  digitalWrite(LED_POWER_OFF, !on ? HIGH : LOW);
}

void update_power_load()
{
  if (millis() - power_load_last < POWER_LOAD_PICK_TIME)
    return;

  // read current power load
  char power_value = analogRead(POWER_LOAD_PIN);
  
  // sum total load with current load
  power_load_sum += power_value;
  ++power_load_count;
  
  // compute load average if having done enought power pick
  if (power_load_count == POWER_LOAD_AVG_PICK_COUNT)
  {
    power_load_avg = (char)(power_load_sum / power_load_count);
    power_load_count = 0;
    power_load_sum = 0;
  }
  
  // reset last measure
  power_load_last = millis();	
}

void handle_byte(char b, source_t src)
{		
  // backup received buffer
  last_received_source = src;
  
  // if r is SERIAL_COMMAND_END, treat the command buffer
  // and reset it
  if (b == COMMAND_VALIDATE)
  {
    command_validated = true;
  }
  else
  {
    // backup command
    last_received_byte = b;
  }
}

void input_read()
{
  // read from the intput button
  if (digitalRead(BTN_POWER_SWITCH) == HIGH)
  {
    // start timing the HIGH elasped time
    if (power_button_high_start == 0)
    {
      power_button_high_start = millis();
    }
  }
  else
  {
    // reset HIGH elapsed time
    power_button_high_start = 0;
  }
  
  // switch state after a button push of at least 200 ms
  if (power_button_high_start != 0 &&
      millis() - power_button_high_start >= INPUT_SWITCH_DELAY_MS)
  {
    char b = (power_load_last > 0) ? CMD_POWER_OFF : CMD_POWER_ON;
    handle_byte(b, SRC_INPUT);
    handle_byte(COMMAND_VALIDATE, SRC_INPUT);
  }
}

void serial_read()
{
	if (Serial.available() > 0)
	{
		// get one char only
		char r = Serial.read();
		handle_byte(r, SRC_SERIAL);
	}
}

void i2c_onReceive(int byte_count)
{
	// command shall be received on 1 byte only
  // only treat last byte
  char r = 0;
  while (Wire.available())
  {
		r = Wire.read();
		handle_byte(r, SRC_I2C);
  }
}

void i2c_onRequest()
{
  // if no
  // send data according to onReceive command
  switch (pending_command)
  { 
    case CMD_I2C_ADDR:
      Wire.write(i2c_addr);
      Wire.write(COMMAND_VALIDATE);
      pending_command = CMD_NONE;
		break;
    case CMD_POWER_LOAD:
			Wire.write(power_load_avg);
			Wire.write(COMMAND_VALIDATE);
			pending_command = CMD_NONE;
		break;
		default:
			// no idea of what is requested !
			// send something to i2c to tell it request is handled
			// but without date
			Wire.write(COMMAND_VALIDATE);
		break;
  }
}

void handle_command()
{
  if (!command_validated)
	return;

//  backup command
	char cmd = last_received_byte;
	source_t src = last_received_source;

	// reset received command
	command_validated = false;
	last_received_byte = CMD_NONE;

  // state command
  switch (pending_command)
  { 
  	case CMD_POWER_ON:  
			ac_power(true); 
			pending_command = CMD_NONE;
		break;
    case CMD_POWER_OFF: 
			ac_power(false); 
			pending_command = CMD_NONE;
		break;
    case CMD_I2C_ADDR:
      switch (pending_source)
      {
        case SRC_SERIAL:
          Serial.write(i2c_addr);
          Serial.write(COMMAND_VALIDATE);
        case SRC_INPUT:
          pending_command = CMD_NONE;
          break;
        default:
          // request from I2C are treated by onRequest, so do not reset state
          break;
      }
		break;
    case CMD_POWER_LOAD:
      switch (pending_source)
      {
        case SRC_SERIAL:
          Serial.write(power_load_avg);
          Serial.write(COMMAND_VALIDATE);
        case SRC_INPUT:
          pending_command = CMD_NONE;
          break;
        default:
          // request from I2C are treated by onRequest, so do not reset state
          break;
      }
		break;
    default:
      // unsupported command
      pending_command = CMD_NONE;
		break;
  }
}
