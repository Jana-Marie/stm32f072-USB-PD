/*
 * tcpm_driver.c
 *
 * Created: 11/11/2017 18:42:26
 *  Author: jason
 */ 

#include "tcpm_driver.h"
#include "main.h"

extern const struct tcpc_config_t tcpc_config[CONFIG_USB_PD_PORT_COUNT];
extern I2C_HandleTypeDef hi2c2;

void Wirewrite(char value[], int size)
{
  HAL_I2C_Master_Transmit(&hi2c2,(fusb302_I2C_SLAVE_ADDR << 1), value, size, 10);
}

char WirerequestFrom(int size)
{
  char value[size];
  HAL_I2C_Master_Receive(&hi2c2,  (fusb302_I2C_SLAVE_ADDR << 1), value, size, 10);
  return value;
}


/* I2C wrapper functions - get I2C port / slave addr from config struct. */
int tcpc_write(int port, int reg, int val)
{
  char data[2] = {reg & 0xFF,val & 0xFF};
  Wirewrite(data,2); 

  return 0;
}

int tcpc_write16(int port, int reg, int val)
{
  char data[3] = {reg & 0xFF,val & 0xFF,(val >> 8) & 0xFF};
  Wirewrite(data,3);

  return 0;
}

int tcpc_read(int port, int reg, int *val)
{
  char data[1] = {reg & 0xFF};
  Wirewrite(data,1);
  *val = WirerequestFrom(1);

  return 0;
}

int tcpc_read16(int port, int reg, int *val)
{

  char data[1] = {reg & 0xFF};
  Wirewrite(data,1);
  *val  = WirerequestFrom(1);
  *val |= (WirerequestFrom(1) << 8);

  return 0;
}

int tcpc_xfer(int port,
	const uint8_t *out, int out_size,
	uint8_t *in, int in_size,
	int flags)
{
  if (out_size)
  {
    Wirewrite(out,out_size);
  }

  if (in_size) {
    for (; in_size>0; in_size--) {
        *in = WirerequestFrom(1);
        in++;
    }
  }
}

