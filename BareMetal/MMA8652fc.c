#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
#define MMA8652_SA 0x1D

/* MMA8652 Registers */
#define MMA8652_REG_WHOAMI 0x0D

#define I2C1_START		I2C1->C1 |= (1 << I2C_C1_MST_SHIFT) | (1 << I2C_C1_TX_SHIFT)
#define I2C1_STOP		I2C1->C1 &= ~((1 << I2C_C1_MST_SHIFT) | (1 << I2C_C1_TX_SHIFT) | (1 << I2C_C1_TXAK_SHIFT))
#define I2C1_RSTART		I2C1->C1 |= (1 << I2C_C1_RSTA_SHIFT) | (1 << I2C_C1_TX_SHIFT)
#define I2C1_XMIT		I2C1->C1 |= (1 << I2C_C1_TX_SHIFT)
#define I2C1_RECV		I2C1->C1 &= ~(1 << I2C_C1_TX_SHIFT)
#define I2C1_MASTER		I2C1->C1 |= (1 << I2C_C1_MST_SHIFT)
#define I2C1_WAIT		while((I2C1->S & (1 << I2C_S_IICIF_SHIFT)) == 0); \
						I2C1->S |= (1 << I2C_S_IICIF_SHIFT)
#define I2C1_READ_WAIT			while((I2C1->S & (1 << I2C_S_IICIF_SHIFT)) == 0); \
								I2C1->S |= (1 << I2C_S_IICIF_SHIFT)

void i2c_write(uint8_t slaveAddr, uint8_t regAddr, uint8_t regVal)
{uint8_t data;



	/* I2C1 Check for Bus Busy */
	while(I2C1->S & (1 << I2C_S_BUSY_SHIFT));

	/* Generate START Condition */
	I2C1_START;

	//I2C1->A1 = (slaveAddr << 1);

	/* Send Slave Address */
	I2C1->D = (slaveAddr << 1);
	I2C1_WAIT;

	/* Send Register Address */
	I2C1->D = regAddr;
	I2C1_WAIT;

	/* Send Slave Address */
	I2C1->D = regVal;
	I2C1_WAIT;

	/* Generate STOP Condition */
	I2C1_STOP;

	return;
}

uint8_t i2c_read(uint8_t slaveAddr, uint8_t regAddr)
{
	uint8_t data;
	uint8_t timeDelay = 6;

	/* I2C1 Check for Bus Busy */
	while(I2C1->S & (1 << I2C_S_BUSY_SHIFT));

	/* Generate START Condition */
	I2C1_START;

	/* Send Slave Address */
	I2C1->D = (slaveAddr << 1);
	I2C1_WAIT;

	/* Send Register Address */
	I2C1->D = regAddr;
	I2C1_WAIT;

	/* Generate Repeated Start */
	I2C1_RSTART;

    /* Add some delay to wait the Re-Start signal. */
    while (timeDelay--)
    {
        __NOP();
    }

	/* Send Slave Address */
	I2C1->D = (slaveAddr << 1) | 0x01;
	I2C1_WAIT;

	I2C1_RECV;

	I2C1->C1 |= (1 << I2C_C1_TXAK_SHIFT);

	/* Dummy Read */
	data = I2C1->D;
	I2C1_READ_WAIT;

	/* Generate STOP Condition */
	I2C1_STOP;

	/* Read the value from Data Register */
	data = I2C1->D;

	return data;
}

#define I2C_RELEASE_SDA_PORT PORTE
#define I2C_RELEASE_SCL_PORT PORTE
#define I2C_RELEASE_SDA_GPIO GPIOE
#define I2C_RELEASE_SDA_PIN 0U
#define I2C_RELEASE_SCL_GPIO GPIOE
#define I2C_RELEASE_SCL_PIN 1U

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < 1000; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortE);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA low */
    for (i = 0; i < 9; i++)
    {
        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}
/*
 * @brief   Application entry point.
 */
int main(void) {
	    uint8_t data;
		uint16_t xoff;
		uint16_t yoff;
		uint16_t zoff;

		/* Enable clock for PORTE */
		SIM->SCGC5 = (1 << SIM_SCGC5_PORTE_SHIFT);

		/* Enable clock for I2C1 */
		SIM->SCGC4 = (1 << SIM_SCGC4_I2C1_SHIFT);

		BOARD_I2C_ReleaseBus();

		/* PORTE pin as I2C1_SCL */
		PORTE->PCR[1] = (6 << PORT_PCR_MUX_SHIFT) | (1 << PORT_PCR_PS_SHIFT) | (1<<PORT_PCR_PE_SHIFT) | (1<<PORT_PCR_SRE_SHIFT);

		/* PORTE pin as I2C1_SDA */
		PORTE->PCR[0] = (6 << PORT_PCR_MUX_SHIFT)  | (1 << PORT_PCR_PS_SHIFT) | (1<<PORT_PCR_PE_SHIFT) | (1<<PORT_PCR_SRE_SHIFT);


		/* I2C1 Frequency Divider */
		I2C1->F = 0x0F;

		/* I2C1 Enable, Master Mode */
		I2C1->C1 = (1 << I2C_C1_IICEN_SHIFT) | (1 << I2C_C1_IICIE_SHIFT);

		I2C1->S |= (1 << I2C_S_IICIF_SHIFT);

		/* I2C1 Check for Bus Busy */
		while(I2C1->S & (1 << I2C_S_BUSY_SHIFT));

		data = i2c_read(MMA8652_SA, MMA8652_REG_WHOAMI);
		while(I2C1->S & (1 << I2C_S_BUSY_SHIFT));

		data = i2c_read(MMA8652_SA, MMA8652_REG_WHOAMI);

		data = i2c_read(MMA8652_SA, MMA8652_REG_WHOAMI);

		i2c_write(MMA8652_SA, 0x2A, 0x01);

		data = i2c_read(MMA8652_SA, 0x2A);

		while(1) {
			/* X Offset LSB */
			data = i2c_read(MMA8652_SA, 0x01);
			xoff = data;

			/* X Offset MSB */
			data = i2c_read(MMA8652_SA, 0x02);
			xoff |= (data << 8);

			/* Y Offset LSB */
			data = i2c_read(MMA8652_SA, 0x03);
			yoff = data;

			/* Y Offset MSB */
			data = i2c_read(MMA8652_SA, 0x04);
			yoff |= (data << 8);

			/* Z Offset LSB */
			data = i2c_read(MMA8652_SA, 0x05);
			zoff = data;

			/* Z Offset MSB */
			data = i2c_read(MMA8652_SA, 0x06);
			zoff |= (data << 8);

			printf("XOFF: 0x%04x YOFF: 0x%04x ZOFF: 0x%04x\n", xoff, yoff, zoff);
		}
    return 0 ;
}
