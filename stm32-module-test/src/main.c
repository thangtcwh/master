#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static void clock_setup(void)
{
	/* Set STM32 to 64 MHz. */
	rcc_clock_setup_in_hsi_out_64mhz();

	/* Enable alternate function peripheral clock. */
	rcc_periph_clock_enable(RCC_AFIO);

	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOC);
}

static void gpio_setup(void)
{

	/* Configure PB4 as GPIO. */
	AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;
	
	/* Set GPIO4 and 5 (in GPIO port B) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	
}

int main(void)
{
	int i;

	clock_setup();
	gpio_setup();

	/* Blink the LEDs on the board. */
	while (1) {
		gpio_toggle(GPIOC, GPIO15);	/* LED on/off */
		for (i = 0; i < 10000; i++)	/* Wait a bit. */
			__asm__("nop");
		
	}

	return 0;
}