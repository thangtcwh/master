
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/common/crc_common_all.h>

#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#define LED_DISCOVERY_USER_PORT	GPIOC
#define LED_DISCOVERY_USER_PIN	GPIO13

#define InDATA	0x1234ffff
// #define LED_DISCOVERY_USER_PIN	GPIO13

#define USART_CONSOLE USART2
uint32_t Inputdata = 0x00000000;//0x12345678;
uint32_t Outputdata;

int _write(int file, char *ptr, int len);

static void clock_setup(void)
{
	rcc_clock_setup_in_hsi_out_24mhz();
	/* Enable clocks for USART2 and DAC*/
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_CRC);

	/* and the ADC and IO ports */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	// rcc_periph_clock_enable(RCC_ADC1);
}

static void usart_setup(void)
{
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

	usart_set_baudrate(USART_CONSOLE, 115200);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
	usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART_CONSOLE);
}

static void crc_setup(void)
{
	crc_reset();
	printf("Data input after CRC calculate %8x\n\r", Inputdata);
	// printf("Data output aftr CRC calculate %08x\n\r", MMIO32(Outputdata));
	//0x4C11DB7
	Outputdata = crc_calculate(Inputdata);
	printf("Data output aftr CRC calculate %8x\n\r", Outputdata);
	printf("Data output aftr CRC calculate %8jx\n\r",(uintmax_t)(Outputdata));
//(uintmax_t)
}

static void crc_setup_1(void)
{
	crc_reset();
	Outputdata = crc_calculate(InDATA);
	printf("Data output after CRC calculate %8jx\n\r", (uintmax_t)Outputdata);
	// printf("Data output aftr CRC calculate %d\r", Outputdata);

}
/**
 * Use USART_CONSOLE as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART_CONSOLE, '\r');
			}
			usart_send_blocking(USART_CONSOLE, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}


int main(void)
{
	int i;
	int j = 0;
	clock_setup();
	usart_setup();
	
	printf("hi guys!\n");
	
	gpio_set_mode(LED_DISCOVERY_USER_PORT, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, LED_DISCOVERY_USER_PIN);
	
	while (1) {
		
		printf("tick: %d \n\r",j++);
		
		gpio_toggle(LED_DISCOVERY_USER_PORT, LED_DISCOVERY_USER_PIN); /* LED on/off */
		
		crc_setup();
		// crc_setup_1();
		for (i = 0; i < 1000000; i++) /* Wait a bit. */
		 	__asm__("NOP");
	
		

		// printf("Data output aftr CRC calculate %d\r", Outputdata);
	}

	return 0;
}



// uint16_t input_adc0 = read_adc_naiive(0);
// uint16_t target = input_adc0 / 2;
// dac_load_data_buffer_single(target, RIGHT12, CHANNEL_2);
// dac_software_trigger(CHANNEL_2);
// uint16_t input_adc1 = read_adc_naiive(1);

// #include <libopencm3/stm32/rcc.h>
// #include <libopencm3/stm32/gpio.h>
// #include <libopencm3/stm32/usart.h>

// static void clock_setup(void)
// {
// 	/* Set STM32 to 64 MHz. */
// 	rcc_clock_setup_in_hsi_out_64mhz();

// 	/* Enable alternate function peripheral clock. */
// 	rcc_periph_clock_enable(RCC_AFIO);

// 	/* Enable GPIOB clock. */
// 	rcc_periph_clock_enable(RCC_GPIOC);
// 	rcc_periph_clock_enable(RCC_GPIOB);
// 	rcc_periph_clock_enable(RCC_USART2);
// }

// static void usart_setup(void)
// {
// 	/* Setup GPIO pin GPIO_USART2_TX. */
// 	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
// 		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

// 	/* Setup UART parameters. */
// 	usart_set_baudrate(USART2, 115200);
// 	usart_set_databits(USART2, 8);
// 	usart_set_stopbits(USART2, USART_STOPBITS_1);
// 	usart_set_mode(USART2, USART_MODE_TX);
// 	usart_set_parity(USART2, USART_PARITY_NONE);
// 	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

// 	/* Finally enable the USART. */
// 	usart_enable(USART2);

// }


// static void gpio_setup(void)
// {

// 	/* Configure PB4 as GPIO. */
// 	// AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;

// 	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
// 		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO9|GPIO10);

// 	/* Set GPIO4 and 5 (in GPIO port B) to 'output push-pull'. */
// 	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
// 		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

// }

// int main(void)
// {
// 	int i;

// 	clock_setup();
// 	gpio_setup();

// 	/* Blink the LEDs on the board. */
// 	while (1) {
// 		gpio_toggle(GPIOC, GPIO13);	/* LED on/off */
// 		for (i = 0; i < 5000000; i++)	/* Wait a bit. */
// 			__asm__("nop");
// 			printf("hellllllllllll0");


// 	}

// 	return 0;
// }

// static void adc_setup(void)
// {
// 	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
// 	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);

// 	/* Make sure the ADC doesn't run during config. */
// 	adc_power_off(ADC1);

// 	/* We configure everything for one single conversion. */
// 	adc_disable_scan_mode(ADC1);
// 	adc_set_single_conversion_mode(ADC1);
// 	adc_disable_external_trigger_regular(ADC1);
// 	adc_set_right_aligned(ADC1);
// 	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

// 	adc_power_on(ADC1);

// 	/* Wait for ADC starting up. */
// 	int i;
// 	for (i = 0; i < 800000; i++) /* Wait a bit. */
// 		__asm__("nop");

// 	adc_reset_calibration(ADC1);
// 	adc_calibrate(ADC1);
// }

// static void dac_setup(void)
// {
// 	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5);
// 	dac_disable(CHANNEL_2);
// 	dac_disable_waveform_generation(CHANNEL_2);
// 	dac_enable(CHANNEL_2);
// 	dac_set_trigger_source(DAC_CR_TSEL2_SW);
// }

// static uint16_t read_adc_naiive(uint8_t channel)
// {
// 	uint8_t channel_array[16];
// 	channel_array[0] = channel;
// 	adc_set_regular_sequence(ADC1, 1, channel_array);
// 	adc_start_conversion_direct(ADC1);
// 	while (!adc_eoc(ADC1));
// 	uint16_t reg16 = adc_read_regular(ADC1);
// 	return reg16;
// }
