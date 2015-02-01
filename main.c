/**
 *   Simple example to drive ws2811 leds with DMA and libopenc3.
 *
 *   Based on the idea developed at the folowing page:
 *   http://eliaselectronics.com/driving-a-ws2812-rgb-led-with-an-stm32/
 *
 *   Copyright (C) 2015  Oliver Dille
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>

#define NUMBER_LEDS		8

#define WS2811_HIGH		20
#define WS2811_LOW		9

typedef struct Rgb {
	uint8_t g;
	uint8_t r;
	uint8_t b;
};

struct Rgb leds[NUMBER_LEDS];

struct Rgb color_ring[NUMBER_LEDS];

uint8_t bufout[48];  // 2 Leds

volatile bool transfered = true;
volatile uint32_t count_leds;

struct Rgb hsv_to_rgb(double h, double s, double v)
{
	double dr, dg, db;
	int16_t i;

	struct Rgb ret;

	i = (int16_t)(h*6);
	double f = h * 6 - i;
	double p = v * (1 - s);
	double q = v * (1 - f * s);
	double t = v * (1 - (1 - f) * s);

	switch(i % 6) {
		case 0: dr = v, dg = t, db = p; break;
		case 1: dr = q, dg = v, db = p; break;
		case 2: dr = p, dg = v, db = t; break;
		case 3: dr = p, dg = q, db = v; break;
		case 4: dr = t, dg = p, db = v; break;
		case 5: dr = v, dg = p, db = q; break;
	}

	ret.r = dr * 255.0;
	ret.g = dg * 255.0;
	ret.b = db * 255.0;

	return ret;
}

static void clock_setup(void)
{
	/* Set STM32 to 72Mhz */
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOC clock */
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable GPIOA clock */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable DMA1 clock */
	rcc_periph_clock_enable(RCC_DMA1);

	/* Enable Timer 3 clock */
	rcc_periph_clock_enable(RCC_TIM3);

	/* Enable Aternate functions io clock */
	rcc_periph_clock_enable(RCC_AFIO);

}

static void gpio_setup(void)
{
	/* GPIOs used for probing / debuging */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO13 | GPIO14);
	/* PWM output */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO_TIM3_CH1);
}

static void timer3_setup()
{
	timer_reset(TIM3);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
	timer_enable_oc_output(TIM3, TIM_OC1);
	timer_set_oc_polarity_high(TIM3, TIM_OC1);

	/* Enable prefetch */
	TIM3_CCMR1 |= TIM_CCMR1_OC1PE;

	TIM3_PSC = 3 - 1;
	timer_set_period(TIM3, 29);
	timer_continuous_mode(TIM3);

	/* Enable capture compare DMA requests */
	TIM3_DIER |=   TIM_DIER_TDE | TIM_DIER_CC1DE;

	TIM3_CCR1 = 0;
	timer_enable_counter(TIM3);
}

static void ws2811_send()
{
	int i, j;

	uint8_t *led8;
	led8 = (uint8_t*)&leds[0];

	for(i=0;i!=6;++i) {

		for(j=0;j!=8;++j) {
			if( ( *(led8 + i) & (1 << j)) > 0) {
				bufout[ (i*8)+(7-j)] = WS2811_HIGH;
			} else {
				bufout[ (i*8)+(7-j)] = WS2811_LOW;
			}
		}

	}
	count_leds = 2;

	transfered = false;

	dma_channel_reset(DMA1, DMA_CHANNEL6);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL6, (uint32_t)&TIM3_CCR1);
	dma_set_memory_address(DMA1, DMA_CHANNEL6, (uint32_t)&bufout);
	dma_set_number_of_data(DMA1, DMA_CHANNEL6, 48);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL6);

	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL6);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL6, DMA_CCR_PSIZE_16BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL6, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, DMA_CHANNEL6, DMA_CCR_PL_VERY_HIGH);
	dma_enable_circular_mode(DMA1, DMA_CHANNEL6);

	timer_set_dma_on_compare_event(TIM3);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL6);
	dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL6);

	dma_enable_channel(DMA1, DMA_CHANNEL6);

	timer_enable_counter(TIM3);

}

int main(void)
{
	int i;
	uint32_t cnt = 0;

	for (i = 0; i < (1<<16); i++)	/* Wait a bit. */
 		__asm__("nop");

	clock_setup();
	gpio_setup();
	timer3_setup();

	nvic_set_priority(NVIC_DMA1_CHANNEL6_IRQ, 0);
	nvic_enable_irq(NVIC_DMA1_CHANNEL6_IRQ);

	for(i=0;i!=NUMBER_LEDS;++i) {
		color_ring[i] = hsv_to_rgb(1.0/NUMBER_LEDS*i,1.0,1.0);
	}

	while (1) {

		for(i=0;i != NUMBER_LEDS ;++i) {
			leds[i] = color_ring[ ( cnt + i ) % NUMBER_LEDS ];
		}

		cnt++;
		cnt %= NUMBER_LEDS;

		while(!transfered);
		ws2811_send();

		for (i = 0; i < (1 << 20); i++) //	 Wait a bit.
 			__asm__("nop");
	}

	return 0;
}

void dma1_channel6_isr(void)
{
	static int i;
	uint8_t *led8;

	/* Half Transfer interupt*/
	if((DMA1_ISR &DMA_ISR_HTIF6) > 0) {
		gpio_set(GPIOC, GPIO14);
		dma_clear_interrupt_flags(DMA1, DMA_CHANNEL6, DMA_ISR_HTIF_BIT);

		if (count_leds >= NUMBER_LEDS) {
			for(i=0;i!=24;++i) {
				bufout[i] = 0;
			}
		} else {
			led8 = (uint8_t*)&leds[count_leds];
			bufout[0]  = (*led8 & 0x80) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[1]  = (*led8 & 0x40) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[2]  = (*led8 & 0x20) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[3]  = (*led8 & 0x10) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[4]  = (*led8 & 0x08) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[5]  = (*led8 & 0x04) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[6]  = (*led8 & 0x02) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[7]  = (*led8 & 0x01) > 0 ? WS2811_HIGH : WS2811_LOW;

			led8++;
			bufout[8]  = (*led8 & 0x80) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[9]  = (*led8 & 0x40) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[10] = (*led8 & 0x20) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[11] = (*led8 & 0x10) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[12] = (*led8 & 0x08) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[13] = (*led8 & 0x04) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[14] = (*led8 & 0x02) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[15] = (*led8 & 0x01) > 0 ? WS2811_HIGH : WS2811_LOW;

			led8++;
			bufout[16] = (*led8 & 0x80) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[17] = (*led8 & 0x40) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[18] = (*led8 & 0x20) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[19] = (*led8 & 0x10) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[20] = (*led8 & 0x08) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[21] = (*led8 & 0x04) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[22] = (*led8 & 0x02) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[23] = (*led8 & 0x01) > 0 ? WS2811_HIGH : WS2811_LOW;

		}
		count_leds++;
	}

	/* Transfer conplete interupt*/
	if((DMA1_ISR &DMA_ISR_TCIF6) > 0) {
		gpio_clear(GPIOC, GPIO14);
		dma_clear_interrupt_flags(DMA1, DMA_CHANNEL6, DMA_ISR_TCIF_BIT);

		/* If the data for all leds has been sent, set the pwm pin low  */
		if (count_leds >= NUMBER_LEDS) {
			for(i=24;i!=48;++i) {
				bufout[i] = 0;
			}
		} else {
			led8 = (uint8_t*)&leds[count_leds];
			bufout[24] = (*led8 & 0x80) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[25] = (*led8 & 0x40) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[26] = (*led8 & 0x20) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[27] = (*led8 & 0x10) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[28] = (*led8 & 0x08) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[29] = (*led8 & 0x04) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[30] = (*led8 & 0x02) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[31] = (*led8 & 0x01) > 0 ? WS2811_HIGH : WS2811_LOW;

			led8++;
			bufout[32] = (*led8 & 0x80) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[33] = (*led8 & 0x40) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[34] = (*led8 & 0x20) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[35] = (*led8 & 0x10) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[36] = (*led8 & 0x08) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[37] = (*led8 & 0x04) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[38] = (*led8 & 0x02) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[39] = (*led8 & 0x01) > 0 ? WS2811_HIGH : WS2811_LOW;

			led8++;
			bufout[40] = (*led8 & 0x80) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[41] = (*led8 & 0x40) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[42] = (*led8 & 0x20) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[43] = (*led8 & 0x10) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[44] = (*led8 & 0x08) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[45] = (*led8 & 0x04) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[46] = (*led8 & 0x02) > 0 ? WS2811_HIGH : WS2811_LOW;
			bufout[47] = (*led8 & 0x01) > 0 ? WS2811_HIGH : WS2811_LOW;

		}

		count_leds++;

		if (count_leds >= NUMBER_LEDS + 6) {
			timer_disable_counter(TIM3);
			dma_disable_channel(DMA1, DMA_CHANNEL6);

			TIM3_CCR1 = 0;
			transfered = true;
		}
	}

}
