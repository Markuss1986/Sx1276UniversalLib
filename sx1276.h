#ifndef _SX1276_H
#define _SX1276_H

#include "sx1276Regs-Fsk.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

/* Frame length */
#define SX1276_PAYLOAD	    23

/* Use your function for select/deselect chip SX1276/1278 with GPIO NSS
 * for example using HAL-function for stm32. Active is low level. */
#define SX1276_SELECT()             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)
#define SX1276_DESELECT()           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)

/* Use your function for set/reset pin RESET chip SX1276/1278 with GPIO
 * for example using HAL-function for stm32 */
#define SX1276_RESET_PIN_RESET()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define SX1276_RESET_PIN_SET()      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

/* Use your function for delay 100 ms for example using HAL-function for stm32 */
#define DELAY_100MS                 HAL_Delay(100)

/* Function for transmit and receive array by SPI to/from SX1276/1278 */
#define SX1276_SPI_TRANSMIT()       HAL_SPI_Transmit(&hspi1, txBuffer, size,10)
#define SX1276_SPI_RECEIVE()        HAL_SPI_Receive(&hspi1, rxBuffer, size,10)

/* Declaring prototype functions */

void		sx1276_init_fsk(void);																/* Initialization sx1276 for FSK mode */
void    sx1276Init(void);

// Functions for switch modes 
void 		sx1276_reset(void);																		/* Function for hardware reset sx1276 */
void 		sx1276_receiver_mode(void);														/* Function for start reciver sx1276 */
void 		sx1276_standby_mode(void);														/* Function for switching to standby mode sx1276 */
void 		sx1276_transmit_mode(void);														/* Function for switching to start transmit sx1276 */
void	  sx1276_sleep_mode(void);															/* Function for switching to sleep mode sx1276 */

// Functions send data to registers sx1276
void		sx1276_write_reg(uint8_t addr, uint8_t data);					/* Function to transmit one byte/array by SPI to registers Sx1276 */
uint8_t sx1276_read_reg(uint8_t addr);												/* Functions to read one byte/array by SPI from registers Sx1276 */

// Functions send/read frame to/from FIFO sx1276
void 		sx1276_read_FIFO(uint8_t* buffer,uint8_t size);				/* Function send frame to FIFO sx1276 */
void 		sx1276_write_FIFO(uint8_t* buffer,uint8_t size);			/* Function read frame from FIFO sx1276 */

// Function fo transmiit framme by radio channel
void 		sx1276_transmit_frame(uint8_t* frame, uint8_t size);	/* Initialization sx1276 for FSK mode */

#endif
