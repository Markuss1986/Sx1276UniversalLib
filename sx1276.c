#include "sx1276.h"

void sx1276_spi_transmit(unsigned char *txBuffer,unsigned char size)
{
    /* Use your own functions to transmit one byte/array with SPI 
			This function send data by SPI without binding to registers
		*/
    SX1276_SPI_TRANSMIT();
}

void sx1276_spi_receive(unsigned char *rxBuffer,unsigned char size)
{
    /* Use your own functions to receive byte/array with SPI 
		 * This function send data by SPI without binding to registers */
    SX1276_SPI_RECEIVE(); 
}

void sx1276_reset(void)
{
    /* Function for hardware reset sx1276 */
		SX1276_DESELECT();
    SX1276_RESET_PIN_RESET();
    DELAY_100MS;
    SX1276_RESET_PIN_SET();
    DELAY_100MS;
}

void sx1276_receiver_mode(void)
{
    /* Function for start reciver sx1276 */
		sx1276_write_reg( REG_OPMODE, RF_OPMODE_LONGRANGEMODE_OFF |
											RF_OPMODE_MODULATIONTYPE_FSK |
											RF_OPMODE_MODULATIONSHAPING_01 |
											RF_OPMODE_RECEIVER );
}

void sx1276_standby_mode(void)
{
    /* Function for switching to standby mode sx1276 */
		sx1276_write_reg( REG_OPMODE, RF_OPMODE_LONGRANGEMODE_OFF |
											RF_OPMODE_MODULATIONTYPE_FSK |
											RF_OPMODE_MODULATIONSHAPING_01 |
											RF_OPMODE_STANDBY);
}

void sx1276_sleep_mode(void)
{
    /* Function for switching to sleep mode sx1276 */
    sx1276_write_reg( REG_OPMODE, RF_OPMODE_LONGRANGEMODE_OFF |
                      RF_OPMODE_MODULATIONTYPE_FSK |
                      RF_OPMODE_MODULATIONSHAPING_01 |
                      RF_OPMODE_SLEEP);
}

void sx1276_transmit_mode(void)
{
    /* Function for switching to start transmit mode sx1276 */
    sx1276_write_reg( REG_OPMODE, RF_OPMODE_LONGRANGEMODE_OFF |
                      RF_OPMODE_MODULATIONTYPE_FSK |
                      RF_OPMODE_MODULATIONSHAPING_01 |
                      RF_OPMODE_TRANSMITTER);
}

void sx1276_write_reg(uint8_t addr, uint8_t data)
{
    /* Function for write one byte (data) to register by address (addr)
     * 0x80 - inform you that we will write to the register, data -  second byte  */
		uint8_t txBuffer[2];
		SX1276_SELECT();
		txBuffer[0]=addr|0x80;
		txBuffer[1]=data;
		sx1276_spi_transmit(txBuffer,2);
		SX1276_DESELECT();
}

uint8_t sx1276_read_reg(uint8_t addr)
{
    /* Function for read one byte  from register by address (addr) */
    /* 0x7F - inform you that we will read from the register */
		uint8_t txBuffer=addr&0x7F;
		uint8_t rxBuffer=0;
		SX1276_SELECT();
		sx1276_spi_transmit(&txBuffer,1);
		sx1276_spi_receive(&rxBuffer,1);
		SX1276_DESELECT();
		return rxBuffer;
}



void sx1276_read_FIFO(uint8_t *buffer,uint8_t size)
{
    /* Function for read frame from stack FIFO sx1276 */
		//Read from FIFO only in standby mode. close Recive 
		sx1276_standby_mode();
	
		// Read data drom FIFO SX1276
		uint8_t SelectReaadModeSX1276=REG_FIFO&0x7F;
		SX1276_SELECT();
		sx1276_spi_transmit(&SelectReaadModeSX1276,1);
		sx1276_spi_receive(buffer,size);
		SX1276_DESELECT();
	
	  // Set recive mode Sx1276
		sx1276_receiver_mode();
}

void sx1276_write_FIFO(uint8_t *buffer,uint8_t size)
{
    /* Function for write frame to stack FIFO sx1276 */
		//Write to FIFO only in sleep mode 
		sx1276_sleep_mode(); 
	  uint8_t SelectWriteModeSX1276=REG_FIFO|0x80;
		SX1276_SELECT();
		sx1276_spi_transmit(&SelectWriteModeSX1276,1);
		sx1276_spi_transmit(buffer,size);
		SX1276_DESELECT();
	
		//Transmit mode (Send FIFO to RF)	
		sx1276_transmit_mode();
}


void sx1276_transmit_frame(uint8_t *frame, uint8_t size)
{
    /* Function for transmit frame with sx1276 by radio channel  */
		sx1276_standby_mode();
		sx1276_write_reg(REG_IRQFLAGS2,0xFF);
		sx1276_write_FIFO(frame,size);

    /* Switching to transmit mode sx1276 */
		sx1276_transmit_mode();

}

void sx1276_init_fsk(void)
{	
		/*Set RESET PIN SX1276 to high, first of all need do this command before main initialization*/
		SX1276_RESET_PIN_SET();
    
	  /* Initialization sx1276 */
    /* Switch sx1276 to sleep mode  */
    sx1276_write_reg( REG_OPMODE, RF_OPMODE_LONGRANGEMODE_OFF |
                      RF_OPMODE_MODULATIONTYPE_FSK |
                      RF_OPMODE_MODULATIONSHAPING_01 |
                      RF_OPMODE_SLEEP);

    /* Set bitrate */
    sx1276_write_reg( REG_BITRATEMSB, 0x06);
    sx1276_write_reg( REG_BITRATELSB, 0x82);
    sx1276_write_reg( REG_BITRATEFRAC, 11);

	  /* Set deviation */
    sx1276_write_reg( REG_FDEVMSB, RF_FDEVMSB_45000_HZ);
    sx1276_write_reg( REG_FDEVLSB, RF_FDEVLSB_45000_HZ);

		/* Set frequency */
    sx1276_write_reg( REG_FRFMSB, RFLR_FRFMSB_434_MHZ);
    sx1276_write_reg( REG_FRFMID, RFLR_FRFMID_434_MHZ);
    sx1276_write_reg( REG_FRFLSB, RFLR_FRFLSB_434_MHZ);

    /* Set Gain LNA */
    sx1276_write_reg( REG_LNA, RF_LNA_GAIN_G6|RF_LNA_BOOST_ON);

    /* Initialization receiver, Auto Freq, LNA, Preamble detect on */
    sx1276_write_reg( REG_RXCONFIG, RF_RXCONFIG_RESTARTRXONCOLLISION_OFF |
                      RF_RXCONFIG_AFCAUTO_OFF	|
                      RF_RXCONFIG_AGCAUTO_OFF |
                      RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT);

    /* Set samplinng RSSI */
    sx1276_write_reg( REG_RSSICONFIG, RF_RSSICONFIG_SMOOTHING_256);

    /* Set power transmitter (revise) */
    sx1276_write_reg( REG_PACONFIG, RF_PACONFIG_MAX_POWER_MASK);

    /* Set (Rise/Fall time of ramp up/down in FSK, Gaussian filter) */
    sx1276_write_reg( REG_PARAMP, RF_PARAMP_1000_US |RF_PARAMP_MODULATIONSHAPING_00);

    /* Set Bandwidth */
    sx1276_write_reg( REG_RXBW, RF_RXBW_MANT_16 | RF_RXBW_EXP_3);

    /* Initialization AFC */
    sx1276_write_reg( REG_AFCFEI,
                      RF_AFCFEI_AFCAUTOCLEAR_OFF |
                      RF_AFCFEI_AFCCLEAR |
                      RF_AFCFEI_AGCSTART);

    /* Set Preamble detect -  on, length preamble detect - 3bytes, number mistakes */
    sx1276_write_reg( REG_PREAMBLEDETECT, RF_PREAMBLEDETECT_DETECTOR_ON |
                      RF_PREAMBLEDETECT_DETECTORSIZE_3 |
                      RF_PREAMBLEDETECT_DETECTORTOL_30);

    /* Initialization Sync Word */
    sx1276_write_reg( REG_SYNCCONFIG, RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_OFF |
                      RF_SYNCCONFIG_PREAMBLEPOLARITY_55 |
                      RF_SYNCCONFIG_SYNCSIZE_4 |
                      RF_SYNCCONFIG_SYNC_ON);

    /* Initialization size preamble */
    sx1276_write_reg( REG_PREAMBLEMSB, 0x00);
    sx1276_write_reg( REG_PREAMBLELSB, 0x03);

    /* Initialization Sync Word */
    sx1276_write_reg( REG_SYNCVALUE1, 0x01);
    sx1276_write_reg( REG_SYNCVALUE2, 0x01);
    sx1276_write_reg( REG_SYNCVALUE3, 0x01);
    sx1276_write_reg( REG_SYNCVALUE4, 0x01);

    /* Fixed length packet,CRC on */
    sx1276_write_reg( REG_PACKETCONFIG1, RF_PACKETCONFIG1_PACKETFORMAT_FIXED	|
                      RF_PACKETCONFIG1_DCFREE_OFF	|
                      RF_PACKETCONFIG1_CRC_ON	|
                      RF_PACKETCONFIG1_CRCAUTOCLEAR_ON |
                      RF_PACKETCONFIG1_ADDRSFILTERING_OFF |
                      RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT);

    /* Set Packet mode on */
    sx1276_write_reg( REG_PACKETCONFIG2, RF_PACKETCONFIG2_DATAMODE_PACKET );

    /* Set transmit current overload current */
    sx1276_write_reg( REG_OCP, RF_OCP_ON | RF_OCP_TRIM_200_MA );

    /* Set Length payload (frame) */
    sx1276_write_reg( REG_PAYLOADLENGTH, SX1276_PAYLOAD);

    /* Set width PLL */
    //SX127xWriteBuffer( REG_PLL, 0x10 | 0x02);

    /* Set GPIO - DIO_0 - DIO5, config */
    sx1276_write_reg( REG_DIOMAPPING1, 0x00);
    sx1276_write_reg( REG_DIOMAPPING2, 0x00);

    /* Set FIFO Interrupt
     * Sx1276 will report an interrupt when the number of bytes in the fifo stack is equal to SX1276_PAYLOAD-1 */
    sx1276_write_reg( REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTARTCONDITION_FIFOTHRESH | SX1276_PAYLOAD-1);
    sx1276_write_reg( REG_SEQCONFIG1,RF_SEQCONFIG1_FROMTX_TORX);
		
		//Set reciver mode
		sx1276_receiver_mode();
}




