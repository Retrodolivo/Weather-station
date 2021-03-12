//List of header files
#include "nrf24.h"

//*** Variables declaration ***//
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define _BOOL(x) (((x)>0) ? 1:0)

//*** Library variables ***//
static uint64_t pipe0_reading_address;
static bool ack_payload_available; /**< Whether there is an ack payload waiting */
static uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
static uint8_t payload_size; /**< Fixed size of payloads */
static bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */ 
static bool p_variant; /* False for RF24L01 and true for RF24L01P */
static bool wide_band; /* 2Mbs data rate in use? */

//*** nrf24L01 pins and handles ***//
//CE and CSN pins
static GPIO_TypeDef		*nrf24_PORT;
static uint16_t				nrf24_CSN_PIN;
static uint16_t				nrf24_CE_PIN;
//SPI handle
static SPI_HandleTypeDef nrf24_hspi;
//Debugging UART handle
static UART_HandleTypeDef nrf24_huart;

//**** Functions prototypes ****//
//Microsecond delay function
void nrf24_DelayMicroSeconds(uint32_t uSec)
{
	uint32_t uSecVar = uSec;
	uSecVar = uSecVar* ((SystemCoreClock/1000000)/3);
	while(uSecVar--);
}

//1. Chip Select function
void nrf24_csn(int state)
{
	if(state) HAL_GPIO_WritePin(nrf24_PORT, nrf24_CSN_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(nrf24_PORT, nrf24_CSN_PIN, GPIO_PIN_RESET);
}
//2. Chip Enable
void nrf24_ce(int state)
{
	if(state) HAL_GPIO_WritePin(nrf24_PORT, nrf24_CE_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(nrf24_PORT, nrf24_CE_PIN, GPIO_PIN_RESET);
}
//3. Read single byte from a register
uint8_t nrf24_read_register(uint8_t reg)
{
	uint8_t spiBuf[3];
	uint8_t retData;
	//Put CSN low
	nrf24_csn(0);
	//Transmit register address
	spiBuf[0] = reg&0x1F;
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 1, 100);
	//Receive data
	HAL_SPI_Receive(&nrf24_hspi, &spiBuf[1], 1, 100);
	retData = spiBuf[1];
	//Bring CSN high
	nrf24_csn(1);
	return retData;
}
//4. Read multiple bytes register
void nrf24_read_registerN(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t spiBuf[3];
	//Put CSN low
	nrf24_csn(0);
	//Transmit register address
	spiBuf[0] = reg&0x1F;
	//spiStatus = nrf24_SPI_Write(spiBuf, 1);
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 1, 100);
	//Receive data
	HAL_SPI_Receive(&nrf24_hspi, buf, len, 100);
	//Bring CSN high
	nrf24_csn(1);
}
//5. Write single byte register
void nrf24_write_register(uint8_t reg, uint8_t value)
{
	uint8_t spiBuf[3];
	//Put CSN low
	nrf24_csn(0);
	//Transmit register address and data
	spiBuf[0] = reg|0x20;
	spiBuf[1] = value;
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 2, 100);
	//Bring CSN high
	nrf24_csn(1);
}
//6. Write multipl bytes register
void nrf24_write_registerN(uint8_t reg, const uint8_t* buf, uint8_t len)
{
	uint8_t spiBuf[3];
	//Put CSN low
	nrf24_csn(0);
	//Transmit register address and data
	spiBuf[0] = reg|0x20;
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 1, 100);
	HAL_SPI_Transmit(&nrf24_hspi, (uint8_t*)buf, len, 100);
	//Bring CSN high
	nrf24_csn(1);
}
//7. Write transmit payload
void nrf24_write_payload(const void* buf, uint8_t len)
{
	uint8_t wrPayloadCmd;
	//Bring CSN low
	nrf24_csn(0);
	//Send Write Tx payload command followed by pbuf data
	wrPayloadCmd = CMD_W_TX_PAYLOAD;
	HAL_SPI_Transmit(&nrf24_hspi, &wrPayloadCmd, 1, 100);
	HAL_SPI_Transmit(&nrf24_hspi, (uint8_t *)buf, len, 100);
	//Bring CSN high
	nrf24_csn(1);
}
//8. Read receive payload
void nrf24_read_payload(void* buf, uint8_t len)
{
	uint8_t cmdRxBuf;
	//Get data length using payload size
	uint8_t data_len = MIN(len, nrf24_getPayloadSize());
	//Read data from Rx payload buffer
	nrf24_csn(0);
	cmdRxBuf = CMD_R_RX_PAYLOAD;
	HAL_SPI_Transmit(&nrf24_hspi, &cmdRxBuf, 1, 100);
	HAL_SPI_Receive(&nrf24_hspi, buf, data_len, 100);
	nrf24_csn(1);
}

//9. Flush Tx buffer
void nrf24_flush_tx(void)
{
	nrf24_write_register(CMD_FLUSH_TX, 0xFF);
}
//10. Flush Rx buffer
void nrf24_flush_rx(void)
{
	nrf24_write_register(CMD_FLUSH_RX, 0xFF);
}
//11. Get status register value
uint8_t nrf24_get_status(void)
{
	uint8_t statReg;
	statReg = nrf24_read_register(REG_STATUS);
	return statReg;
}

//12. Begin function
void nrf24_begin(GPIO_TypeDef *nrf24PORT, uint16_t nrfCSN_Pin, uint16_t nrfCE_Pin, SPI_HandleTypeDef nrfSPI)
{
	//Copy SPI handle variable
	memcpy(&nrf24_hspi, &nrfSPI, sizeof(nrfSPI));
	//Copy Pins and Port variables
	nrf24_PORT = nrf24PORT;
	nrf24_CSN_PIN = nrfCSN_Pin;
	nrf24_CE_PIN = nrfCE_Pin;
	
	//Put pins to idle state
	nrf24_csn(1);
	nrf24_ce(0);
	//5 ms initial delay
	HAL_Delay(5);
	
	//**** Soft Reset Registers default values ****//
	nrf24_write_register(0x00, 0x08);
	nrf24_write_register(0x01, 0x3f);
	nrf24_write_register(0x02, 0x03);
	nrf24_write_register(0x03, 0x03);
	nrf24_write_register(0x04, 0x03);
	nrf24_write_register(0x05, 0x02);
	nrf24_write_register(0x06, 0x0f);
	nrf24_write_register(0x07, 0x0e);
	nrf24_write_register(0x08, 0x00);
	nrf24_write_register(0x09, 0x00);
	uint8_t pipeAddrVar[6];
	pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; 
	nrf24_write_registerN(0x0A, pipeAddrVar, 5);
	pipeAddrVar[4]=0xC2; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; 
	nrf24_write_registerN(0x0B, pipeAddrVar, 5);
	nrf24_write_register(0x0C, 0xC3);
	nrf24_write_register(0x0D, 0xC4);
	nrf24_write_register(0x0E, 0xC5);
	nrf24_write_register(0x0F, 0xC6);
	pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; 
	nrf24_write_registerN(0x10, pipeAddrVar, 5);
	nrf24_write_register(0x11, 0);
	nrf24_write_register(0x12, 0);
	nrf24_write_register(0x13, 0);
	nrf24_write_register(0x14, 0);
	nrf24_write_register(0x15, 0);
	nrf24_write_register(0x16, 0);
	
	nrf24_ACTIVATE_cmd();
	nrf24_write_register(0x1c, 0);
	nrf24_write_register(0x1d, 0);
	printRadioSettings();
	//Initialise retries 15 and delay 1250 usec
	nrf24_setRetries(15, 15);
	//Initialise PA level to max (0dB)
	nrf24_setPALevel(RF24_PA_0dB);
	//Initialise data rate to 1Mbps
	nrf24_setDataRate(RF24_2MBPS);
	//Initalise CRC length to 16-bit (2 bytes)
	nrf24_setCRCLength(RF24_CRC_16);
	//Disable dynamic payload
	nrf24_disableDynamicPayloads();
	//Set payload size
	nrf24_setPayloadSize(32);
	
	//Reset status register
	nrf24_resetStatus();
	//Initialise channel to 76
	nrf24_setChannel(76);
	//Flush buffers
	nrf24_flush_tx();
	nrf24_flush_rx();
	
	nrf24_powerDown();
	
}
//13. Listen on open pipes for reading (Must call nrf24_openReadingPipe() first)
void nrf24_startListening(void)
{
	//Power up and set to RX mode
	nrf24_write_register(REG_CONFIG, nrf24_read_register(REG_CONFIG) | (1UL<<1) |(1UL <<0));
	//Restore pipe 0 address if exists
	if(pipe0_reading_address)
		nrf24_write_registerN(REG_RX_ADDR_P0, (uint8_t *)(&pipe0_reading_address), 5);
	
	//Flush buffers
	nrf24_flush_tx();
	nrf24_flush_rx();
	//Set CE HIGH to start listenning
	nrf24_ce(1);
	//Wait for 130 uSec for the radio to come on
	nrf24_DelayMicroSeconds(150);
}
//14. Stop listening (essential before any write operation)
void nrf24_stopListening(void)
{
	nrf24_ce(0);
	nrf24_flush_tx();
	nrf24_flush_rx();
}
//15. Write(Transmit data), returns true if successfully sent
bool nrf24_write( const void* buf, uint8_t len )
{
	bool retStatus;
	//Start writing
	nrf24_resetStatus();
	nrf24_startWrite(buf,len);
	//Data monitor
  uint8_t observe_tx;
  uint8_t status;
  uint32_t sent_at = HAL_GetTick();
	const uint32_t timeout = 10; //ms to wait for timeout
	do
  {
    nrf24_read_registerN(REG_OBSERVE_TX,&observe_tx,1);
		//Get status register
		status = nrf24_get_status();
  }
  while( ! ( status & ( _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) ) ) && ( HAL_GetTick() - sent_at < timeout ) );
	
//	printConfigReg();
//	printStatusReg();
	
	bool tx_ok, tx_fail;
  nrf24_whatHappened(&tx_ok,&tx_fail, &ack_payload_available);
	retStatus = tx_ok;
	if ( ack_payload_available )
  {
    ack_payload_length = nrf24_getDynamicPayloadSize();
	}
	
	//Power down
	nrf24_available();
	nrf24_flush_tx();
	return retStatus;
}
//16. Check for available data to read
bool nrf24_available(void)
{
	return nrf24_availablePipe(NULL);
}
//17. Read received data
bool nrf24_read( void* buf, uint8_t len )
{
	nrf24_read_payload( buf, len );
	uint8_t rxStatus = nrf24_read_register(REG_FIFO_STATUS) & _BV(BIT_RX_EMPTY);
	nrf24_flush_rx();
	nrf24_getDynamicPayloadSize();
	return rxStatus;
}
//18. Open Tx pipe for writing (Cannot perform this while Listenning, has to call nrf24_stopListening)
void nrf24_openWritingPipe(uint64_t address)
{
	nrf24_write_registerN(REG_RX_ADDR_P0, (uint8_t *)(&address), 5);
  nrf24_write_registerN(REG_TX_ADDR, (uint8_t *)(&address), 5);
	
	const uint8_t max_payload_size = 32;
  nrf24_write_register(REG_RX_PW_P0,MIN(payload_size,max_payload_size));
}
//19. Open reading pipe
void nrf24_openReadingPipe(uint8_t number, uint64_t address)
{
	if (number == 0)
    pipe0_reading_address = address;
	
	if(number <= 6)
	{
		if(number < 2)
		{
			//Address width is 5 bytes
			nrf24_write_registerN(nrf24_ADDR_REGS[number], (uint8_t *)(&address), 5);
		}
		else
		{
			nrf24_write_registerN(nrf24_ADDR_REGS[number], (uint8_t *)(&address), 1);
		}
		//Write payload size
		nrf24_write_register(RF24_RX_PW_PIPE[number],payload_size);
		//Enable pipe
		nrf24_write_register(REG_EN_RXADDR, nrf24_read_register(REG_EN_RXADDR) | _BV(number));
	}
	
}
//20 set transmit retries (rf24_Retries_e) and delay
void nrf24_setRetries(uint8_t delay, uint8_t count)
{
	nrf24_write_register(REG_SETUP_RETR,(delay&0xf)<<BIT_ARD | (count&0xf)<<BIT_ARC);
}

//21. Set RF channel frequency
void nrf24_setChannel(uint8_t channel)
{
	const uint8_t max_channel = 127;
  nrf24_write_register(REG_RF_CH,MIN(channel,max_channel));
}
//22. Set payload size
void nrf24_setPayloadSize(uint8_t size)
{
	const uint8_t max_payload_size = 32;
  payload_size = MIN(size,max_payload_size);
}
//23. Get payload size
uint8_t nrf24_getPayloadSize(void)
{
	return payload_size;
}
//24. Get dynamic payload size, of latest packet received
uint8_t nrf24_getDynamicPayloadSize(void)
{
	return nrf24_read_register(CMD_R_RX_PL_WID);
}
//25. Enable payload on Ackknowledge packet
void nrf24_enableAckPayload(void)
{
	//Need to enable dynamic payload and Ack payload together
	 nrf24_write_register(REG_FEATURE,nrf24_read_register(REG_FEATURE) | _BV(BIT_EN_ACK_PAY) | _BV(BIT_EN_DPL) );
	if(!nrf24_read_register(REG_FEATURE))
	{
		nrf24_ACTIVATE_cmd();
		nrf24_write_register(REG_FEATURE,nrf24_read_register(REG_FEATURE) | _BV(BIT_EN_ACK_PAY) | _BV(BIT_EN_DPL) );
	}
	// Enable dynamic payload on pipes 0 & 1
	nrf24_write_register(REG_DYNPD,nrf24_read_register(REG_DYNPD) | _BV(BIT_DPL_P1) | _BV(BIT_DPL_P0));
}
//26. Enable dynamic payloads
void nrf24_enableDynamicPayloads(void)
{
	//Enable dynamic payload through FEATURE register
	nrf24_write_register(REG_FEATURE,nrf24_read_register(REG_FEATURE) |  _BV(BIT_EN_DPL) );
	if(!nrf24_read_register(REG_FEATURE))
	{
		nrf24_ACTIVATE_cmd();
		nrf24_write_register(REG_FEATURE,nrf24_read_register(REG_FEATURE) |  _BV(BIT_EN_DPL) );
	}
	//Enable Dynamic payload on all pipes
	nrf24_write_register(REG_DYNPD,nrf24_read_register(REG_DYNPD) | _BV(BIT_DPL_P5) | _BV(BIT_DPL_P4) | _BV(BIT_DPL_P3) | _BV(BIT_DPL_P2) | _BV(BIT_DPL_P1) | _BV(BIT_DPL_P0));
  dynamic_payloads_enabled = true;
	
}
void nrf24_disableDynamicPayloads(void)
{
	nrf24_write_register(REG_FEATURE,nrf24_read_register(REG_FEATURE) &  ~(_BV(BIT_EN_DPL)) );
	//Disable for all pipes 
	nrf24_write_register(REG_DYNPD,0);
	dynamic_payloads_enabled = false;
}
//27. Check if module is nrf24L01+ or normal module
bool nrf24_isNRF_Plus(void)
{
	return p_variant;
}
//28. Set Auto Ack for all
void nrf24_setAutoAck(bool enable)
{
	if ( enable )
    nrf24_write_register(REG_EN_AA, 0x3F);
  else
    nrf24_write_register(REG_EN_AA, 0x00);
}
//29. Set Auto Ack for certain pipe
void nrf24_setAutoAckPipe( uint8_t pipe, bool enable )
{
	if ( pipe <= 6 )
  {
    uint8_t en_aa = nrf24_read_register( REG_EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    nrf24_write_register( REG_EN_AA, en_aa ) ;
  }
}
//30. Set transmit power level
void nrf24_setPALevel( rf24_pa_dbm_e level )
{
	uint8_t setup = nrf24_read_register(REG_RF_SETUP) ;
  setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == RF24_PA_0dB)
  {
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_m6dB )
  {
    setup |= _BV(RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_m12dB )
  {
    setup |= _BV(RF_PWR_LOW);
  }
  else if ( level == RF24_PA_m18dB )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }

  nrf24_write_register( REG_RF_SETUP, setup ) ;
}
//31. Get transmit power level
rf24_pa_dbm_e nrf24_getPALevel( void )
{
	rf24_pa_dbm_e result = RF24_PA_ERROR ;
  uint8_t power = nrf24_read_register(REG_RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));

  // switch uses RAM (evil!)
  if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
  {
    result = RF24_PA_0dB ;
  }
  else if ( power == _BV(RF_PWR_HIGH) )
  {
    result = RF24_PA_m6dB ;
  }
  else if ( power == _BV(RF_PWR_LOW) )
  {
    result = RF24_PA_m12dB ;
  }
  else
  {
    result = RF24_PA_m18dB ;
  }

  return result ;
}
//32. Set data rate (250 Kbps, 1Mbps, 2Mbps)
bool nrf24_setDataRate(rf24_datarate_e speed)
{
	bool result = false;
  uint8_t setup = nrf24_read_register(REG_RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  wide_band = false ;
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    wide_band = false ;
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      wide_band = true ;
      setup |= _BV(RF_DR_HIGH);
    }
    else
    {
      // 1Mbs
      wide_band = false ;
    }
  }
  nrf24_write_register(REG_RF_SETUP,setup);

  // Verify our result
  if ( nrf24_read_register(REG_RF_SETUP) == setup )
  {
    result = true;
  }
  else
  {
    wide_band = false;
  }

  return result;
}
//33. Get data rate
rf24_datarate_e nrf24_getDataRate( void )
{
	rf24_datarate_e result ;
  uint8_t dr = nrf24_read_register(REG_RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
  
  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}
//34. Set crc length (disable, 8-bits or 16-bits)
void nrf24_setCRCLength(rf24_crclength_e length)
{
	uint8_t config = nrf24_read_register(REG_CONFIG) & ~( _BV(BIT_CRCO) | _BV(BIT_EN_CRC)) ;
  
  // switch uses RAM
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above. 
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(BIT_EN_CRC);
  }
  else
  {
    config |= _BV(BIT_EN_CRC);
    config |= _BV( BIT_CRCO );
  }
  nrf24_write_register( REG_CONFIG, config );
}
//35. Get CRC length
rf24_crclength_e nrf24_getCRCLength(void)
{
	rf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = nrf24_read_register(REG_CONFIG) & ( _BV(BIT_CRCO) | _BV(BIT_EN_CRC)) ;

  if ( config & _BV(BIT_EN_CRC ) )
  {
    if ( config & _BV(BIT_CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}
//36. Disable CRC
void nrf24_disableCRC( void )
{
	uint8_t disable = nrf24_read_register(REG_CONFIG) & ~_BV(BIT_EN_CRC) ;
  nrf24_write_register( REG_CONFIG, disable ) ;
}
//37. power up
void nrf24_powerUp(void)
{
	nrf24_write_register(REG_CONFIG,nrf24_read_register(REG_CONFIG) | _BV(BIT_PWR_UP));
}
//38. power down
void nrf24_powerDown(void)
{
	nrf24_write_register(REG_CONFIG,nrf24_read_register(REG_CONFIG) & ~_BV(BIT_PWR_UP));
}
//39. Check if data are available and on which pipe (Use this for multiple rx pipes)
bool nrf24_availablePipe(uint8_t* pipe_num)
{
	uint8_t status = nrf24_get_status();

  bool result = ( status & _BV(BIT_RX_DR) );

  if (result)
  {
    // If the caller wants the pipe number, include that
    if ( pipe_num )
      *pipe_num = ( status >> BIT_RX_P_NO ) & 0x7;

    // Clear the status bit
    nrf24_write_register(REG_STATUS,_BV(BIT_RX_DR) );

    // Handle ack payload receipt
    if ( status & _BV(BIT_TX_DS) )
    {
      nrf24_write_register(REG_STATUS,_BV(BIT_TX_DS));
    }
  }
  return result;
}
//40. Start write (for IRQ mode)
void nrf24_startWrite( const void* buf, uint8_t len )
{
  // Transmitter power-up
  nrf24_ce(0);
  nrf24_write_register(REG_CONFIG, ( nrf24_read_register(REG_CONFIG) | _BV(BIT_PWR_UP) ) & ~_BV(BIT_PRIM_RX) );
  nrf24_ce(1);
  nrf24_DelayMicroSeconds(150);

  // Send the payload
  nrf24_write_payload( buf, len );

  // Enable Tx for 15usec
  nrf24_ce(1);
  nrf24_DelayMicroSeconds(15);
  nrf24_ce(0);
}
//41. Write acknowledge payload
void nrf24_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{
	const uint8_t* current = (uint8_t *)buf;
	const uint8_t max_payload_size = 32;
  uint8_t data_len = MIN(len,max_payload_size);
	
  nrf24_csn(0);
	nrf24_write_registerN(CMD_W_ACK_PAYLOAD | ( pipe & 0x7 ) , current, data_len);
  nrf24_csn(1);
}
//42. Check if an Ack payload is available
bool nrf24_isAckPayloadAvailable(void)
{
	bool result = ack_payload_available;
  ack_payload_available = false;
  return result;
}
//43. Check interrupt flags
void nrf24_whatHappened(bool *tx_ok,bool *tx_fail,bool *rx_ready)
{
	uint8_t status = nrf24_get_status();
	*tx_ok = 0;
	nrf24_write_register(REG_STATUS,_BV(BIT_RX_DR) | _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) );
  // Report to the user what happened
  *tx_ok = status & _BV(BIT_TX_DS);
  *tx_fail = status & _BV(BIT_MAX_RT);
  *rx_ready = status & _BV(BIT_RX_DR);
}
//44. Test if there is a carrier on the previous listenning period (useful to check for intereference)
bool nrf24_testCarrier(void)
{
	return nrf24_read_register(REG_CD) & 1;
}
//45. Test if a signal carrier exists (=> -64dB), only for nrf24L01+
bool nrf24_testRPD(void)
{
	return nrf24_read_register(REG_RPD) & 1;
}

//46. Reset Status
void nrf24_resetStatus(void)
{
	nrf24_write_register(REG_STATUS,_BV(BIT_RX_DR) | _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) );
}

//47. ACTIVATE cmd
void nrf24_ACTIVATE_cmd(void)
{
	uint8_t cmdRxBuf[2];
	//Read data from Rx payload buffer
	nrf24_csn(0);
	cmdRxBuf[0] = CMD_ACTIVATE;
	cmdRxBuf[1] = 0x73;
	HAL_SPI_Transmit(&nrf24_hspi, cmdRxBuf, 2, 100);
	nrf24_csn(1);
}
//48. Get AckPayload Size
uint8_t nrf24_GetAckPayloadSize(void)
{
	return ack_payload_length;
}

void printRadioSettings(void)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf, "\r\n**********************************************\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//a) Get CRC settings - Config Register
	reg8Val = nrf24_read_register(0x00);
	if(reg8Val & (1 << 3))
	{
		if(reg8Val & (1 << 2)) sprintf(uartTxBuf, "CRC:\r\n		Enabled, 2 Bytes \r\n");
		else sprintf(uartTxBuf, "CRC:\r\n		Enabled, 1 Byte \r\n");	
	}
	else
	{
		sprintf(uartTxBuf, "CRC:\r\n		Disabled \r\n");
	}
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//b) AutoAck on pipes
	reg8Val = nrf24_read_register(0x01);
	sprintf(uartTxBuf, "ENAA:\r\n		P0:	%d\r\n		P1:	%d\r\n		P2:	%d\r\n		P3:	%d\r\n		P4:	%d\r\n		P5:	%d\r\n",
	_BOOL(reg8Val&(1<<0)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<2)), _BOOL(reg8Val&(1<<3)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<5)));
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//c) Enabled Rx addresses
	reg8Val = nrf24_read_register(0x02);
	sprintf(uartTxBuf, "EN_RXADDR:\r\n		P0:	%d\r\n		P1:	%d\r\n		P2:	%d\r\n		P3:	%d\r\n		P4:	%d\r\n		P5:	%d\r\n",
	_BOOL(reg8Val&(1<<0)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<2)), _BOOL(reg8Val&(1<<3)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<5)));
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//d) Address width
	reg8Val = nrf24_read_register(0x03)&0x03;
	reg8Val +=2;
	sprintf(uartTxBuf, "SETUP_AW:\r\n		%d bytes \r\n", reg8Val);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//e) RF channel
	reg8Val = nrf24_read_register(0x05);
	sprintf(uartTxBuf, "RF_CH:\r\n		%d CH \r\n", reg8Val&0x7F);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//f) Data rate & RF_PWR
	reg8Val = nrf24_read_register(0x06);
	if(reg8Val & (1 << 3)) sprintf(uartTxBuf, "Data Rate:\r\n		2Mbps \r\n");
	else sprintf(uartTxBuf, "Data Rate:\r\n		1Mbps \r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	reg8Val &= (3 << 1);
	reg8Val = (reg8Val>>1);
	if(reg8Val == 0) sprintf(uartTxBuf, "RF_PWR:\r\n		-18dB \r\n");
	else if(reg8Val == 1) sprintf(uartTxBuf, "RF_PWR:\r\n		-12dB \r\n");
	else if(reg8Val == 2) sprintf(uartTxBuf, "RF_PWR:\r\n		-6dB \r\n");
	else if(reg8Val == 3) sprintf(uartTxBuf, "RF_PWR:\r\n		0dB \r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//g) RX pipes addresses
	uint8_t pipeAddrs[6];
	nrf24_read_registerN(0x0A, pipeAddrs, 5);
	sprintf(uartTxBuf, "RX_Pipe0 Addrs:\r\n		%02X,%02X,%02X,%02X,%02X  \r\n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	nrf24_read_registerN(0x0A+1, pipeAddrs, 5);
	sprintf(uartTxBuf, "RX_Pipe1 Addrs:\r\n		%02X,%02X,%02X,%02X,%02X  \r\n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	nrf24_read_registerN(0x0A+2, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe2 Addrs:\r\n		xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	nrf24_read_registerN(0x0A+3, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe3 Addrs:\r\n		xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	nrf24_read_registerN(0x0A+4, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe4 Addrs:\r\n		xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	nrf24_read_registerN(0x0A+5, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe5 Addrs:\r\n		xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	nrf24_read_registerN(0x0A+6, pipeAddrs, 5);
	sprintf(uartTxBuf, "TX Addrs:\r\n		%02X,%02X,%02X,%02X,%02X  \r\n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	//h) RX PW (Payload Width 0 - 32)
	reg8Val = nrf24_read_register(0x11);
	sprintf(uartTxBuf, "RX_PW_P0:\r\n		%d bytes \r\n", reg8Val&0x3F);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = nrf24_read_register(0x11+1);
	sprintf(uartTxBuf, "RX_PW_P1:\r\n		%d bytes \r\n", reg8Val&0x3F);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = nrf24_read_register(0x11+2);
	sprintf(uartTxBuf, "RX_PW_P2:\r\n		%d bytes \r\n", reg8Val&0x3F);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = nrf24_read_register(0x11+3);
	sprintf(uartTxBuf, "RX_PW_P3:\r\n		%d bytes \r\n", reg8Val&0x3F);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = nrf24_read_register(0x11+4);
	sprintf(uartTxBuf, "RX_PW_P4:\r\n		%d bytes \r\n", reg8Val&0x3F);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = nrf24_read_register(0x11+5);
	sprintf(uartTxBuf, "RX_PW_P5:\r\n		%d bytes \r\n", reg8Val&0x3F);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	//i) Dynamic payload enable for each pipe
	reg8Val = nrf24_read_register(0x1c);
	sprintf(uartTxBuf, "DYNPD_pipe:\r\n		P0:	%d\r\n		P1:	%d\r\n		P2:	%d\r\n		P3:	%d\r\n		P4:	%d\r\n		P5:	%d\r\n",
	_BOOL(reg8Val&(1<<0)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<2)), _BOOL(reg8Val&(1<<3)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<5)));
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	//j) EN_DPL (is Dynamic payload feature enabled ?)
	reg8Val = nrf24_read_register(0x1d);
	if(reg8Val&(1<<2)) sprintf(uartTxBuf, "EN_DPL:\r\n		Enabled \r\n");
	else sprintf(uartTxBuf, "EN_DPL:\r\n		Disabled \r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	//k) EN_ACK_PAY
	if(reg8Val&(1<<1)) sprintf(uartTxBuf, "EN_ACK_PAY:\r\n		Enabled \r\n");
	else sprintf(uartTxBuf, "EN_ACK_PAY:\r\n		Disabled \r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	
	sprintf(uartTxBuf, "\r\n**********************************************\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
}

//2. Print Status 
void printStatusReg(void)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = nrf24_read_register(0x07);
	sprintf(uartTxBuf, "STATUS reg:\r\n		RX_DR:		%d\r\n		TX_DS:		%d\r\n		MAX_RT:		%d\r\n		RX_P_NO:	%d\r\n		TX_FULL:	%d\r\n",
	_BOOL(reg8Val&(1<<6)), _BOOL(reg8Val&(1<<5)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(3<<1)), _BOOL(reg8Val&(1<<0)));
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
}
//3. Print Config 
void printConfigReg(void)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = nrf24_read_register(0x00);
	sprintf(uartTxBuf, "CONFIG reg:\r\n		PWR_UP:		%d\r\n		PRIM_RX:	%d\r\n",
	_BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<0)));
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
}

//4. Init Variables
void nrf24_DebugUART_Init(UART_HandleTypeDef nrf24Uart)
{
	memcpy(&nrf24_huart, &nrf24Uart, sizeof(nrf24Uart));
}
//5. FIFO Status
void printFIFOstatus(void)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = nrf24_read_register(0x17);
	sprintf(uartTxBuf, "FIFO Status reg:\r\n		TX_FULL:		%d\r\n		TX_EMPTY:		%d\r\n		RX_FULL:		%d\r\n		RX_EMPTY:		%d\r\n",
	_BOOL(reg8Val&(1<<5)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<0)));
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
}
