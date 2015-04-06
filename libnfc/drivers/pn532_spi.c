/*-
 * Free/Libre Near Field Communication (NFC) library
 *
 * Libnfc historical contributors:
 * Copyright (C) 2009      Roel Verdult
 * Copyright (C) 2009-2013 Romuald Conty
 * Copyright (C) 2010-2012 Romain Tarti猫re
 * Copyright (C) 2010-2013 Philippe Teuwen
 * Copyright (C) 2012-2013 Ludovic Rousseau
 * See AUTHORS file for a more comprehensive list of contributors.
 * Additional contributors of this file:
 * Copyright (C) 2013      Evgeny Boger
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

/**
 * @file pn532_spi.c
 * @brief PN532 driver using SPI bus
 */

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif // HAVE_CONFIG_H

#include "pn532_spi.h"

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <nfc/nfc.h>

#include "drivers.h"
#include "nfc-internal.h"
#include "chips/pn53x.h"
#include "chips/pn53x-internal.h"

#include "stm32f10x.h"
#include "stm32f10x_exti.h"
/***************************************************************************
*                          宏定义
***************************************************************************/
#define NFC_ADDR_W  0X48    //I2C写地址
#define NFC_ADDR_R  0X49    //I2C读地址

#define NFC_DW      0X01    //SPI写指令
#define NFC_SR      0X02    //SPI状态
#define NFC_DR      0X03    //SPI读指示

#define NFC_NSS_L   (GPIOA->ODR &= ~GPIO_Pin_4)     //PA4
#define NFC_NSS_H   (GPIOA->ODR |= GPIO_Pin_4)      //PA4

uint8_t Exti4Flag = 0X00;


#define PN532_SPI_DEFAULT_SPEED 1000000 // 1 MHz
#define PN532_SPI_DRIVER_NAME "pn532_spi"
#define PN532_SPI_MODE SPI_MODE_0

#define LOG_CATEGORY "libnfc.driver.pn532_spi"
#define LOG_GROUP    NFC_LOG_GROUP_DRIVER

/******************************************************************
**                                                       
**  Name		: Exti4Init                                         
**  Description	: 外部中断4初始化
**                                                                         
**  Parameters  : None
**  Returns     : None                                                    
**                                                   
*******************************************************************/
void Exti4Init(void)
{
    EXTI_InitTypeDef    EXTI_InitStructure;
    GPIO_InitTypeDef    GPIO_InitStructure; 
    NVIC_InitTypeDef    NVIC_InitStructure; 
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource4);                 //IO口与中断线连接 将C口连在4号线上 即PC4为中断输入口
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;                     //下降沿
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;                //优先级设置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);   
}

/******************************************************************
**                                                       
**  Name	: EXTI4_IRQHandler                                         
**  Description	: 外部中断4中断处理函数
**                                                                         
**  Parameters  : None
**  Returns     : None                                                    
**                                                   
*******************************************************************/
void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
	    Exti4Flag = 0X01;        
	    /* Clear the  EXTI line 4 pending bit */
	    EXTI_ClearITPendingBit(EXTI_Line4);
    }    
}


/******************************************************************
**                                                       
**  Name	: SPI1_Init                                         
**  Description	: SPI1初始化，主机模式
**                                                                         
**  Parameters  : 无
**  Returns     : 无                                                     
**                                                   
*******************************************************************/
void SPI1_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    SPI_InitTypeDef    SPI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);      //外设时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    //  PA5 SPI1_SCK
    //  PA6 SPI1_MISO
    //  PA7 SPI1_MOSI
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;           // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;        //双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                             //主机
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                         //八位数据
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                                //时钟线空闲低电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                              //数据采样 锁存 从第1个时钟边沿开始。
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                                 //软件控制NSS位 NSS从设备选择引脚 需要SSOE位使能后起作用
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;      //波特率预分频的值
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;                        //低位在前
    SPI_InitStructure.SPI_CRCPolynomial = 7;                                  //CRC校验选择 需要CRCEN位置位后起作用
    SPI_Init(SPI1, &SPI_InitStructure);
    
    SPI_Cmd(SPI1, ENABLE);
}

/******************************************************************
**                                                       
**  Name		: Write_Read_SPI1                                         
**  Description	: SPI1 读写函数，全双工，时钟同步，读写同时进行
**  				注意：SPI全双工通信模式下，每次发送完成后， 
**					都必须等待接收非空标识位置位， 
**					然后读DR清RXNE ，才能再发送下一个数据                                                                       
**  Parameters  : 写入的数据
**  Returns     : 读出的数据                                                     
**                                                   
*******************************************************************/
uint8_t Write_Read_SPI1(uint8_t Byte)
{
    uint16_t i;
    uint8_t BUFF;
    for(i=0; i<65530; i++)
    {
        if( SPI1->SR&0x02 )  //等待上次发送完成
        {
            SPI1->DR = Byte;                                      
            break;
        }
    }
    for(i=0; i<65530; i++)
    {
        if( SPI1->SR&0x01 )   //等待接收完成
        {
            BUFF = SPI1->DR;
            break;
        }
    }

    return BUFF;
}

/******************************************************************
**                                                       
**  Name		: SpiNssInit                                        
**  Description	: NFC的NSS引脚初始化 PA4 SPI接口的NSS引脚单独控制
**
**  Parameters  : 无
**  Returns     : 无                                                                                  
**                                                   
*******************************************************************/
void SpiNssInit(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;			
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;		                    //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		                    //最高输出速率50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);				 	                    //选择端口
}



int
spi_send(const uint8_t *pbtTx, const size_t szTx){
  uint8_t i;
  NFC_NSS_L;                      //选通SPI
  Write_Read_SPI1(NFC_DW);        //写数据
  for(i=0; i<szTx; i++)
  {
      Write_Read_SPI1( pbtTx[i] );
  }
  NFC_NSS_H;                      //关闭SPI
  return 0;
}

int
spi_receive(uint8_t *pbtRx, const size_t szRx, bool lsb_first){
  uint8_t i;
  NFC_NSS_L;
  Write_Read_SPI1(NFC_DR);
  for(i=0; i<szRx; i++)
  {
     pbtRx[i]= Write_Read_SPI1( NFC_DR );
  }
  NFC_NSS_H;
  return 0;
}



/******************************************************************
**                                                       
**  Name		: NfcDelayMs                                         
**  Description	: 1 Ms延时
**
**  Parameters  : 无
**  Returns     : 无                                          
**                                                   
*******************************************************************/
void msleep(unsigned short int mCount)
{
    volatile unsigned long i;
    i = (unsigned long)( mCount*2570 );    //1 Ms延时 
    while (i--)
    {
        __NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();
    }
}


// Internal data structs
const struct pn53x_io pn532_spi_io;
struct pn532_spi_data {
  void* port;
  volatile bool abort_flag;
};

// Prototypes
int     pn532_spi_ack(nfc_device *pnd);
int     pn532_spi_wakeup(nfc_device *pnd);

#define DRIVER_DATA(pnd) ((struct pn532_spi_data*)(pnd->driver_data))

static size_t
pn532_spi_scan(const void* ports[],  size_t szPorts)
{
  size_t device_found = 0;
  return device_found;
}

struct pn532_spi_descriptor {
  char *port;
  uint32_t speed;
};

static void
pn532_spi_close(nfc_device *pnd)
{
  pn53x_idle(pnd);

  pn53x_data_free(pnd);
  nfc_device_free(pnd);
}

static nfc_device *
pn532_spi_open(const void* port)
{

  nfc_device *pnd = NULL;

  pnd = nfc_device_new();
  if (!pnd) {
    return NULL;
  }

  pnd->driver_data = malloc(sizeof(struct pn532_spi_data));
  if (!pnd->driver_data) {
    nfc_device_free(pnd);
    return NULL;
  }
  //DRIVER_DATA(pnd)->port = port;
  
  SPI1_Init();        /* NFC_SPI初始化 PA5-SCK PA6-MISO PA7-MOSI*/
  SpiNssInit();       /* NFC_NSS初始化 PA4-NSS SPI接口的NSS引脚单独控制 */
  Exti4Init();        /* NFC_IRQ初始化 PE4 */
  
  pn532_spi_wakeup(pnd);     /* SPI接口唤醒 */


  // Alloc and init chip's data
  if (pn53x_data_new(pnd, &pn532_spi_io) == NULL) {
    nfc_device_free(pnd);
    return NULL;
  }
  // SAMConfiguration command if needed to wakeup the chip and pn53x_SAMConfiguration check if the chip is a PN532
  CHIP_DATA(pnd)->type = PN532;
  // This device starts in LowVBat mode
  CHIP_DATA(pnd)->power_mode = LOWVBAT;

  // empirical tuning
  CHIP_DATA(pnd)->timer_correction = 48;
  pnd->driver = &pn532_spi_driver;

  DRIVER_DATA(pnd)->abort_flag = false;

  // Check communication using "Diagnose" command, with "Communication test" (0x00)
  if (pn53x_check_communication(pnd) < 0) {
    printf("[Error][pn532_uart] pn53x_check_communication error\r\n");
    pn532_spi_close(pnd);
    return NULL;
  }

  pn53x_init(pnd);
  return pnd;
}

int
pn532_spi_wakeup(nfc_device *pnd)
{
  /* SPI wakeup is basically activating chipselect for several ms.
   * To do so, we are sending harmless command at very low speed  */
  NFC_NSS_L;
  msleep(10);
  NFC_NSS_H;
  return 0;
}

#define PN532_BUFFER_LEN (PN53x_EXTENDED_FRAME__DATA_MAX_LEN + PN53x_EXTENDED_FRAME__OVERHEAD)


static int
pn532_spi_wait_for_data(nfc_device *pnd, int timeout)
{
  static const uint8_t pn532_spi_ready = 0x01;
  static const int pn532_spi_poll_interval = 10; //ms


  int timer = 0;

  int ret;
  while ( Exti4Flag != pn532_spi_ready) {
    if (ret < 0) {
      return ret;
    }

    if (DRIVER_DATA(pnd)->abort_flag) {
      DRIVER_DATA(pnd)->abort_flag = false;
      return NFC_EOPABORTED;
    }

    if (timeout > 0) {
      timer += pn532_spi_poll_interval;
      if (timer > timeout) {
        return NFC_ETIMEOUT;
      }

      msleep(pn532_spi_poll_interval);
    }
  }
  Exti4Flag = 0;

  return NFC_SUCCESS;
}


static int
pn532_spi_receive(nfc_device *pnd, uint8_t *pbtData, const size_t szDataLen, int timeout)
{
  uint8_t  abtRxBuf[5];
  size_t len;

  pnd->last_error = pn532_spi_wait_for_data(pnd, timeout);

  if (NFC_EOPABORTED == pnd->last_error) {
    return pn532_spi_ack(pnd);
  }

  if (pnd->last_error != NFC_SUCCESS) {
    printf("[Error][pn532_uart] %s",  "Unable to wait for SPI data. (RX)");
    goto error;
  }
/*
  pnd->last_error = spi_send_receive(DRIVER_DATA(pnd)->port, &pn532_spi_cmd_dataread, 1, abtRxBuf , 4, true);

  if (pnd->last_error < 0) {
    goto error;
  }
*/
  NFC_NSS_L;  //Begin to read stream
  for (size_t i = 0; i < 4; ++i) {
    abtRxBuf[i] = Write_Read_SPI1(NFC_DR);
  }

  const uint8_t pn53x_long_preamble[3] = { 0x00, 0x00, 0xff };
  if (0 == (memcmp(abtRxBuf, pn53x_long_preamble, 3))) {
    // long preamble

    // omit first byte
    for (size_t i = 0; i < 3; ++i) {
      abtRxBuf[i] = abtRxBuf[i + 1];
    }

    // need one more byte
  /*
    pnd->last_error = pn532_spi_receive_next_chunk(pnd, abtRxBuf + 3, 1);
    if (pnd->last_error != 0) {
      printf("[Error][pn532_uart] %s",  "Unable to receive one more byte for long preamble frame. (RX)");
      goto error;
    }
    */
    abtRxBuf[3] = Write_Read_SPI1(NFC_DR);
  }


  const uint8_t pn53x_preamble[2] = { 0x00, 0xff };
  if (0 != (memcmp(abtRxBuf, pn53x_preamble, 2))) {
    printf("[Error][pn532_uart] %s",  " preamble+start code mismatch");
    pnd->last_error = NFC_EIO;
    goto error;
  }

  if ((0x01 == abtRxBuf[2]) && (0xff == abtRxBuf[3])) {
    // Error frame
    //pn532_spi_receive_next_chunk(pnd, abtRxBuf, 3);
    for (size_t i = 0; i < 3; ++i) {
      abtRxBuf[i] = Write_Read_SPI1(NFC_DR);
    }

    printf("[Error][pn532_uart] %s",  "Application level error detected");
    pnd->last_error = NFC_EIO;
    goto error;
  } else if ((0xff == abtRxBuf[2]) && (0xff == abtRxBuf[3])) {
    // Extended frame
    //pnd->last_error = pn532_spi_receive_next_chunk(pnd, abtRxBuf, 3);

    //if (pnd->last_error != 0) {
    //  printf("[Error][pn532_uart] %s",  "Unable to receive data. (RX)");
    //  goto error;
    //}
    for (size_t i = 0; i < 3; ++i) {
      abtRxBuf[i] = Write_Read_SPI1(NFC_DR);
    }

    // (abtRxBuf[0] << 8) + abtRxBuf[1] (LEN) include TFI + (CC+1)
    len = (abtRxBuf[0] << 8) + abtRxBuf[1] - 2;
    if (((abtRxBuf[0] + abtRxBuf[1] + abtRxBuf[2]) % 256) != 0) {
      printf("[Error][pn532_uart] %s",  "Length checksum mismatch");
      pnd->last_error = NFC_EIO;
      goto error;
    }
  } else {
    // Normal frame
    if (256 != (abtRxBuf[2] + abtRxBuf[3])) {
      // TODO: Retry
      printf("[Error][pn532_uart] %s",  "Length checksum mismatch");
      pnd->last_error = NFC_EIO;
      goto error;
    }

    // abtRxBuf[3] (LEN) include TFI + (CC+1)
    len = abtRxBuf[2] - 2;
  }

  if (len > szDataLen) {
    log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR, "Unable to receive data: buffer too small. (szDataLen: %zu, len: %zu)", szDataLen, len);
    pnd->last_error = NFC_EIO;
    goto error;
  }

  // TFI + PD0 (CC+1)

  //pnd->last_error = pn532_spi_receive_next_chunk(pnd, abtRxBuf, 2);

  //if (pnd->last_error != 0) {
  //  printf("[Error][pn532_uart] %s",  "Unable to receive data. (RX)");
  //  goto error;
  //}
  for (size_t i = 0; i < 2; ++i) {
        abtRxBuf[i] = Write_Read_SPI1(NFC_DR);
  }

  if (abtRxBuf[0] != 0xD5) {
    printf("[Error][pn532_uart] %s",  "TFI Mismatch");
    pnd->last_error = NFC_EIO;
    goto error;
  }

  if (abtRxBuf[1] != CHIP_DATA(pnd)->last_command + 1) {
    printf("[Error][pn532_uart] %s",  "Command Code verification failed");
    pnd->last_error = NFC_EIO;
    goto error;
  }

  if (len) {
    //pnd->last_error = pn532_spi_receive_next_chunk(pnd, pbtData, len);

    //if (pnd->last_error != 0) {
    //  printf("[Error][pn532_uart] %s",  "Unable to receive data. (RX)");
    //  goto error;
    //}
    for (size_t i = 0; i < len; ++i) {
      pbtData[i] = Write_Read_SPI1(NFC_DR);
    }
  }

  //pnd->last_error = pn532_spi_receive_next_chunk(pnd, abtRxBuf, 2);

  //if (pnd->last_error != 0) {
  //  printf("[Error][pn532_uart] %s",  "Unable to receive data. (RX)");
  //  goto error;
  //}
  for (size_t i = 0; i < 2; ++i) {
     abtRxBuf[i] = Write_Read_SPI1(NFC_DR);
  }
  NFC_NSS_H;//Ending

  uint8_t btDCS = (256 - 0xD5);
  btDCS -= CHIP_DATA(pnd)->last_command + 1;
  for (size_t szPos = 0; szPos < len; szPos++) {
    btDCS -= pbtData[szPos];
  }

  if (btDCS != abtRxBuf[0]) {
    printf("[Error][pn532_uart] %s",  "Data checksum mismatch");
    pnd->last_error = NFC_EIO;
    goto error;
  }

  if (0x00 != abtRxBuf[1]) {
    printf("[Error][pn532_uart] %s",  "Frame postamble mismatch");
    pnd->last_error = NFC_EIO;
    goto error;
  }
  // The PN53x command is done and we successfully received the reply
  return len;
error:
  return pnd->last_error;
}

static int
pn532_spi_send(nfc_device *pnd, const uint8_t *pbtData, const size_t szData, int timeout)
{
  int res = 0;

  switch (CHIP_DATA(pnd)->power_mode) {
    case LOWVBAT: {
      /** PN532C106 wakeup. */
      if ((res = pn532_spi_wakeup(pnd)) < 0) {
        return res;
      }
      // According to PN532 application note, C106 appendix: to go out Low Vbat mode and enter in normal mode we need to send a SAMConfiguration command
      if ((res = pn532_SAMConfiguration(pnd, PSM_NORMAL, 1000)) < 0) {
        return res;
      }
    }
    break;
    case POWERDOWN: {
      if ((res = pn532_spi_wakeup(pnd)) < 0) {
        return res;
      }
    }
    break;
    case NORMAL:
      // Nothing to do :)
      break;
  };

  uint8_t  abtFrame[PN532_BUFFER_LEN] = { 0x00, 0x00, 0xff };       // SPI data transfer starts with DATAWRITE (0x01) byte,  Every packet must start with "00 00 ff"
  size_t szFrame = 0;

  if ((res = pn53x_build_frame(abtFrame, &szFrame, pbtData, szData)) < 0) {
    pnd->last_error = res;
    return pnd->last_error;
  }

  res = spi_send(/*DRIVER_DATA(pnd)->port,*/ abtFrame, szFrame);
  if (res != 0) {
    /*log_put*/ printf("[Error][pn532_uart] %s", "Unable to transmit data. (TX)\r\n");
    pnd->last_error = res;
    return pnd->last_error;
  }

  res = pn532_spi_wait_for_data(pnd, timeout);
  if (res != NFC_SUCCESS) {
    /*log_put*/ printf("[Debug][pn532_uart] %s", "Unable to wait for SPI data. (RX)\r\n");
    pnd->last_error = res;
    return pnd->last_error;
  }



  uint8_t abtRxBuf[PN53x_ACK_FRAME__LEN];
  res = spi_receive(/*DRIVER_DATA(pnd)->port*/abtRxBuf, sizeof(abtRxBuf), true);

  if (res != 0) {
    /*log_put*/ printf("[Debug][pn532_uart] %s", "Unable to read ACK\r\n");
    pnd->last_error = res;
    return pnd->last_error;
  }

  if (pn53x_check_ack_frame(pnd, abtRxBuf, sizeof(abtRxBuf)) == 0) {
    // The PN53x is running the sent command
  } else {
    return pnd->last_error;
  }
  return NFC_SUCCESS;
}


int
pn532_spi_ack(nfc_device *pnd)
{
  int res = spi_send(pn53x_ack_frame, sizeof(pn53x_ack_frame));
  return res;
}

static int
pn532_spi_abort_command(nfc_device *pnd)
{
  if (pnd) {
    DRIVER_DATA(pnd)->abort_flag = true;
  }

  return NFC_SUCCESS;
}

const struct pn53x_io pn532_spi_io = {
  .send       = pn532_spi_send,
  .receive    = pn532_spi_receive,
};

const struct nfc_driver pn532_spi_driver = {
  .name                             = PN532_SPI_DRIVER_NAME,
  .scan_type                        = INTRUSIVE,
  .scan                             = pn532_spi_scan,
  .open                             = pn532_spi_open,
  .close                            = pn532_spi_close,
  .strerror                         = pn53x_strerror,

  .initiator_init                   = pn53x_initiator_init,
  .initiator_init_secure_element    = pn532_initiator_init_secure_element,
  .initiator_select_passive_target  = pn53x_initiator_select_passive_target,
  .initiator_poll_target            = pn53x_initiator_poll_target,
  .initiator_select_dep_target      = pn53x_initiator_select_dep_target,
  .initiator_deselect_target        = pn53x_initiator_deselect_target,
  .initiator_transceive_bytes       = pn53x_initiator_transceive_bytes,
  .initiator_transceive_bits        = pn53x_initiator_transceive_bits,
  .initiator_transceive_bytes_timed = pn53x_initiator_transceive_bytes_timed,
  .initiator_transceive_bits_timed  = pn53x_initiator_transceive_bits_timed,
  .initiator_target_is_present      = pn53x_initiator_target_is_present,

  .target_init           = pn53x_target_init,
  .target_send_bytes     = pn53x_target_send_bytes,
  .target_receive_bytes  = pn53x_target_receive_bytes,
  .target_send_bits      = pn53x_target_send_bits,
  .target_receive_bits   = pn53x_target_receive_bits,

  .device_set_property_bool     = pn53x_set_property_bool,
  .device_set_property_int      = pn53x_set_property_int,
  .get_supported_modulation     = pn53x_get_supported_modulation,
  .get_supported_baud_rate      = pn53x_get_supported_baud_rate,
  .device_get_information_about = pn53x_get_information_about,

  .abort_command  = pn532_spi_abort_command,
  .idle           = pn53x_idle,
  .powerdown      = pn53x_PowerDown,
};
