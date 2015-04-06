
#include "stm32_f107_uart.h"
#include "log.h"

//#define PORT_NFC USART2
//#define PORT_LOG USART1

#define SIZE_RX_BUFFER 1024
#define SIZE_TX_BUFFER 2048

u8 RxBuffer[SIZE_RX_BUFFER];

u32 RxFront = 0;
u32 RxRear = 0;

u8 TxBuffer[SIZE_TX_BUFFER];
u32 TxFront = 0;
u32 TxRear = 0;

bool isItEnabled = false;
u8 flag_U1_rev_finish;//串口接收到数据的标志， 1为接收到
u8 RxBufferU1[30];
u32 RxCounterU1=0;
u8 testbyte[8]={0x01,0x56,0x12,0x34,0x56,0x78,0xAB,0x01};


void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {    
        TIM_Cmd(TIM3, ENABLE);
        TIM3->CNT &= 0x0000;//定时器延时设置  每次接受清零 当接受间隔超过定时器定的值时  进入tim中断  认为一帧接受完成
       
        RxBufferU1[RxCounterU1++] = USART_ReceiveData(USART1);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        
        //flag_U1_rev_finish++;
        //测试NFC模块与主机通讯是否有问题，收到主机发送的指令，则返回相应的指令
        if((RxBufferU1[2]==0x12)&&(RxBufferU1[3]==0x34)&&(RxBufferU1[4]==0x56)&&(RxBufferU1[5]==0x78))
        {
            SendDataU1(testbyte,8);
            RxBufferU1[2]=0;
            RxBufferU1[3]=0;
            RxBufferU1[4]=0;
            RxBufferU1[5]=0;
        }
        
    }
  
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
      if(TxRear == TxFront){
       USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
       isItEnabled = false;
       return;
      } else{
        USART_SendData(USART1,TxBuffer[TxRear]);
        //while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        TxRear = (TxRear+1)%SIZE_TX_BUFFER;
      }
    }
}

/**
  * @brief  This function handles USARTy global interrupt request.
  * @param  None
  * @retval None
  */

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        if( ((RxFront+1)%SIZE_RX_BUFFER) == RxRear) return; // buffer is full
        RxBuffer[RxFront] = USART_ReceiveData(USART2);
        RxFront = (RxFront+1)%SIZE_RX_BUFFER;
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}


void TIM3_Configuration(void)
{ 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //UART1
  //  TIM3 Configuration:向上计数中断:
   
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 720;
  TIM_TimeBaseStructure.TIM_Prescaler = 10000-1;//
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update , ENABLE);//开启计数中断
  
}
void TIM3_IRQHandler(void)
{
   TIM_Cmd(TIM3, DISABLE);
   TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
 /****copy Uart0_rev_count的值****/
   RxCounterU1=0;
   flag_U1_rev_finish=1;
}

void InitUart(){
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

  //GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);


    //USART2_TX   PA.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9
  
  //USART2_RX	  PA.3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10
    
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USARTx_Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USARTx_Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  USART_InitStructure.USART_Mode = USART_Mode_Rx |USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
  USART_GetFlagStatus(USART1, USART_FLAG_TC);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE);

  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  // USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  USART_Cmd(USART2, ENABLE);
  RxFront = 0;
  RxRear = 0;
}

void PutToLogBuffer(u8 ch){
  if((TxFront+1)%SIZE_TX_BUFFER == TxRear){return ;}//full
  TxBuffer[TxFront] = ch;
  TxFront = (TxFront+1)%SIZE_TX_BUFFER;
  if(!isItEnabled){
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  }
}

int ReadByte(u32 timeout){
  u8 b;
  while (RxRear == RxFront );//empty-- block untill received
  b = RxBuffer[RxRear];
  RxRear = (RxRear+1)%SIZE_RX_BUFFER;
  return b;
}

int SendByte(u8 byte, u32 timeout){
  USART_SendData(USART2, byte);
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);//block untill finished
  return 0;
}

//往USART1，发送 length长度的数据data
void SendDataU1(u8 *data,u8 length)
{
    u8 i;
    for(i=0;i<length;i++)
    {
        USART_SendData(USART1, data[i]);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)//等得发送完成
        {
        }  
    }
}

