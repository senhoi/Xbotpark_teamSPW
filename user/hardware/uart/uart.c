#include "uart.h"

uint8_t UART_Printf(UART_Frame_t *tx, char *fmt, ...)
{
	if (tx->ready == false)
		return 0;

	tx->sum = 0;
	tx->idx = 0;

	va_list ap;
	va_start(ap, fmt);
	vsprintf((char *)tx->buf, fmt, ap);
	va_end(ap);

	tx->len = strlen((const char *)tx->buf);

	for (int i = 0; i < tx->len; i++)
	{
		tx->sum += tx->buf[i];
	}

	tx->buf_size = tx->len;

	return 1;
}

uint8_t UART_SendArr_8b(UART_Frame_t *tx, void *arr, uint8_t len)
{
	if (tx->ready == false)
		return 0;

	tx->sum = 0;
	tx->idx = 0;
	tx->len = len;

	if (tx->len > UART_MAX_BUF)
		return 0;

	tx->buf[0] = tx->head >> 8;
	tx->buf[1] = tx->head;
	tx->buf[2] = tx->addr;
	tx->buf[3] = tx->func;
	tx->buf[4] = tx->len;

	for (int i = 0; i < tx->len; i++)
	{
		*(tx->buf + 5 + i) = *((uint8_t *)arr + i);
		tx->sum += *((uint8_t *)arr + i);
	}

	tx->buf[5 + tx->len] = tx->sum;
	tx->buf[6 + tx->len] = tx->tail >> 8;
	tx->buf[7 + tx->len] = tx->tail;

	tx->buf_size = len + 8;

	return 1;
}

uint8_t UART_SendArr_16b(UART_Frame_t *tx, void *arr, uint8_t len)
{
	if (tx->ready == false)
		return 0;

	tx->sum = 0;
	tx->idx = 0;
	tx->len = len * 2;

	if (tx->len > UART_MAX_BUF)
		return 0;

	tx->buf[0] = tx->head >> 8;
	tx->buf[1] = tx->head;
	tx->buf[2] = tx->addr;
	tx->buf[3] = tx->func;
	tx->buf[4] = tx->len;

	for (int i = 0; i < tx->len; i++)
	{
		*(tx->buf + 5 + i) = *((uint8_t *)arr + i);
		tx->sum += *((uint8_t *)arr + i);
	}

	tx->buf[5 + tx->len] = tx->sum;
	tx->buf[6 + tx->len] = tx->tail >> 8;
	tx->buf[7 + tx->len] = tx->tail;

	tx->buf_size = tx->len + 8;

	return 1;
}

uint8_t UART_SendArr_32b(UART_Frame_t *tx, void *arr, uint8_t len)
{
	if (tx->ready == false)
		return 0;

	tx->sum = 0;
	tx->idx = 0;
	tx->len = len * 4;

	if (tx->len > UART_MAX_BUF)
		return 0;

	tx->buf[0] = tx->head >> 8;
	tx->buf[1] = tx->head;
	tx->buf[2] = tx->addr;
	tx->buf[3] = tx->func;
	tx->buf[4] = tx->len;

	for (int i = 0; i < tx->len; i++)
	{
		*(tx->buf + 5 + i) = *((uint8_t *)arr + i);
		tx->sum += *((uint8_t *)arr + i);
	}

	tx->buf[5 + tx->len] = tx->sum;
	tx->buf[6 + tx->len] = tx->tail >> 8;
	tx->buf[7 + tx->len] = tx->tail;

	tx->buf_size = tx->len + 8;

	return 1;
}

uint8_t UART_SendArr_64b(UART_Frame_t *tx, void *arr, uint8_t len)
{
	if (tx->ready == false)
		return 0;

	tx->sum = 0;
	tx->idx = 0;
	tx->len = len * 8;

	if (tx->len > UART_MAX_BUF)
		return 0;

	tx->buf[0] = tx->head >> 8;
	tx->buf[1] = tx->head;
	tx->buf[2] = tx->addr;
	tx->buf[3] = tx->func;
	tx->buf[4] = tx->len;

	for (int i = 0; i < tx->len; i++)
	{
		*(tx->buf + 5 + i) = *((uint8_t *)arr + i);
		tx->sum += *((uint8_t *)arr + i);
	}

	tx->buf[5 + tx->len] = tx->sum;
	tx->buf[6 + tx->len] = tx->tail >> 8;
	tx->buf[7 + tx->len] = tx->tail;

	tx->buf_size = tx->len + 8;

	return 1;
}

uint8_t UART_GetArr(UART_Frame_t *rx)
{
	uint8_t sum = 0;

	if (rx->ready != true)
		return 0;

	rx->ready = false;

	if ((uint16_t)(rx->buf[0] << 8 | rx->buf[1]) != rx->head)
		return 0;

	for (int i = 0; i < rx->buf[4]; i++)
		sum += rx->buf[5 + i];

	if (rx->buf[5 + rx->buf[4]] != sum)
		return 0;

	if ((uint16_t)(rx->buf[6 + rx->buf[4]] << 8 | rx->buf[7 + rx->buf[4]]) != rx->tail)
		return 0;

	rx->addr = rx->buf[2];
	rx->func = rx->buf[3];
	rx->len = rx->buf[4];
	for (int i = 0; i < rx->buf[4]; i++)
	{
		rx->dat[i] = rx->buf[5 + i];
	}
	rx->sum = sum;

	return 1;
}

void UART_SetTxFrame(UART_Frame_t *tx, uint16_t head, uint16_t tail, uint8_t addr, uint8_t func)
{
	tx->head = head;
	tx->tail = tail;
	tx->addr = addr;
	tx->func = func;
}

void UART_SetRxFrame(UART_Frame_t *rx, uint16_t head, uint16_t tail)
{
	rx->head = head;
	rx->tail = tail;
}

void UART_ReadData(UART_Frame_t *rx, uint8_t rx_date)
{
	static uint8_t w;
	static uint8_t idx;
	static UART_Frame_t temp_rx;
	
	temp_rx.head = rx->head;
	temp_rx.tail = rx->tail;

	switch (w)
	{
	case 0:
		if (rx_date == (uint8_t)(temp_rx.head >> 8))
			w++;
		
		break;
	case 1:
		if (rx_date == (uint8_t)(temp_rx.head))
			w++;
		else
		{
			w = 0;
			idx = 0;
		}
		break;
	case 2:
		if (rx_date == (uint8_t)(temp_rx.tail >> 8))
			w++;

		break;
	case 3:
		if (rx_date == (uint8_t)(temp_rx.tail))
			w++;
		else
		{
			w = 0;
			idx = 0;
		}
		break;

	default:
		break;
	}

	if (w != 0)
	{
		temp_rx.buf[idx] = rx_date;
		idx++;

		if (w == 4)
		{
			temp_rx.ready = true;
			*rx = temp_rx;
			*rx = temp_rx;
		}
	}
}

/******************** Uart Configuation ********************/

void USART6_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, ENABLE);

	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	USART_DeInit(USART6);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6, &USART_InitStructure);

	USART_ITConfig(USART6, USART_IT_TC, ENABLE);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART6, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

extern UART_Frame_t Uart2PC;
extern UART_Frame_t PC2Uart;

void USART6_IRQHandler(void)
{
	static uint8_t RxData;

	if (USART_GetFlagStatus(USART6, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
		RxData = USART_ReceiveData(USART6);

		UART_ReadData(&PC2Uart, RxData);

		//USART_SendData(USART6, RxData);
		//USART6->DR = RxData;
	}

	if (USART_GetITStatus(USART6, USART_IT_TC))
	{
		USART_ClearITPendingBit(USART6, USART_IT_TC);
		Uart2PC.idx++;
		if (Uart2PC.idx < Uart2PC.buf_size)
			USART_SendData(USART6, Uart2PC.buf[Uart2PC.idx]);
		if (Uart2PC.idx == Uart2PC.buf_size)
		{
			Uart2PC.ready = true;
		}
	}
}

void UART_AutoSend(void)
{
	if (Uart2PC.ready == true)
	{
		if (Uart2PC.idx == 0 && Uart2PC.buf_size != 0)
		{
			USART_SendData(USART6, Uart2PC.buf[Uart2PC.idx]);
			Uart2PC.ready = false;
		}
	}
}
