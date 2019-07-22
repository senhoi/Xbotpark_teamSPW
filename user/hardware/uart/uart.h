#ifndef UART_H
#define UART_H

#include "main.h"

#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#include "stdbool.h"

#define UART_MAX_BUF 255

#define UART_FUNC_DATA 0xF0
#define UART_FUNC_DATA_STR 0xF1
#define UART_FUNC_DATA_U8 0xF2
#define UART_FUNC_DATA_U16 0xF3
#define UART_FUNC_DATA_U32 0xF4
#define UART_FUNC_DATA_U64 0xF5
#define UART_FUNC_DATA_N8 0xF6
#define UART_FUNC_DATA_N16 0xF7
#define UART_FUNC_DATA_N32 0xF8
#define UART_FUNC_DATA_N64 0xF9
#define UART_FUNC_DATA_FLT 0xFA
#define UART_FUNC_DATA_DBL 0xFB

// Tx Frame:
//	________________________________________________________________________________________________________________________
//		HEAD_H	|	HEAD_L	|	ADDR	|	FUNC	|	LEN		|	DATA	|	DATA	|	SUM		|	TAIL_H	|	TAIL_L
//
typedef struct UART_Frame_t
{
	bool ready;
	uint8_t idx;

	uint8_t buf[UART_MAX_BUF + 8];
	uint8_t buf_size;

	uint16_t head;
	uint8_t addr;
	uint8_t func;
	uint8_t len;
	uint8_t dat[UART_MAX_BUF];
	uint8_t sum;
	uint16_t tail;

} UART_Frame_t;

extern UART_Frame_t Uart2PC;

uint8_t UART_Printf(UART_Frame_t *tx, char *fmt, ...);

uint8_t UART_GetArr(UART_Frame_t *rx);

uint8_t UART_SendArr_8b(UART_Frame_t *tx, void *arr, uint8_t len);

uint8_t UART_SendArr_16b(UART_Frame_t *tx, void *arr, uint8_t len);

uint8_t UART_SendArr_32b(UART_Frame_t *tx, void *arr, uint8_t len);

uint8_t UART_SendArr_64b(UART_Frame_t *tx, void *arr, uint8_t len);

#define UART_SendArr_flt UART_SendArr_32b

#define UART_SendArr_dbl UART_SendArr_64b

void UART_SetTxFrame(UART_Frame_t *tx, uint16_t head, uint16_t tail, uint8_t addr, uint8_t func);

void UART_SetRxFrame(UART_Frame_t *rx, uint16_t head, uint16_t tail);

void USART6_Init(void);

void UART_AutoSend(void);

#endif
