#ifndef USER_LIB_H
#define USER_LIB_H

#include "main.h"

typedef float fp32;
typedef double fp64;

typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�������
    fp32 min_value;    //�޷���Сֵ
    fp32 max_value;    //�޷����ֵ
    fp32 frame_period; //ʱ����
} ramp_function_source_t;

typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�˲����������
    fp32 num[1];       //�˲�����
    fp32 frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
//���ٿ���
extern fp32 invSqrt(fp32 num);

//б��������ʼ��
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//б����������
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
//һ���˲���ʼ��
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//һ���˲�����
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//��������
void abs_limit(fp32 *num, fp32 Limit);
//�жϷ���λ
fp32 sign(fp32 value);
//��������
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26����
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//�޷�����
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//ѭ���޷�����
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//�Ƕ� ���޷� 180 ~ -180
fp32 theta_format(fp32 Ang);

void absLimit(float* Val, float Limit);

uint8_t ValueInRange_u(uint32_t Value, uint32_t Min, uint32_t Max);

uint8_t ValueInRange_i(int32_t Value, int32_t Min, int32_t Max);

uint8_t ValueInRange_f(float Value, float Min, float Max);

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
