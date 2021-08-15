#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h" 
   	   		   
//IO��������
#define SDA_IN()  {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0x00<<(11*2);}//PB11����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0x01<<(11*2);} //PB11���ģʽ
//IO��������	 
#define IIC_SCL    PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA	 
#define READ_SDA   PBin(11)  //����SDA 

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	

void IIC_Init2(void);                //��ʼ��IIC��IO��				 
void IIC_Start2(void);				//����IIC��ʼ�ź�
void IIC_Stop2(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte2(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte2(unsigned char ack);//IIC��ȡһ���ֽ�


#endif
















