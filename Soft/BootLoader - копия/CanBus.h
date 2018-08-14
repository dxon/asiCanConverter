#ifndef __CANBUS_H
#define __CANBUS_H

#define CAN_SIZE  256

typedef struct{
  uint32_t  address;
  uint32_t  command;
  uint32_t  mcuStatus;
}TargetStruct;


#define TYPEMSG (ptrBox->Data[0])
#define CHANNEL (ptrBox->Data[1])
#define CMD ((uint16_t)ptrBox->Data[2]+(((uint16_t)ptrBox->Data[3])<<8))
#define CMD_LOW (ptrBox->Data[2])
#define CMD_HIGH (ptrBox->Data[3])
#define DL0 (ptrBox->Data[4])
#define DL1 (ptrBox->Data[5])
#define DL2 (ptrBox->Data[6])
#define DL3 (ptrBox->Data[7])

#define DL32 ( (uint32_t)ptrBox->Data[4]+(((uint32_t)ptrBox->Data[5])<<8)+\
               (((uint32_t)ptrBox->Data[6])<<16)+(((uint32_t)ptrBox->Data[7])<<24))

#define DL16 ( (uint32_t)ptrBox->Data[4]+(((uint32_t)ptrBox->Data[5])<<8) )
#define DL16_L ( (uint16_t)ptrBox->Data[4]+(((uint16_t)ptrBox->Data[5])<<8) )
#define DL16_H ( (uint16_t)ptrBox->Data[6]+(((uint16_t)ptrBox->Data[7])<<8) )

#define TX_SET(a)	Can_Tx.ExtId = a;
#define TX_SET1(a,b)	Can_Tx.ExtId = a; Can_Tx.DLC = 1; Can_Tx.Data[0] = b;

#define TX_SET2(a,b,c)  Can_Tx.ExtId = a; Can_Tx.DLC = 2;\
                        Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;

#define TX_SET3(a,b,c,d)  Can_Tx.ExtId = a; Can_Tx.DLC = 3;\
                          Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;Can_Tx.Data[2] = d;

#define TX_SET4(a,b,c,d,e)  Can_Tx.ExtId = a; Can_Tx.DLC = 4;\
                            Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                            Can_Tx.Data[2] = d; Can_Tx.Data[3] = e;

#define TX_SET5(a,b,c,d,e,f)  Can_Tx.ExtId = a; Can_Tx.DLC = 5;\
                              Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                              Can_Tx.Data[2] = d; Can_Tx.Data[3] = e;\
                              Can_Tx.Data[4] = f;

#define TX_SET6(a,b,c,d,e,f,g)  Can_Tx.ExtId = a; Can_Tx.DLC = 6;\
                                Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                                Can_Tx.Data[2] = d; Can_Tx.Data[3] = e;\
                                Can_Tx.Data[4] = f;Can_Tx.Data[5] = g;

#define TX_SET7(a,b,c,d,e,f,g,h)  Can_Tx.ExtId = a; Can_Tx.DLC = 7;\
                                  Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                                  Can_Tx.Data[2] = d; Can_Tx.Data[3] = e;\
                                  Can_Tx.Data[4] = f; Can_Tx.Data[5] = g;\
                                  Can_Tx.Data[6] = h;


#define TX_SET8(a,b,c,d,e,f,g,h,i) Can_Tx.ExtId = a; Can_Tx.DLC = 8;\
                                   Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                                   Can_Tx.Data[2] = d; Can_Tx.Data[3] = e;\
                                   Can_Tx.Data[4] = f; Can_Tx.Data[5] = g;\
                                   Can_Tx.Data[6] = h; Can_Tx.Data[7] = i;

#define TX_SET_LONG(a,b,c,d,e)  Can_Tx.ExtId = a; Can_Tx.DLC = 8;\
                                Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                                Can_Tx.Data[2] = (uint8_t)d; Can_Tx.Data[3] = (uint8_t)(d>>8);\
                                Can_Tx.Data[4] = (uint8_t)e; Can_Tx.Data[5] = (uint8_t)(e>>8);\
                                Can_Tx.Data[6] = (uint8_t)(e>>16); Can_Tx.Data[7] = (uint8_t)(e>>24);

#define TX_SET_WORD(a,b,c,d,e)  Can_Tx.ExtId = a; Can_Tx.DLC = 6;\
                                Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                                Can_Tx.Data[2] = (uint8_t)d; Can_Tx.Data[3] = (uint8_t)(d>>8);\
                                Can_Tx.Data[4] = (uint8_t)e; Can_Tx.Data[5] = (uint8_t)(e>>8);

#define TX_SET_WORD_BY_WORD(a,b,c,d,e,f)  Can_Tx.ExtId = a; Can_Tx.DLC = 8;\
                                          Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                                          Can_Tx.Data[2] = (uint8_t)d; Can_Tx.Data[3] = (uint8_t)(d>>8);\
                                          Can_Tx.Data[4] = (uint8_t)e; Can_Tx.Data[5] = (uint8_t)(e>>8);\
                                          Can_Tx.Data[6] = (uint8_t)f; Can_Tx.Data[7] = (uint8_t)(f>>8);

#define TX_SET_(a,b,c,d)  Can_Tx.ExtId = a; Can_Tx.DLC = 4;\
                          Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                          Can_Tx.Data[2] = (uint8_t)d; Can_Tx.Data[3] = (uint8_t)(d>>8);


#define TRANSFER_4(a,b,c,d,e)  {Can_Tx.ExtId = a; Can_Tx.DLC = 4;\
                            Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                            Can_Tx.Data[2] = d; Can_Tx.Data[3] = e;\
                            CAN_Transmit(CAN1, &Can_Tx);}

#define TRANSFER_5(a,b,c,d,e,f)  {Can_Tx.ExtId = a; Can_Tx.DLC = 5;\
                                Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                                Can_Tx.Data[2] = d; Can_Tx.Data[3] = e;\
                                Can_Tx.Data[4] = f;\
                                resultTx = CAN_Transmit(CAN1, &Can_Tx);}


#define TRANSFER_6(a,b,c,d,e,f,g)  {Can_Tx.ExtId = a; Can_Tx.DLC = 6;\
                                Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                                Can_Tx.Data[2] = d; Can_Tx.Data[3] = e;\
                                Can_Tx.Data[4] = f;Can_Tx.Data[5] = g;\
                                resultTx = CAN_Transmit(CAN1, &Can_Tx);}


#define TRANSFER_8(a,b,c,d,e,f,g,h,i) { Can_Tx.ExtId = a; Can_Tx.DLC = 8;\
                                   Can_Tx.Data[0] = b; Can_Tx.Data[1] = c;\
                                   Can_Tx.Data[2] = d; Can_Tx.Data[3] = e;\
                                   Can_Tx.Data[4] = f; Can_Tx.Data[5] = g;\
                                   Can_Tx.Data[6] = h; Can_Tx.Data[7] = i;\
                                   CAN_Transmit(CAN1, &Can_Tx);}

void  initCanBus(void);
void  canHandler(void);
CanRxMsg* canGetRxMsg(void);
void  canDeleteRxMsg(void);
#endif

