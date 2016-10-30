#include "BaseIO.h"

BaseIO::BaseIO()
{
    IRQ = 8;
    CE = 9;
    CSN = 10;
    MOSI = 11;
    MISO = 12;
    SCK = 13;

    channel = 3;
    payload = 16;
}
/****************************************
 * 作用：初始引脚
 * 参数：无
 */
void BaseIO::init()
{
    pinMode(CE,  OUTPUT);
    pinMode(CSN, OUTPUT);
    pinMode(MOSI,  OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(IRQ, INPUT);
    pinMode(SCK, OUTPUT);

    ceLow();
    csnHi();
}
/****************************************
 * 作用：获取负载字节大小
 * 参数：无
 */
const uint8_t BaseIO::payloadSize()
{
  return payload;
}
/****************************************
 * 作用：配置当前的数据接收
 * 参数：无
 */
void BaseIO::config()
// Sets the important registers in the MiRF module and powers the module
// in receiving mode
// NB: channel and payload must be set now.
{
    // Set RF channel
    configRegister(RF_CH,channel);

    // Set length of incoming payload
    configRegister(RX_PW_P0, payload);
    configRegister(RX_PW_P1, payload);

    // Start receiver
    powerUpRx();
    flushRx();
}
/****************************************
 * 作用：设置接收端的目的地址,默认使用RX_ADDR_P1，可以修改
 * 参数：adr指向接受者地址数组的指针
 */
void BaseIO::setRADDR(uint8_t * adr,uint8_t RX_ADDR_P = RX_ADDR_P1)
{
    ceLow();
    writeRegister(RX_ADDR_P,adr,BaseIO_ADDR_LEN);
    ceHi();
}
/****************************************
 * 作用：设置本机的地址（本机为发送端时）
 * 参数：adr指向发送地址数组的指针
 */
void BaseIO::setTADDR(uint8_t * adr)
// Sets the transmitting address
{
    /*
     * RX_ADDR_P0 must be set to the sending addr for auto ack to work.
     */

    writeRegister(RX_ADDR_P0,adr,BaseIO_ADDR_LEN);
    writeRegister(TX_ADDR,adr,BaseIO_ADDR_LEN);
}
/****************************************
 * 作用：判断当前数据是否接受完毕
 * 参数：无
 */
bool BaseIO::dataReady()
// Checks if data is available for reading
{
    // See note in getData() function - just checking RX_DR isn't good enough
    uint8_t status = getStatus();

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if ( status & (1 << RX_DR) ) return 1;
    return !rxFifoEmpty();
}
/****************************************
 * 作用：读取接收到的数据到缓冲区
 * 参数：data接收数据的缓冲区
 */
void BaseIO::getData(uint8_t * data)
// Reads payload bytes into data array
{
    csnLow();                               // Pull down chip select
    transfer( R_RX_PAYLOAD );            // Send cmd to read rx payload
    transferSync(data,data,payload); // Read payload
    csnHi();                               // Pull up chip select
    // NVI: per product spec, p 67, note c:
    //  "The RX_DR IRQ is asserted by a new packet arrival event. The procedure
    //  for handling this interrupt should be: 1) read payload through SPI,
    //  2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more
    //  payloads available in RX FIFO, 4) if there are more data in RX FIFO,
    //  repeat from step 1)."
    // So if we're going to clear RX_DR here, we need to check the RX FIFO
    // in the dataReady() function
    configRegister(STATUS,(1<<RX_DR));   // Reset status register
}
/****************************************
 * 作用：判断当前RX FIFO是否为空
 * 参数：无
 */
bool BaseIO::rxFifoEmpty(){
    uint8_t fifoStatus;

    readRegister(FIFO_STATUS,&fifoStatus,sizeof(fifoStatus));
    return (fifoStatus & (1 << RX_EMPTY));
}
/****************************************
 * 作用：判断当前TX FIFO是否为空
 * 参数：无
 */
bool BaseIO::txFifoEmpty(){
    uint8_t fifoStatus;

    readRegister(FIFO_STATUS,&fifoStatus,sizeof(fifoStatus));
    return (fifoStatus & (1 << TX_EMPTY));
}
/****************************************
 * 作用：发送数据包
 * 参数：value指向发送的目标数据
 */
void BaseIO::send(uint8_t * value)
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
    uint8_t status;
    status = getStatus();

    while (PTX) {
        status = getStatus();

        if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
            PTX = 0;
            break;
        }
    }                  // Wait until last paket is send

    ceLow();

    powerUpTx();       // Set to transmitter mode , Power up

    csnLow();                    // Pull down chip select
    transfer( FLUSH_TX );     // Write cmd to flush tx fifo
    csnHi();                    // Pull up chip select

    csnLow();                    // Pull down chip select
    transfer( W_TX_PAYLOAD ); // Write cmd to write payload
    transmitSync(value,payload);   // Write payload
    csnHi();                    // Pull up chip select

    ceHi();                     // Start transmission
}
/****************************************
* 作用：调用send函数，判断寄存器状态，直到所有的
*      数据发送完毕
* 参数：要发送的数组指针
*/
void BaseIO::sendEndWithFinish(uint8_t *value)
{
   send(value);

   while(isSending())delay(10);
}
/****************************************
 * 作用：判断当前是否处于发送中，进行相应的配置
 *      工作模式仅仅两种：1.发送中，保持发送模式
 *                      不变；2.不发送了进
 *                          入接收模式
 * 参数：无
 */
bool BaseIO::isSending(){
    uint8_t status;
    if(PTX){
        status = getStatus();

        /*
         *  if sending successful (TX_DS) or max retries exceded (MAX_RT).
         */

        if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
            powerUpRx();
            return false;
        }

        return true;
    }
    return false;
}
/****************************************
 * 作用：获取寄存器当前的状态
 * 参数：无
 */
uint8_t BaseIO::getStatus(){
    uint8_t rv;
    readRegister(STATUS,&rv,1);
    return rv;
}
/****************************************
 * 作用：CE拉高电压
 * 参数：无
 */
void BaseIO::ceHi(){
    digitalWrite(CE,HIGH);
}
/****************************************
 * 作用：CE拉低电压
 * 参数：无
 */
void BaseIO::ceLow(){
    digitalWrite(CE,LOW);
}
/****************************************
 * 作用：CSN拉高电压
 * 参数：无
 */
void BaseIO::csnHi(){
    digitalWrite(CSN,HIGH);
}
/****************************************
 * 作用：CSN拉低电压
 * 参数：无
 */
void BaseIO::csnLow(){
    digitalWrite(CSN,LOW);
}
/****************************************
 * 作用：配置寄存器设置
 * 参数：reg 目标寄存器，value要配置的值
 */
int8_t BaseIO::configRegister(uint8_t reg, uint8_t value)
{
  uint8_t status;

  csnLow();
  status = transfer(W_REGISTER | (REGISTER_MASK & reg));//spi->transfer(W_REGISTER | (REGISTER_MASK & reg));
  transfer(value);
  csnHi();
  return(status);
}

/****************************************
 * 作用：获取当前寄存器状态
 * 参数：reg 目标寄存器
 *      *value读取到的值保存位置（利用指针特性保存数据）
 *      len 读取的数据长度
 */
uint8_t BaseIO::readRegister(uint8_t reg, uint8_t * value, uint8_t len)
{
  uint8_t status;

  csnLow();
  status = transfer((R_REGISTER | (REGISTER_MASK & reg)));
  transferSync(value,value,len);

  csnHi();
  return(status);
}
/****************************************
 * 作用：读取数据
 * 参数：dataout与datain指向同一个缓冲区，完成
 *      读数据的动作,len为要数据长度
 */
void BaseIO::transferSync(uint8_t *dataout,uint8_t *datain,uint8_t len)
{
    uint8_t i;
    for(i = 0;i < len;i++)
    {
        datain[i] = transfer(dataout[i]);
    }
}
/****************************************
 * 作用：向寄存器写入命令
 * 参数：reg要写入的命令，pBuf参数（如写入RX地址），bytes 写入的字节数
 */
uint8_t BaseIO::writeRegister(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  uint8_t status;

  csnLow();
  status = transfer(W_REGISTER | (REGISTER_MASK & reg));
  transmitSync(pBuf,bytes);
  csnHi();
  return(status);

}
/****************************************
 * 作用：向从机输出数据流
 * 参数：dataout指向要发送数据缓冲区，Len表示
 *      输出的数据长度
 */
void BaseIO::transmitSync(uint8_t *dataout,uint8_t len)
{
    uint8_t i;
    for(i = 0;i < len;i++)
    {
        transfer(*dataout++);
    }
}

/****************************************
 * 作用：配置电源选项为接收模式
 * 参数：无
 */
void BaseIO::powerUpRx()
{
    PTX = 0;
    ceLow();
    configRegister(CONFIG, BaseIO_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) );
    ceHi();
    configRegister(STATUS,(1 << TX_DS) | (1 << MAX_RT));
}
//--------核心函数--------
/************************
 * 作用：交换数据
*/
uint8_t BaseIO::transfer(unsigned char Byte)
{
    uint8_t i;
    for(i=0;i<8;i++)
    {
        if(Byte&0x80)
        {
          digitalWrite(MOSI, 1);
        }
        else
        {
          digitalWrite(MOSI, 0);
        }
        digitalWrite(SCK, 1);
        Byte <<= 1;
        if(digitalRead(MISO) == 1)
        {
          Byte |= 1;
        }
        digitalWrite(SCK, 0);
    }
    return(Byte);
}
/****************************************
 * 作用：刷新接收缓存区
 * 参数：无
 */
void BaseIO::flushRx()
{
    csnLow();
    transfer( FLUSH_RX );
    csnHi();
}
/****************************************
 * 作用：配置发送模式电源选项
 * 参数：无
 */
void BaseIO::powerUpTx()
{
    PTX = 1;
    configRegister(CONFIG, BaseIO_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) );
}
/****************************************
 * 作用：配置电源选项为掉电模式，省电
 * 参数：无
 */
void BaseIO::powerDown()
{
    ceLow();
    configRegister(CONFIG, BaseIO_CONFIG );
}
/****************************************
 * 作用：设置引脚
 * 参数：引脚
 */
void BaseIO::setBaseIOPin(int8_t irq = 8,int8_t ce = 9,int8_t csn = 10,
                  int8_t mosi = 11,int8_t miso = 12,int8_t sck = 13)
{
    IRQ = irq;
    CE = ce;
    CSN = csn;
    MOSI = mosi;
    MISO = miso;
    SCK = sck;
}
/****************************************
 * 作用：设置频道和负载字节数
 * 参数：channel频道号1 -- 127，负载：16--32最好
 */
void BaseIO::setChannelAndPayload(uint8_t c,uint8_t p)
{
    channel = c;
    payload = p;
}
