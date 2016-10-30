#ifndef BASEIO_H
#define BASEIO_H

#include "nrf24l01.h"
#include <stdint.h>
#include <SPI.h>

#define BaseIO_ADDR_LEN	5
#define BaseIO_CONFIG ((1<<EN_CRC) | (0<<CRCO) )

class BaseIO
{
public:
    BaseIO();
    /****************************************
     * 作用：设置引脚
     * 参数：引脚
     */
    void setBaseIOPin(int8_t irq,int8_t ce,int8_t csn,
                      int8_t mosi,int8_t miso,int8_t sck);
    /****************************************
     * 作用：初始引脚
     * 参数：无
     */
    void init();
    /****************************************
     * 作用：配置当前的数据接收
     * 参数：无
     */
    void config();
    /****************************************
     * 作用：发送数据包
     * 参数：value指向发送的目标数据
     */
    void send(uint8_t *value);
    /****************************************
     * 作用：调用send函数，判断寄存器状态，直到所有的
     *      数据发送完毕
     * 参数：要发送的数组指针
     */
    void sendEndWithFinish(uint8_t *value);
    /****************************************
     * 作用：设置接收端的目的地址,默认使用RX_ADDR_P1，可以修改
     * 参数：adr指向接受者地址数组的指针
     */
    void setRADDR(uint8_t * adr,uint8_t RX_ADDR_P);
    /****************************************
     * 作用：设置本机的地址（本机为发送端时）
     * 参数：adr指向发送地址数组的指针
     */
    void setTADDR(uint8_t * adr);
    /****************************************
     * 作用：判断当前数据是否接受完毕
     * 参数：无
     */
    bool dataReady();
    /****************************************
     * 作用：判断当前是否处于发送中，进行相应的配置
     *      工作模式仅仅两种：1.发送中，保持发送模式
     *                      不变；2.不发送了进
     *                      入接收模式
     * 参数：无
     */
    bool isSending();
    /****************************************
     * 作用：判断当前RX FIFO是否为空
     * 参数：无
     */
    bool rxFifoEmpty();
    /****************************************
     * 作用：判断当前TX FIFO是否为空
     * 参数：无
     */
    bool txFifoEmpty();
    /****************************************
     * 作用：读取接收到的数据到缓冲区
     * 参数：data接收数据的缓冲区
     */
    void getData(uint8_t * data);
    /****************************************
     * 作用：获取寄存器当前的状态
     * 参数：无
     */
    uint8_t getStatus();
    /****************************************
     * 作用：向从机输出数据流
     * 参数：dataout指向要发送数据缓冲区，Len表示
     *      输出的数据长度
     */
    void transmitSync(uint8_t *dataout,uint8_t len);
    /****************************************
     * 作用：读取数据
     * 参数：dataout与datain指向同一个缓冲区，完成
     *      读数据的动作,len为要数据长度
     */
    void transferSync(uint8_t *dataout,uint8_t *datain,uint8_t len);
    /****************************************
     * 作用：配置寄存器设置
     * 参数：reg 目标寄存器，value要配置的值
     */
    int8_t configRegister(uint8_t reg, uint8_t value);
    /****************************************
     * 作用：获取当前寄存器状态
     * 参数：reg 目标寄存器
     *      *value读取到的值保存位置（利用指针特性保存数据）
     *      len 读取的数据长度
     */
    uint8_t readRegister(uint8_t reg, uint8_t * value, uint8_t len);//
    /****************************************
     * 作用：向寄存器写入命令
     * 参数：reg要写入的命令，pBuf参数（如写入RX地址），bytes 写入的字节数
     */
    uint8_t writeRegister(uint8_t reg, uint8_t * value, uint8_t len);//
    /****************************************
     * 作用：配置电源选项为接收模式
     * 参数：无
     */
    void powerUpRx();//
    /****************************************
     * 作用：配置发送模式电源选项
     * 参数：无
     */
    void powerUpTx();//
    /****************************************
     * 作用：配置掉电模式，为了省电
     * 参数：无
     */
    void powerDown();//
    /****************************************
     * 作用：核心函数，用于交换信息 8-bits
     * 参数：要交换的数据
     */
    uint8_t transfer(unsigned char Byte);//
    /****************************************
     * 作用：CSN拉高电压
     * 参数：无
     */
    void csnHi();//
    /****************************************
     * 作用：CSN拉低电压
     * 参数：无
     */
    void csnLow();//
    /****************************************
     * 作用：CE拉高电压
     * 参数：无
     */
    void ceHi();//
    /****************************************
     * 作用：CE拉低电压
     * 参数：无
     */
    void ceLow();//
    /****************************************
     * 作用：刷新接收缓存区
     * 参数：无
     */
    void flushRx();//
    /****************************************
     * 作用：设置频道和负载字节数
     * 参数：channel频道号1 -- 127，负载：16--32最好
     */
    void setChannelAndPayload(uint8_t c,uint8_t p);
        /****************************************
     * 作用：获取负载字节大小
     * 参数：无
     */
    const uint8_t payloadSize();
private:
    int8_t IRQ;
    int8_t CE;
    int8_t CSN;

    int8_t MOSI;
    int8_t MISO;
    int8_t SCK;

    uint8_t PTX;

    uint8_t channel;
    uint8_t payload;
};

#endif // BASEIO_H
