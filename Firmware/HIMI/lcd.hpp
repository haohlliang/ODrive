#ifndef _LCD_HPP_
#define _LCD_HPP_
#include <stdlib.h>

#include "lcd_font.h"
#include "lcd_pic.h"
#include "odrive_main.h"
#include "stdarg.h"
#include "stdbool.h" 
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "string.h"

#define USE_HORIZONTAL 3 //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
    #define LCD_W 135
    #define LCD_H 240

#else
    #define LCD_W 240
    #define LCD_H 135
#endif

typedef struct
{
    GPIO_TypeDef * SCLK_GPIO_Port, *MOSI_GPIO_Port, *RES_GPIO_Port, *DC_GPIO_Port, *CS_GPIO_Port, *BLK_GPIO_Port;
    uint16_t SCLK_GPIO_Pin, MOSI_GPIO_Pin, RES_GPIO_Pin, DC_GPIO_Pin, CS_GPIO_Pin, BLK_GPIO_Pin;
    void (*WritePin)(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
} LCD_TypeDef;

extern uint8_t Usart_Flag,Can_Flag;  //定义串口和can的flag



void LCD_GPIO(LCD_TypeDef * LCD_InitStruct);                               //初始化GPIO
void LCD_Init(void);                                                       //初始化GPIO
void LCD_Writ_Bus(uint8_t dat);                                            //模拟SPI时序
void LCD_WR_DATA8(uint8_t dat);                                            //写入一个字节
void LCD_WR_DATA16(uint16_t dat);                                            //写入两个字节
void LCD_WR_REG(uint8_t dat);                                              //写入一个指令
void LCD_SetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);  //设置坐标函数




class LCD
{
public:
    LCD(void);
    enum ColorType
    {
        WHITE = 0xFFFF,       //白色
        BLACK = 0x0000,       //黑色
        BLUE = 0x001F,        //蓝色
        BRED = 0XF81F,        //色
        GRED = 0XFFE0,        //色
        GBLUE = 0X07FF,       //色
        RED = 0xF800,         //红色
        MAGENTA = 0xF81F,     //色
        GREEN = 0x07E0,       //绿色
        CYAN = 0x7FFF,        //色
        YELLOW = 0xFFE0,      //黄色
        BROWN = 0XBC40,       //棕色
        BRRED = 0XFC07,       //棕红色
        GRAY = 0X8430,        //灰色
        DARKBLUE = 0X01CF,    //深蓝色
        LIGHTBLUE = 0X7D7C,   //浅蓝色
        GRAYBLUE = 0X5458,    //灰蓝色
        LIGHTGREEN = 0X841F,  //浅绿色
        LGRAY = 0XC618,       //浅灰色(PANNEL),窗体背景色
        LGRAYBLUE = 0XA651,   //浅灰蓝色(中间层颜色)
        LBBLUE = 0X2B12       //浅棕蓝色(选择条目的反色)
    };
    typedef struct
    {
        ColorType BackColor;
        ColorType TextColor;
        ColorType ErrColor;
        ColorType WarnColor;
        ColorType NormColor;
        ColorType TableColor;
    } LCDColor_TypeDef;

    LCDColor_TypeDef Color = {LCD::BLACK, LCD::WHITE, LCD::RED, LCD::YELLOW, LCD::GREEN, LCD::GRAY};

    enum FontSize
    {
        FontSize_12 = 12,
        FontSize_16 = 16,
        FontSize_24 = 24,
        FontSize_32 = 32
    };

    const char tab_name[9][2][32] =
    {
        {"        axis0", "        axis1"},
        {"Pos EST", "Pos EST"},        // odrv0.axis0.controller.pos_setpoint, odrv0.axis0.encoder.pos_estimate
        {"Vel EST", "Vel EST"},        // odrv0.axis0.controller.vel_setpoint, odrv0.axis0.encoder.vel_estimate
        {"Torque Set", "Torque Set"},  // odrv0.axis0.controller.torque_setpoint
        {"Encoder", "Encoder"},        // odrv0.axis0.encoder.error  odrv0.axis1.encoder.shadow_count
        {"DRV8301 ", "DRV8301 "},      // odrv0.axis0.motor.gate_driver.drv_fault
        {"Current", "Current"},        // odrv0.axis0.motor.current_control.Ibus
        {"Fet Temp", "Fet Temp"},      // odrv0.axis0.fet_thermistor.temperature
        {"Encoder error", "DCBUS"},  // odrv0.axis0.motor_thermistor.temperature
       
    };

    char tab_val[9][2][32] =
    {
        {"", ""},
        {"", ""},
        {"", ""},
        {"", ""},
        {"", ""},
        {"", ""},
        {"", ""},
        {"", ""},
        {"", ""},
    };

    osThreadId thread_id_;
    const uint32_t stack_size_ = 2048;  // Bytes
    volatile bool thread_id_valid_ = false;

    void Setup(void);
    void Loop(void);
    void StartThread(void);

    void GetParameter(const char * name, char * val, int size) ;
    void Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);                                //指定区域填充颜色
    void DrawPoint(uint16_t x, uint16_t y, uint16_t color);                                                               //在指定位置画一个点
    void DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                                    //在指定位置画一条线
    void DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                               //在指定位置画一个矩形
    void DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);                                                 //在指定位置画一个圆
    void DrawTab(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint16_t row, uint16_t col, uint16_t color);  //画一个表格
    void ShowChinese(uint16_t x, uint16_t y, uint8_t * s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);         //显示汉字串
    void ShowText(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode, const char * str, ...);
    void ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[]);  //显示图片
    void LCD_ShowFloatNum1(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);

private:
    void ShowChinese12x12(uint16_t x, uint16_t y, uint8_t * s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode); //显示单个12x12汉字
    void ShowChinese16x16(uint16_t x, uint16_t y, uint8_t * s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode); //显示单个16x16汉字
    void ShowChinese24x24(uint16_t x, uint16_t y, uint8_t * s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode); //显示单个24x24汉字
    void ShowChinese32x32(uint16_t x, uint16_t y, uint8_t * s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode); //显示单个32x32汉字
    void ShowChar(uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);         //显示一个字符
    void ShowString(uint16_t x, uint16_t y, const uint8_t * p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode); //显示字符串
};




#endif
