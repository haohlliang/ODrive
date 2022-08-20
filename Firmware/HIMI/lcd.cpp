#include <lcd.hpp>

#include "..\communication\interface_uart.h"

LCD_TypeDef lcdIO;
uint8_t Usart_Flag=0,Can_Flag=0;
/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(uint8_t dat) {
    uint8_t i;
    lcdIO.WritePin(lcdIO.CS_GPIO_Port, lcdIO.CS_GPIO_Pin, GPIO_PIN_RESET);
    for (i = 0; i < 8; i++) {
        lcdIO.WritePin(lcdIO.SCLK_GPIO_Port, lcdIO.SCLK_GPIO_Pin, GPIO_PIN_RESET);
        if (dat & 0x80) {
            lcdIO.WritePin(lcdIO.MOSI_GPIO_Port, lcdIO.MOSI_GPIO_Pin, GPIO_PIN_SET);
        } else {
            lcdIO.WritePin(lcdIO.MOSI_GPIO_Port, lcdIO.MOSI_GPIO_Pin, GPIO_PIN_RESET);
        }
        lcdIO.WritePin(lcdIO.SCLK_GPIO_Port, lcdIO.SCLK_GPIO_Pin, GPIO_PIN_SET);
        dat <<= 1;
    }
    lcdIO.WritePin(lcdIO.CS_GPIO_Port, lcdIO.CS_GPIO_Pin, GPIO_PIN_SET);
}

/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(uint8_t dat) {
    LCD_Writ_Bus(dat);
}

/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA16(uint16_t dat) {
    LCD_Writ_Bus(dat >> 8);
    LCD_Writ_Bus(dat);
}

/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(uint8_t dat) {
    lcdIO.WritePin(lcdIO.DC_GPIO_Port, lcdIO.DC_GPIO_Pin, GPIO_PIN_RESET);  //写命令
    LCD_Writ_Bus(dat);
    lcdIO.WritePin(lcdIO.DC_GPIO_Port, lcdIO.DC_GPIO_Pin, GPIO_PIN_SET);  //写数据
}

/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_SetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA16(x1+52);
		LCD_WR_DATA16(x2+52);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA16(y1+40);
		LCD_WR_DATA16(y2+40);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA16(x1+53);
		LCD_WR_DATA16(x2+53);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA16(y1+40);
		LCD_WR_DATA16(y2+40);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA16(x1+40);
		LCD_WR_DATA16(x2+40);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA16(y1+53);
		LCD_WR_DATA16(y2+53);
		LCD_WR_REG(0x2c);//储存器写
	}
	else
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA16(x1+40);
		LCD_WR_DATA16(x2+40);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA16(y1+52);
		LCD_WR_DATA16(y2+52);
		LCD_WR_REG(0x2c);//储存器写
	}
}

void LCD_GPIO(LCD_TypeDef *LCD_InitStruct) {
    lcdIO.WritePin = LCD_InitStruct->WritePin;
    lcdIO.SCLK_GPIO_Port = LCD_InitStruct->SCLK_GPIO_Port;
    lcdIO.MOSI_GPIO_Port = LCD_InitStruct->MOSI_GPIO_Port;
    lcdIO.RES_GPIO_Port = LCD_InitStruct->RES_GPIO_Port;
    lcdIO.DC_GPIO_Port = LCD_InitStruct->DC_GPIO_Port;
    lcdIO.CS_GPIO_Port = LCD_InitStruct->CS_GPIO_Port;
    lcdIO.BLK_GPIO_Port = LCD_InitStruct->BLK_GPIO_Port;
    lcdIO.SCLK_GPIO_Pin = LCD_InitStruct->SCLK_GPIO_Pin;
    lcdIO.MOSI_GPIO_Pin = LCD_InitStruct->MOSI_GPIO_Pin;
    lcdIO.RES_GPIO_Pin = LCD_InitStruct->RES_GPIO_Pin;
    lcdIO.DC_GPIO_Pin = LCD_InitStruct->DC_GPIO_Pin;
    lcdIO.CS_GPIO_Pin = LCD_InitStruct->CS_GPIO_Pin;
    lcdIO.BLK_GPIO_Pin = LCD_InitStruct->BLK_GPIO_Pin;
}

void LCD_Init() {
    lcdIO.WritePin(lcdIO.RES_GPIO_Port, lcdIO.RES_GPIO_Pin, GPIO_PIN_RESET);  //复位
    HAL_Delay(100);
    lcdIO.WritePin(lcdIO.RES_GPIO_Port, lcdIO.RES_GPIO_Pin, GPIO_PIN_SET);  //复位
    HAL_Delay(100);
    lcdIO.WritePin(lcdIO.BLK_GPIO_Port, lcdIO.BLK_GPIO_Pin, GPIO_PIN_SET);  //复位//打开背光
    HAL_Delay(100);
    LCD_WR_REG(0x11);
    HAL_Delay(120);
    LCD_WR_REG(0x36);
    if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
	else LCD_WR_DATA8(0xA0);

	LCD_WR_REG(0x3A);
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x33); 

	LCD_WR_REG(0xB7); 
	LCD_WR_DATA8(0x35);  

	LCD_WR_REG(0xBB);
	LCD_WR_DATA8(0x19);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x2C);

	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x12);   

	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x20);  

	LCD_WR_REG(0xC6); 
	LCD_WR_DATA8(0x0F);    

	LCD_WR_REG(0xD0); 
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1);

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2B);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x54);
	LCD_WR_DATA8(0x4C);
	LCD_WR_DATA8(0x18);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0B);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x23);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2C);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x44);
	LCD_WR_DATA8(0x51);
	LCD_WR_DATA8(0x2F);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x20);
	LCD_WR_DATA8(0x23);

	LCD_WR_REG(0x21); 

	LCD_WR_REG(0x29); 
}

#include "autogen/type_info.hpp"
#include "communication/ascii_protocol.hpp"

// 表格长宽
#define NUM_ROW 9
#define NUM_COL 2

LCD::LCD() {
    LCD_TypeDef lcd_parameters = {
        .SCLK_GPIO_Port = SRC_SCL_GPIO_Port,
        .MOSI_GPIO_Port = SRC_SDA_GPIO_Port,
        .RES_GPIO_Port = SRC_RES_GPIO_Port,
        .DC_GPIO_Port = SRC_DC_GPIO_Port,
        .CS_GPIO_Port = SRC_CS_GPIO_Port,
        .BLK_GPIO_Port = SRC_BL_GPIO_Port,
        .SCLK_GPIO_Pin = SRC_SCL_Pin,
        .MOSI_GPIO_Pin = SRC_SDA_Pin,
        .RES_GPIO_Pin = SRC_RES_Pin,
        .DC_GPIO_Pin = SRC_DC_Pin,
        .CS_GPIO_Pin = SRC_CS_Pin,
        .BLK_GPIO_Pin = SRC_BL_Pin,
        .WritePin = HAL_GPIO_WritePin,
    };

    LCD_GPIO(&lcd_parameters);
}

void LCD::GetParameter(const char *name, char *val, int size) {
    char res[32];
    Introspectable property = root_obj.get_child(name, strlen(name) + 1);
    const StringConvertibleTypeInfo *type_info = dynamic_cast<const StringConvertibleTypeInfo *>(property.get_type_info());
    if (!type_info) {
        uart4_stream_output_ptr->process_bytes((const uint8_t *)"type_info error", strlen("type_info error"), nullptr);
        strncpy(val, "", 8);
    } else {
        if (!type_info->get_string(property, res, size)) {
            uart4_stream_output_ptr->process_bytes((const uint8_t *)"type_info get_string error", strlen("type_info get_string error"), nullptr);
            strncpy(val, "", 8);
        } else {
            strncpy(val, (const char *)res, 8);
        }
    }
}

int i=0;
void LCD::Setup() {
    // 两次初始化确保屏幕初始化正常
    LCD_Init();
    Fill(0, 0, LCD_W, LCD_H, Color.BackColor);
    LCD_Init();
    Fill(0, 0, LCD_W, LCD_H, Color.BackColor);

    ShowText(LCD_W * 2 / 4 + 80, (LCD_H - 1) * 0 / NUM_ROW + 20, WHITE, BLACK, FontSize_16, 0, "V");
    ShowText(LCD_W * 2 / 4 + 40, (LCD_H - 1) * 0 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "DCBUS");
    ShowText(LCD_W * 0 / 4 + 65, (LCD_H - 1) *0 / NUM_ROW + 2, WHITE, BLACK, FontSize_12, 0,"M0:");
    ShowText(LCD_W * 0 / 4 + 105, (LCD_H - 1) *0 / NUM_ROW + 2, GREEN, BLACK, FontSize_12, 0,"C");
    ShowText(LCD_W * 0 / 4 + 65, (LCD_H - 1) *0 / NUM_ROW + 15, WHITE, BLACK, FontSize_12, 0,"M1:");
    ShowText(LCD_W * 0 / 4 + 105, (LCD_H - 1) *0 / NUM_ROW + 15, GREEN, BLACK, FontSize_12, 0,"C");

	for(i = 1; i < 7; i++)
    {
        ShowText(LCD_W * 0 / 4 + 90, (LCD_H - 1) *(i+2) / NUM_ROW + 2, WHITE, BLACK, FontSize_12, 0, "|");
        ShowText(LCD_W * 0 / 4 + 100, (LCD_H - 1) *(i+2) / NUM_ROW + 2, WHITE, BLACK, FontSize_12, 0, "|");
    }


    ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *3 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "MODE0:");
    ShowText(LCD_W * 0 / 4 + 45, (LCD_H - 1) *3 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "[");

    //ShowText(LCD_W * 0 / 4 + 50, (LCD_H - 1) *2 / NUM_ROW + 2, BROWN, BLACK, FontSize_12, 0, "Pos");
    //ShowText(LCD_W * 0 / 4 + 50, (LCD_H - 1) *2 / NUM_ROW + 2, GREEN, BLACK, FontSize_12, 0, "Vel");
    //ShowText(LCD_W * 0 / 4 + 50, (LCD_H - 1) *2 / NUM_ROW + 2, BLUE, BLACK, FontSize_12, 0, " Iq");
    //ShowText(LCD_W * 0 / 4 + 50, (LCD_H - 1) *3 / NUM_ROW + 2, RED, BLACK, FontSize_12, 0, "NUL");
    ShowText(LCD_W * 0 / 4 + 75, (LCD_H - 1) *3 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "]");

    ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *4.5 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "C:");
    ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *6/ NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "P:");
    ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *7.5 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "V:");

    ShowText(LCD_W * 2 / 4 + 4, (LCD_H - 1) *3 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "MODE1:");
    ShowText(LCD_W * 2 / 4 + 45, (LCD_H - 1) *3 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "[");
    ShowText(LCD_W * 2 / 4 + 75, (LCD_H - 1) *3 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "]");
   // ShowText(LCD_W * 2/ 4 + 52, (LCD_H - 1) *2 / NUM_ROW + 2, BROWN, BLACK, FontSize_12, 0, "Pos");
    //ShowText(LCD_W * 2 / 4 + 52, (LCD_H - 1) *2 / NUM_ROW + 2, GREEN, BLACK, FontSize_12, 0, "Vel");  
    //ShowText(LCD_W * 2 / 4 + 52, (LCD_H - 1) *2 / NUM_ROW + 2, BLUE, BLACK, FontSize_12, 0, " Iq");
   // ShowText(LCD_W * 2 / 4 + 50, (LCD_H - 1) *3 / NUM_ROW + 2, RED, BLACK, FontSize_12, 0, "NUL");

    ShowText(LCD_W * 2 / 4 + 2, (LCD_H - 1) *4.5 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "C:");
    ShowText(LCD_W * 2 / 4 + 2, (LCD_H - 1) *6/ NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "P:");
    ShowText(LCD_W * 2 / 4 + 2, (LCD_H - 1) *7.5 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "V:");

    // 创建线程
    StartThread();
}

void LCD::Loop() {
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {


        int len = sizeof(tab_val[1][0]);
        vTaskDelayUntil(&xLastWakeTime, 200);

        //显示当前电压值0.0
	    LCD_ShowFloatNum1(LCD_W * 2 / 4 + 40, (LCD_H - 1) * 0 / NUM_ROW + 20, vbus_voltage, 4, GREEN, BLACK, FontSize_16);
        if(Usart_Flag==1){
            ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *0 / NUM_ROW + 2, YELLOW, BLACK, FontSize_24, 0, "USART");}
        else if(Can_Flag==1){
            ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *0 / NUM_ROW + 2, GRAYBLUE, BLACK, FontSize_24, 0, "CAN");}
        else if((vbus_voltage==12||vbus_voltage<9)&&(Usart_Flag==0||Can_Flag==0)){
            ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *0 / NUM_ROW + 2, RED, BLACK, FontSize_24, 0, "NO-DC");}       
        else{
            ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *0 / NUM_ROW + 2, GREEN, BLACK, FontSize_24, 0, "READY");}

        // axis 0
        GetParameter("axis0.fet_thermistor.temperature", tab_val[7][0], sizeof(len));
        GetParameter("axis0.motor.current_control.Iq_setpoint", tab_val[6][0], 6);// 电机电流，此值为 Iq + Id    
        GetParameter("axis0.encoder.pos_estimate", tab_val[1][0], 6);  //当前预测到的位置值。
        GetParameter("axis0.encoder.vel_estimate", tab_val[2][0], 6);//当前估算转速

        //M0的mos温度
        ShowText(LCD_W * 0 / 4 + 85, (LCD_H - 1) *0 / NUM_ROW + 2, RED, BLACK, FontSize_12, 0,  (const char *)tab_val[7][0]);
        //M0的位置
        ShowText(LCD_W * 0 / 4 + 40, (LCD_H - 1) *6/ NUM_ROW + 2, BLUE, BLACK, FontSize_16, 0, (const char *)tab_val[1][0]);
        //M0的电流
        ShowText(LCD_W * 0 / 4 + 40, (LCD_H - 1) *4.5 / NUM_ROW + 2, BLUE, BLACK, FontSize_16, 0, (const char *)tab_val[6][0]);
        //M0的速度
        ShowText(LCD_W * 0 / 4 + 40, (LCD_H - 1) *7.5 / NUM_ROW + 2, BLUE, BLACK, FontSize_16, 0, (const char *)tab_val[2][0]);


        // axis 1
        GetParameter("axis1.fet_thermistor.temperature", tab_val[7][1], sizeof(len));  //MOS管温度
        GetParameter("axis1.encoder.pos_estimate", tab_val[1][1], 6);  //当前预测到的位置值。
        GetParameter("axis1.motor.current_control.Iq_setpoint", tab_val[6][1], 6); // 电机电流，此值为 Iq + Id
        GetParameter("axis1.encoder.vel_estimate", tab_val[2][1], 6); //当前估算转速


        //M1的mos温度
        ShowText(LCD_W * 0 / 4 + 85, (LCD_H - 1) *0 / NUM_ROW + 15, RED, BLACK, FontSize_12, 0,  (const char *)tab_val[7][1]);
         //M1的位置       
        ShowText(LCD_W * 2 / 4 + 40, (LCD_H - 1) *6/ NUM_ROW + 2, BLUE, BLACK, FontSize_16, 0,  (const char *)tab_val[1][1]);
        //M1的电流
        ShowText(LCD_W * 2 / 4 + 40, (LCD_H - 1) *4.5 / NUM_ROW + 2, BLUE, BLACK, FontSize_16, 0, (const char *)tab_val[6][1]);
        //M1的速度
        ShowText(LCD_W * 2 / 4 + 40, (LCD_H - 1) *7.5 / NUM_ROW + 2, BLUE, BLACK, FontSize_16, 0, (const char *)tab_val[2][1]);


		switch(axes[0]->controller_.config_.control_mode){         

			case 0:
                ShowText(LCD_W * 0 / 4 + 52, (LCD_H - 1) *3 / NUM_ROW + 2, RED, BLACK, FontSize_16, 0, "NUL");

				break;
			case 1:
                ShowText(LCD_W * 0 / 4 + 52, (LCD_H - 1) *3 / NUM_ROW + 2, BLUE, BLACK, FontSize_16, 0, " Tor");

				break;			
				case 2:
                    ShowText(LCD_W * 0 / 4 + 52, (LCD_H - 1) *3 / NUM_ROW + 2, GREEN, BLACK, FontSize_16, 0, "Vel");  
				break;			
				case 3:			
                     ShowText(LCD_W * 0/ 4 + 52, (LCD_H - 1) *3 / NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "Pos");
				break;				
			default:	break;		
        }	

		switch(axes[1]->controller_.config_.control_mode){         

			case 0:
                ShowText(LCD_W * 2 / 4 + 52, (LCD_H - 1) *3 / NUM_ROW + 2, RED, BLACK, FontSize_16, 0, "NUL");

				break;
			case 1:
                ShowText(LCD_W * 2 / 4 + 52, (LCD_H - 1) *3 / NUM_ROW + 2, BLUE, BLACK, FontSize_16, 0, " Tor");

				break;			
				case 2:
                    ShowText(LCD_W * 2 / 4 + 52, (LCD_H - 1) *3 / NUM_ROW + 2, GREEN, BLACK, FontSize_16, 0, "Vel");  
				break;			
				case 3:			
                     ShowText(LCD_W * 2/ 4 + 52, (LCD_H - 1) *3 / NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "Pos");
				break;				
			default:	break;		
        }




     /*   
        int len = sizeof(tab_val[1][0]);
        // axis 0
        GetParameter("axis0.encoder.pos_estimate", tab_val[1][0], sizeof(len));
        GetParameter("axis0.encoder.vel_estimate", tab_val[2][0], sizeof(len));
        GetParameter("axis0.controller.torque_setpoint", tab_val[3][0], sizeof(len));
        GetParameter("axis0.encoder.shadow_count", tab_val[4][0], sizeof(len));
        GetParameter("axis0.motor.gate_driver.drv_fault", tab_val[5][0], sizeof(len));
        GetParameter("axis0.motor.current_control.Ibus", tab_val[6][0], sizeof(len));
        GetParameter("axis0.fet_thermistor.temperature", tab_val[7][0], sizeof(len));

        GetParameter("odrv0.axis0.encoder.error", tab_val[8][0], sizeof(len));//编码器状态值


        // axis 1
        GetParameter("axis1.encoder.pos_estimate", tab_val[1][1], sizeof(len));  //当前预测到的位置值。
        GetParameter("axis1.encoder.vel_estimate", tab_val[2][1], sizeof(len)); //当前估算转速
        GetParameter("axis1.controller.torque_setpoint", tab_val[3][1], sizeof(len));  //设定的电机输出的力矩大小
        GetParameter("axis1.encoder.shadow_count", tab_val[4][1], sizeof(len));  //编码器值
        GetParameter("axis1.motor.gate_driver.drv_fault", tab_val[5][1], sizeof(len)); //DRV8301状态值
        GetParameter("axis1.motor.current_control.Ibus", tab_val[6][1], sizeof(len)); // 电机电流，此值为 Iq + Id
        GetParameter("axis1.fet_thermistor.temperature", tab_val[7][1], sizeof(len));  //MOS管温度

        //GetParameter("axis1.motor_thermistor.temperature", tab_val[8][1], sizeof(len));

        //显示当前电压值0.0
		LCD_ShowFloatNum1(LCD_W * 3 / 4 + 6, (LCD_H - 1) * 8 / NUM_ROW + 2, vbus_voltage,4,LIGHTGREEN, Color.BackColor,FontSize_12);


        
        //更新表项
        for (int i = 0; i < 9; i++) {
            ShowText(LCD_W * 1 / 4 + 10, (LCD_H - 1) * i / NUM_ROW + 2, Color.TextColor, Color.BackColor, FontSize_12, 0, (const char *)tab_val[i][0]);
            ShowText(LCD_W * 3 / 4 + 10, (LCD_H - 1) * i / NUM_ROW + 2, Color.TextColor, Color.BackColor, FontSize_12, 0, (const char *)tab_val[i][1]);
        }

*/
    }
}

static void srceen_loop(void *ctx) {
    reinterpret_cast<LCD *>(ctx)->Loop();
    reinterpret_cast<LCD *>(ctx)->thread_id_valid_ = false;
}

void LCD::StartThread() {
    osThreadDef(srceen, srceen_loop, osPriorityNormal, 0, stack_size_ / sizeof(StackType_t));
    thread_id_ = osThreadCreate(osThread(srceen), this);
    thread_id_valid_ = true;
}
/********************************************************************************
    @FunctionName:    Fill
    @Description:     在指定区域填充颜色
    @FunctionAuthor:  佚名
    @parameters:
                    NAME      TYPE       DESCRIBE
                    @xsta:    uint16_t   起始坐标
                    @ysta:    uint16_t   起始坐标
                    @xend:    uint16_t   终止坐标
                    @yend:    uint16_t   终止坐标
                    @color:   uint16_t   要填充的颜色
    @Return:          void
    @Other:
********************************************************************************/
void LCD::Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color) {
    uint16_t i, j;
    LCD_SetAddress(xsta, ysta, xend - 1, yend - 1);  //设置显示范围
    for (i = ysta; i < yend; i++) {
        for (j = xsta; j < xend; j++) {
            LCD_WR_DATA16(color);
        }
    }
}

/********************************************************************************
    @FunctionName:    DrawPoint
    @Description:     在指定位置画点
    @FunctionAuthor:  佚名
    @parameters:
                    NAME      TYPE       DESCRIBE
                    @x:       uint16_t   画点坐标
                    @y:       uint16_t   画点坐标
                    @color:   uint16_t   点颜色
    @Return:          void
    @Other:
********************************************************************************/
void LCD::DrawPoint(uint16_t x, uint16_t y, uint16_t color) {
    LCD_SetAddress(x, y, x, y);  //设置光标位置
    LCD_WR_DATA16(color);
}

/********************************************************************************
    @FunctionName:    DrawLine
    @Description:     画线
    @FunctionAuthor:  佚名
    @parameters:
                    NAME      TYPE       DESCRIBE
                    @x1:      uint16_t   起始坐标
                    @y1:      uint16_t   起始坐标
                    @x2:      uint16_t   起始坐标
                    @y2:      uint16_t   起始坐标
                    @color:   uint16_t   线的颜色
    @Return:          void
    @Other:
********************************************************************************/
void LCD::DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1;  //计算坐标增量
    delta_y = y2 - y1;
    uRow = x1;  //画线起点坐标
    uCol = y1;
    if (delta_x > 0) {
        incx = 1;
    }  //设置单步方向
    else if (delta_x == 0) {
        incx = 0;
    }  //垂直线
    else {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0) {
        incy = 1;
    } else if (delta_y == 0) {
        incy = 0;
    }  //水平线
    else {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y) {
        distance = delta_x;
    }  //选取基本增量坐标轴
    else {
        distance = delta_y;
    }
    for (t = 0; t < distance + 1; t++) {
        DrawPoint(uRow, uCol, color);  //画点
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
}

void LCD::DrawTab(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint16_t row, uint16_t col, uint16_t color) {
    int i, j;
    for (i = 0; i <= row; i++) {
        DrawLine(
            x0,
            y0 + height * i / row,
            x0 + width,
            y0 + height * i / row,
            color);
    }
    for (j = 0; j <= col; j++) {
        DrawLine(
            x0 + width * j / col,
            y0,
            x0 + width * j / col,
            y0 + height,
            color);
    }
}

/********************************************************************************
    @FunctionName:    DrawRectangle
    @Description:     画矩形
    @FunctionAuthor:  佚名
    @parameters:
                    NAME      TYPE       DESCRIBE
                    @x1:      uint16_t   起始坐标
                    @y1:      uint16_t   起始坐标
                    @x2:      uint16_t   终止坐标
                    @y2:      uint16_t   终止坐标
                    @color:   uint16_t   矩形的颜色
    @Return:          void
    @Other:
********************************************************************************/
void LCD::DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    DrawLine(x1, y1, x2, y1, color);
    DrawLine(x1, y1, x1, y2, color);
    DrawLine(x1, y2, x2, y2, color);
    DrawLine(x2, y1, x2, y2, color);
}

/********************************************************************************
    @FunctionName:    DrawCircle
    @Description:     画圆
    @FunctionAuthor:  佚名
    @parameters:
                    NAME      TYPE      DESCRIBE
                    @x0:      uint16_t   圆心坐标
                    @y0:      uint16_t   圆心坐标
                    @r:       uint8_t    半径
                    @color:   uint16_t   圆的颜色
    @Return:          void
    @Other:
********************************************************************************/
void LCD::DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color) {
    int a, b;
    a = 0;
    b = r;
    while (a <= b) {
        DrawPoint(x0 - b, y0 - a, color);  //3
        DrawPoint(x0 + b, y0 - a, color);  //0
        DrawPoint(x0 - a, y0 + b, color);  //1
        DrawPoint(x0 - a, y0 - b, color);  //2
        DrawPoint(x0 + b, y0 + a, color);  //4
        DrawPoint(x0 + a, y0 - b, color);  //5
        DrawPoint(x0 + a, y0 + b, color);  //6
        DrawPoint(x0 - b, y0 + a, color);  //7
        a++;
        if ((a * a + b * b) > (r * r))  //判断要画的点是否过远
        {
            b--;
        }
    }
}

/********************************************************************************
    @FunctionName:    ShowChinese
    @Description:     显示汉字串
    @FunctionAuthor:  佚名
    @parameters:
                    NAME      TYPE       DESCRIBE
                    @x:       uint16_t   显示坐标
                    @y:       uint16_t   显示坐标
                    @s:       uint8_t*   要显示的汉字串
                    @fc:      uint16_t   字的颜色
                    @bc:      uint16_t   字的背景色
                    @sizey:   uint8_t    字号
                    @mode:    uint8_t    0非叠加模式  1叠加模式
    @Return:          void
    @Other:
********************************************************************************/
void LCD::ShowChinese(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
    while (*s != 0) {
        if (sizey == 12) {
            ShowChinese12x12(x, y, s, fc, bc, sizey, mode);
        } else if (sizey == 16) {
            ShowChinese16x16(x, y, s, fc, bc, sizey, mode);
        } else if (sizey == 24) {
            ShowChinese24x24(x, y, s, fc, bc, sizey, mode);
        } else if (sizey == 32) {
            ShowChinese32x32(x, y, s, fc, bc, sizey, mode);
        } else {
            return;
        }
        s += 2;
        x += sizey;
    }
}

void LCD::ShowChinese12x12(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
    uint8_t i, j, m = 0;
    uint16_t k;
    uint16_t HZnum;        //汉字数目
    uint16_t TypefaceNum;  //一个字符所占字节大小
    uint16_t x0 = x;
    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    HZnum = sizeof(tfont12) / sizeof(typFNT_GB12);  //统计汉字数目
    for (k = 0; k < HZnum; k++) {
        if ((tfont12[k].Index[0] == *(s)) && (tfont12[k].Index[1] == *(s + 1))) {
            LCD_SetAddress(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode)  //非叠加方式
                    {
                        if (tfont12[k].Msk[i] & (0x01 << j)) {
                            LCD_WR_DATA16(fc);
                        } else {
                            LCD_WR_DATA16(bc);
                        }
                        m++;
                        if (m % sizey == 0) {
                            m = 0;
                            break;
                        }
                    } else  //叠加方式
                    {
                        if (tfont12[k].Msk[i] & (0x01 << j)) {
                            DrawPoint(x, y, fc);
                        }  //画一个点
                        x++;
                        if ((x - x0) == sizey) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
    }
}

void LCD::ShowChinese16x16(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
    uint8_t i, j, m = 0;
    uint16_t k;
    uint16_t HZnum;        //汉字数目
    uint16_t TypefaceNum;  //一个字符所占字节大小
    uint16_t x0 = x;
    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    HZnum = sizeof(tfont16) / sizeof(typFNT_GB16);  //统计汉字数目
    for (k = 0; k < HZnum; k++) {
        if ((tfont16[k].Index[0] == *(s)) && (tfont16[k].Index[1] == *(s + 1))) {
            LCD_SetAddress(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode)  //非叠加方式
                    {
                        if (tfont16[k].Msk[i] & (0x01 << j)) {
                            LCD_WR_DATA16(fc);
                        } else {
                            LCD_WR_DATA16(bc);
                        }
                        m++;
                        if (m % sizey == 0) {
                            m = 0;
                            break;
                        }
                    } else  //叠加方式
                    {
                        if (tfont16[k].Msk[i] & (0x01 << j)) {
                            DrawPoint(x, y, fc);
                        }  //画一个点
                        x++;
                        if ((x - x0) == sizey) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
    }
}

void LCD::ShowChinese24x24(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
    uint8_t i, j, m = 0;
    uint16_t k;
    uint16_t HZnum;        //汉字数目
    uint16_t TypefaceNum;  //一个字符所占字节大小
    uint16_t x0 = x;
    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    HZnum = sizeof(tfont24) / sizeof(typFNT_GB24);  //统计汉字数目
    for (k = 0; k < HZnum; k++) {
        if ((tfont24[k].Index[0] == *(s)) && (tfont24[k].Index[1] == *(s + 1))) {
            LCD_SetAddress(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode)  //非叠加方式
                    {
                        if (tfont24[k].Msk[i] & (0x01 << j)) {
                            LCD_WR_DATA16(fc);
                        } else {
                            LCD_WR_DATA16(bc);
                        }
                        m++;
                        if (m % sizey == 0) {
                            m = 0;
                            break;
                        }
                    } else  //叠加方式
                    {
                        if (tfont24[k].Msk[i] & (0x01 << j)) {
                            DrawPoint(x, y, fc);
                        }  //画一个点
                        x++;
                        if ((x - x0) == sizey) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响.
    }
}

void LCD::ShowChinese32x32(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
    uint8_t i, j, m = 0;
    uint16_t k;
    uint16_t HZnum;        //汉字数目
    uint16_t TypefaceNum;  //一个字符所占字节大小
    uint16_t x0 = x;
    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    HZnum = sizeof(tfont32) / sizeof(typFNT_GB32);  //统计汉字数目
    for (k = 0; k < HZnum; k++) {
        if ((tfont32[k].Index[0] == *(s)) && (tfont32[k].Index[1] == *(s + 1))) {
            LCD_SetAddress(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode)  //非叠加方式
                    {
                        if (tfont32[k].Msk[i] & (0x01 << j)) {
                            LCD_WR_DATA16(fc);
                        } else {
                            LCD_WR_DATA16(bc);
                        }
                        m++;
                        if (m % sizey == 0) {
                            m = 0;
                            break;
                        }
                    } else  //叠加方式
                    {
                        if (tfont32[k].Msk[i] & (0x01 << j)) {
                            DrawPoint(x, y, fc);
                        }  //画一个点
                        x++;
                        if ((x - x0) == sizey) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
    }
}

/********************************************************************************
    @FunctionName:    ShowChar
    @Description:     显示单个字符
    @FunctionAuthor:  佚名
    @parameters:
                    NAME      TYPE       DESCRIBE
                    @x:       uint16_t   显示坐标
                    @y:       uint16_t   显示坐标
                    @chr:     uint8_t    要显示的字符
                    @fc:      uint16_t   字的颜色
                    @bc:      uint16_t   字的背景色
                    @sizey:   uint8_t    字号
                    @mode:    uint8_t    0非叠加模式  1叠加模式
    @Return:          void
    @Other:
********************************************************************************/
void LCD::ShowChar(uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
    uint8_t temp, sizex, t, m = 0;
    uint16_t i, TypefaceNum;  //一个字符所占字节大小
    uint16_t x0 = x;
    sizex = sizey / 2;
    TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
    chr = chr - ' ';                                     //得到偏移后的值
    LCD_SetAddress(x, y, x + sizex - 1, y + sizey - 1);  //设置光标位置
    for (i = 0; i < TypefaceNum; i++) {
        if (sizey == 12) {
            temp = ascii_1206[chr][i];
        }  //调用6x12字体
        else if (sizey == 16) {
            temp = ascii_1608[chr][i];
        }  //调用8x16字体
        else if (sizey == 24) {
            temp = ascii_2412[chr][i];
        }  //调用12x24字体
        else if (sizey == 32) {
            temp = ascii_3216[chr][i];
        }  //调用16x32字体
        else {
            return;
        }
        for (t = 0; t < 8; t++) {
            if (!mode)  //非叠加模式
            {
                if (temp & (0x01 << t)) {
                    LCD_WR_DATA16(fc);
                } else {
                    LCD_WR_DATA16(bc);
                }
                m++;
                if (m % sizex == 0) {
                    m = 0;
                    break;
                }
            } else  //叠加模式
            {
                if (temp & (0x01 << t)) {
                    DrawPoint(x, y, fc);
                }  //画一个点
                x++;
                if ((x - x0) == sizex) {
                    x = x0;
                    y++;
                    break;
                }
            }
        }
    }
}

/********************************************************************************
    @FunctionName:    ShowString
    @Description:     显示字符串
    @FunctionAuthor:  矛盾聚合体
    @parameters:
                    NAME      TYPE       DESCRIBE
                    @x:       uint16_t   显示坐标
                    @y:       uint16_t   显示坐标
                    @p:       uint8_t*   要显示的字符串
                    @fc:      uint16_t   字的颜色
                    @bc:      uint16_t   字的背景色
                    @sizey:   uint8_t    字号
                    @mode:    uint8_t    0非叠加模式  1叠加模式
    @Return:          void
    @Other:
********************************************************************************/
void LCD::ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
    while (*p != '\0') {
        ShowChar(x, y, *p, fc, bc, sizey, mode);
        x += sizey / 2;
        p++;
    }
}

/********************************************************************************
    @FunctionName:    ShowText
    @Description:     显示字符串
    @FunctionAuthor:  矛盾聚合体
    @parameters:
                    NAME      TYPE       DESCRIBE
                    @x:       uint16_t   显示坐标
                    @y:       uint16_t   显示坐标
                    @fc:      uint16_t   字的颜色
                    @bc:      uint16_t   字的背景色
                    @sizey:   uint8_t    字号
                    @mode:    uint8_t    0非叠加模式  1叠加模式
                    @str:     uint8_t*   要显示的字符串
    @Return:          void
    @Other:
********************************************************************************/
void LCD::ShowText(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode, const char *str, ...) {
    char strFormatted[256];
    va_list args;         //存放可变参数的数据结构
    va_start(args, str);  //初始化可变参数,需要传一个va_list类型变量,和可变参数之前的参数,这里是str
    vsprintf(strFormatted, str, args);
    ShowString(x, y, (const uint8_t *)strFormatted, fc, bc, sizey, mode);  //此函数再头文件 stdio中
    va_end(args);
}

/********************************************************************************
    @FunctionName:    ShowPicture
    @Description:     显示图片
    @FunctionAuthor:  佚名
    @parameters:
                    NAME      TYPE      DESCRIBE
                    @x:       uint16_t   起点坐标
                    @y:       uint16_t   起点坐标
                    @length:  uint16_t   图片长度
                    @width:   uint16_t   图片宽度
                    @pic[]:   constuint8_t 图片数组
    @Return:          void
    @Other:
********************************************************************************/
void LCD::ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[]) {
    uint16_t i, j;
    uint32_t k = 0;
    LCD_SetAddress(x, y, x + length - 1, y + width - 1);
    for (i = 0; i < length; i++) {
        for (j = 0; j < width; j++) {
            LCD_WR_DATA8(pic[k * 2]);
            LCD_WR_DATA8(pic[k * 2 + 1]);
            k++;
        }
    }
}

/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;
	return result;
}



/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD::LCD_ShowFloatNum1(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp,sizex;
	uint16_t num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}
