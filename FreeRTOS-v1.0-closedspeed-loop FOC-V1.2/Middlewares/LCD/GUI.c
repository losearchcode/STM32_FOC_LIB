#include "GUI.h"

uint8_t Title_Width = 30;
uint8_t Item_Width = 30;
uint8_t GUI_Item_Sizey = 24;
uint8_t GUI_Title_Sizey = 32;

GUI_Parameter_t GUI1_SubMenu_Para[]=
																{	
																	{"Cur_Ang: ", &LCD_Show_Parameter.AS5600_angle_Show, NULL,FLOAT},
																	{"El_Ang:  ", &LCD_Show_Parameter.xCurrent_electricalAngle_Show, NULL,FLOAT},
																	{"Vel:     ", &LCD_Show_Parameter.xCurrent_Vel_Show, NULL,FLOAT},
																	{"Motor_PP:", &FOC_Init_Parameter[0].Motor_PP, NULL,INT},
																	{"Sen_DIR: ", &FOC_Init_Parameter[0].Sensor_DIR, NULL,INT},
																	{"xElTime: ", &LCD_Show_Parameter.xElapsedTime_Show, NULL,INT},
																	{"LED_Hz:  ", &LCD_Show_Parameter.LED1_Delay_time_Show, 1,INT},
																};

								
GUI_Parameter_t GUI2_SubMenu_Para[]=
																{	
																	{"P:       ", &(PID_Vel_Loop_Parameter[0].P), 1,FLOAT},
																	{"I:       ", &(PID_Vel_Loop_Parameter[0].I), 1,FLOAT},
																	{"D:       ", &(PID_Vel_Loop_Parameter[0].D), 1,FLOAT},
																	{"E:       ", &(PID_Vel_Loop_Parameter[0].error_prev), NULL,FLOAT},
																	{"Target:  ", &Vel_Target, 1,FLOAT},
																	{"Vel:     ", &LCD_Show_Parameter.xCurrent_Vel_Show, NULL,FLOAT},
																};

																
																
GUI_Parameter_t GUI3_SubMenu_Para[]=
																{	
																	{"P:       ", &(PID_Angle_Loop_Parameter[0].P), 1,FLOAT},
																	{"I:       ", &(PID_Angle_Loop_Parameter[0].I), 1,FLOAT},
																	{"D:       ", &(PID_Angle_Loop_Parameter[0].D), 1,FLOAT},
																	{"E:       ", &(PID_Angle_Loop_Parameter[0].error_prev), NULL,FLOAT},
																	{"Target:  ", &Angle_Target, 1,FLOAT},
																	{"Angle:   ", &Sensor_AS5600_Parameter[0].Angle_prev, NULL,FLOAT},
																};

GUI_Parameter_t GUI4_SubMenu_Para[]=
																{	
																	{"Contrast:", (void *)50, NULL,INT},
																	{"Bright:  ", (void *)100, 1,INT},
																};																																
// 初始化一级菜单
GUI_MenuItem_t GUI_MainMenu[Main_Menu_Num] = {
    {"Gnenal Info", NULL, GUI1_SubMenu_Para, 7},
    {"Speed_PID", NULL, GUI2_SubMenu_Para, 6},
    {"Angle_PID", NULL, GUI3_SubMenu_Para, 6},
    {"Help", NULL, GUI4_SubMenu_Para, 2}
};

GUI_Index_t GUI_Index_Current;

/*******															
初始化二级菜单,以此类推
MenuItem viewSubMenu[] = {
    {"Display Settings", NULL, &param1, 1},
    {"Color Settings", NULL, &param2, 1}
};
*****/

void GUI_Init(uint8_t GUI_Item_Sizey_)
{
	GUI_Title_Sizey = GUI_Item_Sizey_+8;
	GUI_Item_Sizey  = GUI_Item_Sizey_;
	Title_Width = GUI_Title_Sizey+4;
	Item_Width = GUI_Item_Sizey+4;
	GUI_Index_Current.Main_Menu_Index = 0;
	GUI_Index_Current.Main_Menu_Last_Index = 0;
	GUI_Index_Current.Submenu_Index = 0;
	GUI_Index_Current.Submenu_Last_Index = 0;
	GUI_Index_Current.Edit_Flag = 0;
}

void GUI_Show_Static(void)
{
	uint8_t Title_Temp;
	
	uint8_t title_len;
	
	title_len = strlen(GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Menu_Text);
	
	LCD_Fill(0,0,240,Title_Width,LIGHTBLUE);
	LCD_Fill(0,Title_Width,240,240,WHITE);
	LCD_DrawLine(110,Title_Width,110,240,LIGHTBLUE);
	
	LCD_ShowString(120-(title_len*8),0,(uint8_t *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Menu_Text,RED,LIGHTBLUE,GUI_Title_Sizey,0);
	
	if(GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].NumParameters != 0)
		for (Title_Temp = 0; Title_Temp<GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].NumParameters ; Title_Temp++)
		{
				LCD_ShowString(0,Title_Width+Title_Temp*Item_Width,(uint8_t *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[Title_Temp].Name,RED,WHITE,GUI_Item_Sizey,0);
		}
}

uint8_t processParameter(GUI_Parameter_t *param) 
{
    uint8_t result_value=0;
		switch (param->ValueType) {
        case INT:
            // 处理整数类型
						result_value = 1;
            break;
        case FLOAT:
            // 处理浮点数类型
            result_value = 2;
            break;
        case DOUBLE:
            // 处理双精度浮点数类型
            result_value = 3;
            break;
        default:
            // 处理未知类型或错误情况
//            printf("Unknown type\n");
            break;
			}
		return result_value;
}

void GUI_Show_Dynamic(void)
{
	uint8_t Item_Temp;
	uint8_t num1;
	if(GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].NumParameters != 0)
	{
		for (Item_Temp = 0; Item_Temp<GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].NumParameters ; Item_Temp++)
		{
			if(Item_Temp != GUI_Index_Current.Submenu_Index)
				LCD_ShowString(220,Title_Width+Item_Temp*Item_Width," ",RED,WHITE,GUI_Item_Sizey,0);
			else if(GUI_Index_Current.Edit_Flag == 0)
				LCD_ShowString(220,Title_Width+Item_Temp*Item_Width,"<",RED,WHITE,GUI_Item_Sizey,0);
			else
				LCD_ShowString(220,Title_Width+Item_Temp*Item_Width,"*",RED,WHITE,GUI_Item_Sizey,0);
			
			num1 = processParameter(&GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[Item_Temp]);
			switch (num1){
				case 1:
					LCD_ShowIntNum(120,Title_Width+Item_Temp*Item_Width,*(long*)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[Item_Temp].Value,4,RED,WHITE,GUI_Item_Sizey);
					break;
				case 2:
					if(GUI_Index_Current.Main_Menu_Index == 0 || Item_Temp>=3)
						LCD_ShowFloatNum1(120,Title_Width+Item_Temp*Item_Width,*(float *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[Item_Temp].Value,6,RED,WHITE,GUI_Item_Sizey);
					else
						LCD_ShowFloatNum2(120,Title_Width+Item_Temp*Item_Width,*(float *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[Item_Temp].Value,6,RED,WHITE,GUI_Item_Sizey);
					
//						LCD_ShowFloatNum2(120,Title_Width+Item_Temp*Item_Width,0.00024,6,RED,WHITE,GUI_Item_Sizey);
					break;
				case 3:
					if(GUI_Index_Current.Main_Menu_Index == 0 || Item_Temp>=3)
						LCD_ShowFloatNum1(120,Title_Width+Item_Temp*Item_Width,*(float *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[Item_Temp].Value,6,RED,WHITE,GUI_Item_Sizey);
					else
						LCD_ShowFloatNum2(120,Title_Width+Item_Temp*Item_Width,*(float *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[Item_Temp].Value,6,RED,WHITE,GUI_Item_Sizey);
					break;
				default:
            // 处理未知类型或错误情况
						printf("Unknown type\n");
            break;
				}
		}
	}
}

void EC11_GUI_Callback(void)
{
	uint8_t Menu_Param_Type;
	
	if(EC11.EC11_Analyze_Value == 5)
			{
				if(GUI_Index_Current.Edit_Flag == 0)
				{
					GUI_Index_Current.Submenu_Index --;
					if(GUI_Index_Current.Submenu_Index ==255)
					{
						GUI_Index_Current.Submenu_Index = GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].NumParameters-1;
					}
				}
				else
				{
					if(GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Edit_Flag == 0)
					{
						GUI_Index_Current.Edit_Flag = 0;
						GUI_Index_Current.Submenu_Index --;
						if(GUI_Index_Current.Submenu_Index ==255)
						{
							GUI_Index_Current.Submenu_Index = GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].NumParameters-1;
						}
					}
					else
					{
						Menu_Param_Type = processParameter(&GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index]);
						switch (Menu_Param_Type){
						case 1:
							*(long *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Value -= 1;
							break;
						case 2:
							if(GUI_Index_Current.Main_Menu_Index != 0 && GUI_Index_Current.Submenu_Index < 4 )
								*(float *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Value -= 0.00001;
							else
								*(float *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Value -= 1;	
							break;
						case 3:
							if(GUI_Index_Current.Main_Menu_Index == 0 && GUI_Index_Current.Submenu_Index < 4 )
								*(double *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Value -= 0.00001;
							else
								*(double *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Value -= 1;	
							break;
						default:
							// 处理未知类型或错误情况
							printf("EC11 Unknown type\n");
							break;
						}
					}
				}
				
			}
			
			if(EC11.EC11_Analyze_Value == 6)
			{
				if(GUI_Index_Current.Edit_Flag == 0)
				{
					GUI_Index_Current.Submenu_Index ++;
					if(GUI_Index_Current.Submenu_Index ==GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].NumParameters)
					{
						GUI_Index_Current.Submenu_Index = 0;
					}
				}
				else
				{
					if(GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Edit_Flag == 0)
					{
						GUI_Index_Current.Edit_Flag = 0;
						GUI_Index_Current.Submenu_Index ++;
						if(GUI_Index_Current.Submenu_Index ==GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].NumParameters)
						{
							GUI_Index_Current.Submenu_Index = 0;
						}
					}
					else
					{
						Menu_Param_Type = processParameter(&GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index]);
						switch (Menu_Param_Type){
						case 1:
							*(long *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Value += 1;
							break;
						case 2:
							if(GUI_Index_Current.Main_Menu_Index != 0 && GUI_Index_Current.Submenu_Index < 4 )
								*(float *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Value += 0.00001;
							else
								*(float *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Value += 1;	
							break;
						case 3:
							if(GUI_Index_Current.Main_Menu_Index == 0 && GUI_Index_Current.Submenu_Index < 4 )
								*(double *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Value += 0.00001;
							else
								*(double *)GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Value += 1;	
							break;
						default:
							// 处理未知类型或错误情况
							printf("EC11 Unknown type\n");
							break;
						}
					}
				}
				
			}
			
			if(EC11.EC11_Analyze_Value == 1)
			{
				if(GUI_Index_Current.Edit_Flag == 1 )
					GUI_Index_Current.Edit_Flag = 0;
				else if(GUI_MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[GUI_Index_Current.Submenu_Index].Edit_Flag)
					GUI_Index_Current.Edit_Flag = 1;
			}
			
			if(EC11.EC11_Analyze_Value == 8)
			{
				GUI_Index_Current.Main_Menu_Index ++;
				if(GUI_Index_Current.Main_Menu_Index == Main_Menu_Num)
					GUI_Index_Current.Main_Menu_Index = 0;
			}
			if(EC11.EC11_Analyze_Value == 7)
			{
				GUI_Index_Current.Main_Menu_Index --;
				if(GUI_Index_Current.Main_Menu_Index == 255)
					GUI_Index_Current.Main_Menu_Index = Main_Menu_Num-1;
			}
}

