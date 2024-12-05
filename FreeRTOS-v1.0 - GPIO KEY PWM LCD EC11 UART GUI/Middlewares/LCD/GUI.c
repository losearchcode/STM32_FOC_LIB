#include "GUI.h"

uint8_t Title_Width = 30;
uint8_t Item_Width = 30;
uint8_t GUI_Item_Sizey = 24;
uint8_t GUI_Title_Sizey = 32;

GUI_Parameter_t GUI1_SubMenu_Para[]=
																{	
																	{"LED_Hz:", (void *)50, NULL},
																	{"Brightness:", (void *)100, 1},
																};
																
GUI_Parameter_t GUI2_SubMenu_Para[]=
																{	
																	{"P:", (void *)50, NULL},
																	{"I:", (void *)100, 1},
																	{"D:", (void *)50, NULL},
																	{"E:", (void *)100, 1},
																};

GUI_Parameter_t GUI3_SubMenu_Para[]=
																{	
																	{"P:", (void *)50, NULL},
																	{"I:", (void *)100, 1},
																	{"D:", (void *)50, NULL},
																	{"E:", (void *)100, 1},
																};

GUI_Parameter_t GUI4_SubMenu_Para[]=
																{	
																	{"Contrast:", (void *)50, NULL},
																	{"Brightness:", (void *)100, 1},
																};																																
// 初始化一级菜单
GUI_MenuItem_t MainMenu[Main_Menu_Num] = {
    {"Gnenal Info", NULL, GUI1_SubMenu_Para, 2},
    {"Low_Speed_PID", NULL, GUI2_SubMenu_Para, 4},
    {"High_Speed_PID", NULL, GUI3_SubMenu_Para, 4},
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
	Title_Width = GUI_Title_Sizey+6;
	Item_Width = GUI_Item_Sizey+6;
	GUI_Index_Current.Main_Menu_Index = 2;
	GUI_Index_Current.Main_Menu_Last_Index = 0;
	GUI_Index_Current.Submenu_Index = 0;
	GUI_Index_Current.Submenu_Last_Index = 0;
	
	LCD_Fill(0,0,240,Title_Width,LIGHTBLUE);
	LCD_Fill(0,Title_Width,240,240,WHITE);
	LCD_DrawLine(90,Title_Width,90,240,LIGHTBLUE);
}

void GUI_Show_Static(void)
{
	uint8_t i;
	
	uint8_t title_len;
	
	title_len = strlen(MainMenu[GUI_Index_Current.Main_Menu_Index].Menu_Text);
	
	
	LCD_ShowString(120-(title_len*8),0,(uint8_t *)MainMenu[GUI_Index_Current.Main_Menu_Index].Menu_Text,RED,LIGHTBLUE,GUI_Title_Sizey,0);
	
	if(MainMenu[GUI_Index_Current.Main_Menu_Index].NumParameters != 0)
		for (i = 0; i<MainMenu[GUI_Index_Current.Main_Menu_Index].NumParameters ; i++)
		{
				LCD_ShowString(0,Title_Width+i*Item_Width,(uint8_t *)MainMenu[GUI_Index_Current.Main_Menu_Index].Parameters[i].Name,RED,WHITE,GUI_Item_Sizey,0);
		}
}

void GUI_Show_Dynamic(void* parameter)
{
	uint8_t* param = (uint8_t*)parameter;
	
	uint8_t LED_Hz =	* param;
	LCD_ShowIntNum(120,120,(uint8_t)(9-LED_Hz),2,RED,WHITE,24);
	
	LCD_ShowIntNum(120,160,EC11.EC11_Analyze_Value,2,RED,WHITE,24);
	uint8_t time = EC11_SCAN_PERIOD_MS;
	LCD_ShowIntNum(80,160,time,2,RED,WHITE,24);
	
	LCD_ShowIntNum(120,200,EC11.EC11_Analyze_Last_Value,2,RED,WHITE,24);
}

