#ifndef __LCD_GUI_H
#define __LCD_GUI_H

#include	"main.h"

#define Main_Menu_Num 4 


typedef struct {
		uint8_t Main_Menu_Index;
		uint8_t Submenu_Index;
		uint8_t Main_Menu_Last_Index;
		uint8_t Submenu_Last_Index;
		uint8_t Edit_Flag;
} GUI_Index_t;


typedef struct{
    char *Name;
    void *Value;  // 指向实际值的指针
    uint8_t Edit_Flag;
		enum { INT, FLOAT, DOUBLE } ValueType; // 添加类型标识
} GUI_Parameter_t;

typedef struct {
    char *Menu_Text;
    struct MenuItem *SubMenu;  // 子菜单
    GUI_Parameter_t *Parameters;     // 参数列表
    uint8_t NumParameters;      // 参数数量
} GUI_MenuItem_t;

extern GUI_MenuItem_t GUI_MainMenu[Main_Menu_Num];
extern GUI_Index_t GUI_Index_Current;

void GUI_Init(uint8_t GUI_Item_Sizey_);
void GUI_Show_Static(void);
uint8_t processParameter(GUI_Parameter_t *param) ;
void GUI_Show_Dynamic(void);
void EC11_GUI_Callback(void);

#endif

