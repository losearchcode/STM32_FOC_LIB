#ifndef __LCD_GUI_H
#define __LCD_GUI_H

#include	"main.h"

#define Main_Menu_Num 4 


typedef struct {
		uint8_t Main_Menu_Index;
		uint8_t Submenu_Index;
		uint8_t Main_Menu_Last_Index;
		uint8_t Submenu_Last_Index;
} GUI_Index_t;


typedef struct{
    char *Name;
    void *Value;  // 指向实际值的指针
    uint8_t Edit_Flag;
} GUI_Parameter_t;

typedef struct {
    char *Menu_Text;
    struct MenuItem *SubMenu;  // 子菜单
    GUI_Parameter_t *Parameters;     // 参数列表
    uint8_t NumParameters;      // 参数数量
} GUI_MenuItem_t;

void GUI_Init(uint8_t GUI_Item_Sizey_);
void GUI_Show_Static(void);
void GUI_Show_Dynamic(void* parameter);

#endif

