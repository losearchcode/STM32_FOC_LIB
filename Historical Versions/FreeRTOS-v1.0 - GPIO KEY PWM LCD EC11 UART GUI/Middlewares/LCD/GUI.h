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
    void *Value;  // ָ��ʵ��ֵ��ָ��
    uint8_t Edit_Flag;
} GUI_Parameter_t;

typedef struct {
    char *Menu_Text;
    struct MenuItem *SubMenu;  // �Ӳ˵�
    GUI_Parameter_t *Parameters;     // �����б�
    uint8_t NumParameters;      // ��������
} GUI_MenuItem_t;

void GUI_Init(uint8_t GUI_Item_Sizey_);
void GUI_Show_Static(void);
void GUI_Show_Dynamic(void* parameter);

#endif

