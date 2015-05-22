#ifndef _MENU_H
#define _MENU_H

extern void Menu(void);
extern void LcdClear(void);
extern void Menu_Putchar(char c);
extern char DisplayBuff[80];
extern unsigned char DispPtr;

extern unsigned char MaxMenue;
extern unsigned char MenuePunkt;
extern unsigned char RemoteKeys;

#define LCD_printfxy(x,y,format, args...)  { DispPtr = (y) * 20 + (x); _printf_P(&Menu_Putchar,PSTR(format) , ## args);}
#define LCD_printf(format, args...)        {  _printf_P(&Menu_Putchar, PSTR(format) , ## args);}

#endif //_MENU_H
