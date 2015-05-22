 #ifndef _DEBUG_H
 #define _DEBUG_H
// ----------------------------------------------
#define CMD_NONE			0x00
#define CMD_RAW_OUTPUT		0x01
#define CMD_ERROR_MSG		0x02
#define CMD_WARNING_MSG		0x04
#define CMD_GREEN_MSG		0x08

// debug console in MK-Tool can also handle ANSI ESC seq. 
#define ANSI_ATTRIBUTE_OFF		"\033[0m"
#define ANSI_BOLD				"\033[1m"
#define ANSI_UNDERSCORE			"\033[4m"
#define ANSI_BLINK				"\033[5m"
#define ANSI_INVERSE			"\033[7m"
#define ANSI_INVISIBLE			"\033[8m"

#define ANSI_COLOR_BLACK		"\033[30m"
#define ANSI_COLOR_RED			"\033[31m"
#define ANSI_COLOR_GREEN		"\033[32m"
#define ANSI_COLOR_YELLOW		"\033[33m"
#define ANSI_COLOR_BLUE			"\033[34m"
#define ANSI_COLOR_VIOLETT		"\033[35m"
#define ANSI_COLOR_KOBALTBLUE	"\033[36m"
#define ANSI_COLOR_WHITE		"\033[37m"

#define ANSI_CLEAR				"\033[2J"
#define ANSI_HOME				"\033[H"

// macros for easier use 
#ifdef DEBUG															// only include functions if DEBUG is defined in main.h

#define Debug(format, args...)			{   _printf_P(&Debug_Putchar, PSTR(format) , ## args); DebugSend(CMD_NONE); } 
#define Debug_Raw(format, args...)		{   _printf_P(&Debug_Putchar, PSTR(format) , ## args); DebugSend(CMD_RAW_OUTPUT); } 
#define Debug_Warning(format, args...)	{   _printf_P(&Debug_Putchar, PSTR(format) , ## args); DebugSend(CMD_WARNING_MSG); } 
#define Debug_Error(format, args...)	{   _printf_P(&Debug_Putchar, PSTR(format) , ## args); DebugSend(CMD_ERROR_MSG); } 
#define Debug_OK(format, args...)		{   _printf_P(&Debug_Putchar, PSTR(format) , ## args); DebugSend(CMD_GREEN_MSG); } 

struct str_Debug
{
 unsigned char Cmd;			// bitcoded command 
 char Text[32]; 
};

extern struct str_Debug    tDebug;
unsigned char SendDebugOutput;

void Debug_Putchar(char c);
void DebugSend(unsigned char cmd);

#else						// dummy macros (won't waste flash, if #DEBUG is disabled)
#define Debug(format, args...)			;
#define Debug_Raw(format, args...)		;
#define Debug_Warning(format, args...)	;
#define Debug_Error(format, args...)	;
#define Debug_OK(format, args...)		;
#endif

// ----------------------------------------------
#endif
