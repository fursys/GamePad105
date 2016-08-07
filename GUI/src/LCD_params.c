#include "GUI.h"

#define LEFT_INDENT 3
#define TOP_LINE 101

int lcdpf_CursorPos_Y= 0;
uint8_t lcdpf_EditMode = 0;

void DrawHeaders ()
{
    uint8_t line = TOP_LINE;
    LCDPutStrROM("LCD parameters", 121, LEFT_INDENT, SMALL, GREEN, BLACK);
    LCDPutStrROM("Brightness", line, LEFT_INDENT, SMALL, GREEN, BLACK);
    line -= 10;
    LCDPutStrROM("RGB", line, LEFT_INDENT, SMALL, GREEN, BLACK);
    line -= 10;
    LCDPutStrROM("LAO top to bottom", line, LEFT_INDENT, SMALL, GREEN, BLACK);
    line -= 10;
    LCDPutStrROM("Write Y direction", line, LEFT_INDENT, SMALL, GREEN, BLACK);
    line -= 10;
    LCDPutStrROM("Mirror X", line, LEFT_INDENT, SMALL, GREEN, BLACK);
    line -= 10;
    LCDPutStrROM("Mirror Y", line, LEFT_INDENT, SMALL, GREEN, BLACK);
    line -= 10;
    LCDPutStrROM("Software rotate", line, LEFT_INDENT, SMALL, GREEN, BLACK);
}
void DrawValue (uint8_t line, uint8_t cursor)
{
    char str[4];
    char stri[4];
    int len;
    if (!line)
    {
        memset(str, 32, 4);//init result string
        itoa (Parameters.LCD_Britness, stri, &len);
        memcpy(str,stri,len+1);
        if (cursor) LCDPutStr (str, TOP_LINE - line*10, LEFT_INDENT + 108, SMALL, BLACK, lcdpf_EditMode ? GREEN : RED);
        else LCDPutStr (str, TOP_LINE - line*10, LEFT_INDENT + 108, SMALL, GREEN, BLACK);
    }
    else
    {
        if ((line == 6) && (Parameters.LCD_ScreenRotation == 1))
        {
            if (cursor) LCDPutStrROM("ON ", TOP_LINE - line*10, LEFT_INDENT + 108, SMALL, BLACK, lcdpf_EditMode ? GREEN : RED);
            else LCDPutStrROM("ON ", TOP_LINE - line*10, LEFT_INDENT + 108, SMALL, GREEN, BLACK);
        }
        else if ((line == 6) && (Parameters.LCD_ScreenRotation == 0))
        {
            if (cursor) LCDPutStrROM("OFF", TOP_LINE - line*10, LEFT_INDENT + 108, SMALL, BLACK, lcdpf_EditMode ? GREEN : RED);
            else LCDPutStrROM("OFF", TOP_LINE - line*10, LEFT_INDENT + 108, SMALL, GREEN, BLACK);
        }
        else if (Parameters.LCD_MADCTL & (4<<line))
        {
            if (cursor) LCDPutStrROM("ON ", TOP_LINE - line*10, LEFT_INDENT + 108, SMALL, BLACK, lcdpf_EditMode ? GREEN : RED);
            else LCDPutStrROM("ON ", TOP_LINE - line*10, LEFT_INDENT + 108, SMALL, GREEN, BLACK);
        }
        else
        {
            if (cursor) LCDPutStrROM("OFF", TOP_LINE - line*10, LEFT_INDENT + 108, SMALL, BLACK, lcdpf_EditMode ? GREEN : RED);
            else LCDPutStrROM("OFF", TOP_LINE - line*10, LEFT_INDENT + 108, SMALL, GREEN, BLACK);
        }
    }
}

void DrawAllValues ()
{
    for (int i = 0; i < 7;i++)
    {
        if (i==lcdpf_CursorPos_Y) DrawValue(i,1);
        else DrawValue(i,0);
    }
}

void LCD_parameters_form (uint8_t event)
{
    	switch (event)
	{
		case NO_EVENT:

			break;
		case KEY_DOWN_EVENT:
		    if (lcdpf_EditMode)
            {
                if (Parameters.LCD_Britness < MAX_LCD_BRITNESS)
                {
                    Parameters.LCD_Britness++;
                    DrawValue(lcdpf_CursorPos_Y,1);
                }
            }
            else
            {
                DrawValue(lcdpf_CursorPos_Y,0);
                lcdpf_CursorPos_Y++;
                if (lcdpf_CursorPos_Y > 6) lcdpf_CursorPos_Y = 0;
                DrawValue(lcdpf_CursorPos_Y,1);
            }
			break;
		case KEY_UP_EVENT:
		    if (lcdpf_EditMode)
            {
                if (Parameters.LCD_Britness > 0)
                {
                    Parameters.LCD_Britness--;
                    DrawValue(lcdpf_CursorPos_Y,1);
                }
            }
            else
            {
                DrawValue(lcdpf_CursorPos_Y,0);
                lcdpf_CursorPos_Y--;
                if (lcdpf_CursorPos_Y < 0) lcdpf_CursorPos_Y = 6;
                DrawValue(lcdpf_CursorPos_Y,1);
            }
			break;
		case KEY_ENTER_EVENT:
		    if (lcdpf_CursorPos_Y == 6)
            {
                Parameters.LCD_ScreenRotation ^= 1;
                LCDSetRotation(Parameters.LCD_ScreenRotation);
                LCDSetRect(0, 0, 131, 131, FILL, BLACK);
                DrawHeaders ();
                DrawAllValues ();
            }
		    else if (lcdpf_CursorPos_Y)
            {
                Parameters.LCD_MADCTL ^= 4<<lcdpf_CursorPos_Y;
                LCDSetMADCTL (Parameters.LCD_MADCTL);
                LCDSetRect(0, 0, 131, 131, FILL, BLACK);
                DrawHeaders ();
                DrawAllValues ();
            }
            else
            {
                if (lcdpf_EditMode) lcdpf_EditMode = 0;
                else lcdpf_EditMode = 1;
                DrawValue(lcdpf_CursorPos_Y,1);
            }

			break;
		case KEY_ESC_EVENT:
            CurrentFormProc = Menu_Form;
            CurrentFormProc(FORM_INIT_EVENT);
			break;
		case FORM_INIT_EVENT:
			//event = NO_EVENT;
            LCDSetRect(0, 0, 131, 131, FILL, BLACK);
			DrawHeaders ();
			DrawAllValues ();
			LCDFlushTxBuffer();
			break;
		 default:
			break;
	}
}
