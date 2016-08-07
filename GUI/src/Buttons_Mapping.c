#include "GUI.h"
#include <MyLib.h>

char str[23];
char stri[4];
int len;
uint8_t bm_editMode = 0;
uint8_t bm_CursorPos = 0;
//------------------------------------------------------------------------------------------------
uint8_t bm_Replace (uint8_t from, uint8_t to)
{
    for (int i=0;i<32;i++)
    {
        if (Parameters.ButtonsMapping [i] == from)
        {
            Parameters.ButtonsMapping [i] = to;
            return i;
        }
    }
    return 255;
}
//------------------------------------------------------------------------------------------------
uint8_t bm_Detect (void)
{
    for (int i=0;i<25;i++)
    {
        if (Buttons & (1<<i)) return i;
    }
    return 255;
}
//------------------------------------------------------------------------------------------------
void bm_PrintValue (uint8_t index, uint8_t cursor)
{
    uint8_t x;
    uint8_t y;

    y = index / 8;
    x = index - y*8;

    memset(str, 32, 22);//init result string
    itoa (Parameters.ButtonsMapping [index], stri, &len );

    memcpy(str+(4-len),stri,len+1);

    if (cursor) LCDPutStr (str, 111 - x*10, 6+y*24, SMALL, BLACK, bm_editMode ? GREEN : RED);
    else LCDPutStr (str, 111 - x*10, 6+y*24, SMALL, GREEN, BLACK);

    //LCDPutStr (str, 111 - x*10, 6+y*24, SMALL, GREEN, BLACK);
}
//------------------------------------------------------------------------------------------------
void Buttons_Mapping_form (uint8_t event)
{
    switch (event)
    {
		case NO_EVENT:
            if (bm_editMode)
            {
                uint8_t bd = bm_Detect();
                if (bd != 255)
                {
                    //uint8_t old = Parameters.ButtonsMapping [bm_CursorPos];
                    uint8_t rp = bm_Replace(bd,Parameters.ButtonsMapping [bm_CursorPos]);
                    Parameters.ButtonsMapping [bm_CursorPos] = bd;

                    bm_editMode = 0;
                    bm_PrintValue (bm_CursorPos,1);
                    if (rp !=255) bm_PrintValue (rp,0);
                }
            }
			break;
		case KEY_DOWN_EVENT:
            if (!bm_editMode)
            {
                if (bm_CursorPos < 31)
                {
                    bm_CursorPos++;
                    bm_PrintValue (bm_CursorPos,1);
                    bm_PrintValue (bm_CursorPos-1,0);
                }
                else
                {
                    bm_CursorPos = 0;
                    bm_PrintValue (bm_CursorPos,1);
                    bm_PrintValue (31,0);
                }
            }
			break;
		case KEY_UP_EVENT:
            if (!bm_editMode)
            {
                if (bm_CursorPos > 0)
                {
                    bm_CursorPos--;
                    bm_PrintValue (bm_CursorPos,1);
                    bm_PrintValue (bm_CursorPos+1,0);
                }
                else
                {
                    bm_CursorPos = 31;
                    bm_PrintValue (bm_CursorPos,1);
                    bm_PrintValue (0,0);
                }
            }
			break;
		case KEY_ENTER_EVENT:
            bm_editMode++;
            bm_editMode &=1;
            bm_PrintValue (bm_CursorPos,1);
			break;
		case KEY_ESC_EVENT:
            CurrentFormProc = Menu_Form;
            CurrentFormProc(FORM_INIT_EVENT);
			break;
		case FORM_INIT_EVENT:
			//event = NO_EVENT;
            LCDSetRect(0, 0, 131, 131, FILL, BLACK);
            LCDPutStrROM("Buttons mapping", 121, 1, SMALL, GREEN, BLACK);
            bm_CursorPos = 0;
            bm_PrintValue (bm_CursorPos,1);
            for (int i=1;i<32;i++)
            {
                bm_PrintValue (i,0);
            }

/*
            for (int i=0;i<8;i++)
            {
                memset(str, 32, 22);//init result string
                for (int j=0;j<4;j++)
                {
                    itoa (Parameters.ButtonsMapping [i+j*8], stri, &len );
                    memcpy(str+(j*4 + 1),stri,len);
                }
                str [18] = 0;
                LCDPutStr (str, 111 - i*10, 1, SMALL, GREEN, BLACK);
            }
*/
			LCDFlushTxBuffer();
			break;
		 default:
			break;
	}
}
