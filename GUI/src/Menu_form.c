#include "GUI.h"




//------------------------------------------------------------------------------------------------
static unsigned char SelectedMenu = 1;			// Start with first entry (apart from header)
// Define single menu entry
typedef const struct MenuStructure
{	const char *text;
	uint8_t submenu;
	uint8_t up;
	uint8_t down;
	uint8_t enter;
  	void ( *fp )( uint8_t event );
	// unsigned char value;
}MenuEntry;

// Menu strings
const char menu_000[] = " [Main Screen]";  	// 0
const char menu_001[] = "  Settings";  		// 1
const char menu_002[] = "  PID values";		// 2
const char menu_003[] = "  Options3";  		// 3
const char menu_004[] = "  ADC values"; 	// 4
const char menu_005[] = "  ADC calibration";// 5
const char menu_006[] = "  Options6";  		// 6
const char menu_007[] = "  Options7";  		// 7
const char menu_008[] = "  Trimmer";  		// 8
const char menu_009[] = "  Options9";	// 9
const char menu_010[] = "  return";  		// 10

const char menu_100[] = " [Header1]";  		// 11
const char menu_101[] = "  LCD parameters"; // 12
const char menu_102[] = "  Option102"; 	    // 13
const char menu_103[] = "  Ch.params values"; 	// 14 calibration params form
const char menu_104[] = "  Butons mapping";  	// 15
const char menu_105[] = "  Option105";  	// 16
const char menu_106[] = "  Reset parameters";  	// 17
const char menu_107[] = "  Save parameters";	// 18
const char menu_108[] = "  return";  		// 19

const char menu_200[] = " [Header2]";  		// 20
const char menu_201[] = "  Option201";  	// 21
const char menu_202[] = "  Option202";  	// 22
const char menu_203[] = "  Option203";  	// 23
const char menu_204[] = "  Option204";  	// 24
const char menu_205[] = "  Option205";  	// 25
const char menu_206[] = "  Option206";  	// 26
const char menu_207[] = "  Option207";  	// 27
const char menu_208[] = "  return";  		// 28

const char menu_300[] = " [Header3]";  		// 29
const char menu_301[] = "  Option301";  	// 30
const char menu_302[] = "  Option302";  	// 31
const char menu_303[] = "  Option303";  	// 32
const char menu_304[] = "  Option304";  	// 33
const char menu_305[] = "  return";  		// 34

// Array of entries
MenuEntry menu[] =
{
	{menu_000,  1,  0,  0,  0,  0}, 		// 0
	{menu_001,  1,  1,  2, 12,  0},
	{menu_002,  1,  1,  3, 21,  0},
	{menu_003,  1,  2,  4, 30,  0},
	{menu_004,  1,  3,  5,  4,  ADC_form},
	{menu_005,  1,  4,  6,  5,  ADC_calibration_form},
	{menu_006,  1,  5,  7,  6,  0},
	{menu_007,  1,  6,  8,  7,  0},
	{menu_008,  1,  7,  9,  8,  Trimmer_form},
	{menu_009,  1,  8, 10,  9,  0},
	{menu_010,  1,  9, 10, 1,  0}, 		// 10

	{menu_100,  2,  0,  0,  0,  0},			// 11
	{menu_101,  2, 12, 13, 12,  LCD_parameters_form},
	{menu_102,  2, 12, 14, 13,  0},
	{menu_103,  2, 13, 15, 14,  Calibration_params_form},
	{menu_104,  2, 14, 16, 15,  Buttons_Mapping_form},
	{menu_105,  2, 15, 17, 16,  0},
	{menu_106,  2, 16, 18, 17,  Reset_parameters_form},
	{menu_107,  2, 17, 19, 18,  Save_parameters_form},
	{menu_108,  2, 18, 19,  1,  0},			// 19

	{menu_200,  3,  0,  0,  0,  0},			// 20
	{menu_201,  3, 21, 22, 21,  0},
	{menu_202,  3, 21, 23, 22,  0},
	{menu_203,  3, 22, 24, 23,  0},
	{menu_204,  3, 23, 25, 24,  0},
	{menu_205,  3, 24, 26, 25,  0},
	{menu_206,  3, 25, 27, 26,  0},
	{menu_207,  3, 26, 28, 27,  0},
	{menu_208,  3, 27, 28,  2,  0},			// 28

	{menu_300,  4,  0,  0,  0,  0},			// 29
	{menu_301,  4, 30, 31, 30,  0},
	{menu_302,  4, 30, 32, 31,  0},
	{menu_303,  4, 31, 33, 32,  0},
	{menu_304,  4, 32, 34, 33,  0},
	{menu_305,  4, 33, 34,  3,  0}			// 34
};

const uint8_t LastMenuItem = 34;
//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------
void DrawMenu (uint8_t f, uint8_t t)
{
	uint8_t pos = 0;

	LCDSetRect(0, 0, MENU_X_POS+8, 131, FILL, BLACK);//clear screen
	while (f <= t)
	{
		if (f==SelectedMenu) LCDPutStrROM(menu[f].text, MENU_X_POS-8*pos, MENU_Y_POS, SMALL, BLACK, RED);
		else LCDPutStrROM(menu[f].text, MENU_X_POS-8*pos, MENU_Y_POS, SMALL, WHITE, BLACK);
		f++;
		pos++;
	}
	LCDFlushTxBuffer();
}
//------------------------------------------------------------------------------------------------
void CalcFromTill (uint8_t * from, uint8_t * till)
{
	uint8_t submenu = menu[SelectedMenu].submenu;
	*from = SelectedMenu;
	while ((menu[*from-1].submenu == submenu) && (--(*from) > 0));
	while (SelectedMenu - *from > MAX_MENU_ROWS) *from += MAX_MENU_ROWS;

	while ((menu[*till+1].submenu == submenu) && (*till < *from + MAX_MENU_ROWS) && (++(*till)<LastMenuItem));
}

//------------------------------------------------------------------------------------------------
void Menu_Form (uint8_t event)
{
	uint8_t from = SelectedMenu;
	uint8_t till = SelectedMenu;


	uint8_t LastSelected = SelectedMenu;

	switch (event)
	{
		case NO_EVENT:
			break;
		case FORM_INIT_EVENT:
			//event = NO_EVENT;
			LCDSetRect(0, 0, 131, 131, FILL, BLACK);


			CalcFromTill (&from, &till);
			DrawMenu (from,till);
			/*
			while (from <= till)
			{
				if (from==SelectedMenu) LCDPutStrROM(menu[from].text, MENU_X_POS-8*pos, MENU_Y_POS, SMALL, BLACK, RED);
				else LCDPutStrROM(menu[from].text, MENU_X_POS-8*pos, MENU_Y_POS, SMALL, WHITE, BLACK);
				from++;
				pos++;
			}
			*/
			break;
		case KEY_DOWN_EVENT:
			SelectedMenu = menu[SelectedMenu].down;

			from = SelectedMenu;
			CalcFromTill (&from, &till);

			if (SelectedMenu!=from)
			{
				LCDPutStrROM(menu[LastSelected].text, MENU_X_POS-8*(LastSelected-from), MENU_Y_POS, SMALL, WHITE, BLACK);
				LCDPutStrROM(menu[SelectedMenu].text, MENU_X_POS-8*(SelectedMenu-from), MENU_Y_POS, SMALL, BLACK, RED);
				LCDFlushTxBuffer();
			}
			else
			{
				DrawMenu (from,till);//Redraw
			}
			//vTaskDelay( FORM_BUT_DELAY / portTICK_RATE_MS );
			break;
		case KEY_UP_EVENT:
			SelectedMenu = menu[SelectedMenu].up;
			CalcFromTill (&from, &till);
			if (SelectedMenu != till)
			{
				LCDPutStrROM(menu[LastSelected].text, MENU_X_POS-8*(LastSelected-from), MENU_Y_POS, SMALL, WHITE, BLACK);
				LCDPutStrROM(menu[SelectedMenu].text, MENU_X_POS-8*(SelectedMenu-from), MENU_Y_POS, SMALL, BLACK, RED);
				LCDFlushTxBuffer();
			}
			else
			{
				DrawMenu (from,till);//Redraw
			}
			//vTaskDelay( FORM_BUT_DELAY / portTICK_RATE_MS );
			break;
		case KEY_ENTER_EVENT: // KEY_ENTER_EVENT:

			if (menu[SelectedMenu].fp != NULL)
			{
				SelectedMenu = menu[SelectedMenu].enter;
				CurrentFormProc = menu[SelectedMenu].fp;
				CurrentFormProc(FORM_INIT_EVENT);
			}
			else
			{
				SelectedMenu = menu[SelectedMenu].enter;
				from=till=SelectedMenu;
				CalcFromTill (&from, &till);
				DrawMenu (from,till);//Redraw
				//vTaskDelay( FORM_BUT_DELAY / portTICK_RATE_MS );
			}
			break;
		case KEY_ESC_EVENT:
                CalcFromTill (&from, &till);
			    SelectedMenu = menu[till].enter;
				from=till=SelectedMenu;
				CalcFromTill (&from, &till);
				DrawMenu (from,till);//Redraw
            break;
		 default:
			break;
	}


}
