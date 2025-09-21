#include "msp430.h"
#include "stdio.h"
#include "Junior_Design.h"

static unsigned long int d1;
int d_1, d_10;
char Data2[32];

int main(void)
{
    Init_HW();
    LCD_Initialize();

    while(1)
    {
        LCD_CLRscreen();

        Ultrasonic_Trigger();

        Delay_Timer(DELAYT_100ms);

        d1 = Ultrasonic_Echo_Read();

        int distance_mm = (int)((float)d1 * 3.4 /  2.0);

        d_1 = (int)((float)d1 * 0.066929);

        d_10 = (int)((float)d1 * 0.66929) - d_1 * 10;

        LCD_Cursor_Position(0x40);

        LCD_Cursor_On_No_Blink();

        sprintf(Data2, "%dmm %d.%din", distance_mm, d_1, d_10);

        LCD_String(Data2);

        DelayTimer(DELAY_500ms);
    }
}
