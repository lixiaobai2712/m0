#include "ti_msp_dl_config.h"
#include "car_app.h"

int main(void)
{
       SYSCFG_DL_init();
    CarApp_Init();

    while (1) {
        CarApp_RunCycle();
    }
}
