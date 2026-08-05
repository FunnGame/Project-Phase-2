#include "app_main.h"

int main(void)
{
    Status_t status;

    status = App_Init();

    if (status != STATUS_OK)
    {
        while (1)
        {
        }
    }

    while (1)
    {
        App_Run();
    }
}
