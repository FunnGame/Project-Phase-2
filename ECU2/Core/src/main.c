#include "control_app.h"

int main(void) {
    ControlApp_Init();
    while (1) {
        ControlApp_Update();
    }

    return 0;
}
