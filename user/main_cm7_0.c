#include "zf_common_headfile.h"
#include "application/camera_helper/app_camera_helper.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    app_camera_helper_init();

    while(true)
    {
        app_camera_helper_tick();
    }
}
