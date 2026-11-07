//
// Created by 35264 on 2026/11/7.
//

#include "include/my_tasks.h"



[[ noreturn ]] void robot::tasks::controller_task(void* pvParameter)
{

    while(true)
    {
        auto sharing_contents = static_cast<robot::sharing_contents*>(pvParameter);

    }
}