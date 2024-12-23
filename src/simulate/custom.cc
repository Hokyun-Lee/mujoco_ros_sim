#include "custom.h"

void initialize_mycontroller()
{
    // initialize the controller
    printf("Controller initialized\n");
}

void mycontroller(const mjModel *m, mjData *d)
{
    // control the joint
    printf("test control time : %f joint state : %f\n", d->time, d->qpos[0]);
    d->ctrl[0] = 1;
}
