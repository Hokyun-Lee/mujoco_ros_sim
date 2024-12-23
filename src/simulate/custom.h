#ifndef MYCONTROLLER_H
#define MYCONTROLLER_H

#include <mujoco/mjdata.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>

void initialize_mycontroller();
void mycontroller(const mjModel* m, mjData* d);

#endif