#include "custom.h"

Eigen::VectorXd e_ctrl;
Eigen::VectorXd eq_pos;
Eigen::VectorXd eq_vel;
Eigen::VectorXd eq_acc;

void initialize_mycontroller(const mjModel *m, mjData *d)
{
    // initialize the controller

    if (m->nkey > 0)
    {
        mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);
    }
    // mju_copy(d->qpos,m->key_qpos + i*m->nq,m->nq)
    // control the joint

    e_ctrl.setZero(m->nu);
    eq_pos.setZero(m->nq);
    eq_vel.setZero(m->nv);
    eq_acc.setZero(m->nv);

    // std::cout << "act dim : " << m->nu << std::endl;
    // std::cout << "qpos dim : " << m->nq << std::endl;
    // std::cout << "qvel dim : " << m->nv << std::endl;
    // std::cout << "qacc dim : " << m->nv << std::endl;


    //get init joint status
    std::copy(d->qpos, d->qpos + m->nq, eq_pos.data());
    std::copy(d->qvel, d->qvel + m->nv, eq_vel.data());
    std::copy(d->qacc, d->qacc + m->nv, eq_acc.data());

    // std::cout << eq_pos.transpose() << std::endl;

    e_ctrl = eq_pos.segment(7, m->nu);
    printf("Controller initialized with key 0 / %d\n", m->nkey);

}

void mycontroller(const mjModel *m, mjData *d)
{
    //get current joint status
    std::copy(d->qpos, d->qpos + m->nq, eq_pos.data());
    std::copy(d->qvel, d->qvel + m->nv, eq_vel.data());
    std::copy(d->qacc, d->qacc + m->nv, eq_acc.data());



    // std::cout << "qpos : "<< eq_pos.transpose()<<std::endl;
    // ctrl_vec.setZero();
    e_ctrl(12) = sin(d->time);

    mju_copy(d->ctrl, e_ctrl.data(), m->nu);


}
