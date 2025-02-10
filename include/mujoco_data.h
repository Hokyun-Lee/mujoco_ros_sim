#ifndef MUJOCO_DATA_H
#define MUJOCO_DATA_H

#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include <atomic>
using atomic_size_t = std::atomic<size_t>;

#define memory_order_relaxed std::memory_order_relaxed
#define memory_order_acquire std::memory_order_acquire
#define memory_order_release std::memory_order_release

#define atomic_store_explicit std::atomic_store_explicit
#define atomic_load_explicit std::atomic_load_explicit

#define MAX_DOF 50
#define RINGBUFFER_CAPACITY 8

typedef struct
{
    double sim_time;
    int mujoco_cnt;

    double joint_position[MAX_DOF];
    double joint_velocity[MAX_DOF];
    double joint_acc[MAX_DOF];

    double imu_orientation[4];
    double imu_angular_velocity[3];
    double imu_linear_acceleration[3];

} mujoco_status_container;

typedef struct
{
    double command_time;
    double target_torque[MAX_DOF];
} mujoco_command_container;

typedef struct
{
    atomic_size_t writeIndex;
    atomic_size_t readIndex;

    mujoco_status_container buffer[RINGBUFFER_CAPACITY];
} spsc_mujoco_joint_status;

typedef struct
{
    atomic_size_t writeIndex;
    atomic_size_t readIndex;

    mujoco_command_container buffer[RINGBUFFER_CAPACITY];
} spsc_mujoco_joint_command;

typedef struct
{
    spsc_mujoco_joint_status joint_status_buffer;
    spsc_mujoco_joint_command joint_command_buffer;
} mujoco_shm;

static inline void spsc_mujoco_joint_status_init(spsc_mujoco_joint_status *rb)
{
    atomic_store_explicit(&rb->writeIndex, 0, memory_order_relaxed);
    atomic_store_explicit(&rb->readIndex, 0, memory_order_relaxed);
}

static inline bool spsc_mujoco_joint_status_push(spsc_mujoco_joint_status *rb, const mujoco_status_container *data)
{
    size_t currentWrite = atomic_load_explicit(&rb->writeIndex, memory_order_relaxed);
    size_t nextWrite = (currentWrite + 1) & (RINGBUFFER_CAPACITY - 1);

    size_t currentRead = atomic_load_explicit(&rb->readIndex, memory_order_acquire);
    if (nextWrite == currentRead)
    {
        return false;
    }

    rb->buffer[currentWrite] = *data;

    atomic_store_explicit(&rb->writeIndex, nextWrite, memory_order_release);

    return true;
}

static inline bool spsc_mujoco_joint_status_pop(spsc_mujoco_joint_status *rb, mujoco_status_container *data)
{
    size_t currentRead = atomic_load_explicit(&rb->readIndex, memory_order_relaxed);
    size_t currentWrite = atomic_load_explicit(&rb->writeIndex, memory_order_acquire);

    if (currentRead == currentWrite)
    {
        return false;
    }

    *data = rb->buffer[currentRead];

    atomic_store_explicit(&rb->readIndex, (currentRead + 1) & (RINGBUFFER_CAPACITY - 1), memory_order_release);

    return true;
}

static inline void spsc_mujoco_joint_command_init(spsc_mujoco_joint_command *rb)
{
    atomic_store_explicit(&rb->writeIndex, 0, memory_order_relaxed);
    atomic_store_explicit(&rb->readIndex, 0, memory_order_relaxed);
}

static inline bool spsc_mujoco_joint_command_push(spsc_mujoco_joint_command *rb, const mujoco_command_container *data)
{
    size_t currentWrite = atomic_load_explicit(&rb->writeIndex, memory_order_relaxed);
    size_t nextWrite = (currentWrite + 1) & (RINGBUFFER_CAPACITY - 1);

    size_t currentRead = atomic_load_explicit(&rb->readIndex, memory_order_acquire);
    if (nextWrite == currentRead)
    {
        return false;
    }

    rb->buffer[currentWrite] = *data;

    atomic_store_explicit(&rb->writeIndex, nextWrite, memory_order_release);

    return true;
}

static inline bool spsc_mujoco_joint_command_pop(spsc_mujoco_joint_command *rb, mujoco_command_container *data)
{
    size_t currentRead = atomic_load_explicit(&rb->readIndex, memory_order_relaxed);
    size_t currentWrite = atomic_load_explicit(&rb->writeIndex, memory_order_acquire);

    if (currentRead == currentWrite)
    {
        return false;
    }

    *data = rb->buffer[currentRead];

    atomic_store_explicit(&rb->readIndex, (currentRead + 1) & (RINGBUFFER_CAPACITY - 1), memory_order_release);

    return true;
}

static inline mujoco_shm *get_mujoco_shm()
{
    key_t key = ftok("/tmp", 68);
    if (key == -1)
    {
        perror("ftok");
        exit(1);
    }

    int shmid = shmget(key, sizeof(mujoco_shm), 0666 | IPC_CREAT);
    if (shmid == -1)
    {
        perror("shmget");
        exit(1);
    }

    mujoco_shm *shm = (mujoco_shm *)shmat(shmid, NULL, 0);
    if (shm == (mujoco_shm *)-1)
    {
        perror("shmat");
        exit(1);
    }

    return shm;
}

static inline void detach_mujoco_shm(mujoco_shm *shm)
{
    if (shmdt(shm) == -1)
    {
        perror("shmdt");
        exit(1);
    }
}

#ifdef __cplusplus
#undef memory_order_relaxed
#undef memory_order_acquire
#undef memory_order_release

#undef atomic_store_explicit
#undef atomic_load_explicit

#endif

#endif