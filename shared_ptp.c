#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/sem.h>
#include <unistd.h>
#include <signal.h>
#include "shared_ptp.h"

int ptp_shared_memory_id, ptp_semaphore_id;
struct SharedPtpData *shared_ptp_data;

void InitializePtpSemaphore(int *semaphore_id) {
    // Create semaphore
    *semaphore_id = semget(kPtpSemKey, 1, IPC_CREAT | 0666);
    if (*semaphore_id < 0) {
        perror("ptp semget");
        exit(1);
    }

    // Initialize semaphore if not already initialized
    if (semctl(*semaphore_id, 0, GETVAL) == 0) {
        semctl(*semaphore_id, 0, SETVAL, 1);
    }
}

void AcquirePtpSemaphoreLock(int semaphore_id) {
    struct sembuf sb = {0, -1, 0};
    if (semop(semaphore_id, &sb, 1) == -1) {
        perror("ptp semop lock");
        exit(1);
    }
}

void ReleasePtpSemaphoreLock(int semaphore_id) {
    struct sembuf sb = {0, 1, 0};
    if (semop(semaphore_id, &sb, 1) == -1) {
        perror("ptp semop unlock");
        exit(1);
    }
}

void InitializePtpSharedMemory(int *shared_memory_id, struct SharedPtpData **shared_data) {
    // Lock the semaphore to ensure exclusive access during initialization
    AcquirePtpSemaphoreLock(ptp_semaphore_id);
    printf("sizeof(struct SharedPtpData): %zu\n", sizeof(struct SharedPtpData));
    // Create shared memory segment
    *shared_memory_id = shmget(kPtpShmKey, sizeof(struct SharedPtpData), IPC_CREAT | IPC_EXCL | 0666);
    int created = 0;  // Flag to check if the memory was created by this process

    if (*shared_memory_id < 0) {
        if (errno == EEXIST) {
            // Shared memory already exists. Size must be <= existing size to succeed.
            // Request minimal size to obtain shmid regardless of existing segment size.
            *shared_memory_id = shmget(kPtpShmKey, 1, 0666);
            if (*shared_memory_id < 0) {
                perror("ptp shmget1");
                ReleasePtpSemaphoreLock(ptp_semaphore_id);  // Unlock semaphore before exiting
                exit(1);
            }
        } else {
            perror("ptp shmget2");
            ReleasePtpSemaphoreLock(ptp_semaphore_id);  // Unlock semaphore before exiting
            exit(1);
        }
    } else {
        printf("PTP shared memory created.\n");
        created = 1;  // Mark that this process created the shared memory
    }

    // Attach shared memory
    *shared_data = (struct SharedPtpData *)shmat(*shared_memory_id, NULL, 0);
    if (*shared_data == (void *)-1) {
        perror("ptp shmat");
        ReleasePtpSemaphoreLock(ptp_semaphore_id);  // Unlock semaphore before exiting
        exit(1);
    }

    // Validate segment size against our expected structure size.
    // If smaller, continue but warn; writers/readers must avoid overruns.
    {
        struct shmid_ds shminfo;
        if (shmctl(*shared_memory_id, IPC_STAT, &shminfo) == 0) {
            if ((size_t)shminfo.shm_segsz < sizeof(struct SharedPtpData)) {
                fprintf(stderr,
                        "ptp shared memory size %zu < expected %zu; using legacy layout\n",
                        (size_t)shminfo.shm_segsz, sizeof(struct SharedPtpData));
            }
        }
    }

    // Initialize shared memory to zero if it was created by this process
    if (created) {
        memset(*shared_data, 0, sizeof(struct SharedPtpData));
        (*shared_data)->version = PTP_VERSION_NUMBER;
        printf("PTP shared memory initialized to zero.\n");
    }

    // Unlock the semaphore after initialization is complete
    ReleasePtpSemaphoreLock(ptp_semaphore_id);
}

void CleanupPtpWithoutExit(int shared_memory_id, int semaphore_id, struct SharedPtpData *shared_data) {
    if (shared_data != NULL) {
        // Detach from shared memory
        if (shmdt(shared_data) == -1) {
            perror("ptp shmdt");
        }
    }

    if (shared_memory_id >= 0) {
        // Check if this is the last process using the shared memory
        struct shmid_ds shminfo;
        if (shmctl(shared_memory_id, IPC_STAT, &shminfo) == 0) {
            if (shminfo.shm_nattch == 0) {
                // No processes attached, remove the shared memory
                if (shmctl(shared_memory_id, IPC_RMID, NULL) == -1) {
                    perror("ptp shmctl IPC_RMID");
                } else {
                    printf("PTP shared memory removed.\n");
                }
            }
        }
    }
}

void CleanupPtp(int shared_memory_id, int semaphore_id, struct SharedPtpData *shared_data) {
    CleanupPtpWithoutExit(shared_memory_id, semaphore_id, shared_data);
    
    // Remove semaphore
    if (semaphore_id >= 0) {
        if (semctl(semaphore_id, 0, IPC_RMID) == -1) {
            perror("ptp semctl IPC_RMID");
        } else {
            printf("PTP semaphore removed.\n");
        }
    }
}

void SetupPtpSignalHandlers(void (*handler)(int)) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handler;
    sa.sa_flags = SA_RESTART;

    if (sigaction(SIGINT, &sa, NULL) == -1) {
        perror("ptp sigaction SIGINT");
    }
    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        perror("ptp sigaction SIGTERM");
    }
} 