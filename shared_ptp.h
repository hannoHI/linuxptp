#ifndef SHARED_PTP_H_
#define SHARED_PTP_H_

#define kPtpShmSize 512
#define kPtpShmKey 0x5054  // PT in hex
#define kPtpSemKey 0x504C  // PL in hex

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <stdint.h>
#include <time.h>

// Version number for the PTP shared data structure
#define PTP_VERSION_NUMBER 1

#ifdef __cplusplus
extern "C" {
#endif

struct SharedPtpData {
    int version;
    
    // PTP Core Data (written by PTP, read by ptp_mon)
    int master_offset;           // Master offset in nanoseconds
    float mean_path_delay;       // Mean path delay in nanoseconds
    int utc_offset;             // Current UTC offset in seconds
    char mode[9];               // Port state (SLAVE, MASTER, LISTENING, etc.)
    
    // PTP Hardware Clock Data (written by PTP, read by ptp_mon)
    float drift;                // Frequency drift in parts per billion (ppb)
    time_t ptp_seconds;         // Current PTP time (seconds)
    time_t ptp_nanoseconds;     // Current PTP time (nanoseconds)
    
    // PTP Status Flags (written by PTP, read by ptp_mon)
    // int ptp_is_up;              // PTP daemon is running
    // int ptp_is_synced;          // PTP is synchronized
    // int ptp_domain;             // PTP domain number
    
    // Timestamp (written by PTP when data is updated)
    // time_t last_ptp_update;     // Last PTP data update timestamp

    // GPS info
    int visible_satellites;

    int64_t ts_offset; // ts2phc offset
    double ts_frequency; // ts2phc freq
    int ts_servo_state; // ts2phc servo state
    
    
    // Reserved for future expansion
    char reserved[64];
};

// Function declarations
void InitializePtpSemaphore(int *semaphore_id);
void AcquirePtpSemaphoreLock(int semaphore_id);
void ReleasePtpSemaphoreLock(int semaphore_id);
void InitializePtpSharedMemory(int *shared_memory_id, struct SharedPtpData **shared_data);
void CleanupPtpWithoutExit(int shared_memory_id, int semaphore_id, struct SharedPtpData *shared_data);
void CleanupPtp(int shared_memory_id, int semaphore_id, struct SharedPtpData *shared_data);
void SetupPtpSignalHandlers(void (*handler)(int));

extern int ptp_shared_memory_id, ptp_semaphore_id;
extern struct SharedPtpData *shared_ptp_data;

#ifdef __cplusplus
}
#endif

#endif // SHARED_PTP_H_ 