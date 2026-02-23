/**
 * @file nmea.c
 * @note Copyright (C) 2020 Richard Cochran <richardcochran@gmail.com>
 * @note SPDX-License-Identifier: GPL-2.0+
 */
#include <malloc.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ipc.h>
//#include <sys/shm.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
//#include "shared_data.h"

#include "nmea.h"
#include "print.h"

static int gps_count = 0;
static int glonass_count = 0;
static int galileo_count = 0;
static int total_visible_sats = 0;

static time_t last_gps_time = 0;
static time_t last_glonass_time = 0;
static time_t last_galileo_time = 0;

int temp_gps_count = 0;
int temp_glonass_count = 0;
int temp_galileo_count = 0;

pthread_mutex_t shm_mutex = PTHREAD_MUTEX_INITIALIZER;
//int shm_id;
//struct SharedData *shared_data;

/*void initialize_shared_memory() {
    // Create shared memory segment
    shm_id = shmget(SHM_KEY, sizeof(struct SharedData), IPC_CREAT | IPC_EXCL | 0666);
    if (shm_id < 0) {
        if (errno == EEXIST) {
            // Shared memory already exists
            shm_id = shmget(SHM_KEY, sizeof(struct SharedData), 0666);
            if (shm_id < 0) {
                perror("shmget");
                exit(1);
            }
        } else {
            perror("shmget");
            exit(1);
        }
    } else {
        pr_debug("Shared memory created.\n");
    }

    // Attach shared memory
    shared_data = (struct SharedData *)shmat(shm_id, NULL, 0);
    if (shared_data == (void *)-1) {
        perror("shmat");
        exit(1);
    }
}
*/

void update_shared_memory2() {
    //sem_lock(semid);
    shared_ptp_data->visible_satellites = total_visible_sats;
    if(total_visible_sats == 0){
     shared_ptp_data->ts_offset = 0;
     shared_ptp_data->ts_frequency = 0;
     shared_ptp_data->ts_servo_state = 0;
    }
    //sem_unlock(semid);
}

void reset_counts_if_needed() {
    time_t current_time = time(NULL);

    if (current_time - last_gps_time > 2) {
        gps_count = 0;
    }
    if (current_time - last_glonass_time > 2) {
        glonass_count = 0;
    }
    if (current_time - last_galileo_time > 2) {
        galileo_count = 0;
    }
    total_visible_sats = gps_count + glonass_count + galileo_count;
    //pr_debug("total_visible_sats = %d", total_visible_sats);
    update_shared_memory2();
}

void process_gsv_message(const char *sentence) {
    int total_msgs, msg_num, sat_count;
    int prn, elevation, azimuth, snr;
    int valid_snr_count = 0;
    time_t current_time = time(NULL);

    // Parse the GSV message
    if (sscanf(sentence + 6, "%d,%d,%d", &total_msgs, &msg_num, &sat_count) != 3) {
        return;
    }

    const char *ptr = sentence;
    for (int i = 0; i < 4; ++i) {
        if (sscanf(ptr, "%*[^,],%*[^,],%*[^,],%*[^,],%d,%d,%d,%d", &prn, &elevation, &azimuth, &snr) == 4) {
            if (snr > 25) {
                valid_snr_count++;
            }
        }
        ptr = strchr(ptr, ',');
        if (ptr == NULL) {
            break;
        }
        ptr++;
    }

    // Process GPS GSV messages
    if (strncmp(sentence, "GPGSV", 5) == 0) {
        if (msg_num == 1) {
            temp_gps_count = 0; // Reset the temp count for a new sequence
        }
        temp_gps_count += valid_snr_count;
        if (msg_num == total_msgs) {
            gps_count = temp_gps_count; // Update the total count when the last message is received
            last_gps_time = current_time;
        }
    }

    // Process GLONASS GSV messages
    else if (strncmp(sentence, "GLGSV", 5) == 0) {
        if (msg_num == 1) {
            temp_glonass_count = 0; // Reset the temp count for a new sequence
        }
        temp_glonass_count += valid_snr_count;
        if (msg_num == total_msgs) {
            glonass_count = temp_glonass_count; // Update the total count when the last message is received
            last_glonass_time = current_time;
        }
    }

    // Process Galileo GSV messages
    else if (strncmp(sentence, "GAGSV", 5) == 0) {
        if (msg_num == 1) {
            temp_galileo_count = 0; // Reset the temp count for a new sequence
        }
        temp_galileo_count += valid_snr_count;
        if (msg_num == total_msgs) {
            galileo_count = temp_galileo_count; // Update the total count when the last message is received
            last_galileo_time = current_time;
        }
    }

    reset_counts_if_needed();
}

#define NMEA_CHAR_MIN	' '
#define NMEA_CHAR_MAX	'~'
#define NMEA_MAX_LENGTH	256

enum nmea_state {
	NMEA_IDLE,
	NMEA_HAVE_DOLLAR,
	NMEA_HAVE_STARTG,
	NMEA_HAVE_STARTX,
	NMEA_HAVE_BODY,
	NMEA_HAVE_CSUMA,
	NMEA_HAVE_CSUM_MSB,
	NMEA_HAVE_CSUM_LSB,
	NMEA_HAVE_PENULTIMATE,
};

struct nmea_parser {
	char sentence[NMEA_MAX_LENGTH + 1];
	char payload_checksum[3];
	enum nmea_state state;
	uint8_t checksum;
	int offset;
};

static void nmea_reset(struct nmea_parser *np);

static void nmea_accumulate(struct nmea_parser *np, char c)
{
	if (c < NMEA_CHAR_MIN || c > NMEA_CHAR_MAX) {
		nmea_reset(np);
		return;
	}
	if (np->offset == NMEA_MAX_LENGTH) {
		nmea_reset(np);
	}
	np->sentence[np->offset++] = c;
	np->checksum ^= c;
}

static int nmea_parse_symbol(struct nmea_parser *np, char c)
{
	switch (np->state) {
	case NMEA_IDLE:
		if (c == '$') {
			np->state = NMEA_HAVE_DOLLAR;
		}
		break;
	case NMEA_HAVE_DOLLAR:
		if (c == 'G') {
			np->state = NMEA_HAVE_STARTG;
			nmea_accumulate(np, c);
		} else {
			nmea_reset(np);
		}
		break;
	case NMEA_HAVE_STARTG:
		np->state = NMEA_HAVE_STARTX;
		nmea_accumulate(np, c);
		break;
	case NMEA_HAVE_STARTX:
		np->state = NMEA_HAVE_BODY;
		nmea_accumulate(np, c);
		break;
	case NMEA_HAVE_BODY:
		if (c == '*') {
			np->state = NMEA_HAVE_CSUMA;
		} else {
			nmea_accumulate(np, c);
		}
		break;
	case NMEA_HAVE_CSUMA:
		np->state = NMEA_HAVE_CSUM_MSB;
		np->payload_checksum[0] = c;
		break;
	case NMEA_HAVE_CSUM_MSB:
		np->state = NMEA_HAVE_CSUM_LSB;
		np->payload_checksum[1] = c;
		break;
	case NMEA_HAVE_CSUM_LSB:
		if (c == '\n') {
			/*skip the CR*/
			return 0;
		}
		if (c == '\r') {
			np->state = NMEA_HAVE_PENULTIMATE;
		} else {
			nmea_reset(np);
		}
		break;
	case NMEA_HAVE_PENULTIMATE:
		if (c == '\n') {
			return 0;
		}
		nmea_reset(np);
		break;
	}
	return -1;
}

static void nmea_reset(struct nmea_parser *np)
{
	memset(np, 0, sizeof(*np));
}

static int nmea_scan_rmc(struct nmea_parser *np, struct nmea_rmc *result)
{
	int cnt, i, msec = 0;
	char *ptr, status;
	uint8_t checksum;
	struct tm tm = {0};

	//pr_debug("nmea sentence: %s", np->sentence);
	cnt = sscanf(np->payload_checksum, "%02hhx", &checksum);
	if (cnt != 1) {
		return -1;
	}
	if (checksum != np->checksum) {
		pr_err("checksum mismatch 0x%02hhx != 0x%02hhx on %s",
		       checksum, np->checksum, np->sentence);
		return -1;
	}
    // Check if the sentence is a GSV message
    if (strncmp(np->sentence, "GPGSV", 5) == 0 || strncmp(np->sentence, "GLGSV", 5) == 0 || strncmp(np->sentence, "GAGSV", 5) == 0) {
        process_gsv_message(np->sentence);
        return -1; // Return early to avoid double processing
    }
	//pr_debug("nmea sentence: %s", np->sentence);
	if (strncmp(&np->sentence[2],"RMC",3) !=0){
		//pr_debug("!RMC");
		return -1;
	}
    pr_debug("nmea sentence: %s", np->sentence);
	
	cnt = sscanf(np->sentence,
		     "G%*cRMC,%2d%2d%2d.%d,%c",
		     &tm.tm_hour, &tm.tm_min, &tm.tm_sec, &msec, &status);
	if (cnt != 5) {
		cnt = sscanf(np->sentence,
			     "G%*cRMC,%2d%2d%2d,%c",
			     &tm.tm_hour, &tm.tm_min, &tm.tm_sec, &status);
		if (cnt != 4) {
			return -1;
		}
		
	}
	ptr = np->sentence;
	for (i = 0; i < 9; i++) {
		ptr = strchr(ptr, ',');
		if (!ptr) {
			return -1;
		}
		ptr++;
	}
	cnt = sscanf(ptr, "%2d%2d%2d", &tm.tm_mday, &tm.tm_mon, &tm.tm_year);
	pr_debug("nmea time: %02d/%02d/%02d %02d:%02d:%02d %c", tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, status);
    if (cnt != 3) {
		return -1;
	}
	/* Convert an inserted leap second to ambiguous 23:59:59 */
	if (tm.tm_sec == 60)
		tm.tm_sec = 59;
	tm.tm_year += 100;
	tm.tm_mon--;
	tm.tm_isdst = 0;
	result->ts.tv_sec = mktime(&tm);
	result->ts.tv_nsec = msec * 1000000UL;
	result->fix_valid = status == 'A' ? true : false;
	return 0;
}

int nmea_parse(struct nmea_parser *np, const char *ptr, int buflen,
	       struct nmea_rmc *result, int *parsed)
{
	int count = 0;
	while (buflen) {
		if (!nmea_parse_symbol(np, *ptr)) {
			if (!nmea_scan_rmc(np, result)) {
				nmea_reset(np);
				*parsed = count + 1;
				return 0;
			}
			nmea_reset(np);
		}
		buflen--;
		count++;
		ptr++;
	}
	*parsed = count;
	return -1;
}

struct nmea_parser *nmea_parser_create(void)
{
	struct nmea_parser *np;
	np = malloc(sizeof(*np));
	if (!np) {
		return NULL;
	}
	nmea_reset(np);
	/* Ensure that mktime(3) returns a value in the UTC time scale. */
	setenv("TZ", "UTC", 1);
	return np;
}

void nmea_parser_destroy(struct nmea_parser *np)
{
	free(np);
}
