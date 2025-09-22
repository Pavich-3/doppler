#ifndef INC_DRONE_TRACKER_H_
#define INC_DRONE_TRACKER_H_

#include "main.h"
#include "arm_math.h"
#include "stdbool.h"

#define AUDIO_BUFFER_SIZE 2048
#define ANALYSIS_SIZE 1024
#define FFT_SIZE 1024
#define NUM_STAGES 4

typedef enum {
	STATE_LISTENING,
	STATE_TRACKING
} TrackerState_t;

typedef struct {
	TrackerState_t current_state;
	uint32_t detection_counter;
	uint32_t lost_counter;

    int32_t i2s1_rx_buffer[AUDIO_BUFFER_SIZE * 2];
    int32_t i2s4_rx_buffer[AUDIO_BUFFER_SIZE * 2];

    float32_t mic_samples[4][ANALYSIS_SIZE];
    float32_t processing_buffer[ANALYSIS_SIZE];
    float32_t fft_output[FFT_SIZE];
    float32_t magnitudes[FFT_SIZE / 2];
    float32_t correlation_output[2 * ANALYSIS_SIZE - 1];
    float32_t filter_state[NUM_STAGES * 4];

    arm_biquad_casd_df1_inst_f32 filter_instance;
    arm_rfft_fast_instance_f32 fft_instance;

    volatile bool i2s1_data_ready;
    volatile bool i2s4_data_ready;
    volatile int32_t* i2s1_active_ptr;
    volatile int32_t* i2s4_active_ptr;

    float32_t azimuth;
    float32_t elevation;

} DroneTracker_t;

void DroneTracker_Init(DroneTracker_t* tracker);
void DroneTracker_Process(DroneTracker_t* tracker);


#endif
