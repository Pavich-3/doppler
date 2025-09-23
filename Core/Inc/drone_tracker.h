#ifndef INC_DRONE_TRACKER_H_
#define INC_DRONE_TRACKER_H_

#include "main.h"
#include "arm_math.h"
#include "stdbool.h"

// --- Configuration Constants ---

#define AUDIO_BUFFER_SIZE           2048  // Size of one half of the DMA circular buffer (in 32-bit samples).
#define ANALYSIS_SIZE               1024  // Number of samples to use for FFT and correlation analysis.
#define FFT_SIZE                    1024  // Size of the Fast Fourier Transform.
#define NUM_STAGES                  4     // Number of stages for the Biquad IIR filter.
#define ANGLE_SMOOTHING_WINDOW_SIZE 10    // Number of recent angle values to average for smoothing.

// --- Type Definitions ---

/**
 * @brief Defines the operational states of the drone tracker.
 */
typedef enum {
	STATE_LISTENING, // The system is waiting to detect a drone signature.
	STATE_TRACKING   // The system has detected a drone and is actively calculating its position.
} TrackerState_t;


/**
 * @brief Main structure holding all data and state for the drone tracker application.
 */
typedef struct {
	// --- State Machine ---
	TrackerState_t current_state;       // The current operational state of the tracker.
	uint32_t detection_counter;         // Counts consecutive frames with a potential drone signature.
	uint32_t lost_counter;              // Counts consecutive frames where the drone signature is lost.

    // --- Raw I2S DMA Buffers ---
    // These are large circular buffers filled by DMA transfers from the I2S peripherals.
    // The size is doubled to support double-buffering (Half/Full transfer complete interrupts).
    int32_t i2s1_rx_buffer[AUDIO_BUFFER_SIZE * 2]; // For microphones 1 & 2 (e.g., Azimuth).
    int32_t i2s4_rx_buffer[AUDIO_BUFFER_SIZE * 2]; // For microphones 3 & 4 (e.g., Elevation).

    // --- Signal Processing Buffers ---
    float32_t mic_samples[4][ANALYSIS_SIZE];         // De-interleaved, float-converted samples for each of the 4 microphones.
    float32_t processing_buffer[ANALYSIS_SIZE];      // A general-purpose buffer for in-place operations like filtering and FFT.
    float32_t fft_output[FFT_SIZE];                  // Output of the real FFT (complex numbers).
    float32_t magnitudes[FFT_SIZE / 2];              // Magnitude of each frequency bin from the FFT.
    float32_t correlation_output[2 * ANALYSIS_SIZE - 1]; // Output of the cross-correlation function.
    float32_t filter_state[NUM_STAGES * 4];          // State buffer required by the ARM Biquad filter function.

    // --- ARM DSP Library Instances ---
    arm_biquad_casd_df1_inst_f32 filter_instance; // Instance for the Biquad IIR filter.
    arm_rfft_fast_instance_f32 fft_instance;         // Instance for the real Fast Fourier Transform.

    // --- DMA Synchronization Flags & Pointers (must be volatile) ---
    volatile bool i2s1_data_ready;      // Flag set by ISR when new data is available from I2S1.
    volatile bool i2s4_data_ready;      // Flag set by ISR when new data is available from I2S4.
    volatile int32_t* i2s1_active_ptr;  // Pointer to the half of the DMA buffer that is ready for processing (I2S1).
    volatile int32_t* i2s4_active_ptr;  // Pointer to the half of the DMA buffer that is ready for processing (I2S4).

    // --- Angle Calculation Results ---
    float32_t azimuth;                  // Raw calculated azimuth angle in degrees.
    float32_t elevation;                // Raw calculated elevation angle in degrees.

    // --- Angle Smoothing Buffers ---
    float32_t azimuth_history[ANGLE_SMOOTHING_WINDOW_SIZE];     // Circular buffer for storing recent azimuth values.
    float32_t elevation_history[ANGLE_SMOOTHING_WINDOW_SIZE];   // Circular buffer for storing recent elevation values.
    uint32_t azimuth_history_index;     // Current index in the azimuth history buffer.
    uint32_t elevation_history_index;   // Current index in the elevation history buffer.

    // --- Final Smoothed Angles ---
    float32_t smoothed_azimuth;         // Smoothed azimuth angle, ready for servo control.
    float32_t smoothed_elevation;       // Smoothed elevation angle, ready for servo control.

} DroneTracker_t;

// --- Function Prototypes ---

/**
 * @brief Initializes the DroneTracker structure and its components (DSP instances, state).
 * @param tracker Pointer to the DroneTracker_t instance.
 */
void DroneTracker_Init(DroneTracker_t* tracker);

/**
 * @brief Main processing function, called repeatedly in the main loop.
 * It handles state transitions, signal processing, and angle calculation.
 * @param tracker Pointer to the DroneTracker_t instance.
 */
void DroneTracker_Process(DroneTracker_t* tracker);


#endif /* INC_DRONE_TRACKER_H_ */
