/**
 * @file drone_tracker.c
 * @brief Core logic implementation for the acoustic drone tracker.
 *
 * This file contains the state machine, signal processing pipeline, and algorithms
 * for detecting a drone, calculating its direction using Time Difference of Arrival (TDOA),
 * and controlling servos to track it.
 */

#include "drone_tracker.h"
#include <math.h> // Required for asinf() and M_PI

// External handle for the timer used for PWM servo control.
extern TIM_HandleTypeDef timerHandle;

// --- Algorithm Constants ---

// Multiplier for drone detection. A drone is detected if the max FFT peak is
// this many times greater than the average energy of the signal.
#define DRONE_THRESHOLD 15.0f

// --- Physical and System Constants ---

// Audio sampling rate in Hz. Must match the I2S peripheral configuration.
static const float32_t SAMPLE_RATE = 32000.0f;
// The physical distance between a pair of microphones in meters.
static const float32_t MIC_DISTANCE = 0.15f;
// The speed of sound in meters per second.
static const float32_t SOUND_SPEED = 343.0f;


// --- Servo Control Constants ---

// The operational range of the servos in degrees.
const float32_t input_angle_min = -60.0f;
const float32_t input_angle_max = 60.0f;

// The PWM pulse widths in microseconds corresponding to the servo's min, center, and max positions.
const uint32_t pulse_min = 750;
const uint32_t pulse_center = 1500;
const uint32_t pulse_max = 2250;


// --- Filter Definition ---

// Pre-calculated coefficients for the Biquad IIR filter.
// Each stage has 5 coefficients: b0, b1, b2, a1, a2.
static float32_t filter_coeffs[NUM_STAGES * 5] = {
    0.01733589f, 0.03467177f, 0.01733589f, 0.72652296f, -0.17059273f,
    1.00000000f, 2.00000000f, 1.00000000f, 0.87825900f, -0.55492900f,
    1.00000000f, -2.00000000f, 1.00000000f, 1.88500209f, -0.88900017f,
    1.00000000f, -2.00000000f, 1.00000000f, 1.95595489f, -0.95944615f,
};


// --- Static Function Prototypes ---

static void deinterleave_and_convert(volatile int32_t* dma_buffer, float32_t* left_channel_out, float32_t* right_channel_out, uint32_t num_samples);
static float32_t calculate_tdoa_angle(DroneTracker_t* tracker, float32_t* micA_samples, float32_t* micB_samples);
static float32_t smooth_angle(float32_t new_angle, float32_t* history_buffer, uint32_t* history_index);
static void set_servo_position(uint32_t channel, float32_t angle);


/**
 * @brief Initializes the DroneTracker structure.
 */
void DroneTracker_Init(DroneTracker_t* tracker)
{
	// Start in the listening state.
	tracker->current_state = STATE_LISTENING;
	tracker->detection_counter = 0;
	tracker->lost_counter = 0;

    // Initialize ARM DSP library instances.
    arm_biquad_cascade_df1_init_f32(&tracker->filter_instance, NUM_STAGES, filter_coeffs, tracker->filter_state);
    arm_rfft_fast_init_f32(&tracker->fft_instance, FFT_SIZE);
}

/**
 * @brief Main processing loop for the drone tracker.
 */
void DroneTracker_Process(DroneTracker_t* tracker)
{
    // Wait until new data is ready from the primary microphone pair.
    // The second pair (I2S4) is only processed when tracking, so we don't check its flag here.
    if (!tracker->i2s1_data_ready) return;

    // --- Buffer Management ---
    // Copy the active DMA buffer pointer to a local variable for safe access.
    volatile int32_t* local_i2s1_ptr = tracker->i2s1_active_ptr;
    // Reset the data ready flag to allow the ISR to set it again for the next block.
    tracker->i2s1_data_ready = false;


    // --- Drone Detection Pipeline ---
    // 1. Unpack the stereo audio data from the DMA buffer into two separate float arrays.
    // NOTE: Swapped the order of deinterleave and copy from the original code for correctness.
    deinterleave_and_convert(local_i2s1_ptr, tracker->mic_samples[0], tracker->mic_samples[1], ANALYSIS_SIZE);

    // 2. Copy one channel to a processing buffer for in-place operations.
    arm_copy_f32(tracker->mic_samples[0], tracker->processing_buffer, ANALYSIS_SIZE);

    // 3. Apply the Biquad filter to the signal.
    arm_biquad_cascade_df1_f32(&tracker->filter_instance, tracker->processing_buffer, tracker->processing_buffer, ANALYSIS_SIZE);

    // 4. Perform a Fast Fourier Transform (FFT).
    arm_rfft_fast_f32(&tracker->fft_instance, tracker->processing_buffer, tracker->fft_output, 0);

    // 5. Calculate the magnitude of each frequency bin.
    arm_cmplx_mag_f32(tracker->fft_output, tracker->magnitudes, FFT_SIZE / 2);

    // 6. Calculate the mean energy and find the maximum peak in the spectrum.
    float32_t max_peak, mean_energy;
    uint32_t max_peak_index;
    arm_mean_f32(tracker->magnitudes, FFT_SIZE / 2, &mean_energy);
    arm_max_f32(tracker->magnitudes, FFT_SIZE / 2, &max_peak, &max_peak_index);

    // 7. Determine if a drone is present based on the peak-to-mean ratio.
    bool is_drone_signature_present = (max_peak > mean_energy * DRONE_THRESHOLD);


    // --- State Machine ---
    switch (tracker->current_state)
    {
        case STATE_LISTENING:
            if (is_drone_signature_present)
            {
                // Increment counter to debounce detection.
                tracker->detection_counter++;
                if (tracker->detection_counter >= 3) // Require 3 consecutive detections
                {
                    // Transition to TRACKING state.
                    tracker->current_state = STATE_TRACKING;
                    tracker->lost_counter = 0; // Reset lost counter.
                }
            }
            else
                // Reset counter if signature is not present.
                tracker->detection_counter = 0;
            break;

        case STATE_TRACKING:
            if (is_drone_signature_present)
            {
                // Reset the lost counter since we still detect the drone.
                tracker->lost_counter = 0;

                // Check if the secondary microphone pair data is ready.
                if (tracker->i2s4_data_ready)
                {
                    // Get a local pointer and clear the flag.
                    volatile int32_t* local_i2s4_ptr = tracker->i2s4_active_ptr;
                    tracker->i2s4_data_ready = false;

                    // Process the secondary mic pair only when needed (saves CPU).
                    deinterleave_and_convert(local_i2s4_ptr, tracker->mic_samples[2], tracker->mic_samples[3], ANALYSIS_SIZE);

                    // Calculate azimuth (horizontal) and elevation (vertical) angles.
                    tracker->azimuth = calculate_tdoa_angle(tracker, tracker->mic_samples[0], tracker->mic_samples[1]);
                    tracker->elevation = calculate_tdoa_angle(tracker, tracker->mic_samples[2], tracker->mic_samples[3]);

                    // Apply a moving average filter to stabilize the servo output.
                    tracker->smoothed_azimuth = smooth_angle(tracker->azimuth, tracker->azimuth_history, &tracker->azimuth_history_index);
                    tracker->smoothed_elevation = smooth_angle(tracker->elevation, tracker->elevation_history, &tracker->elevation_history_index);

                    // Update the servo positions.
                    set_servo_position(TIM_CHANNEL_2, tracker->smoothed_azimuth);
                    set_servo_position(TIM_CHANNEL_3, tracker->smoothed_elevation);
                }
            }
            else
            {
                // Increment lost counter if signature is missing.
                tracker->lost_counter++;
                if (tracker->lost_counter >= 10) // Require 10 consecutive misses
                {
                    // Transition back to LISTENING state.
                    tracker->current_state = STATE_LISTENING;
                    tracker->detection_counter = 0; // Reset detection counter.
                }
            }
            break;
    }
}

/**
 * @brief De-interleaves a stereo I2S DMA buffer and converts samples to float.
 */
static void deinterleave_and_convert(volatile int32_t* dma_buffer, float32_t* left_channel_out, float32_t* right_channel_out, uint32_t num_samples)
{
    // The I2S DMA buffer contains interleaved left and right channel samples.
    for (uint32_t i = 0; i < num_samples; i++)
    {
        // Sample format might need adjustment based on I2S configuration (e.g., bit shifting for 24-bit data).
        right_channel_out[i] = (float32_t)dma_buffer[2 * i];
        left_channel_out[i]  = (float32_t)dma_buffer[2 * i + 1];
    }
}

/**
 * @brief Calculates the angle of arrival of a sound source using TDOA.
 */
static float32_t calculate_tdoa_angle(DroneTracker_t* tracker, float32_t* micA_samples, float32_t* micB_samples)
{
    // 1. Perform cross-correlation to find the time delay between the two microphone signals.
    arm_correlate_f32(micA_samples, ANALYSIS_SIZE, micB_samples, ANALYSIS_SIZE, tracker->correlation_output);

    // 2. Find the index of the maximum value in the correlation result.
    float32_t max_corr_value;
    uint32_t  max_corr_index;
    arm_max_f32(tracker->correlation_output, (2 * ANALYSIS_SIZE - 1), &max_corr_value, &max_corr_index);

    // 3. Convert the index to a sample delay. The center of the correlation output corresponds to zero delay.
    int32_t k_max = max_corr_index - (ANALYSIS_SIZE - 1);

    // 4. Convert sample delay to time delay in seconds.
    float32_t delta_t = (float32_t)k_max / SAMPLE_RATE;

    // 5. Calculate the sine of the angle using the TDOA formula: sin(theta) = (delta_t * speed_of_sound) / mic_distance.
    float32_t sin_theta = (delta_t * SOUND_SPEED) / MIC_DISTANCE;

    // 6. Clamp the value to the valid range [-1.0, 1.0] to prevent math errors in asinf().
    if (sin_theta > 1.0f)  sin_theta = 1.0f;
    if (sin_theta < -1.0f) sin_theta = -1.0f;

    // 7. Calculate the angle in radians and then convert to degrees.
    float32_t angle_rad = asinf(sin_theta);
    float32_t angle_deg = angle_rad * (180.0f / M_PI);

    return angle_deg;
}

/**
 * @brief Smooths an angle measurement using a simple moving average filter.
 */
static float32_t smooth_angle(float32_t new_angle, float32_t* history_buffer, uint32_t* history_index)
{
	// Add the new angle to the circular history buffer.
	history_buffer[*history_index] = new_angle;

	// Increment the index, wrapping around if necessary.
	*history_index = (*history_index + 1) % ANGLE_SMOOTHING_WINDOW_SIZE;

	// Calculate the sum of all values in the history buffer.
	float32_t sum = 0.0f;
	for (int i = 0; i < ANGLE_SMOOTHING_WINDOW_SIZE; i++)
		sum += history_buffer[i];

	// Return the average.
	return sum / ANGLE_SMOOTHING_WINDOW_SIZE;
}

/**
 * @brief Sets a servo's position by converting an angle to a PWM pulse width.
 */
static void set_servo_position(uint32_t channel, float32_t angle)
{
	// 1. Constrain the angle to the servo's physical limits.
	if (angle < input_angle_min) angle = input_angle_min;
	if (angle > input_angle_max) angle = input_angle_max;

	// 2. Linearly map the angle from its input range [-60, 60] to the PWM pulse range [750, 2250].
    float32_t mapped_value = (angle - input_angle_min) * (pulse_max - pulse_min) / (input_angle_max - input_angle_min) + pulse_min;

	// 3. Update the timer's compare register to set the new PWM duty cycle.
	__HAL_TIM_SET_COMPARE(&timerHandle, channel, (uint32_t)mapped_value);
}
