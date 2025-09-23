#include "drone_tracker.h"

extern TIM_HandleTypeDef timerHandle;

#define DRONE_THRESHOLD 15.0f

static const float32_t SAMPLE_RATE = 32000.0f;
static const float32_t MIC_DISTANCE = 0.15f;
static const float32_t SOUND_SPEED = 343.0f;

const float32_t input_angle_min = -60.0f;
const float32_t input_angle_max = 60.0f;

const uint32_t pulse_min = 750;
const uint32_t pulse_center = 1500;
const uint32_t pulse_max = 2250;

static float32_t filter_coeffs[NUM_STAGES * 5] = {
    0.01733589f, 0.03467177f, 0.01733589f, 0.72652296f, -0.17059273f,
    1.00000000f, 2.00000000f, 1.00000000f, 0.87825900f, -0.55492900f,
    1.00000000f, -2.00000000f, 1.00000000f, 1.88500209f, -0.88900017f,
    1.00000000f, -2.00000000f, 1.00000000f, 1.95595489f, -0.95944615f,
};

static void deinterleave_and_convert(volatile int32_t* dma_buffer, float32_t* left_channel_out, float32_t* right_channel_out, uint32_t num_samples);
static float32_t calculate_tdoa_angle(DroneTracker_t* tracker, float32_t* micA_samples, float32_t* micB_samples);
static float32_t smooth_angle(float32_t new_angle, float32_t* history_buffer, uint32_t* history_index);
static void set_servo_position(uint32_t channel, float32_t angle);

void DroneTracker_Init(DroneTracker_t* tracker)
{
	tracker->current_state = STATE_LISTENING;
	tracker->detection_counter = 0;
	tracker->lost_counter = 0;

    arm_biquad_cascade_df1_init_f32(&tracker->filter_instance, NUM_STAGES, filter_coeffs, tracker->filter_state);
    arm_rfft_fast_init_f32(&tracker->fft_instance, FFT_SIZE);
}

void DroneTracker_Process(DroneTracker_t* tracker)
{
    if (!tracker->i2s1_data_ready || !tracker->i2s4_data_ready) return;

    volatile int32_t* local_i2s1_ptr = tracker->i2s1_active_ptr;
    volatile int32_t* local_i2s2_ptr = tracker->i2s4_active_ptr;
    tracker->i2s1_data_ready = false;
    tracker->i2s4_data_ready = false;

    deinterleave_and_convert(local_i2s1_ptr, tracker->mic_samples[0], tracker->mic_samples[1], ANALYSIS_SIZE);
    arm_copy_f32(tracker->mic_samples[0], tracker->processing_buffer, ANALYSIS_SIZE);

    arm_biquad_cascade_df1_f32(&tracker->filter_instance, tracker->processing_buffer, tracker->processing_buffer, ANALYSIS_SIZE);
    arm_rfft_fast_f32(&tracker->fft_instance, tracker->processing_buffer, tracker->fft_output, 0);
    arm_cmplx_mag_f32(tracker->fft_output, tracker->magnitudes, FFT_SIZE / 2);

    float32_t max_peak, mean_energy;
    uint32_t max_peak_index;
    arm_mean_f32(tracker->magnitudes, FFT_SIZE / 2, &mean_energy);
    arm_max_f32(tracker->magnitudes, FFT_SIZE / 2, &max_peak, &max_peak_index);

    bool is_drone_signature_present = (max_peak > mean_energy * DRONE_THRESHOLD);

    switch (tracker->current_state)
    {
        case STATE_LISTENING:
            if (is_drone_signature_present)
            {
                tracker->detection_counter++;
                if (tracker->detection_counter >= 3)
                {
                    tracker->current_state = STATE_TRACKING;
                    tracker->lost_counter = 0;
                }
            }
            else
                tracker->detection_counter = 0;
            break;

        case STATE_TRACKING:
            if (is_drone_signature_present)
            {
                tracker->lost_counter = 0;

                deinterleave_and_convert(local_i2s2_ptr, tracker->mic_samples[2], tracker->mic_samples[3], ANALYSIS_SIZE);

                tracker->azimuth = calculate_tdoa_angle(tracker, tracker->mic_samples[0], tracker->mic_samples[1]);
                tracker->elevation = calculate_tdoa_angle(tracker, tracker->mic_samples[2], tracker->mic_samples[3]);

                tracker->smoothed_azimuth = smooth_angle(tracker->azimuth, tracker->azimuth_history, &tracker->azimuth_history_index);
                tracker->smoothed_elevation = smooth_angle(tracker->elevation, tracker->elevation_history, &tracker->elevation_history_index);

                set_servo_position(TIM_CHANNEL_2, tracker->smoothed_azimuth);
                set_servo_position(TIM_CHANNEL_3, tracker->smoothed_elevation);
            }
            else
            {
                tracker->lost_counter++;
                if (tracker->lost_counter >= 10)
                {
                    tracker->current_state = STATE_LISTENING;
                    tracker->detection_counter = 0;
                }
            }
            break;
    }
}

static void deinterleave_and_convert(volatile int32_t* dma_buffer, float32_t* left_channel_out, float32_t* right_channel_out, uint32_t num_samples)
{
    for (uint32_t i = 0; i < num_samples; i++)
    {
        right_channel_out[i] = (float32_t)dma_buffer[2 * i];
        left_channel_out[i]  = (float32_t)dma_buffer[2 * i + 1];
    }
}

static float32_t calculate_tdoa_angle(DroneTracker_t* tracker, float32_t* micA_samples, float32_t* micB_samples)
{
    arm_correlate_f32(micA_samples, ANALYSIS_SIZE, micB_samples, ANALYSIS_SIZE, tracker->correlation_output);

    float32_t max_corr_value;
    uint32_t  max_corr_index;
    arm_max_f32(tracker->correlation_output, (2 * ANALYSIS_SIZE - 1), &max_corr_value, &max_corr_index);

    int32_t k_max = max_corr_index - (ANALYSIS_SIZE - 1);
    float32_t delta_t = (float32_t)k_max / SAMPLE_RATE;
    float32_t sin_theta = (delta_t * SOUND_SPEED) / MIC_DISTANCE;

    if (sin_theta > 1.0f)  sin_theta = 1.0f;
    if (sin_theta < -1.0f) sin_theta = -1.0f;

    float32_t angle_rad = asinf(sin_theta);
    float32_t angle_deg = angle_rad * (180.0f / M_PI);

    return angle_deg;
}

static float32_t smooth_angle_optimized(float32_t new_angle, float32_t* history_buffer, uint32_t* history_index, float32_t* current_sum)
{
    *current_sum -= history_buffer[*history_index];

    *current_sum += new_angle;
    history_buffer[*history_index] = new_angle;

    *history_index = (*history_index + 1) % ANGLE_SMOOTHING_WINDOW_SIZE;

    return *current_sum / ANGLE_SMOOTHING_WINDOW_SIZE;
}

static void set_servo_position(uint32_t channel, float32_t angle)
{
	if (angle < input_angle_min) angle = input_angle_min;
	if (angle > input_angle_max) angle = input_angle_max;

    float32_t mapped_value = (angle - input_angle_min) * (pulse_max - pulse_min) / (input_angle_max - input_angle_min) + pulse_min;

	__HAL_TIM_SET_COMPARE(&timerHandle, channel, (uint32_t)mapped_value);
}
