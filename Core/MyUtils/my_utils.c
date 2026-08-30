/*
 * my_utils.c
 *
 *  Created on: Jul 17, 2026
 *      Author: Thomas Bourgeois
 *
 *
 * Helper Functions for the STM32F4 project, including mapping values and converting ADC readings to voltages.
 */

#include "my_utils.h"
#include <stdbool.h>

/**
 * @brief  Maps a value from one range to another (like Arduino's map()).
 * @param  x: The input value to map.
 * @param  in_min: The lower bound of the input value's current range.
 * @param  in_max: The upper bound of the input value's current range.
 * @param  out_min: The lower bound of the target range.
 * @param  out_max: The upper bound of the target range.
 * @retval The value of x mapped from [in_min, in_max] to [out_min, out_max].
 * @note   Does not clamp the result — if x is outside [in_min, in_max],
 *         the returned value will be outside [out_min, out_max] as well.
 * @note   If in_min == in_max, this will divide by zero.
 */
float util_map(float x, float in_min, float in_max, float out_min,
		float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Converts a 12-bit ADC reading to a voltage.
 * @param val Raw ADC value (0–4095).
 * @return Voltage in volts (0.0–3.3V), assuming 3.3V reference.
 */
float util_adc_to_voltage(uint16_t val) {
	return ((float) val * 3.3f) / 4095.0f;
}

/**
 * @brief   Averages the most recent samples for one channel from an
 *          interleaved multi-channel ADC/DMA buffer.
 *
 * @details Buffer layout is round-robin per scan, e.g. for 3 channels:
 *          [ch0, ch1, ch2, ch0, ch1, ch2, ...]. Sums every `num_channels`-th
 *          sample starting at `channel_index` and returns the mean.
 *          Non-blocking -- only reads memory already written by DMA.
 *
 * @param[in] buf                 Interleaved raw sample buffer (volatile: written by DMA; const: read-only here).
 * @param[in] num_channels        Number of interleaved channels (ADC ranks).
 * @param[in] samples_per_channel Number of most-recent samples to average.
 * @param[in] channel_index       Zero-based channel index (rank order).
 *
 * @return  Mean of the raw ADC counts (0-4095 for 12-bit), not yet
 *          converted to a voltage.
 *
 * @note    Assumes `buf` holds at least `num_channels * samples_per_channel`
 *          entries; `channel_index` must be < `num_channels`. No bounds
 *          checking is performed.
 */
float util_average_channel(const volatile uint16_t *buf, uint32_t num_channels,
		uint32_t samples_per_channel, uint32_t channel_index) {

	uint32_t sum = 0;
	for (uint32_t i = 0; i < samples_per_channel; i++) {
		sum += buf[(i * num_channels) + channel_index];
	}
	return (float) sum / (float) samples_per_channel;
}

/**
 * @brief Checks whether at least one of the specified bit(s) is set in a bitmask.
 *
 * Performs a bitwise AND between current_fault and faults_to_check and returns
 * true if the result is non-zero. If faults_to_check contains multiple bits,
 * this returns true as long as ANY one of those bits is set — it does not
 * require all of them to be set.
 *
 * @param current_fault   Bitmask of currently active fault/state flags.
 * @param faults_to_check Single flag, or a mask of multiple flags, to test for.
 * @return true if any bit in faults_to_check is also set in current_fault; false otherwise.
 */
bool has_fault(uint32_t current_fault, uint32_t faults_to_check) {
	// Non-zero result means at least one matching bit was found.
	return (current_fault & faults_to_check) != 0;
}

/**
 * @brief Checks whether ALL of the specified faults are set in a bitmask.
 *
 * Performs a bitwise AND between current_fault and faults_to_check and
 * returns true only if the result equals faults_to_check exactly, meaning
 * every bit in faults_to_check is also present in current_fault.
 *
 * @param current_fault   Bitmask of currently active fault/state flags.
 * @param faults_to_check Mask of flags that must ALL be present.
 * @return true if every bit in faults_to_check is set in current_fault; false if any is missing.
 */
bool has_all_faults(uint32_t current_fault, uint32_t faults_to_check) {
	// Result must match the mask exactly, all required bits must be set.
	return (current_fault & faults_to_check) == faults_to_check;
}
