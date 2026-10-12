/*
    PiLink.hpp

    Declares the Pi serial link configuration and public communication helpers.
*/

#pragma once

#include <cstddef>
#include <cstdint>

namespace comm {

// Simple Pi -> V5 serial protocol (USB stdin via PROS COBS stream "sin"):
// [0] 0xAA
// [1] 0x55
// [2] payload length (N)
// [3..3+N-1] payload bytes
// [3+N] CRC8 over length + payload (poly 0x07, init 0x00)
struct PiLinkConfig {
  // port/baudrate retained for compatibility; USB stdin ignores these.
  std::uint8_t port;
  std::int32_t baudrate;
  std::uint8_t max_payload;
};

/**
 * @brief Starts the Pi serial link receive task.
 *
 * @param config Serial protocol configuration.
 */
void startPiLink(const PiLinkConfig& config);

/**
 * @brief Enables or disables incoming Pi pose updates.
 *
 * @param enabled True to accept Pi updates, false to ignore them.
 */
void setPiLinkEnabled(bool enabled);

/**
 * @brief Checks whether incoming Pi pose updates are enabled.
 *
 * @return True when Pi updates are enabled.
 */
bool isPiLinkEnabled();

// Send a calibration request to the Pi and wait for a response.
// Returns true on success, false on failure/timeout.
/**
 * @brief Requests an OTOS calibration from the Pi and waits for a response.
 *
 * @param timeout_ms Maximum time to wait for the response, in milliseconds.
 * @return True if calibration succeeds before timeout.
 */
bool requestOtosCalibrate(std::uint32_t timeout_ms = 5000);

// Copies the last received payload as ASCII (non-printables become '.').
// Returns true if a message has been received.
/**
 * @brief Copies the most recent Pi payload into an ASCII buffer.
 *
 * @param out Destination buffer.
 * @param out_size Size of the destination buffer.
 * @param out_count Optional pointer that receives the total received message count.
 * @return True if a message was available and copied.
 */
bool getLastPiMessage(char* out, std::size_t out_size, std::uint32_t* out_count);

}  // namespace comm
