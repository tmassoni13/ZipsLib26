/*
    PiLink.cpp

    Handles the serial protocol used to receive Pi pose data and calibration
    responses.
*/

#include "comm/PiLink.hpp"

#include "Chassis/ChassisController.hpp"
#include "pros/apix.h"
#include "pros/rtos.hpp"
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace comm {
namespace {

constexpr std::uint8_t kHeader1 = 0xAA;
constexpr std::uint8_t kHeader2 = 0x55;
constexpr std::uint32_t kPollDelayMs = 10;
constexpr std::size_t kLastMessageSize = 65;
constexpr std::size_t kMaxParseBuf = 96;
constexpr std::uint32_t kCalibPollMs = 10;

PiLinkConfig g_config{1, 115200, 64};
pros::Mutex g_mutex;
char g_last_message[kLastMessageSize] = {0};
std::uint32_t g_msg_count = 0;
std::uint32_t g_byte_count = 0;
std::uint8_t g_last_byte = 0;
bool g_started = false;
bool g_enabled = true;
enum class CalibStatus { kIdle, kPending, kSuccess, kFail };
CalibStatus g_calib_status = CalibStatus::kIdle;

// Updates the CRC8 accumulator with one byte.
std::uint8_t crc8_update(std::uint8_t crc, std::uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; ++i) {
    if ((crc & 0x80) != 0) {
      crc = static_cast<std::uint8_t>((crc << 1) ^ 0x07);
    } else {
      crc = static_cast<std::uint8_t>(crc << 1);
    }
  }
  return crc;
}

// Copies a binary payload into the last-message ASCII buffer.
void copy_payload_as_ascii(const std::uint8_t* payload, std::uint8_t len) {
  std::uint8_t copy_len = len;
  if (copy_len >= kLastMessageSize) {
    copy_len = static_cast<std::uint8_t>(kLastMessageSize - 1);
  }

  for (std::uint8_t i = 0; i < copy_len; ++i) {
    const std::uint8_t b = payload[i];
    g_last_message[i] = (b >= 32 && b <= 126) ? static_cast<char>(b) : '.';
  }
  g_last_message[copy_len] = '\0';
  ++g_msg_count;
}

// Sends a framed PiLink packet over stdout.
void send_packet(const std::uint8_t* payload, std::uint8_t len) {
  if (payload == nullptr || len == 0) return;
  std::uint8_t crc = crc8_update(0, len);
  std::putchar(static_cast<int>(kHeader1));
  std::putchar(static_cast<int>(kHeader2));
  std::putchar(static_cast<int>(len));
  for (std::uint8_t i = 0; i < len; ++i) {
    const std::uint8_t b = payload[i];
    crc = crc8_update(crc, b);
    std::putchar(static_cast<int>(b));
  }
  std::putchar(static_cast<int>(crc));
  std::fflush(stdout);
}

// Checks whether a payload is a calibration response message.
bool is_calib_response(const std::uint8_t* payload, std::uint8_t len, CalibStatus* status) {
  if (payload == nullptr || status == nullptr) return false;
  if (len == 6 && std::memcmp(payload, "CAL_OK", 6) == 0) {
    *status = CalibStatus::kSuccess;
    return true;
  }
  if (len == 8 && std::memcmp(payload, "CAL_FAIL", 8) == 0) {
    *status = CalibStatus::kFail;
    return true;
  }
  return false;
}

// Parses an ASCII pose payload into a Pose2D.
bool parse_pose_from_payload(const std::uint8_t* payload, std::uint8_t len,
                             zips::Pose2D* out_pose) {
  if (out_pose == nullptr || payload == nullptr || len == 0) {
    return false;
  }

  char buf[kMaxParseBuf];
  std::size_t copy_len = len;
  if (copy_len >= sizeof(buf)) copy_len = sizeof(buf) - 1;
  std::memcpy(buf, payload, copy_len);
  buf[copy_len] = '\0';

  double x = 0.0;
  double y = 0.0;
  double t = 0.0;
  const int parsed = std::sscanf(buf, " %lf , %lf , %lf", &x, &y, &t);
  if (parsed != 3) {
    return false;
  }

  out_pose->x = x;
  out_pose->y = y;
  out_pose->theta = t;
  return true;
}

// Runs the PiLink receive loop and dispatches valid payloads.
void pi_link_task(void*) {
  enum class State { kWaitHeader1, kWaitHeader2, kReadLen, kReadPayload, kReadCrc };
  State state = State::kWaitHeader1;
  std::uint8_t length = 0;
  std::uint8_t index = 0;
  std::uint8_t crc = 0;
  std::uint8_t payload[255] = {0};
  while (true) {
    const int ch = std::getchar();
    if (ch == EOF) {
      pros::delay(kPollDelayMs);
      continue;
    }

    const std::uint8_t byte = static_cast<std::uint8_t>(ch & 0xFF);
    g_mutex.take();
    ++g_byte_count;
    g_last_byte = byte;
    g_mutex.give();
    switch (state) {
      case State::kWaitHeader1:
        state = (byte == kHeader1) ? State::kWaitHeader2 : State::kWaitHeader1;
        break;
      case State::kWaitHeader2:
        if (byte == kHeader2) {
          state = State::kReadLen;
        } else {
          state = State::kWaitHeader1;
        }
        break;
      case State::kReadLen:
        length = byte;
        if (length == 0 || length > g_config.max_payload) {
          state = State::kWaitHeader1;
          break;
        }
        crc = crc8_update(0, length);
        index = 0;
        state = State::kReadPayload;
        break;
      case State::kReadPayload:
        payload[index++] = byte;
        crc = crc8_update(crc, byte);
        if (index >= length) {
          state = State::kReadCrc;
        }
        break;
      case State::kReadCrc:
        if (byte == crc) {
          CalibStatus status = CalibStatus::kIdle;
          if (is_calib_response(payload, length, &status)) {
            g_mutex.take();
            g_calib_status = status;
            g_mutex.give();
          } else {
            zips::Pose2D pose{};
            if (g_enabled && parse_pose_from_payload(payload, length, &pose)) {
              zips::chassis.setPoseFromOtos(pose);
            }
          }
          g_mutex.take();
          copy_payload_as_ascii(payload, length);
          g_mutex.give();
        }
        state = State::kWaitHeader1;
        break;
    }
  }
}

}  // namespace

// Starts the PiLink background task if it has not already started.
void startPiLink(const PiLinkConfig& config) {
  if (g_started) {
    return;
  }
  g_config = config;
  pros::c::serctl(SERCTL_ENABLE_COBS, nullptr);
  pros::c::serctl(SERCTL_ACTIVATE, reinterpret_cast<void*>(0x006e6973));  // "sin\0"
  pros::c::serctl(SERCTL_ACTIVATE, reinterpret_cast<void*>(0x74756f73));  // "sout"
  g_started = true;
  pros::Task(pi_link_task, nullptr, TASK_PRIORITY_DEFAULT, 2048, "PiLink");
}

// Enables or disables PiLink pose updates.
void setPiLinkEnabled(bool enabled) {
  g_enabled = enabled;
}

// Returns whether PiLink pose updates are enabled.
bool isPiLinkEnabled() {
  return g_enabled;
}

// Requests OTOS calibration from the Pi and waits for the result.
bool requestOtosCalibrate(std::uint32_t timeout_ms) {
  const std::uint8_t payload[] = {'C','A','L'};
  g_mutex.take();
  g_calib_status = CalibStatus::kPending;
  g_mutex.give();
  send_packet(payload, static_cast<std::uint8_t>(sizeof(payload)));

  const std::uint32_t start = pros::c::millis();
  while (true) {
    g_mutex.take();
    CalibStatus status = g_calib_status;
    g_mutex.give();
    if (status == CalibStatus::kSuccess) return true;
    if (status == CalibStatus::kFail) return false;
    if (pros::c::millis() - start >= timeout_ms) {
      return false;
    }
    pros::delay(kCalibPollMs);
  }
}

// Copies the latest received Pi payload into a caller-provided buffer.
bool getLastPiMessage(char* out, std::size_t out_size, std::uint32_t* out_count) {
  if (out == nullptr || out_size == 0) {
    return false;
  }

  g_mutex.take();
  if (g_msg_count == 0) {
    g_mutex.give();
    return false;
  }

  std::snprintf(out, out_size, "%s", g_last_message);
  if (out_count != nullptr) {
    *out_count = g_msg_count;
  }
  g_mutex.give();
  return true;
}

}  // namespace comm


