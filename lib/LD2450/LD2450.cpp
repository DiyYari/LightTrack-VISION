#include "LD2450.h"

#include <math.h>
#include <string.h>

namespace {
constexpr uint8_t kFrameHeader[] = {0xFD, 0xFC, 0xFB, 0xFA};
}

LD2450::LD2450() : serial_(nullptr), buffer_{0}, bufferLength_(0) { resetTargets(); }

void LD2450::begin(Stream& serial) {
  serial_ = &serial;
  resetTargets();
  bufferLength_ = 0;
}

LD2450::RadarTarget LD2450::getTarget(size_t index) const {
  if (index >= kSupportedTargets) {
    return RadarTarget{};
  }
  return targets_[index];
}

void LD2450::resetTargets() {
  for (auto& target : targets_) {
    target = RadarTarget{};
  }
}

void LD2450::shiftBuffer(size_t count) {
  if (count == 0 || count > bufferLength_) return;
  const size_t remaining = bufferLength_ - count;
  if (remaining > 0) {
    memmove(buffer_, buffer_ + count, remaining);
  }
  bufferLength_ = remaining;
}

size_t LD2450::findHeaderIndex() const {
  if (bufferLength_ < sizeof(kFrameHeader)) return SIZE_MAX;
  for (size_t i = 0; i <= bufferLength_ - sizeof(kFrameHeader); ++i) {
    bool match = true;
    for (size_t j = 0; j < sizeof(kFrameHeader); ++j) {
      if (buffer_[i + j] != kFrameHeader[j]) {
        match = false;
        break;
      }
    }
    if (match) return i;
  }
  return SIZE_MAX;
}

size_t LD2450::expectedFrameSize() const {
  if (bufferLength_ < sizeof(kFrameHeader) + 2) return 0;
  const uint16_t payloadLength = buffer_[4] | (static_cast<uint16_t>(buffer_[5]) << 8);
  // Guard against unrealistic payload sizes to avoid buffer overflow.
  if (payloadLength > kFrameBufferSize - 6) return 0;
  return static_cast<size_t>(payloadLength) + 6;  // header + length fields
}

bool LD2450::decodeFrame(const uint8_t* frame, size_t length) {
  if (length < sizeof(kFrameHeader) + 2) return false;
  const uint16_t payloadLength = frame[4] | (static_cast<uint16_t>(frame[5]) << 8);
  const size_t expectedLength = static_cast<size_t>(payloadLength) + 6;
  if (length < expectedLength) return false;

  const size_t payloadStart = 6;
  const size_t payloadAvailable = length > payloadStart ? length - payloadStart : 0;

  size_t validCount = 0;
  for (size_t i = 0; i < kSupportedTargets; ++i) {
    const size_t base = payloadStart + i * 6;
    if (base + 6 > payloadStart + payloadAvailable) {
      targets_[i] = RadarTarget{};
      continue;
    }

    const int16_t x = static_cast<int16_t>(frame[base] | (frame[base + 1] << 8));
    const int16_t y = static_cast<int16_t>(frame[base + 2] | (frame[base + 3] << 8));
    const int16_t speed = static_cast<int16_t>(frame[base + 4] | (frame[base + 5] << 8));

    RadarTarget target;
    target.x = x;
    target.y = y;
    target.speed = speed;
    target.distance = static_cast<int>(roundf(sqrtf(static_cast<float>(x) * static_cast<float>(x) +
                                                   static_cast<float>(y) * static_cast<float>(y))));
    target.valid = !(x == 0 && y == 0 && speed == 0);

    targets_[i] = target;
    if (target.valid) ++validCount;
  }

  // If the payload includes fewer than the expected bytes for three targets, consider it invalid.
  const size_t minimumPayloadBytes = kSupportedTargets * 6;
  return payloadAvailable >= minimumPayloadBytes && validCount > 0;
}

size_t LD2450::read() {
  if (serial_ == nullptr) return 0;

  while (serial_->available() && bufferLength_ < kFrameBufferSize) {
    buffer_[bufferLength_++] = static_cast<uint8_t>(serial_->read());
  }

  size_t lastValidCount = 0;

  while (true) {
    const size_t headerIndex = findHeaderIndex();
    if (headerIndex == SIZE_MAX) {
      bufferLength_ = 0;  // no header found, drop buffer
      break;
    }
    if (headerIndex > 0) {
      shiftBuffer(headerIndex);
      continue;
    }

    const size_t frameSize = expectedFrameSize();
    if (frameSize == 0) {
      // Invalid length detected, drop one byte and resync.
      shiftBuffer(1);
      continue;
    }
    if (frameSize > bufferLength_) break;  // wait for more data

    if (decodeFrame(buffer_, frameSize)) {
      size_t validTargets = 0;
      for (const auto& target : targets_) {
        if (target.valid) ++validTargets;
      }
      lastValidCount = validTargets;
    }

    shiftBuffer(frameSize);
  }

  return lastValidCount;
}
