#pragma once

#include <Arduino.h>

class LD2450 {
 public:
  struct RadarTarget {
    bool valid = false;
    int x = 0;
    int y = 0;
    int distance = 0;
    int speed = 0;
  };

  LD2450();

  void begin(Stream& serial);
  size_t read();

  size_t getSensorSupportedTargetCount() const { return kSupportedTargets; }
  RadarTarget getTarget(size_t index) const;

 private:
  static constexpr size_t kSupportedTargets = 3;
  static constexpr size_t kFrameBufferSize = 128;

  Stream* serial_;
  uint8_t buffer_[kFrameBufferSize];
  size_t bufferLength_;
  RadarTarget targets_[kSupportedTargets];

  void resetTargets();
  void shiftBuffer(size_t count);
  size_t findHeaderIndex() const;
  size_t expectedFrameSize() const;
  bool decodeFrame(const uint8_t* frame, size_t length);
};
