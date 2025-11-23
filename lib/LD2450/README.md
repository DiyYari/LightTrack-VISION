# LD2450 Arduino Library

Lightweight parser for the Hi-Link LD2450 radar module tailored for the LightTrack VISION sketch. The driver consumes raw bytes from a `Stream`, buffers frames, and exposes up to three parsed targets with position, speed, and computed distance.

## Usage

```cpp
#include <LD2450.h>

LD2450 ld2450;

void setup() {
  Serial1.begin(256000, SERIAL_8N1, RX_PIN, TX_PIN);
  ld2450.begin(Serial1);
}

void loop() {
  if (ld2450.read() > 0) {
    for (size_t i = 0; i < ld2450.getSensorSupportedTargetCount(); ++i) {
      auto target = ld2450.getTarget(i);
      if (target.valid) {
        Serial.printf("Target %u -> x:%d y:%d dist:%d speed:%d\n", i, target.x, target.y, target.distance, target.speed);
      }
    }
  }
}
```

The `read()` call ingests any available bytes, parses complete frames, and refreshes the internal targets. You can call it frequently inside a task or loop to keep data current.
