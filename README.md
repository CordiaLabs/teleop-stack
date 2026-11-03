# Cordia Labs — Teleoperation & Synced Recording Stack

Synchronized dual SO-101 teleoperation, cameras, and 200 Hz tactile data logging in Hugging Face LeRobot format.

```
Kinematics: Dual SO-101 (12× Feetech STS3215 @ 1 Mbps)
Vision:     Overhead 720p workspace view + 2× wrist views (referenced to arm base)
Tactile:    4-channel thin-film FSR stream @ 200 Hz over UDP
Clock:      Single hardware-synced timebase across all modalities
Dataset:    LeRobot v2.1 Parquet / MP4 format
```

---

## Overview

Cloth manipulation usually fails at the point of contact. A gripper closes, the fabric slips two millimeters, and the fold comes out crooked. Vision models miss the slip because cameras can't see pressure.

This stack integrates:
1. **Follower Arm Feedback:** High-speed 1 Mbps serial driver polling all 12 Feetech STS3215 servos at 50 Hz.
2. **Leader Teleoperation:** Low-latency follower tracking commanded by passive leader arms.
3. **Point-of-Contact Tactile Stream:** 200 Hz piezoresistive FSR telemetry streamed over UDP via the ESP32 tactile bridge.
4. **Three Camera Streams:** Overhead 720p workspace view (cropped to ~78 cm at ~1.3 mm per pixel) plus two wrist cameras referenced to the arm base.
5. **Unified Clock Sync:** All vision frames, joint states, and contact forces are synced to one clock and written straight to LeRobot's dataset format. When a fold fails, the slip is in the log.

---

## Repository Structure

```
├── hardware/
│   ├── arms.py           # Fast Feetech STS3215 1 Mbps serial bus manager
│   ├── cameras.py        # Non-blocking OpenCV multi-camera capture
│   ├── tactile.py        # 200 Hz ESP32 UDP tactile telemetry receiver
│   └── force_proxy.py    # Servo current and position-error fallback estimator
├── teleop/
│   └── record.py         # LeRobot v2.1 dataset recorder
├── scripts/
│   ├── readiness_check.py # Comprehensive hardware bring-up and verification
│   └── find_devices.py    # Serial port & camera index discovery
└── requirements.txt
```

---

## Quick Start

### 1. Installation
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install lerobot
```

### 2. Device Discovery
Scan for connected Feetech bus adapters and USB cameras:
```bash
python3 scripts/find_devices.py
```

### 3. Readiness Check
Verify serial communication, 200 Hz tactile UDP stream, and camera feeds before recording:
```bash
python3 scripts/readiness_check.py
```

### 4. Teleoperate & Record
```bash
python3 teleop/record.py \
  --repo-id cordialabs/cloth-v0-pilot \
  --fps 30 \
  --tactile-hz 200
```

---

## License

Licensed under the [Apache License, Version 2.0](LICENSE).
