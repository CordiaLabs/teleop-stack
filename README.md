# Cordia Labs — Teleoperation & Synced Recording Stack

Synchronized dual SO-101 teleoperation, cameras, and 200 Hz tactile data logging in Hugging Face LeRobot format for a $1,000 dual-arm robot that folds laundry.

```
Upstream:   Hugging Face LeRobot (Apache-2.0)
Overlay:    Custom 1 Mbps Feetech STS3215 bus driver + 200 Hz UDP tactile bridge
Kinematics: Dual SO-101 (12× Feetech STS3215 @ 1 Mbps)
Vision:     Overhead 720p workspace view + 2× wrist views (referenced to arm base)
Tactile:    4-channel thin-film FSR stream @ 200 Hz over UDP
Clock:      Single hardware-synced timebase across all modalities
Dataset:    LeRobot v2.1 Parquet / MP4 format
License:    Apache-2.0
```

---

## Relationship to Upstream (Hugging Face LeRobot)

This repository is an explicit hardware telemetry and communication overlay on top of [Hugging Face LeRobot](https://github.com/huggingface/lerobot). We do not reinvent robot learning primitives; we provide the low-latency hardware integration and multimodal synchronization needed to capture reliable contact data for cloth manipulation.

### What is Upstream (LeRobot):
* **Dataset Format & Packaging:** v2.1 Parquet episode structures, feature schemas, and MP4 video chunking.
* **Coordinate Standards:** Arm joint naming conventions and normalized action/observation spaces.
* **Policy Architecture:** Downstream training pipelines (ACT, Diffusion Policy) consuming the recorded episodes.

### What is Ours (Cordia Labs):
* **`hardware/arms.py`:** Direct 1 Mbps serial driver for dual Feetech STS3215 buses (12 servos across two Waveshare adapters). Bypasses high-latency generic serial wrappers to guarantee steady 50 Hz control loops.
* **`hardware/tactile.py`:** Dedicated UDP client listening on port 5006 for the 200 Hz binary telemetry stream broadcast by the ESP32 tactile bridge.
* **`hardware/force_proxy.py`:** Secondary contact estimator calculating grip pressure from servo `Present_Current` (addr 69) and $|Goal - Present|$ position error.
* **`teleop/record.py`:** Hardware-synced multimodal recording loop. Synchronizes asynchronous 200 Hz tactile readings, 50 Hz joint states, and 3× 30 Hz UVC cameras to a single hardware clock, written directly into LeRobot dataset format.
* **`scripts/readiness_check.py` & `scripts/find_devices.py`:** Hardware pre-flight diagnostics verifying bus voltages, packet loss, and camera indexing before recording sessions.

---

## Why Synchronized Contact Logging?

Cloth manipulation usually fails at the point of contact. A gripper closes, the fabric slips two millimeters, and the fold comes out crooked. Vision models miss the slip because cameras can't see pressure.

By hardware-syncing thin-film FSR sensors streaming at 200 Hz with overhead and wrist camera frames, the slip is captured directly in the log. When an autonomous fold fails during policy training or evaluation, the model has the physical contact history to learn why.

---

## Repository Structure

```
├── hardware/
│   ├── arms.py           # [Cordia] Fast Feetech STS3215 1 Mbps serial bus manager
│   ├── cameras.py        # [Cordia] Non-blocking OpenCV multi-camera capture
│   ├── tactile.py        # [Cordia] 200 Hz ESP32 UDP tactile telemetry receiver
│   └── force_proxy.py    # [Cordia] Servo current and position-error fallback estimator
├── teleop/
│   └── record.py         # [Cordia + LeRobot] Synced episode recorder for LeRobot v2.1
├── scripts/
│   ├── readiness_check.py # [Cordia] Comprehensive hardware bring-up and verification
│   └── find_devices.py    # [Cordia] Serial port & camera index discovery
├── requirements.txt
├── NOTICE
└── LICENSE
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

## License & Attribution

Licensed under the [Apache License, Version 2.0](LICENSE).  
Portions of dataset formatting and recording conventions derive from Hugging Face LeRobot (Apache-2.0). See [NOTICE](NOTICE).
