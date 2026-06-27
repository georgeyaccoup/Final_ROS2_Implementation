<div align="center">

```
███████╗ █████╗  █████╗ ████████╗
██╔════╝██╔══██╗██╔══██╗╚══██╔══╝
███████╗███████║███████║   ██║
╚════██║██╔══██║██╔══██║   ██║
███████║██║  ██║██║  ██║   ██║
╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝
```

# Solar-Powered Automated Agricultural Technology
### Pear Sorting & Packaging System — ROS2 Production Workspace

[![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/foxy/)
[![Python](https://img.shields.io/badge/Python-3.8-3776AB?logo=python&logoColor=white)](https://python.org)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20Nano-76B900?logo=nvidia&logoColor=white)](https://developer.nvidia.com/embedded/jetson-nano)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?logo=docker&logoColor=white)](https://docker.com)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

*A graduation project & Agri-Tech startup targeting Egyptian SME packhouses.*
*Detects infected pears, sorts them with servo actuators, and balances two conveyor belts — all in under 1 second.*

</div>

---

## Table of Contents

- [System Overview](#-system-overview)
- [Hardware Architecture](#-hardware-architecture)
- [Belt Layout & ROI Grid](#-belt-layout--roi-grid)
- [Node Architecture](#-node-architecture)
- [Data Flow Diagram](#-data-flow-diagram)
- [Topic Reference](#-topic-reference)
- [The 9-Step Vision Pipeline](#-the-9-step-vision-pipeline)
- [Speed Control Algorithm](#-speed-control-algorithm)
- [The 1-Second Action Cycle](#-the-1-second-action-cycle)
- [Custom Messages](#-custom-messages)
- [IoT Fields & Database Schema](#-iot-fields--database-schema)
- [SCADA Dashboard](#-scada-dashboard)
- [Workspace Structure](#-workspace-structure)
- [Getting Started](#-getting-started)
- [Configuration Reference](#-configuration-reference)
- [Calibration Guide](#-calibration-guide)
- [Troubleshooting](#-troubleshooting)

---

## 🌿 System Overview

SAAT is a fully autonomous pear inspection and sorting machine. Pears arrive on a loading conveyor, pass under an Intel RealSense D455 depth camera, get individually inspected by a classical computer vision pipeline across a 2×3 zone grid, and are physically sorted by six servo-actuated ejection arms — all within a **1-second processing window** per inspection cycle.

```
                    ┌─────────────────────────────────────────────┐
                    │           SAAT SYSTEM OVERVIEW              │
                    └─────────────────────────────────────────────┘

  ┌──────────────┐      ┌────────────────────┐      ┌──────────────────┐
  │              │      │                    │      │                  │
  │  LOADING     │      │   VISION ZONE      │      │  PACKING         │
  │  CONVEYOR    │─────▶│   (6 zones: A1–B3) │─────▶│  CONVEYOR        │
  │  (Conv 1)    │      │   RealSense D455   │      │  (Conv 2)        │
  │  175 cm      │      │   above the belt   │      │  115 cm          │
  │              │      │                    │      │                  │
  └──────────────┘      └────────────────────┘      └──────────────────┘
        │                         │                         │
    PWM → LPF                  6 Servo                  PWM → LPF
    → PLC Chan A             Actuators                 → PLC Chan B
   (GPIO Pin 11)            (PCA9685 I2C)             (GPIO Pin 13)
        │                         │                         │
        └─────────────────────────┴─────────────────────────┘
                                  │
                        NVIDIA Jetson Nano
                      (ROS2 Foxy in Docker)
                                  │
                         SQLite Database
                       SCADA Dashboard :8080
```

**Key constraint enforced in hardware and software:**
> `Conv1_Voltage + Conv2_Voltage = 3.3 V` at all times.
> Minimum voltage on either belt is **0.1 V** (never true-zero to PLC).

---

## 🔧 Hardware Architecture

| Component | Specification | Interface |
|---|---|---|
| **Compute** | NVIDIA Jetson Nano (4 GB) | — |
| **OS** | Ubuntu 18.04 + ROS2 Foxy (Docker) | — |
| **Vision Sensor** | Intel RealSense D455 | USB 3.0 |
| **Conveyor Motor** | PLC-driven, 0–3.3 V analog input | GPIO Pin 11/13 → LPF → PLC |
| **Servo Driver** | Adafruit PCA9685 (16-channel PWM) | I2C Bus 1 (Pins 3 & 5) |
| **Servo Motors** | MG995/MG996R (5V, 6 units) | PCA9685 Ch 0–5 |
| **PWM Frequency** | 500 Hz software PWM | Jetson.GPIO |
| **Belt lengths** | Conv1 = 175 cm, Conv2 = 115 cm | — |
| **Camera resolution** | 1280 × 720 px @ 30 fps | Both colour + depth |

### PCA9685 Channel Map

```
PCA9685 Board (I2C Address 0x40)
├── Channel 0  →  Zone A1 servo
├── Channel 1  →  Zone A2 servo
├── Channel 2  →  Zone A3 servo
├── Channel 3  →  Zone B1 servo
├── Channel 4  →  Zone B2 servo
└── Channel 5  →  Zone B3 servo
```

### GPIO Pin Map (Jetson Nano BOARD numbering)

```
Pin  3  (I2C SDA)  ──┐
Pin  5  (I2C SCL)  ──┴──▶ PCA9685 (servo driver)
Pin 11  (GPIO)     ──────▶ Conv1 PWM → Low-Pass Filter → PLC Channel A
Pin 13  (GPIO)     ──────▶ Conv2 PWM → Low-Pass Filter → PLC Channel B
```

---

## 🗺️ Belt Layout & ROI Grid

The camera looks straight down at the belt. The 1280×720 frame is divided into two rows and three columns, giving six independent inspection zones.

```
                    ◀────────── 1280 px ──────────▶

                ┌────────────┬────────────┬────────────┐  ▲
                │            │            │            │  │
                │     A1     │     A2     │     A3     │  360 px
                │            │            │            │  │
                ├────────────┼────────────┼────────────┤  ▼
                │            │            │            │  ▲
                │     B1     │     B2     │     B3     │  360 px
                │            │            │            │  │
                └────────────┴────────────┴────────────┘  ▼

                ◀── 427px ──▶◀── 427px ──▶◀── 426px ──▶

  Direction of belt travel ──────────────────────────────▶
```

**Pixel ROI definitions** (used in `saat_params.yaml` and `frame_divider_node`):

```python
ROI_DEFINITIONS = {
    "A1": (0,    0,   427, 360),   # top-left
    "A2": (427,  0,   854, 360),   # top-centre
    "A3": (854,  0,  1280, 360),   # top-right
    "B1": (0,   360,  427, 720),   # bottom-left
    "B2": (427, 360,  854, 720),   # bottom-centre
    "B3": (854, 360, 1280, 720),   # bottom-right
}
```

**Speed pipeline column splits** (separate from the vision ROI grid):

```
┌──────────┬────────────────────────────────────────┬──────────┐
│  Conv 1  │              Vision Strip              │  Conv 2  │
│  cols    │           cols 257 – 1024              │  cols    │
│  0–256   │        (centroid tracking)             │ 1025–1280│
└──────────┴────────────────────────────────────────┴──────────┘
```

---

## 🧩 Node Architecture

The system is composed of **29 ROS2 nodes** (18 unique node types, several instantiated ×6 for each belt zone).

```
                        ┌──────────────────────────────────────────────────────────┐
                        │                   NODE MAP                               │
                        └──────────────────────────────────────────────────────────┘

LAYER 0: STARTUP
  ╔═══════════════════╗
  ║  vision_init_node ║  ← runs ONCE, broadcasts latched /vision_params
  ╚═══════════════════╝

LAYER 1: CAPTURE
  ╔══════════════════════╗
  ║  frame_capture_node  ║  ← RealSense D455 → /raw_frame + /raw_depth
  ╚══════════════════════╝

LAYER 2: FRAME DIVISION
  ╔══════════════════════╗  ╔══════════════════════════╗  ╔══════════════════╗
  ║  frame_divider_node  ║  ║ frame_speed_divider_node ║  ║ volume_divider   ║
  ║  → /zone_frame/A1–B3 ║  ║ → /speed_frame/conv1     ║  ║ → /depth_frame/  ║
  ║  (6 colour subframes)║  ║   /speed_frame/vision    ║  ║   A1–B3          ║
  ╚══════════════════════╝  ║   /speed_frame/conv2     ║  ╚══════════════════╝
                            ╚══════════════════════════╝

LAYER 3: VISION (×6, one per zone)
  ╔══════════╗  ╔══════════╗  ╔══════════╗  ╔══════════╗  ╔══════════╗  ╔══════════╗
  ║ A1_vision║  ║ A2_vision║  ║ A3_vision║  ║ B1_vision║  ║ B2_vision║  ║ B3_vision║
  ║ 9-step   ║  ║ 9-step   ║  ║ 9-step   ║  ║ 9-step   ║  ║ 9-step   ║  ║ 9-step   ║
  ║ pipeline ║  ║ pipeline ║  ║ pipeline ║  ║ pipeline ║  ║ pipeline ║  ║ pipeline ║
  ╚══════════╝  ╚══════════╝  ╚══════════╝  ╚══════════╝  ╚══════════╝  ╚══════════╝
       ↓              ↓              ↓              ↓              ↓              ↓
   /A1/detection  /A2/detection  /A3/detection  /B1/detection  /B2/detection  /B3/detection
                        (saat_interfaces/InfectionResult)

LAYER 4: ACTION + MEASUREMENT (×6, one per zone)
  ╔════════════╗   ╔═════════════════╗   ╔══════════════════════╗
  ║ Ax_action  ║   ║ Ax_data_        ║   ║ Ax_area_node         ║
  ║ PCA9685    ║   ║ collector       ║   ║ 2D silhouette area   ║
  ║ servo ctrl ║   ║ pear ID + data  ║   ╚══════════════════════╝
  ╚════════════╝   ╚═════════════════╝
        ↓                  ↓
  /Ax/action         /Ax/pear_data
  "ACCEPTED"         (PearData msg)
  "REJECTED"
  "IDLE"

LAYER 5: AGGREGATORS
  ╔═══════════════════════════╗  ╔══════════════════════╗  ╔════════════════════╗
  ║ infection_description_node║  ║ volume_estimation    ║  ║ mass_estimation    ║
  ║ all 6 zones → JSON IoT   ║  ║ depth → cm³          ║  ║ cm³ × density → g  ║
  ╚═══════════════════════════╝  ╚══════════════════════╝  ╚════════════════════╝

LAYER 6: SPEED PIPELINE
  ╔════════════════╗   ╔═════════════════╗   ╔═════════════════╗
  ║ main_speed_node║   ║ conv1_speed_node║   ║ conv2_speed_node║
  ║ 7-step centroid║   ║ optical flow    ║   ║ optical flow    ║
  ║ tracker        ║   ║ Conv1 feedback  ║   ║ Conv2 feedback  ║
  ╚════════════════╝   ╚═════════════════╝   ╚═════════════════╝
          ↓                      ↓                    ↓
    /main_speed           /conv1_speed_feedback  /conv2_speed_feedback
          ↓
  ╔════════════════╗   ╔═════════════════════╗
  ║ servo_speed    ║   ║ speed_publisher_node║
  ║ PID for servo  ║   ║ dual-channel PWM    ║
  ║ sweep speed    ║   ║ GPIO 11 + GPIO 13   ║
  ╚════════════════╝   ║ PID + V1+V2=3.3V   ║
                       ╚═════════════════════╝
                                 ↓
                         /speed_to_plc  → PLC → belt motors

LAYER 7: DATABASE + WEB
  ╔═══════════════════════╗   ╔══════════════════════════╗
  ║ data_collection_node  ║   ║ webpage_publisher_node   ║
  ║ SQLite 13-field schema║   ║ Flask SCADA @ :8080      ║
  ║ write order A1→B3     ║   ║ /  /database  /api/status║
  ╚═══════════════════════╝   ╚══════════════════════════╝
```

---

## 🔄 Data Flow Diagram

```
  RealSense D455
       │
       ├──▶ /raw_frame (BGR8, 1280×720)
       │          │
       │          ├──▶ frame_divider_node ──▶ /zone_frame/A1 … /zone_frame/B3
       │          │                                │                │
       │          │                          vision_node      area_node
       │          │                           (×6 zones)      (×6 zones)
       │          │                                │                │
       │          │                       /Ax/detection       /Ax/area
       │          │                          │       │              │
       │          │                    action_node  data_collector  │
       │          │                          │       │              │
       │          │                    PCA9685     /Ax/pear_data    │
       │          │                    servo                        │
       │          │                                                 │
       │          └──▶ frame_speed_divider ──▶ /speed_frame/vision  │
       │                                            │               │
       │                                     main_speed_node        │
       │                                            │               │
       └──▶ /raw_depth (16UC1, mm)                  │      volume_estimation
                  │                                 │               │
                  └──▶ volume_divider ─▶ /depth_frame/Ax           │
                                                    │     mass_estimation
                                                    │               │
                                              /Ax/volume     /Ax/mass
                                                    │               │
                                             ┌──────▼───────────────▼──────┐
                                             │      data_collection_node    │
                                             │   (SQLite · 13-field schema) │
                                             └──────────────┬───────────────┘
                                                            │
                                                     /iot_status (JSON)
                                                            │
                                                   webpage_publisher_node
                                                     Flask  http://:8080
  speed_publisher_node ◀── main_speed_node ◀── /main_speed
         │
    GPIO 11 → LPF → PLC Chan A (Conv1)
    GPIO 13 → LPF → PLC Chan B (Conv2)
```

---

## 📡 Topic Reference

All topics, their message types, publishers, and subscribers:

| Topic | Message Type | Published by | Subscribed by |
|---|---|---|---|
| `/raw_frame` | `sensor_msgs/Image` | `frame_capture_node` | `frame_divider_node`, `frame_speed_divider_node` |
| `/raw_depth` | `sensor_msgs/Image` | `frame_capture_node` | `volume_divider_node` |
| `/vision_params` | `std_msgs/String` (JSON, **latched**) | `vision_init_node` | all 6 `vision_node`, all 6 `area_node` |
| `/zone_frame/A1` … `/B3` | `sensor_msgs/Image` | `frame_divider_node` | `A1_vision` … `B3_vision`, `A1_area_node` … `B3_area_node` |
| `/depth_frame/A1` … `/B3` | `sensor_msgs/Image` | `volume_divider_node` | `volume_estimation_node` |
| `/speed_frame/conv1` | `sensor_msgs/Image` | `frame_speed_divider_node` | `conv1_speed_node` |
| `/speed_frame/vision` | `sensor_msgs/Image` | `frame_speed_divider_node` | `main_speed_node` |
| `/speed_frame/conv2` | `sensor_msgs/Image` | `frame_speed_divider_node` | `conv2_speed_node` |
| `/A1/detection` … `/B3/detection` | `saat_interfaces/InfectionResult` | `A1_vision` … `B3_vision` | `Ax_action`, `Ax_data_collector`, `infection_description_node`, `main_speed_node` |
| `/A1/action` … `/B3/action` | `std_msgs/String` | `A1_action` … `B3_action` | `Ax_data_collector`, `data_collection_node` |
| `/A1/area` … `/B3/area` | `std_msgs/Float32` | `A1_area_node` … `B3_area_node` | `Ax_data_collector`, `volume_estimation_node` |
| `/A1/volume` … `/B3/volume` | `std_msgs/Float32` | `volume_estimation_node` | `mass_estimation_node`, `data_collection_node` |
| `/A1/mass` … `/B3/mass` | `std_msgs/Float32` | `mass_estimation_node` | `data_collection_node` |
| `/A1/pear_data` … `/B3/pear_data` | `saat_interfaces/PearData` | `Ax_data_collector` | `data_collection_node` |
| `/infection_description` | `std_msgs/String` (JSON) | `infection_description_node` | `data_collection_node`, `webpage_publisher_node` |
| `/centroid_time_speed` | `std_msgs/Float32` | `main_speed_node` | `servo_speed_node` |
| `/main_speed` | `saat_interfaces/SpeedCommand` | `main_speed_node` | `speed_publisher_node`, `servo_speed_node` |
| `/conv1_speed_feedback` | `std_msgs/Float32` | `conv1_speed_node` | `speed_publisher_node` |
| `/conv2_speed_feedback` | `std_msgs/Float32` | `conv2_speed_node` | `speed_publisher_node` |
| `/servo_cmd` | `std_msgs/Float32` | `servo_speed_node` | `speed_publisher_node` |
| `/speed_to_plc` | `saat_interfaces/SpeedCommand` | `speed_publisher_node` | `data_collection_node`, `webpage_publisher_node` |
| `/area_volume_mass` | `std_msgs/String` (JSON) | `mass_estimation_node` | `webpage_publisher_node` |
| `/iot_status` | `std_msgs/String` (JSON) | `data_collection_node` | `webpage_publisher_node` |

> **QoS note:** All image topics use `depth=1` (drop-oldest). `/vision_params` uses `TRANSIENT_LOCAL` + `RELIABLE` (latched) so late-starting vision nodes always receive it.

---

## 👁️ The 9-Step Vision Pipeline

Each of the 6 zone-specific `vision_node` instances runs this pipeline independently and in parallel on every incoming sub-frame.

```
Input: colour sub-frame (BGR8, ~427×360 px)
          │
          ▼
┌─────────────────────────────────────────────────────────┐
│  Step 1 │  CLAHE                                        │
│         │  Adaptive histogram equalisation on the L     │
│         │  channel of LAB space. Corrects uneven belt   │
│         │  lighting without blowing out highlights.     │
│         │  clip_limit=2.0, tileSize=(8,8)               │
└─────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────┐
│  Step 2 │  Bilateral Filter                             │
│         │  Edge-preserving noise reduction.             │
│         │  d=9, σ_colour=75, σ_space=75                 │
│         │  Reduces speckle without blurring pear edges. │
└─────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────┐
│  Step 3 │  Colour Space Conversion (parallel)           │
│         │  BGR → HSV  (hue-based pear body detection)  │
│         │  BGR → LAB  (skin texture discrimination)     │
└─────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────┐
│  Step 4 │  Colour Range Masking                         │
│         │  HSV mask: H[10–95] S[40–255] V[40–255]      │
│         │    → captures green to deep yellow pears      │
│         │  LAB mask: L[50–255] A[120–150] B[130–200]   │
│         │    → captures pear-skin characteristics       │
└─────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────┐
│  Step 5 │  Bitwise AND — Mask Fusion                    │
│         │  combined = HSV_mask AND LAB_mask             │
│         │  A pixel must satisfy BOTH colour models      │
│         │  to be classified as pear body.               │
└─────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────┐
│  Step 6 │  Morphological Cleanup                        │
│         │  MORPH_OPEN  (3×3 ellipse) → remove noise    │
│         │  MORPH_CLOSE (3×3 ellipse) → fill blemish    │
│         │  holes caused by brown spots on pear skin.    │
└─────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────┐
│  Step 7 │  Otsu's Thresholding (pear-only)             │
│         │  1. Isolate grayscale pixels INSIDE pear mask │
│         │  2. Run Otsu to find optimal threshold T      │
│         │  3. Infection = pixels darker than 0.75 × T  │
│         │  This adapts to each individual pear's        │
│         │  baseline brightness automatically.           │
└─────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────┐
│  Step 8 │  Contour Analysis                             │
│         │  Find infection region contours               │
│         │  Compute: infection_area, centroid (x,y),     │
│         │  dominant RGB colour via cv2.mean()           │
│         │  infection_ratio = inf_area / pear_area       │
└─────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────┐
│  Step 9 │  Output — Publish InfectionResult             │
│         │  is_infected = infection_ratio > 0.03 (3%)   │
│         │  Publishes to /{zone_id}/detection            │
│         │  All fields are zero when no pear detected.   │
└─────────────────────────────────────────────────────────┘
          │
          ▼
Output: saat_interfaces/InfectionResult
```

### Classification Decision

```
infection_ratio < 3%  →  ACCEPTED  →  servo holds at 0°   (pass-through)
infection_ratio ≥ 3%  →  REJECTED  →  servo moves to 90°  (ejects pear)
                                        servo returns after 0.4 s
```

---

## ⚡ Speed Control Algorithm

`main_speed_node` implements a 7-step centroid tracking algorithm to compute the reference belt speed.

```
  Frame N-1 centroids: [(x₁,y₁), (x₂,y₂), …]   (stored with timestamp t₁)
  Frame N   centroids: [(x₁',y₁'), (x₂',y₂'), …] (current, timestamp t₂)

  Step 1 │ Detect pears via HSV mask on /speed_frame/vision
  Step 2 │ Count pears → check overflow (> 6 pears?)
          │
          │   pear_count == 0   →  EMPTY   → Conv1=3.3V, Conv2=0.1V
          │   pear_count > 6    →  CROWDED → Conv1=0.1V, Conv2=3.3V
          │   0 < count ≤ 6    →  NORMAL  → compute speed below
          │
  Step 3 │ Find centroids for all detected pears
  Step 4 │ Match each centroid to its nearest previous-frame counterpart
  Step 5 │ Per-pear speed:
          │     displacement_px = √((x'-x)² + (y'-y)²)
          │     speed_px_s      = displacement_px / (t₂ - t₁)
          │     speed_m_s       = speed_px_s × px_to_m   (0.0005 m/px default)
  Step 6 │ Average over all moving pears (displacement > 5 px threshold)
  Step 7 │ Map to voltage and publish SpeedCommand:
          │     Conv2_V = (ref_speed_m_s / 0.5) × 3.3
          │     Conv1_V = 3.3 - Conv2_V          ← constraint enforced here
          │     Both clamped to [0.1 V, 3.3 V]
```

### PID Control Loop (in `speed_publisher_node`, runs at 10 Hz)

```
  Setpoint  ──▶ [PID₁] ──▶ Conv1 PWM duty ──▶ GPIO 11 ──▶ LPF ──▶ PLC
  (from         [PID₂] ──▶ Conv2 PWM duty ──▶ GPIO 13 ──▶ LPF ──▶ PLC
  /main_speed)
                  ▲                ▲
                  │                │
           /conv1_speed_feedback  /conv2_speed_feedback
           (optical flow from     (optical flow from
            conv strip)            conv strip)
```

---

## ⏱️ The 1-Second Action Cycle

Every pear that enters the vision zone must be fully processed and physically ejected (if rejected) within one second. Here is how the timing budget is allocated:

```
  0 ms ──── Frame arrives at frame_capture_node
  │
  ├──  ~5 ms   frame_capture_node publishes /raw_frame
  ├──  ~2 ms   frame_divider_node crops and publishes /zone_frame/Ax
  ├── ~30 ms   vision_node runs 9-step pipeline
  │              Step 1 (CLAHE)         ~3 ms
  │              Step 2 (Bilateral)     ~8 ms
  │              Step 3-5 (masks)       ~4 ms
  │              Step 6 (morphology)    ~3 ms
  │              Step 7 (Otsu)          ~5 ms
  │              Step 8 (contours)      ~4 ms
  │              Step 9 (publish)       ~3 ms
  ├──  ~1 ms   action_node receives InfectionResult + commands servo
  ├── ~400 ms  servo physically moves to reject position (MG995 at 5V)
  ├── ~400 ms  servo returns to accept position (return_delay_s = 0.4)
  └── ~162 ms  remaining headroom for jitter / belt alignment

  TOTAL: < 1000 ms ✓
```

> All 6 zones process in **parallel** — there is never a sequential bottleneck across zones. The 1-second window applies per-zone, per-pear independently.

---

## 📨 Custom Messages

The `saat_interfaces` package defines four custom message types.

### `InfectionResult.msg`
Published by each vision node to `/{zone_id}/detection`.

```
std_msgs/Header header
string  zone_id             # "A1" … "B3"
bool    pear_detected       # True if a pear is present in the zone
bool    is_infected         # True if infection_ratio > threshold (0.03)
float32 infection_ratio     # infected_area / pear_area  (0.0 – 1.0)
float32 infection_area_px   # area of infection regions in pixels²
float32 pear_area_px        # total pear silhouette area in pixels²
float32 infection_x         # centroid of infection (sub-frame coords)
float32 infection_y
uint8   infection_r         # dominant RGB colour of infection region
uint8   infection_g
uint8   infection_b
float32 pear_centroid_x     # centroid of pear body (used by speed node)
float32 pear_centroid_y
```

### `PearData.msg`
Published by each data_collector node to `/{zone_id}/pear_data`.

```
std_msgs/Header header
string  pear_id             # "A1_00001", "B3_00042", etc.
string  zone_id             # "A1" … "B3"
string  pear_status         # "ACCEPTED" | "REJECTED"
string  pear_category       # "SMALL" | "BIG"  (threshold: 15,000 px²)
float32 infection_area_px
float32 infection_x
float32 infection_y
uint8   infection_r
uint8   infection_g
uint8   infection_b
float32 infection_ratio
float32 pear_surface_area_px
float32 pear_volume_cm3     # from volume_estimation_node
float32 pear_mass_g         # from mass_estimation_node
```

### `SpeedCommand.msg`
Published by `main_speed_node` to `/main_speed` and by `speed_publisher_node` to `/speed_to_plc`.

```
std_msgs/Header header
float32 reference_speed_ms  # computed centroid-tracking reference (m/s)
float32 conv1_voltage       # target voltage for Conv1 (0.1 – 3.3 V)
float32 conv2_voltage       # target voltage for Conv2 (0.1 – 3.3 V)
float32 servo_voltage       # target voltage for servo sweep speed
string  belt_state          # "EMPTY" | "NORMAL" | "CROWDED"
int32   pear_count          # current pear count in vision zone
```

### `MotorStatus.msg`
System-wide motor summary for IoT dashboard.

```
std_msgs/Header header
string[6]  zone_ids
bool[6]    servo_active
string[6]  last_action      # "ACCEPTED" | "REJECTED" | "IDLE"
float32    conv1_speed_ms
float32    conv2_speed_ms
float32    conv1_voltage
float32    conv2_voltage
int32      batch_accepted
int32      batch_rejected
int32      completed_packages
```

---

## 🗄️ IoT Fields & Database Schema

The SQLite database (`/saat_data/saat_records.db`) stores one row per detected pear. **Write order: A1 → A2 → A3 → B1 → B2 → B3.**

| # | Field | Type | Source Node |
|---|---|---|---|
| 1 | `pear_id` | `TEXT PRIMARY KEY` | `data_collector_node` (e.g. `A1_00001`) |
| 2 | `zone_id` | `TEXT` | zone identifier |
| 3 | `timestamp` | `REAL` | ROS2 header stamp |
| 4 | `pear_status` | `TEXT` | `action_node` (`ACCEPTED` / `REJECTED`) |
| 5 | `pear_category` | `TEXT` | `data_collector_node` (`BIG` / `SMALL`) |
| 6 | `infection_area_px` | `REAL` | `vision_node` |
| 7 | `infection_location` | `TEXT` | `vision_node` (JSON: `{"x":…,"y":…}`) |
| 8 | `infection_color_rgb` | `TEXT` | `vision_node` (JSON: `[R,G,B]`) |
| 9 | `infection_ratio` | `REAL` | `vision_node` |
| 10 | `pear_surface_area_px` | `REAL` | `area_node` |
| 11 | `pear_volume_cm3` | `REAL` | `volume_estimation_node` |
| 12 | `pear_mass_g` | `REAL` | `mass_estimation_node` |
| 13 | `belt_speed_ms` | `REAL` | `main_speed_node` |

The `/iot_status` JSON payload (published at **0.1 Hz** to the status dashboard) aggregates these 13 fields into a system-wide snapshot.

---

## 🖥️ SCADA Dashboard

The `webpage_publisher_node` hosts a live Flask web server at `http://<jetson-ip>:8080`.

```
  http://localhost:8080           →  Status page     (auto-refreshes every 10 s)
  http://localhost:8080/database  →  SQLite viewer   (200 most-recent pears)
  http://localhost:8080/api/status →  JSON API       (raw 13-field IoT payload)
```

**Design reference:** dark IIoT SCADA aesthetic from the project UI reference.

| Token | Value | Used for |
|---|---|---|
| Background | `#0d1117` | Page background |
| Surface | `#161b22` | Cards, panels |
| Border | `#30363d` | Card outlines |
| Accent green | `#00ff88` | Accepted, running state |
| Amber | `#f59e0b` | Warnings, small category |
| Red | `#ef4444` | Rejected, critical alarms |
| Blue | `#3b82f6` | Belt speed, conv voltage |
| Font | `JetBrains Mono` | All text (monospace, industrial) |

---

## 📁 Workspace Structure

```
saat_ws/
│
├── docker/
│   ├── Dockerfile          ← L4T arm64 base, ROS2 Foxy, all pip deps
│   └── entrypoint.sh       ← sources ROS2 + workspace overlay at container start
│
└── src/
    │
    ├── saat_interfaces/    ← Custom message definitions (build first)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       ├── InfectionResult.msg
    │       ├── PearData.msg
    │       ├── SpeedCommand.msg
    │       └── MotorStatus.msg
    │
    └── saat_core/          ← All 18 node implementations
        ├── CMakeLists.txt
        ├── package.xml
        │
        ├── config/
        │   └── saat_params.yaml   ← ALL tunable parameters in one file
        │
        ├── launch/
        │   └── saat_launch.py     ← Master launch (staggered T+0…T+6s)
        │
        └── saat_core/             ← Python package (18 nodes)
            ├── __init__.py
            │
            ├── frame_capture_node.py         ← RealSense D455 capture
            ├── frame_divider_node.py         ← Crops A1–B3 subframes
            ├── frame_speed_divider_node.py   ← Crops Conv1/Vision/Conv2 strips
            ├── volume_divider_node.py        ← Routes depth frames to zones
            │
            ├── vision_init_node.py           ← Broadcasts latched vision params
            ├── vision_node.py                ← 9-step pipeline (×6 instances)
            │
            ├── action_node.py                ← PCA9685 servo control (×6)
            ├── data_collector_node.py        ← Assembles PearData records (×6)
            ├── infection_description_node.py ← Aggregates all 6 zone detections
            │
            ├── area_node.py                  ← 2D pear silhouette area (×6)
            ├── volume_estimation_node.py     ← Depth → volume cm³
            ├── mass_estimation_node.py       ← Volume × density → mass g
            │
            ├── main_speed_node.py            ← 7-step centroid speed tracker
            ├── conv_speed_node.py            ← Optical flow belt feedback (×2)
            ├── servo_speed_node.py           ← PID for servo sweep speed
            ├── speed_publisher_node.py       ← Dual PWM + PID + V1+V2=3.3V
            │
            ├── data_collection_node.py       ← SQLite DB + IoT publisher
            └── webpage_publisher_node.py     ← Flask SCADA dashboard
```

---

## 🚀 Getting Started

### Prerequisites

```bash
# On Jetson Nano host
sudo apt-get install docker.io docker-compose
sudo usermod -aG docker $USER

# Allow Docker to access GPIO
sudo groupadd -f gpio
sudo usermod -aG gpio $USER
```

### Option A — Docker (Recommended)

```bash
# 1. Clone the repository
git clone https://github.com/your-org/saat_ws.git
cd saat_ws

# 2. Build the image (first build ~15–20 min on Nano)
docker build -t saat:latest -f docker/Dockerfile .

# 3. Run with hardware access
docker run --rm -it \
  --privileged \
  --device /dev/video0 \
  --device /dev/i2c-1 \
  -v /sys/bus/usb:/sys/bus/usb \
  -v $(pwd)/saat_data:/saat_data \
  -p 8080:8080 \
  saat:latest
```

### Option B — Native (without Docker)

```bash
# 1. Install ROS2 Foxy (Ubuntu 18.04)
# Follow: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

# 2. Install Python dependencies
pip3 install pyrealsense2 adafruit-circuitpython-servokit \
             adafruit-blinka flask opencv-python numpy

# 3. Build the workspace
cd saat_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select saat_interfaces saat_core

# 4. Source and launch
source install/setup.bash
ros2 launch saat_core saat_launch.py
```

### Verifying the Launch

The launch file brings up all nodes in 6 timed stages. Watch for these log lines:

```
[SAAT] T+0: Starting vision_init_node...
[vision_init_node] classical_vision_initialization complete.
[SAAT] T+1: Starting RealSense capture...
[frame_capture_node] RealSense D455 started: 1280×720 colour@30fps depth@30fps
[SAAT] T+2: Starting frame dividers...
[SAAT] T+3: Starting 6 vision nodes + 6 area nodes...
[A1_vision] Vision params received — pipeline active.
...
[SAAT] T+6: Starting database node + SCADA dashboard...
[SAAT] ✅ Full pipeline online. Dashboard at http://localhost:8080
```

### Checking Individual Nodes

```bash
# List all running SAAT nodes
ros2 node list | grep -E "vision|action|speed|saat"

# Echo detection from zone A1
ros2 topic echo /A1/detection

# Watch belt speed commands
ros2 topic echo /speed_to_plc

# Watch IoT status (fires every 10 s)
ros2 topic echo /iot_status

# Check topic frequencies
ros2 topic hz /raw_frame           # expect ~30 Hz
ros2 topic hz /A1/detection        # expect ~30 Hz
ros2 topic hz /speed_to_plc        # expect ~10 Hz
ros2 topic hz /iot_status          # expect ~0.1 Hz
```

---

## ⚙️ Configuration Reference

All system parameters live in a single file: `src/saat_core/config/saat_params.yaml`.

### Most Important Parameters to Check Before First Run

```yaml
# ── Camera resolution ──────────────────────────────────────
frame_capture_node:
  ros__parameters:
    frame_width:  1280       # Must match camera hardware
    frame_height: 720        # Must match camera hardware
    color_fps:    30         # Reduce to 15 if CPU usage is too high

# ── ROI grid ───────────────────────────────────────────────
frame_divider_node:
  ros__parameters:
    rois:
      A1: [0,   0,   427, 360]  # (x_start, y_start, x_end, y_end)
      A2: [427, 0,   854, 360]
      A3: [854, 0,  1280, 360]
      B1: [0,   360, 427, 720]
      B2: [427, 360, 854, 720]
      B3: [854, 360, 1280, 720]

# ── Vision thresholds ──────────────────────────────────────
vision_init_node:
  ros__parameters:
    infection_ratio_threshold: 0.03  # 3% → tune this first
    max_depth_mm: 380                # distance from camera to belt (mm)
    hsv_lower: [10, 40, 40]          # widen if pears are not detected
    hsv_upper: [95, 255, 255]        # narrow if false positives appear

# ── Servo configuration ────────────────────────────────────
A1_action:
  ros__parameters:
    accept_angle:  0.0     # degrees — pear passes through
    reject_angle:  90.0    # degrees — pear is ejected
    return_delay_s: 0.4    # seconds before servo returns

# ── PWM / PLC interface ────────────────────────────────────
speed_publisher_node:
  ros__parameters:
    gpio_pin_conv1: 11     # BOARD numbering
    gpio_pin_conv2: 13     # BOARD numbering
    pwm_frequency:  500    # Hz — matches LPF cutoff design
    min_voltage:    0.1    # V — never send true-zero to PLC

# ── PID gains (start conservative, tune up) ───────────────
    kp_conv1: 1.0          # proportional gain
    ki_conv1: 0.1          # integral gain
    kd_conv1: 0.05         # derivative gain

# ── Physical pear density ──────────────────────────────────
mass_estimation_node:
  ros__parameters:
    pear_density_g_cm3: 0.96   # standard pear density
```

---

## 🎯 Calibration Guide

Three measurements must be done on the physical machine before the system works accurately. Everything else is already tuned from the report.

### 1. Pixel-to-Meter Scale (`px_to_m`)

This converts centroid displacement in pixels to real-world metres per second.

```bash
# Mount a ruler horizontally across the belt at operating height.
# Run the camera and note how many pixels correspond to 10 cm.
# px_to_m = 0.10 / pixel_count_for_10cm

# Example: 200 pixels = 10 cm → px_to_m = 0.10 / 200 = 0.0005
# Edit saat_params.yaml:
#   main_speed_node:
#     ros__parameters:
#       px_to_m: 0.0005   ← your measured value
```

### 2. Infection Ratio Threshold (`infection_ratio_threshold`)

```bash
# Run 30 pears through the system — 15 known-good, 15 known-infected.
# Watch the published infection_ratio values:
ros2 topic echo /A1/detection | grep infection_ratio

# Find the natural gap between good (low ratio) and infected (high ratio) pears.
# Set the threshold at the midpoint of that gap.
# Typical range: 0.02 (2%) to 0.05 (5%)
```

### 3. PID Gains for Belt Speed

Start with these conservative values and increase gradually:

```
Step 1: Set Ki=0, Kd=0. Increase Kp until belt responds but doesn't overshoot.
Step 2: Add Ki slowly (0.05 increments) to eliminate steady-state error.
Step 3: Add Kd (0.01 increments) to dampen oscillation if any appears.

Starting point:   Kp=0.5,  Ki=0.05, Kd=0.01
Typical final:    Kp=1.0,  Ki=0.1,  Kd=0.05
```

---

## 🔍 Troubleshooting

### Camera not detected

```bash
# Check if RealSense is visible over USB
rs-enumerate-devices

# If not found, check USB 3.0 connection (USB 2.0 is not enough for D455)
lsusb | grep Intel

# Inside Docker, ensure the device is passed through:
docker run --privileged -v /sys/bus/usb:/sys/bus/usb …
```

### Vision nodes not processing frames

```bash
# Check if /vision_params was received (it must arrive before /zone_frame/*)
ros2 topic echo /vision_params

# Check vision_init_node ran first
ros2 node info /vision_init_node

# If it crashed, restart it manually:
ros2 run saat_core vision_init_node.py
```

### Servos not moving

```bash
# Check I2C bus is visible
i2cdetect -y 1
# Should show 0x40 for PCA9685

# Check adafruit-blinka is installed correctly
python3 -c "from adafruit_servokit import ServoKit; k=ServoKit(channels=16); print('OK')"

# Test a single servo manually
python3 -c "
from adafruit_servokit import ServoKit
k = ServoKit(channels=16)
k.servo[0].angle = 90   # Zone A1 → reject position
import time; time.sleep(1)
k.servo[0].angle = 0    # Zone A1 → accept position
"
```

### GPIO PWM not working

```bash
# Verify Jetson.GPIO is installed and the user is in the gpio group
python3 -c "import Jetson.GPIO as GPIO; print(GPIO.VERSION)"
groups | grep gpio

# If not in group:
sudo usermod -aG gpio $USER && newgrp gpio

# Test manually (requires sudo or gpio group)
python3 -c "
import Jetson.GPIO as GPIO, time
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.output(11, GPIO.HIGH)
time.sleep(1)
GPIO.output(11, GPIO.LOW)
GPIO.cleanup()
print('GPIO 11 OK')
"
```

### SCADA dashboard not accessible

```bash
# Check Flask is running inside the node
ros2 node info /webpage_publisher_node

# Check port is open
ss -tlnp | grep 8080

# If running in Docker, confirm port is forwarded
docker ps   # should show 0.0.0.0:8080->8080/tcp

# Access from another machine on the same network:
http://<jetson-ip>:8080
```

### High CPU usage on Jetson Nano

```bash
# Reduce camera FPS in saat_params.yaml
frame_capture_node:
  ros__parameters:
    color_fps: 15   # down from 30

# Enable Jetson performance mode
sudo nvpmodel -m 0       # max power (10W mode)
sudo jetson_clocks       # lock clocks to maximum
```

---

## 📝 Development Notes

### Running a single node for debugging

```bash
source install/setup.bash

# Run any node in isolation (uses saat_params.yaml for defaults)
ros2 run saat_core vision_node.py \
  --ros-args -r __node:=A1_vision -p zone_id:=A1 \
  --params-file src/saat_core/config/saat_params.yaml
```

### Re-building after code changes

```bash
# With --symlink-install, Python node changes are live immediately.
# Only re-build if you change CMakeLists.txt, package.xml, or .msg files:
colcon build --symlink-install --packages-select saat_interfaces saat_core
source install/setup.bash
```

### Viewing the database directly

```bash
sqlite3 /saat_data/saat_records.db

sqlite> SELECT pear_id, pear_status, infection_ratio, pear_mass_g
        FROM pear_records
        ORDER BY timestamp DESC
        LIMIT 20;

sqlite> SELECT pear_status, COUNT(*) FROM pear_records GROUP BY pear_status;
sqlite> .quit
```

<div align="center">

**Built with ROS2 Foxy · OpenCV · Intel RealSense · NVIDIA Jetson Nano**

*Made in Egypt 🇪🇬*

</div>
