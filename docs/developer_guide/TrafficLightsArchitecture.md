# Traffic Lights Architecture

This document describes the traffic lights architecture in scenario_simulator_v2, including the channel concept, state management, and merge logic.

## Channel Concept

A physical traffic light can be observed through different methods:

- **Conventional**: Optical observation (camera-based)
- **V2I**: Network-based observation (V2X communication)

A single traffic light has two channels: conventional and v2i.

```
Physical Traffic Light
├── conventional channel (optical observation)
│   ├── GroundTruth
│   └── Detected
└── v2i channel (network observation)
    ├── GroundTruth
    └── Detected
```

### GroundTruth and Detected

Each channel has two types of states:

| State | Description | Unknown State |
|-------|-------------|---------------|
| **GroundTruth** | True state of the traffic light | Not allowed |
| **Detected** | State as perceived by Autoware | Allowed |

Note that "unknown" is Inappropriate for ground truth.

!!! note "Separated Ground Truth"
    Ideally, the GroundTruth of conventional and v2i channels should have the same state.
    However, in scenario_simulator_v2, each channel has its own GroundTruth separately to preserve backward compatibility with scenarios that treat conventional and V2I traffic lights independently.

### Channel Synchronization

Actual V2I traffic lights are physical traffic lights with network communication capabilities. Therefore, the states of conventional and v2i channels should be identical.

- `EnableTrafficSignalV2IFeatureAction` adds V2I output capability to a traffic light
- When enabled, channel synchronization is performed in the background

!!! note "Backward Compatibility (scenario_simulator_v2 v21.2.2 and earlier)"
    Synchronization is **not** enabled unless `EnableTrafficSignalV2IFeatureAction` is explicitly executed.
    This ensures that scenarios written for earlier versions, where conventional and V2I traffic lights were treated as independent equipment, continue to work as expected.

## State Management

### Detected State Lifecycle

Detected states **override** GroundTruth states for Autoware perception simulation. There are two ways to set Detected states, each with different lifecycle management:

#### `Phase` Definition (Automatic Lifecycle)

Detected states defined in OpenSCENARIO XML `TrafficSignalController`'s `Phase` (using `conventional_detected` / `v2i_detected` specification) are **automatically managed**:

- On Phase transition, the previous Phase's Detected states are **cleared before** the new Phase is evaluated
- Scenario writers only need to define Detected states within a Phase
- GroundTruth of the next Phase is not masked by stale Detected states
- Each Phase evaluation first clears all its defined states, then sets new values

#### `TrafficSignalStateAction` (Manual Lifecycle)

Detected states set via OpenSCENARIO XML `TrafficSignalStateAction` are **not automatically cleared**:

- States persist until explicitly changed by another TrafficSignalStateAction
- Each StateAction clears the previous state for the same lanelet_id before setting the new one
- Use this for cases requiring manual lifecycle management (e.g., testing specific perception scenarios)

#### Combined Usage

Both Phase definitions and TrafficSignalStateAction target the **same Detected state storage** per channel. They do not maintain separate state pools.

- **Last-write-wins**: The most recent setting takes effect for a given lanelet_id
- **Phase transition clears only Phase-defined states**: Only the lanelet_ids defined in that Phase are cleared
- **StateAction states persist across Phase transitions**: StateAction settings for different lanelet_ids remain unaffected by Phase transitions

### Output Generation

When generating traffic light output, Detected states override GroundTruth:

- If a Detected state exists for a lanelet_id → **Replace** GroundTruth with Detected
- If no Detected state exists → Use GroundTruth as-is
- If Detected exists for a lanelet_id not in GroundTruth → **Add** as new entry

## Message Formats

### Format Types

| Format | Purpose | Notes |
|--------|---------|-------|
| C++ (`TrafficLight`) | Internal state management | Holds both way_id and regulatory_elements_ids |
| Proto (`TrafficSignal`) | Inter-process communication | Hub for all conversions |
| `traffic_simulator_msgs` | GroundTruth for external systems | Stable interface, prioritizes backward compatibility |
| `autoware_perception_msgs` | Autoware perception simulation | Follows Autoware specs, version-dependent types |

`traffic_simulator_msgs` provides a stable interface unaffected by Autoware specification changes, while `autoware_perception_msgs` uses different types depending on architecture version (`TrafficSignalArray` for ≤20230906, `TrafficLightGroupArray` for ≥20240605).

### Output Topics

#### Conventional

| Topic | Source | Message Type |
|-------|--------|-------------|
| `/simulation/traffic_lights` | traffic_simulator | `traffic_simulator_msgs::msg::TrafficLightArrayV1` |
| `/perception/traffic_light_recognition/internal/traffic_signals` | simple_sensor_simulator | Architecture-dependent |

#### V2I

| Topic | Source | Message Type |
|-------|--------|-------------|
| `/perception/traffic_light_recognition/external/traffic_signals` | traffic_simulator | Architecture-dependent |
| `/v2x/traffic_signals` (legacy) | traffic_simulator | Architecture-dependent |

### ID Types

Traffic lights have two types of IDs, both derived from lanelet2 map concepts:

| ID | lanelet2 Element | Description |
|----|------------------|-------------|
| **way_id** | Way | Physical traffic light ID representing individual signal head geometry |
| **relation_id** | Relation (TrafficLightRegulatoryElement) | Semantic group ID representing traffic lights that control the same traffic flow |

In lanelet2, a `Way` represents the physical geometry of a single traffic light head, while a `Relation` (specifically `TrafficLightRegulatoryElement`) groups multiple traffic light heads that semantically belong together—for example, all signal heads controlling vehicles approaching an intersection from the same direction.

### Conversion Design

To minimize maintenance cost when specifications change, Proto is used as the single hub for all conversions. This avoids implementing conversions between every format pair.

| Conversion | Location |
|------------|----------|
| C++ → Proto | `traffic_light.hpp` |
| Proto → `traffic_simulator_msgs` | `traffic_light_publisher.cpp` |
| Proto → `autoware_perception_msgs` | `conversions.hpp` |

Only the C++ → Proto → ROS direction is implemented. Reverse conversions are not needed.

## Reference

### Traffic Signal ID Format

```
<lanelet_id> [<type>][_detected]
```

- `<type>`: `conventional` or `v2i` (default: `conventional`)
- `_detected`: If specified, targets Detected channel instead of GroundTruth

| Format Example | Type | Detected | Target Channel |
|---------------|------|----------|----------------|
| `34802` | conventional (default) | false (default) | GroundTruth (Conventional) |
| `34802 v2i` | v2i | false (default) | GroundTruth (V2I) |
| `34802 conventional_detected` | conventional (default) | true | Detected (Conventional) |
| `34802 v2i_detected` | v2i | true | Detected (V2I) |
