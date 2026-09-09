# System Overview

## High-level architecture

```text
                  ┌─────────────────────────────┐
                  │      Primary Controller     │
                  │                             │
IR line sensors ─▶│ Line following              │
Front ultrasonic ─▶│ Obstacle avoidance          │
Side ultrasonic ──▶│ Object detection            │
Gripper servos ──▶│ Object collection/deposit   │
Ground RGB ──────▶│ Deposit-zone detection      │
                  │                             │
                  │     Serial @ 57600 baud     │
                  └─────────────┬───────────────┘
                                │
                                ▼
                  ┌─────────────────────────────┐
                  │   Secondary Controller      │
                  │                             │
                  │ TCS34725 colour sensor      │
                  │ 16x2 LCD status display     │
                  │ Buzzer feedback              │
                  └─────────────────────────────┘
```

## Main software components

| Component | Purpose |
|---|---|
| `Scan()` | Reads the three IR sensors |
| `UpdateDirection()` | Converts line position into steering correction |
| `SenseDistance()` | Detects and avoids front obstacles |
| `SenseSide()` | Detects and approaches recyclable objects |
| `GrabCanSequence()` | Operates the gripper to collect an object |
| `ReturnToTrack()` | Reacquires the line after manipulation |
| `detectColour()` | Detects the ground/deposit-zone colour |
| `DropCanSequence()` | Releases an object at its destination |
| `sendMessage()` | Sends robot state to the secondary controller |
| `readColour()` | Classifies the collected object's RGB colour |

## Communication states

The primary controller sends messages to the secondary controller to coordinate the user-facing status and colour-sensing behaviour.

```text
SEARCHING
    │
    ▼
GRABBING ────────► colour measurement
    │
    ▼
object colour returned
    │
    ▼
line following / ground colour detection
    │
    ▼
DEPOSITING ──────► object released
    │
    ▼
return to track
    │
    └──── repeat

OBJECT IN FRONT ─► warning buzzer

FINISHED ────────► completion melody
```
