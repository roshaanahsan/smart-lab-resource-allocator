![Smart Lab Resource Allocator](https://img.shields.io/badge/Smart%20Lab%20Resource%20Allocator-Silicon%20to%20Cloud-8f7fe0?style=for-the-badge&labelColor=171129)

> **A fully automated lab equipment checkout counter — RFID authentication, fair FIFO queuing, and real-time deadlock detection via DFS on a resource-allocation graph. All running on an ESP32.**

---

![ESP32](https://img.shields.io/badge/ESP32-Microcontroller-8f7fe0?style=flat-square&logo=espressif&logoColor=white&labelColor=171129)
![C++](https://img.shields.io/badge/C++-Firmware-8f7fe0?style=flat-square&logo=cplusplus&logoColor=white&labelColor=171129)
![RC522](https://img.shields.io/badge/RC522-RFID%20Auth-8f7fe0?style=flat-square&labelColor=171129)
![WiFi](https://img.shields.io/badge/AsyncWebServer-WiFi%20Dashboard-8f7fe0?style=flat-square&logo=wifi&logoColor=white&labelColor=171129)
![NVS](https://img.shields.io/badge/NVS%20Flash-Persistent%20Storage-8f7fe0?style=flat-square&labelColor=171129)
![DSA](https://img.shields.io/badge/DFS%20Cycle%20Detection-Deadlock%20Engine-8f7fe0?style=flat-square&labelColor=171129)

---


<p align="center">
  <img src="assets/posters/poster_hero.png" width="100%" alt="Smart Lab Allocator — Hero"/>
</p>

## Executive Summary

University labs run on paper registers — equipment goes missing, students argue over queues, and nobody detects deadlocks until tempers flare. I engineered a standalone checkout counter on an ESP32 that authenticates students via RFID, manages fair equipment queues in firmware, and runs a DFS cycle-detection algorithm on a live resource-allocation graph after every transaction — surfacing deadlocks on a 16×2 LCD with a buzzer alarm before they become a problem.

---


## System Architecture


<p align="center">
  <img src="assets/posters/poster_how_it_works.png" width="100%" alt="How It Works"/>
</p>

<p align="center">
  <img src="assets/posters/poster_tech_details.png" width="100%" alt="Technical Details"/>
</p>


## The Problem This Solves

| Old Way | This System |
|---|---|
| Paper register — lost, altered, illegible | NVS flash storage — survives power cuts |
| "Who's next?" arguments | FIFO queue per item — enforced in firmware |
| Deadlocks go undetected until conflict | DFS after every transaction — cycle found instantly |
| No visibility into lab state | WiFi dashboard — live status from any device on network |
| Silent handovers — wrong person gets item | Physical presence required to claim reservation |

---

## Technical Stack

### Hardware — Silicon Layer

| Component | Spec | Role |
|---|---|---|
| ESP32 DevKit | 240MHz dual-core, 4MB flash | Brain, web server, NVS storage |
| RC522 RFID | MIFARE Classic, SPI | Student authentication |
| 4×4 Membrane Keypad | GPIO matrix | Serial number entry + navigation |
| 16×2 I²C LCD | 0x27 address | Live system feedback |
| Green / Yellow / Red LEDs | GPIO direct drive | Success / Queued / Deadlock states |
| Active Buzzer | GPIO PWM | Event beeps + deadlock alarm |
| Momentary Push Button | INPUT_PULLDOWN | Full state export to Serial |

### Firmware

![C++](https://img.shields.io/badge/C++-Firmware-8f7fe0?style=flat-square&logo=cplusplus&logoColor=white&labelColor=171129)
![Arduino](https://img.shields.io/badge/Arduino%20Framework-ESP32-8f7fe0?style=flat-square&logo=arduino&logoColor=white&labelColor=171129)
![SPI](https://img.shields.io/badge/SPI-RC522%20Bus-8f7fe0?style=flat-square&labelColor=171129)
![I2C](https://img.shields.io/badge/I²C-LCD%20Bus-8f7fe0?style=flat-square&labelColor=171129)

Custom state machine with 11 UI states, debounced keypad handling, 15-second idle timeout, and full graph rebuild on every borrow/return/queue event.

### WiFi Layer

![AsyncWebServer](https://img.shields.io/badge/ESPAsyncWebServer-Async%20HTTP-8f7fe0?style=flat-square&labelColor=171129)
![JSON](https://img.shields.io/badge/JSON%20API-Live%20Data-8f7fe0?style=flat-square&labelColor=171129)
![CSV](https://img.shields.io/badge/CSV%20Export-Download-8f7fe0?style=flat-square&labelColor=171129)

Runs in AP mode (SSID: `LabSystem`) by default. Any device on the network can view live equipment status, download a CSV report, or inspect the raw resource-allocation graph as text.

---

## Engineering Deep Dive

### The Resource-Allocation Graph

The core data structure is a `graph[25][25]` adjacency matrix where nodes `0–9` represent students and nodes `10–24` represent equipment items.

- **Assignment edge**: `Equipment → Student` (item is held by that student)
- **Request edge**: `Student → Equipment` (student is waiting for that item)

Every borrow, return, and queue operation mutates this graph. DFS runs immediately after.

### DFS Cycle Detection

```cpp
bool dfsCycleDetect(int v, bool* visited, bool* recStack, int* path, int* pathLen) {
  visited[v] = true; recStack[v] = true;
  path[(*pathLen)++] = v;
  for (int u = 0; u < MAX_NODES; u++) {
    if (graph[v][u] == 0) continue;
    if (!visited[u]) {
      if (dfsCycleDetect(u, visited, recStack, path, pathLen)) return true;
    } else if (recStack[u]) {
      path[(*pathLen)++] = u; // back-edge found → deadlock
      return true;
    }
  }
  recStack[v] = false; (*pathLen)--;
  return false;
}
```

When a cycle is found, the path is extracted and scrolled across the LCD — e.g. `05→000000→18→001000→05` — while the red LED fires and the buzzer alarms.

### The Real-World Queue Logic

Unlike academic examples, this system never silently hands over equipment. When a student returns an item and someone is queued, the LCD shows `Next: Student1` — but Azaz must physically scan his card and press `A` to claim it. A third student attempting to borrow the reserved item is rejected: `Reserved for Student1`. This prevents phantom assignments.

### The Octal Trap

Equipment serials were originally written as `001000`, `001001` in C/C++. A leading `0` means **octal (base-8)**. `001000` (octal) = `512` decimal — not `1000`. The system was silently mis-routing equipment lookups. Fix: store serials as plain integers, display with `%06d` format.

---

## Features

- **RFID Authentication** — MIFARE Classic cards, UID-based student lookup
- **FIFO Queue per Item** — circular array, position displayed on LCD
- **Live Deadlock Detection** — DFS cycle detection after every transaction
- **16×2 LCD State Machine** — 11 UI states, idle timeout, scrolling alerts
- **NVS Persistent Storage** — students and equipment survive power cuts
- **WiFi Dashboard** — live HTML status page, JSON API, CSV export
- **LED + Buzzer Feedback** — green/yellow/red per transaction outcome
- **Serial Export Button** — full system state dump on demand
- **Admin Registration** — new RFID cards registered live via keypad

---

## Hardware Gallery

<p align="center">
  <img src="assets/hardware/front.jpeg" width="48%" alt="Front Panel"/>
  <img src="assets/hardware/panel_front.jpeg" width="48%" alt="Panel Front Close-up"/>
</p>

<p align="center">
  <img src="assets/hardware/side.jpeg" width="48%" alt="Side View"/>
  <img src="assets/hardware/back.jpeg" width="48%" alt="Back View"/>
</p>

<p align="center">
  <img src="assets/hardware/panel_back.jpeg" width="48%" alt="Panel Back — Wiring"/>
  <img src="assets/hardware/breadboard_1.jpeg" width="48%" alt="Breadboard Wiring"/>
</p>

<p align="center">
  <img src="assets/hardware/breadboard_2.jpeg" width="96%" alt="Full Breadboard Layout"/>
</p>

---

## Demo

> 📹 Demo video avalible on Linkidin & X — full walkthrough showing RFID scan, queue, deadlock trigger, LCD scroll, WiFi dashboard, and CSV export.

**Demonstration sequence:**
1. Boot — LCD alternates `System Ready!` with IP address
2. Roshaan scans → borrows Oscilloscope `000000` → green LED
3. Azaz scans → same item → yellow LED, queued
4. Roshaan returns → LCD: `Next: Student1`
5. Azaz claims reserved item → green LED
6. Deadlock created → red LED + buzzer + scrolling path `05→000000→18→001000→05`
7. Admin registers new card + web dashboard accessed from phone

---

## Project Context

Built in 2026. Published as part of my engineering portfolio. The goal was to take DSA theory — graphs, queues, cycle detection — off the whiteboard and run it on real hardware solving a genuine problem.

---

## Contact

[![Email](https://img.shields.io/badge/Email-roshaanahsan.pro%40gmail.com-8f7fe0?style=flat-square&logo=gmail&logoColor=white&labelColor=171129)](mailto:roshaanahsan.pro@gmail.com)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-roshaanahsan-8f7fe0?style=flat-square&logo=linkedin&logoColor=white&labelColor=171129)](https://linkedin.com/in/roshaanahsan)
[![GitHub](https://img.shields.io/badge/GitHub-roshaanahsan-8f7fe0?style=flat-square&logo=github&logoColor=white&labelColor=171129)](https://github.com/roshaanahsan)
[![X](https://img.shields.io/badge/X-roshaanahsan-8f7fe0?style=flat-square&logo=x&logoColor=white&labelColor=171129)](https://x.com/roshaanahsan)
