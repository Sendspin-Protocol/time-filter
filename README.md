# Resonate Time Filter

A Kalman filter-based time synchronization library for maintaining accurate client-server timestamp synchronization with microsecond-level precision.

## Overview

This library implements a reference two-dimensional Kalman filter that tracks clock offset and drift between client and server using NTP-style time message exchange. It provides adaptive synchronization with automatic recovery from network disruptions and clock adjustments.

## Features

- **NTP-style time synchronization** using 4-timestamp message exchange
- **Kalman filter** for optimal offset and drift estimation
- **Adaptive forgetting factor** for quick recovery from clock adjustments
- **Thread-safe** time conversions and time updates
- **Microsecond-level precision** using double-precision arithmetic

## Core API

```cpp
// Constructor
// process_std_dev: Standard deviation of offset process noise in microseconds
// drift_process_std_dev: Standard deviation of drift process noise in µs/s
// forget_factor: Forgetting factor (>1) for recovery from disruptions
// adaptive_cutoff: Fraction of max_error (0-1) that triggers forgetting (default: 0.75)
// min_samples: Minimum samples before adaptive forgetting (default: 100)
ResonateTimeFilter(double process_std_dev, double drift_process_std_dev,
                   double forget_factor, double adaptive_cutoff = 0.75,
                   uint8_t min_samples = 100);

// Update filter with computed offset and uncertainty from NTP exchange
// measurement: ((T2-T1)+(T3-T4))/2 in microseconds
// max_error: ((T4-T1)-(T3-T2))/2 in microseconds
// time_added: Client timestamp when measurement was taken in microseconds
void update(int64_t measurement, int64_t max_error, int64_t time_added);

// Convert between client and server timestamps
int64_t compute_server_time(int64_t client_time) const;
int64_t compute_client_time(int64_t server_time) const;

// Get Kalman offset covariance as a proxy for synchronization accuracy
int64_t get_error() const;
```

## Recommended Values

Based on preliminary experiments, the following constructor parameters provide good synchronization performance:

```cpp
ResonateTimeFilter filter(
    0.01,    // process_std_dev: 0.01 µs offset noise
    0.0,     // drift_process_std_dev: 0.0 µs/s drift noise
    1.001    // forget_factor: 1.001 for adaptive forgetting
    // adaptive_cutoff: 0.75 (use default)
    // min_samples: 100 (use default)
);
```

These values balance tracking responsiveness with stability for typical network conditions.

## Documentation

See [docs/theory.md](docs/theory.md) for detailed mathematical documentation of the Kalman filter implementation and time synchronization protocol.

[![A project from the Open Home Foundation](https://www.openhomefoundation.org/badges/ohf-project.png)](https://www.openhomefoundation.org/)
