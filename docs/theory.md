# Clock Synchronization with Kalman Filters and NTP-Style Time Messages

The Resonate time synchronization system uses an NTP-style time message exchange combined with a two-dimensional Kalman filter to maintain accurate client-server timestamp synchronization. The system not only computes the offset between client and server clocks but also tracks and compensates for clock drift, using microsecond-level precision. We start, in Section 1, with a description of the NTP-style time messages. Then, we describe the two-dimensional Kalman filter in Section 2. Next, we show how the filter's state updates with new messages and describes how it uses an adaptive forgetting factor to quickly adapt when the client clock is drastically inaccurate due to external conditions. In section 4, we give formulas for converting timestamps between the two time domains.
## 1. NTP-Style Time Message Exchange Protocol

### 1.1 Message Exchange Flow

The system follows the classic NTP four-timestamp model:

1. **T₁ (client_transmitted)**: Client sends a time request message, recording the transmission timestamp
2. **T₂ (server_received)**: Server receives the message and records the receipt timestamp
3. **T₃ (server_transmitted)**: Server sends a response with both T₂ and T₃ timestamps
4. **T₄ (client_received)**: Client receives the response and records the receipt timestamp

### 1.2 Offset and Delay Calculation

The system computes:

**Offset Calculation:**

$$\text{offset} = \frac{(T_2 - T_1) + (T_3 - T_4)}{2}$$

This formula derives from the fundamental equations:
- $T_2 = T_1 + \text{offset} + \text{forward\_delay}$
- $T_4 = T_3 + \text{offset} - \text{backward\_delay}$

Assuming symmetric delays (forward_delay ≈ backward_delay), solving these equations yields the offset formula.

**Round-Trip Delay Calculation:**

$$\text{delay} = (T_4 - T_1) - (T_3 - T_2)$$

$$\text{max\_error} = \frac{\text{delay}}{2}$$

The delay represents the total network round-trip time minus the server processing time. The maximum error is half this delay, representing the worst-case uncertainty when network delays are asymmetric.

## 2. Kalman Filter Architecture

### 2.1 State Variables

The Kalman filter tracks a two-dimensional state vector:

$$\mathbf{x} = \begin{bmatrix} \text{offset} \\ \text{drift} \end{bmatrix}$$

Where:
- **offset**: The current timestamp offset between client and server
- **drift**: The rate of change of the offset (clock drift rate)

### 2.2 Covariance Matrix

The system maintains a 2×2 covariance matrix:

$$\mathbf{P} = \begin{bmatrix}
\sigma^2_{\text{offset}} & \sigma_{\text{offset,drift}} \\
\sigma_{\text{offset,drift}} & \sigma^2_{\text{drift}}
\end{bmatrix}$$

Where:
- $\sigma^2_{\text{offset}}$: Variance of the offset estimate
- $\sigma^2_{\text{drift}}$: Variance of the drift estimate
- $\sigma_{\text{offset,drift}}$: Covariance between offset and drift

## 3. Kalman Updates

### 3.1 Initialization Phase

**First Update (count = 0):**
- Sets initial offset directly from measurement: $\text{offset}_0 = z_0$
- Initializes offset covariance from measurement variance: $\sigma^2_{\text{offset},0} = \sigma^2_{\text{measurement}}$
- Sets drift to zero: $\text{drift}_0 = 0$

**Second Update (count = 1):**
- Computes initial drift:
  $$\text{drift}_1 = \frac{z_1 - \text{offset}_0}{\Delta t}$$
- Estimates drift covariance:
  $$\sigma^2_{\text{drift},1} = \frac{\sigma^2_{\text{offset},0} + \sigma^2_{\text{measurement},1}}{(\Delta t)^2}$$

### 3.2 Kalman Filter Prediction Step

**State Prediction:**

$$\hat{\mathbf{x}}_{k|k-1} = \mathbf{F} \mathbf{x}_{k-1|k-1}$$

Where the state transition matrix is:

$$\mathbf{F} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}$$

This yields:
$$\hat{\text{offset}}_{k|k-1} = \text{offset}_{k-1} + \text{drift}_{k-1} \cdot \Delta t$$

**Covariance Prediction:**

The prediction equation follows the standard Kalman filter formulation:

$$\mathbf{P}_{k|k-1} = \mathbf{F} \mathbf{P}_{k-1|k-1} \mathbf{F}^T + \mathbf{Q}$$

Expanding the matrix multiplication:

$$\sigma^2_{\text{offset},k|k-1} = \sigma^2_{\text{offset},k-1} + 2\sigma_{\text{offset,drift},k-1}\Delta t + \sigma^2_{\text{drift},k-1}\Delta t^2 + q_{\text{offset}}\Delta t$$

$$\sigma_{\text{offset,drift},k|k-1} = \sigma_{\text{offset,drift},k-1} + \sigma^2_{\text{drift},k-1}\Delta t$$

$$\sigma^2_{\text{drift},k|k-1} = \sigma^2_{\text{drift},k-1} + q_{\text{drift}}\Delta t$$

The process noise includes two independent components:
- $q_{\text{offset}}\Delta t$ accounts for clock jitter and short-term instabilities
- $q_{\text{drift}}\Delta t$ accounts for clock frequency wander and long-term drift variations

### 3.3 Measurement Update Step

**Innovation/Residual:**

$$y_k = z_k - \mathbf{H}\hat{\mathbf{x}}_{k|k-1} = z_k - \hat{\text{offset}}_{k|k-1}$$

Where $\mathbf{H} = [1, 0]$ is the observation matrix (we only observe offset, not drift directly).

**Innovation Covariance:**

$$S_k = \mathbf{H}\mathbf{P}_{k|k-1}\mathbf{H}^T + R_k = \sigma^2_{\text{offset},k|k-1} + \sigma^2_{\text{measurement},k}$$

**Kalman Gain:**

$$\mathbf{K}_k = \mathbf{P}_{k|k-1}\mathbf{H}^T S_k^{-1} = \begin{bmatrix}
\frac{\sigma^2_{\text{offset},k|k-1}}{S_k} \\
\frac{\sigma_{\text{offset,drift},k|k-1}}{S_k}
\end{bmatrix}$$

**State Update:**

$$\mathbf{x}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k y_k$$

Which expands to:
- $\text{offset}_{k|k} = \hat{\text{offset}}_{k|k-1} + K_{\text{offset},k} \cdot y_k$
- $\text{drift}_{k|k} = \text{drift}_{k-1} + K_{\text{drift},k} \cdot y_k$

**Covariance Update:**

Using the simplified form:

$$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k\mathbf{H})\mathbf{P}_{k|k-1}$$

This yields:
- $\sigma^2_{\text{offset},k|k} = \sigma^2_{\text{offset},k|k-1} - K_{\text{offset},k} \cdot \sigma^2_{\text{offset},k|k-1}$
- $\sigma^2_{\text{drift},k|k} = \sigma^2_{\text{drift},k|k-1} - K_{\text{drift},k} \cdot \sigma_{\text{offset,drift},k|k-1}$
- $\sigma_{\text{offset,drift},k|k} = \sigma_{\text{offset,drift},k|k-1} - K_{\text{drift},k} \cdot \sigma^2_{\text{offset},k|k-1}$

### 3.4 Adaptive Forgetting Factor

The adaptive forgetting mechanism operates in two phases to balance initial stability with responsiveness:

During an initial stabilization period, the filter accumulates measurements without applying forgetting. This builds a reliable baseline before enabling aggressive adaptation.

After stabilization, when the residual exceeds a configurable adaptive cutoff percentage $c$ of the max_error:

$$\text{if } |y_k| > c \cdot \text{max\_error}$$

The system applies a forgetting factor $\lambda^2$ to all covariances:

$$\mathbf{P}_{k|k-1} \leftarrow \lambda^2 \cdot \mathbf{P}_{k|k-1}$$

This adaptive mechanism allows the filter to:
- Build stable initial estimates without premature forgetting
- Quickly adapt to sudden time jumps due to the server or client clock drift rate changing
- Maintain stability during normal operation
- Balance between trusting historical estimates and new measurements

## 4. Time Conversion Functions

### 4.1 Client to Server Conversion

$$T_{\text{server}} = T_{\text{client}} + \text{offset} + \text{drift} \cdot (T_{\text{client}} - T_{\text{last\_update}})$$

### 4.2 Server to Client Conversion

Starting from the forward equation:
$$T_{\text{server}} = T_{\text{client}} + \text{offset} + \text{drift} \cdot (T_{\text{client}} - T_{\text{last\_update}})$$

Rearranging:
$$T_{\text{server}} = (1 + \text{drift}) \cdot T_{\text{client}} + \text{offset} - \text{drift} \cdot T_{\text{last\_update}}$$

Solving for $T_{\text{client}}$:
$$T_{\text{client}} = \frac{T_{\text{server}} - \text{offset} + \text{drift} \cdot T_{\text{last\_update}}}{1 + \text{drift}}$$

## Conclusion

This implementation uses NTP-style time messages and Kalman filtering to accurately synchronize a client's clock. The two-dimensional offset and drift state provides better long-term accuracy compared to offset-only approaches, while the adaptive forgetting factor ensures robustness to underlying clock rate changes.
