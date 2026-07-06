# Scenario event log (Python reference simulation)

## S1 — MAIN generator failure → transfer to APU

| t [s] | event |
|-------|-------|
| 0.020 | K_MAIN CLOSE |
| 1.092 | K_MAIN OPEN |
| 1.142 | K_APU CLOSE |
| 1.022 | bus < 18 V until 1.143 s (120 ms outage) |

## S2 — Cascading failure: MAIN → APU → battery

| t [s] | event |
|-------|-------|
| 0.020 | K_MAIN CLOSE |
| 1.092 | K_MAIN OPEN |
| 1.142 | K_APU CLOSE |
| 3.091 | K_APU OPEN |
| 3.141 | K_BATT CLOSE |
| 1.022 | bus < 18 V until 1.143 s (120 ms outage) |
| 3.022 | bus < 18 V until 3.142 s (120 ms outage) |

## S3 — MAIN recovery → qualified retransfer

| t [s] | event |
|-------|-------|
| 0.020 | K_MAIN CLOSE |
| 1.092 | K_MAIN OPEN |
| 1.142 | K_APU CLOSE |
| 4.124 | K_APU OPEN |
| 4.174 | K_MAIN CLOSE |
| 1.022 | bus < 18 V until 1.143 s (120 ms outage) |
| 4.136 | bus < 18 V until 4.174 s (38 ms outage) |

## S4 — 30 ms sag + load step: no nuisance transfer

| t [s] | event |
|-------|-------|
| 0.020 | K_MAIN CLOSE |
| 1.022 | bus < 18 V until 1.042 s (20 ms outage) |

## S5 — Why break-before-make

Naive 30 ms make-before-break overlap during retransfer drives a peak of **37 A into the battery** (negative source current = back-feed). The BBM controller never draws reverse current: min battery current 0.00 A.
