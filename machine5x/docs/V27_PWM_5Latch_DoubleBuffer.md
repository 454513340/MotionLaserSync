# V27 ECI3808 PWM / Five-Axis Latch / Double-Buffer Design

## 1. Purpose

V27 targets the current commissioning state: XYZ scales are not installed yet, while C/B already have feedback. The project intentionally assumes for the current test that the ECI3808 continuous REGIST path can capture XYZ command position at the same IN0/R0 event as C/B feedback. This is an explicit commissioning assumption to be verified on the target controller; it is not presented as a generic guarantee for every ECI firmware.

The design goal is to keep trigger-time work in hardware and move non-time-critical work out of interrupts.

## 2. Timing path

```text
PC hardware-capture button
  -> laser receiver armed
  -> new TABLE session while PWM is OFF
  -> all five REGIST(104,...) FIFOs armed
  -> 20 Hz PWM probe on OUT14
       -> same electrical edge to scanCONTROL Trigger and ECI IN0
       -> exactly one raw five-axis latch
  -> PC verifies exactly 1 Profile + 1 Pose
  -> PC writes enable=1
  -> BASIC switches PWM0 to 200 Hz
```

Normal scan trigger time is therefore defined by PWM hardware and the IN0 latch event, not by BASIC loop timing.

## 3. Double buffer

### Buffer A — continuous latch FIFOs

Five independent continuous REGIST tables are armed on the same R0 input. Each has one count word plus 128 positions. Buffer A can absorb about 640 ms at 200 Hz if BASIC is temporarily delayed.

```text
X  950..1078
Y 1079..1207
Z 1208..1336
C 1337..1465
B 1466..1594
```

BASIC only consumes a raw index after all five count values agree. A persistent count skew is a scan fault.

### Buffer B — PC-facing interleaved ring

```text
header 330..349
data   370..945
capacity = 96 records
record = [ordinal,X,Y,Z,C,B]
```

Buffer B is a single-producer/single-consumer ring. BASIC publishes `writeIndex` only after all six record words are complete. PC writes `readIndex` only after a complete batch read succeeds. This prevents half-record publication and prevents silent overwrite when PC stalls.

## 4. Header

The first 20 field positions remain compatible with the V22 layout:

| Offset | Owner | Meaning |
|---:|---|---|
|0|BASIC|capacity=96|
|1|BASIC|writeIndex|
|2|PC|readIndex|
|3|BASIC|fault|
|4|PC/BASIC|enable: 0 stop, 1 continuous, 2 probe|
|5|BASIC|published pose count|
|6|BASIC|confirmed latch/trigger count|
|7|BASIC|magic=270909|
|8|BASIC|record words=6|
|9|BASIC|continuous period=5 ms|
|10|PC|session request|
|11|BASIC|session acknowledge|
|12|PC|heartbeat|
|13|BASIC|feedbackMask|
|14|BASIC|timing=4 HardwareLatched|
|15|BASIC|latchMask=31|
|16|BASIC|commandMask|
|17|BASIC|latch input=0|
|18|BASIC|trigger output=14|
|19|BASIC|REGIST mode=4|

For the current ATYPE 1/1/1/4/4 setup, `feedbackMask=24`, `commandMask=7`, `latchMask=31`. After XYZ are changed to ATYPE=4, BASIC automatically publishes 31/0/31.

## 5. Probe rationale

Starting 200 Hz immediately makes it impossible for the host to prove that its first received laser profile corresponds to controller ordinal zero if a profile is lost at startup. V27 therefore uses a low-rate 20 Hz PWM probe. BASIC polls the raw latch count in a normal task and stops PWM after the first latch, leaving about 50 ms before a possible second probe pulse. The PC refuses to continue unless it has exactly one profile and one pose.

## 6. Stop and tail drain

PC writes `enable=0`. BASIC disables PWM first, then enters a stop/drain state and copies any raw FIFO samples already frozen before the PWM shutoff into Buffer B. The TABLE reader continues during this tail phase. The laser receiver is stopped only after the profile count stabilizes, then queued callbacks are delivered and final counts are checked.

## 7. Fault policy

Synchronization faults are fail-closed: PWM is disabled, capture enable is cleared, the fault code is latched, and `RAPIDSTOP(2)` is issued. Important faults include Buffer-B near-full, session mutation during capture, stale PC heartbeat, persistent five-axis raw count skew, multi-pulse startup probe, invalid PC read index, and ordinal overflow.

This is deliberately conservative: a scan with uncertain pose/profile correspondence is diagnostic only and must not be saved as a valid reconstruction.

## 8. XYZ scale upgrade

When XYZ scales are physically installed:

1. wire and verify encoder signals;
2. change X/Y/Z `ATYPE` from 1 to 4;
3. verify/set `ENCODER_RATIO` and direction;
4. verify `DPOS` and `MPOS` track correctly at low speed.

No TABLE ABI or upper-computer code change is required. `SourceMask()` derives the new feedback/command masks from ATYPE.

## 9. What still requires machine testing

- ZDevelop compilation and RAM/ROM size on ECI3808 2.96;
- actual OUT14/PWM0 polarity and pulse width;
- the electrical edge corresponding to REGIST mode 3 versus 4;
- OUT14 -> sensor Trigger + IN0 wiring and common-ground/isolation requirements;
- current commissioning assumption that ATYPE=1 XYZ continuous REGIST returns the desired latched command position on this exact ECI3808 firmware;
- profile loss/reorder behavior under 200 Hz, five-axis interpolation and network load.
