#set page(paper: "us-letter", margin: 1.0in)
#set text(size: 11pt, font: "New Computer Modern")
#set heading(numbering: none)
#set math.equation(numbering: "(1)")

= RS05 Motor Notes for Bistable Hook Excitation

This document holds the RS05 motor details that were removed from the main
bistable derivation. The derivation only keeps the actuator velocity-limit
equations needed by the optimization model.

== Motor Parameters

#figure(table(
  columns: 3,
  [*Parameter*], [*Value*], [*Notes*],
  [Rated torque], [1.6 N$dot$m], [Continuous rating with heat sinking],
  [Peak torque], [5.5 N$dot$m], [Short-duration command limit],
  [No-load output speed], [480 RPM $plus.minus$10%], [$omega_("max") approx 50.3$ rad/s at the output shaft],
  [Torque constant $K_t$], [$0.94 "N" dot "m" / A_("rms")$], [Converts Iq current to torque],
  [Gear ratio], [7.75:1], [Planetary reduction],
  [Drive mode], [FOC], [Field-oriented control],
  [Maximum phase current], [$11 A_("pk")$], [Torque is still limited to the peak torque range],
  [CAN bus rate], [1 Mbps], [CAN 2.0B extended frame],
  [Encoder], [14-bit absolute], [Used for snap detection and state feedback],
), caption: [RS05 parameters used by the bistable simulations.])

== Velocity Limit

The 480 RPM value is a shaft angular velocity limit, not a direct frequency
limit. For an oscillating shaft command at frequency $f$ and angular amplitude
$Theta$,
$ v_("peak") = Theta dot 2 pi f <= omega_("max") $

Therefore,
$ Theta_("max")(f) = (omega_("max"))/(2 pi f) $

With a lever arm $r$, the maximum linear stroke is
$ delta_("max")(f) = (omega_("max") r)/(2 pi f) $

== CAN Control Modes

The run mode is selected by writing `run_mode` at index `0x7005` with
Communication Type 18, then enabling the motor with Type 3.

#figure(table(
  columns: 3,
  [*Value*], [*Mode*], [*Use*],
  [0], [Operation], [MIT-style torque/position/velocity command. Set $K_p=K_d=0$ for pure torque feed-forward through $t_("ff")$.],
  [3], [Current], [Direct Iq command through `iq_ref`; no position or speed loop.],
  [5], [Position CSP], [Cyclic synchronous position; useful only if the position trajectory and velocity limit are acceptable.],
), caption: [RS05 control modes relevant to sinusoidal excitation.])

== Streaming A Sine Command

The motor firmware does not provide an onboard sine-wave generator. A controller
must compute each sample and stream it over CAN.

For operation mode, send Type 1 frames with
$ t_("ff")(t) = tau_0 sin(2 pi f t), quad K_p = 0, quad K_d = 0 $

For current mode, write
$ i_q(t) = (tau_0)/(K_t) sin(2 pi f t) $
to `iq_ref` at index `0x7006`. For example, a 2.87 N$dot$m torque amplitude
corresponds to about 3.05 A using $K_t = 0.94$ N$dot$m/A.

== ESP32 Timing

At 1 Mbps CAN, a 29-bit extended frame is roughly 110 bits, or about
110 $mu$s on the bus. Two motors commanded at 500 Hz each require about
1000 frames/s, so the command traffic is roughly 11% bus utilization before
feedback frames. A 1 ms hardware-timer loop on the ESP32 is adequate if WiFi
and Bluetooth are disabled and the CAN/timer work is kept off latency-sensitive
tasks.
