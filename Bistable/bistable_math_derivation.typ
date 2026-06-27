#set page(paper: "us-letter", margin: 1.1in)
#set text(size: 11pt, font: "New Computer Modern")
#show heading: set block(above: 1.0em, below: 0.5em)
#set heading(numbering: none)
#set math.equation(numbering: "(1)")


#let title = [
  Bistable Mechanism work
]

#set page(
  header: align(center,title),
numbering: "1")
#counter(page).update(1)
#show heading.where(level: 1): it => {

  pagebreak(weak: true)

  align(center + horizon)[

    #text(size: 28pt, weight: "bold")[

      #if it.numbering != none {

        numbering(it.numbering, ..counter(heading).at(it.location()))

        h(0.5em)

      }

      #it.body

    ]

  ]

  pagebreak()

}

#show heading.where(level: 2): set text(size: 16pt)

#show heading.where(level: 3): set text(size: 14pt)
#show heading.where(level: 4): set text(size: 13pt)
#show heading.where(level: 5): set text(size: 11pt)


#pagebreak()

==== Introduction <sec-intro>

A _bistable_ mechanical system has two stable equilibrium positions
separated by an energy barrier. If it is not disturbed, it stays in whichever
well it already occupies. Moving from one well to the other requires enough
input energy to cross the intervening saddle; the rapid transition is the
_snap-through_ event [@qiu2004; @liu2023].
For the Rolly hook, the first well holds the hook retracted. A RobStride RS05
quasi-direct-drive motor drives the surrounding structure with a sinusoidal
force or displacement. Near resonance, the hook motion grows until the relative
coordinate crosses the saddle. After snap-through the second well holds the hook
deployed without continuous motor power. The model here is therefore concerned
with three quantities: the barrier height, the natural frequency about the
retracted state, and the actuator amplitude needed to reach the saddle.
The physical model derived and simulated here is *Simulation 2 (Physical)*: a
compliant curved-beam bistable mechanism following the geometry of Qiu et al. [@qiu2004]
and the 3D-printed Design 9 of Zolfagharian et al. [@zolfagharian2025], with a companion
two-degree-of-freedom tuned mass damper (TMD) model [@denhartog1956].

==== Variable Map <sec-variable-map>

The MATLAB variable names are mapped to the symbols in this document below.
This is also the convention used in the regenerated figures.

- `h_apex` = $h$: undeformed beam apex height.
- `t_beam` = $t$: beam thickness.
- `l_span` = $l$: horizontal span between beam end supports.
- `Q_beam` or `Q` = $Q=h/t$: bistability ratio. The scripts require $Q >= 2.35$.
- `n_cells` = $N$: number of parallel bistable beams. Parallel beams share the same displacement and add force/stiffness.
- `d_stroke_1cell` = $d_(f,1)$: stroke of one beam.
- `d_stroke` = $d_f$: mechanism stroke used by the ODE. In the parallel model this equals `d_stroke_1cell`.
- `d_saddle` = $d_s$: unstable saddle displacement; snap-through occurs when $x_("rel")$ crosses this value.
- `F_push_1cell` = $F_("push,1")$: peak force for one beam.
- `F_push_total` = $F_("push,N")$: total pack peak force, $N F_("push,1")$.
- `A_coeff` = $A$: cubic force coefficient in equation @eq-Fext-cubic.
- `k_eff_s1` = $k_("eff,1")$: small-signal stiffness about State 1.
- `m_hook` = $m_h$: hook/reference mass.
- `M_plate` = $M_P$: rigid plate, motor housing, and external-block mass.
- `xP` = $x_P$: absolute plate displacement.
- `xr` or `x_rel` = $x_("rel")$: hook displacement relative to the plate.
- `A_min_c2` = $A_("min")$: minimum prescribed plate amplitude for Configuration 2.
==== Compliant Curved-Beam Bistable Mechanism <sec-sim2>

===== Beam Geometry - Qiu Cosine Profile <ssec-geometry>

Qiu et al. [@qiu2004] proposed the initial (stress-free) shape of the
curved beam as the first buckling mode of a fixed-end Euler beam, a
cosine profile:
$ y_0(xi) = (h)/(2)[1 - cos((2 pi xi)/(l))],
    0  <=  xi  <=  l $ <eq-beamprofile>

where $h$ is the apex height [m] and $l$ is the horizontal span [m]. The beam
thickness is $t$ [m], and the material elastic modulus is $E$ [Pa].
Key geometric parameters:
$ Q &= (h)/(t)  "(apex-height-to-thickness ratio)" $ <eq-Qdef>

$ I &= (t^3)/(12)  "(second moment of area per unit width)" $ <eq-Idef>

====== Bistability Condition <ssec-bistability-condition>

For a curved beam to exhibit bistability, two design conditions matter most
[@qiu2004; @zolfagharian2025]:
+ *Q-ratio condition*: The apex height must substantially exceed
the beam thickness:
$ Q = (h)/(t)  >=  2.35 $ <eq-Qcond>

Qiu et al. report that a second-mode-constrained curved beam becomes
bistable once this geometric ratio is large enough; their analysis and
force-displacement plots put the threshold at about $Q=2.35$
[@qiu2004]. The MATLAB model now enforces $Q >= 2.35$ directly.
+ *Mode-2 suppression*: For a single beam, buckling mode 2 is
asymmetric and prevents robust bistability. In the Zolfagharian Design 9, a
_double-curved beam_ connected by a central membrane at mid-span
suppresses mode 2, so the snap-through path follows the symmetric
constrained-beam behavior described by Qiu et al. [@qiu2004].

===== Force-Displacement Relationship <ssec-FD>

====== Strain Energy Formulation

Using the Euler-Bernoulli beam assumptions, let the initial shape be
$y_0(xi)$ and the transverse displacement from that shape be $w(xi)$, so
$y(xi) = y_0(xi) - w(xi)$ where $w$ is the transverse displacement
relative to the initial shape. The bending strain energy is then
[@qiu2004]:
$ U_b = (E I)/(2) integral_0^l ((dif^2 w)/(dif xi^2))^2 dif xi $ <eq-Ub>

The axial compressive strain energy (due to shortening of the beam's projected
length) is:
$ U_a = (E A)/(2l)(integral_0^l (dif y_0)/(dif xi)
        (dif w)/(dif xi) dif xi)^2 $ <eq-Ua>

where $A = t dot  b$ is the cross-sectional area (for width $b$).
The force-displacement relationship follows from virtual work:
$F = partial(U_b + U_a)/partial d$, where $d$ is the mid-point displacement
(snapping direction).
====== Normalized Force-Displacement

Qiu et al. [@qiu2004] show that the force-displacement curve depends
only on $Q$ and the normalized displacement $macron(d) = d/h$. For the
double-beam (mode-2 suppressed) configuration, the curve has the
characteristic S-shape:
- *State 1* ($macron(d) = 0$): zero force (equilibrium).
- *Peak push force* $F_("push")$ at $macron(d)  approx  0.27$:
maximum force required to initiate snap-through.
- *Saddle* ($macron(d) = macron(d)_s$): force returns to zero
(unstable equilibrium).
- *Valley force* $F_("pop") < 0$ at $macron(d)  approx  0.82$:
mechanism pulls itself toward State 2.
- *State 2* ($macron(d) = 1$): zero force (second equilibrium).

====== Cubic Polynomial Fit <sssec-cubic>

The simulation uses a cubic fit for the applied force in the compression-test
convention. This is not a replacement for Qiu's modal solution or
Zolfagharian's FEA; it is a compact surrogate with the correct equilibrium
points and peak force:
$ F_("ext")(d) = A d (d - d_s) (d - d_f) $ <eq-Fext-cubic>

The coefficient $A$ is calibrated so the force at the chosen peak location
matches the Design 9 force level from Zolfagharian et al. [@zolfagharian2025]:
$ A = (F_("push"))/(d_("push") (d_("push") - d_s) (d_("push") - d_f)) $ <eq-Acoeff>

where $d_("push")  approx  0.27 d_f$ is the location of the force peak.
The _internal restoring force_ on an attached mass (Newton's third law)
is:
$ F_("restore")(d) = -F_("ext")(d) = -A d (d - d_s) (d - d_f) $ <eq-Frestore2>

===== Potential Energy (Curved-Beam Model) <ssec-potential2>

Integrating the restoring force analytically:
$ V(d) &= -integral_0^d F_("restore")(xi) dif xi
         = A integral_0^d xi (xi-d_s) (xi-d_f) dif xi $

$ &= A integral_0^d [xi^3 - (d_s+d_f)xi^2 + d_s d_f xi]dif xi $

$ &= A[(d^4)/(4) - ((d_s+d_f))/(3)d^3 + (d_s d_f)/(2)d^2] $ <eq-V2>

The energy barrier from State 1 to the saddle is:
$ Delta E_1 = V(d_s) - V(0)
  = A[(d_s^4)/(4) - ((d_s+d_f))/(3)d_s^3 + (d_s d_f)/(2)d_s^2] $ <eq-barrier2>

===== Natural Frequencies at Each Stable State <ssec-natfreq2>

Linearizing $F_("restore")(d)$ about each equilibrium:
*State 1* ($d = 0$):
$ k_("eff,1") = .-(dif F_("restore"))/(dif d)|_("d=0")
  = A d_s d_f $ <eq-keff1>

$ omega_("n,1") = sqrt((k_("eff,1"))/(m)),  
  f_("n,1") = (1)/(2 pi)sqrt((A d_s d_f)/(m)) $ <eq-fn1>

*State 2* ($d = d_f$):
$ k_("eff,2") = .-(dif F_("restore"))/(dif d)|_("d=d_f")
  = A d_f (d_f - d_s) $ <eq-keff2>

$ omega_("n,2") = sqrt((k_("eff,2"))/(m)),  
  f_("n,2") = (1)/(2 pi)sqrt((A d_f (d_f - d_s))/(m)) $ <eq-fn2>

Since $d_s < d_f$ by definition, and typically $d_s  approx  0.55 d_f$:
$k_("eff,2") = A d_f(d_f-d_s)  approx  0.45 A d_f^2$, while
$k_("eff,1") = A d_s d_f  approx  0.55 A d_f^2$, so
$k_("eff,2") < k_("eff,1")$ and thus $f_("n,2") < f_("n,1")$.
===== Parallel Beam-Pack Mechanics <ssec-stacked>

The current robot layout is treated as a parallel pack of identical curved
beams. Each beam sees the same apex displacement, so the travel does not add:
$ d_("f,N") &= d_("f,1")  "(mechanism stroke)" $ <eq-stackstroke>

$ d_("s,N") &= d_("s,1")  "(saddle position)" $ <eq-stacksaddle>

The forces add across the pack:
$ F_("push,N") = N F_("push,1") $ <eq-FpushN>

The cubic coefficient therefore scales linearly:
$ A_N = N A_1 $ <eq-AN>

and the State-1 stiffness becomes:
$ k_("eff,1,N") = A_N d_s d_f = N k_("eff,1,1") $ <eq-keffN>

Thus adding parallel beams raises stiffness, stored energy, and natural
frequency:
$ f_("n,1,N") = sqrt(N) f_("n,1,1") $ <eq-fnN>

This is the source of the earlier displacement error: a series-cell assumption
multiplied the stroke and saddle travel, while the physical beam pack should
share displacement and add force.
===== 1-DOF Hook Snap-Through Model <ssec-1dof>

The hook mass $m$ is connected to the fixed robot body through the bistable
mechanism. External vibration $F_("exc")(t)$ is transmitted through
the robot structure. The equation of motion is:
$ m dot.double(d) + c dot(d) + F_("restore")(d) = F_("exc")(t) $ <eq-1dof2>

where $c = 2 zeta m omega_("n,1")$. The excitation is a linear chirp
(frequency-modulated sinusoid):
$ F_("exc")(t) = F_0 sin[2 pi(f_("lo") t
    + (f_("hi") - f_("lo"))/(2t_f)t^2)] $ <eq-chirp>

where $f_("lo")$, $f_("hi")$ are the chirp bounds [Hz] and
$t_f$ is the simulation duration [s]. As the instantaneous frequency passes
through $f_("n,1")$, resonance amplifies the displacement until snap-through.
===== 2-DOF Tuned Mass Damper Model <ssec-2dof>

Following Zolfagharian et al. [@zolfagharian2025] (their Figure 4), the
main structure mass $m_1$ is supported by spring $k_("main")$ and
driven harmonically. The TMD mass $m_2$ is attached to the same flexible ruler
at distance $L_2$ from the fixed end, where the bistable mechanism
repositions it.
The coupled equations of motion are:
$ m_1 dot.double(x)_1 + c_("main") dot(x)_1 + k_("main")x_1
    + k_b(x_1 - x_2) + c_b(dot(x)_1 - dot(x)_2) &= F_0 sin(omega t) $ <eq-2dof1>

$ m_2 dot.double(x)_2 - k_b(x_1 - x_2) - c_b(dot(x)_1 - dot(x)_2) &= 0 $ <eq-2dof2>

where $k_b$ and $c_b$ are the stiffness and damping of the bistable TMD
connection, linearized at the current stable state.
==== Stiffness of the Flexible Ruler

The effective spring stiffness of a flexible cantilever beam (ruler) at
distance $L$ from the fixed support is [@zolfagharian2025]:
$ k = (3E I_r)/(L^3) $ <eq-rulerspring>

where $E_r$ is the ruler's elastic modulus, $I_r$ is its second moment of area.
When the bistable mechanism shifts the TMD mass from $L_1$ to $L_2 = L_1 + d_f$:
$ k_("TMD,2") = (3E_r I_r)/(L_2^3)
                     = (3E_r I_r)/((L_1 + d_f)^3)
                     < k_("TMD,1") $ <eq-kTMDshift>

This reduction in TMD stiffness shifts the TMD natural frequency downward,
changing the frequency at which it suppresses main-mass vibration.
==== Steady-State Frequency Response

In the frequency domain, the steady-state response of $x_1$ to
$F_0 e^(i omega t)$ is $X_1 e^(i omega t)$. The impedance formulation gives:
$ mat((Z_("11"), Z_("12")), (Z_("21"), Z_("22")))
  mat((X_1), (X_2)) =
  mat((F_0), (0)) $ <eq-impedance>

where
$ Z_("11") &= (k_("main") + k_b - m_1 omega^2) + i omega(c_("main") + c_b) $

$ Z_("12") &= Z_("21") = -(k_b + i omega c_b) $

$ Z_("22") &= (k_b - m_2 omega^2) + i omega c_b $

Solving by Cramer's rule:
$ X_1 = (F_0 Z_("22"))/(Z_("11")Z_("22") - Z_("12")^2) $ <eq-X1>

The amplitude $|X_1(omega)|$ vs. $omega/(2 pi)$ [Hz] is plotted for three
configurations: no TMD, TMD in State 1, TMD in State 2.
==== Base Excitation via Motor Torque Oscillation Through Shared Structure <sec-baseexcit>

===== Physical Setup <ssec-setup-base>

====== Mechanism component chain

The Zolfagharian Design 9 bistable mechanism consists of the following rigid
bodies connected in series:
+ *Robot chassis / ground* - the fixed reference frame.
+ *Spring isolators* ($k_s$, $c_s$) - soft mounting elements
(rubber mounts, vibration isolators) between the chassis and the plate
assembly.
+ *Rigid plate assembly* ($M_P$) - one rigid body comprising the
plate, the motor housing, and the _external blocks_.  The external
blocks are the side walls to which the bistable beam ends are pinned.
+ *Bistable curved beams* - the compliant Qiu cosine-profile beams
spanning between the external blocks.  These provide the nonlinear
restoring force $F_("restore")(x_("rel"))$ and potential
energy $V(x_("rel"))$.
+ *Hook* ($m_h$, also called the _reference mass_) - the
deployable element attached at the beam apex.  In State 1 the beam is
arched and the hook is retracted; in State 2 the beam has snapped through
and the hook is deployed.

The hook is connected to the plate assembly _only_ through the
bistable beams - there is no rigid link.  When the plate assembly vibrates,
the hook cannot follow instantaneously; the resulting *relative displacement* $x_("rel")$ between the beam apex (hook) and the beam ends
(external blocks on plate) is the quantity that drives snap-through.
#figure(```text
  ROBOT CHASSIS
        |
        |  k_s, c_s   <- spring isolators / rubber feet
        |
  RIGID ASSEMBLY  (M_P = motor + plate + external blocks)
        |                  external blocks = beam end pinned supports
        |
      BEAM      BEAM    <- bistable curved beams (Qiu cosine profile)
        |          |
        +----+-----+
             |
           HOOK  (m_h = reference mass at beam apex)
```, caption: [Physical component chain for the Zolfagharian bistable deployment mechanism.]) <fig-chain>

===== Two excitation configurations

*Configuration 1 - Motor on plate assembly.*

The motor housing is bolted directly to the plate (part of $M_P$).  The
motor shaft drives an eccentric mass or proof mass whose reaction force
enters the plate through the bearing.  The plate assembly vibrates; the hook
responds via base excitation through the beams.
$ "Motor"   ->   "Plate" + "External blocks"   ->  
"Beams"   ->   "Hook" $

*Configuration 2 - Motor off plate, drives via bearing.*

The motor housing is _not_ part of the plate assembly.  The motor output
shaft drives the plate+external-blocks assembly directly through a bearing
coupling (lever arm, crank, or shaft).  In this configuration the motor can
_prescribe_ the plate displacement:
$ x_P(t) = A sin(omega_("n,1") t) $

The hook responds to this base excitation through the beams.  The motor
housing mass is excluded from $M_P$.
$ "Motor"   ->   "Bearing"   ->  
"Plate" + "External blocks"   ->   "Beams"   ->   "Hook" $

Configuration 2 avoids the tuned-mass-damper anti-resonance problem because the
motor enforces the plate amplitude
regardless of the hook's dynamics.
===== Coordinates

Two coordinate systems are used throughout:
- $x_P(t)$: absolute displacement of the rigid plate assembly [m]
(positive in the snap direction, i.e. from State 1 toward State 2)
- $x_H(t)$: absolute hook displacement [m]
- $x_("rel") (t)  equiv  x_H(t) - x_P(t)$: hook displacement
_relative to the plate assembly_ [m] - this is the quantity that
enters the bistable restoring force and potential energy

Snap-through occurs when $x_("rel")$ crosses the saddle point
$d_s$ with positive velocity.
== Motor Torque Excitation Force <ssec-torqueexcit>

In torque/current control mode the RS05 applies a sinusoidal output torque
$tau_("out") (t)$.  Through a coupling of moment arm $r_("arm")$
[m] this becomes a linear force on the plate:
$ F_("motor")(t) = (tau_("out"))/(r_("arm")) sin(omega_d t)
   equiv  F_0 sin(omega_d t) $ <eq-Ftorque>

where $tau_("out")$ is the *measured output torque* read from the
motor's torque-speed (T-N) curve at the instantaneous operating condition, and
$omega_d$ is the drive frequency set by the controller command.  The amplitude
$F_0$ is therefore _independent of frequency_ - a key difference from a
rotating-unbalance source whose amplitude grew as $omega^2$.
For a linear chirp sweep from $f_("lo")$ to $f_("hi")$ over total
time $t_f$, the instantaneous frequency and accumulated phase are:
$ f_("inst") (t) &= f_("lo") + (f_("hi") - f_("lo"))/(t_f) t $ <eq-finst>

$ phi(t) &= 2 pi[f_("lo") t
    + (f_("hi") - f_("lo"))/(2t_f) t^2] $ <eq-phi>

The torque-driven force during a chirp is:
$ F_("motor")(t) = F_0 sin(phi(t)) $ <eq-FtorqueChirp>

The amplitude envelope $F_0$ is constant throughout the sweep, so the force
delivered at $f_("n,1")$ equals the force at any other frequency in the band -
the mechanism receives full excitation amplitude at every point in the chirp.
== Gearbox Effects on Vibration Transmission <ssec-gearbox>

Any gearbox in the drivetrain between the motor rotor and the bistable plate
acts as a mechanical filter.  The gear ratio $n_g$ amplifies the rotor's
reflected inertia and creates a low-pass cut-off frequency above which
vibrations are attenuated before reaching the plate.  Two architectures
relevant to bistable excitation are treated below.
===== Planetary Gearboxes <sssec-planetary>

A planetary gearbox achieves its ratio $n_g$ through a sun-planet-ring
arrangement.  Typical ratios range from 3:1 to about 20:1 per stage; multi-stage
units reach 100:1.  QDD motors use a single low-ratio planetary stage
($n_g  <=  10$) specifically to preserve backdrivability and broadband force
transmission.
*Reflected rotor inertia.*  At the output shaft, the effective inertia is:
$ J_("eff") = J_("rotor") n_g^2 + J_("gears") + J_("output") $ <eq-Jeff>

The $n_g^2$ factor dominates: even a modest 5:1 ratio makes the rotor inertia
appear $25 times$ larger at the output.  Combined with the finite gear-mesh
stiffness $k_("gear")$, this creates a mechanical low-pass cut-off
frequency:
$ f_("cut")  approx  (1)/(2 pi)sqrt((k_("gear"))/(J_("eff"))) $ <eq-fcut>

Vibrations at frequencies above $f_("cut")$ are attenuated before
reaching the plate; those well below $f_("cut")$ pass through with
little attenuation.  A low gear ratio keeps $f_("cut")$ high, so the
motor remains substantially backdrivable and transmits a broad vibration
band - a favorable property for this application.
*Design rule:* verify that $f_("n,1") < f_("cut")$ so that the
bistable resonance lies in the pass-band of the actuator's mechanical filter.
This check should use the actual mounted motor and linkage geometry, not the
bare motor datasheet value.
*Gear-mesh frequency.*  The planet-gear tooth contacts produce a
secondary vibration at:
$ f_("mesh") = n_g f_("rotor") N_("teeth") $ <eq-fmesh>

where $N_("teeth")$ is the planet tooth count.  If $f_("mesh")$
happens to coincide with $f_("n,1")$, this harmonic can contribute beneficially to
snap-through excitation without any additional control effort.  A low single-stage
ratio keeps $f_("mesh")$ well above the bistable resonance band, avoiding
unintended parametric excitation.
===== High-Ratio Straight-Tooth Gearboxes <sssec-highratiospur>

Multi-stage spur or helical gearboxes achieve ratios of $n_g = 20$:1 to
$n_g > 100$:1 by cascading two or more stages of spur (straight-tooth) or
helical gears.  They are inexpensive and compact relative to harmonic or
cycloidal alternatives at the same ratio, but carry two significant penalties
for bistable vibration excitation.
*Reflected inertia penalty.*  Because $n_g$ is large, the $n_g^2$ term
in equation @eq-Jeff dominates overwhelmingly.  The cut-off frequency $f_("cut")$
drops accordingly:
$ f_("cut")   prop   (1)/(n_g)
    (J_("eff")  approx  J_("rotor") n_g^2) $ <eq-fcut-highrat>

At $n_g = 50$, a motor with $f_("cut") = 500$ Hz at $n_g = 1$ drops to
roughly $10$ Hz - potentially _below_ the bistable resonance.  Vibrations
above this cut-off are mechanically filtered; the plate never receives the
high-frequency force content needed for resonant snap-through.
*Backlash dead-band.*  Multi-stage spur gearboxes accumulate backlash
$theta_("bl")$ (typically 5-30 arc-min total) from each mesh.  Through the
coupling lever arm $r$ this creates a linear dead-band:
$ delta_("bl") = theta_("bl") r $ <eq-deadband>

During each oscillation half-cycle the drive must traverse $delta_("bl")$ before
force transmission resumes.  The effective plate amplitude is reduced to:
$ A_("eff") = A_("plate") - delta_("bl") $ <eq-Aeff-bl>

For snap-through, the minimum plate amplitude from equation @eq-Amin-c2 must be met
_net of backlash_: $A_("eff")  >=  A_("min")$, i.e.\
$A_("plate")  >=  2 zeta_h d_s + delta_("bl")$.  If $delta_("bl")$ is
comparable to $d_s$, this constraint can become severe.
*Gear-mesh noise.*  Multi-stage meshes generate harmonics at integer
multiples of $f_("mesh")$ (equation @eq-fmesh).  If any harmonic coincides
with $f_("n,1")$, it can excite unintended snap-through; this is harder to avoid
in a multi-stage box than in a single-stage planetary.
*Design rule.*  High-ratio straight-tooth gearboxes are suitable for
bistable excitation only when $f_("n,1")  <<  f_("cut") (n_g)$, the
accumulated backlash $delta_("bl")  <<  d_s$, and the drive frequency is well
below the first gear-mesh harmonic.  In practice, these conditions are difficult
to satisfy simultaneously at gear ratios above $tilde  20$:1; a low-ratio QDD
or cycloidal stage is strongly preferred.
== Two-Degree-of-Freedom Equations of Motion <ssec-2dof-base>

The plate is modelled as a rigid body on a linear elastic support (stiffness
$k_s$, damping $c_s$). The hook mass $m_h$ is connected to the plate through the
bistable element (restoring force $F_("restore")(x_("rel"))$,
equation @eq-Frestore2) and a viscous damper $c_h$.

//

*Plate equation of motion:*
//

$ M_P dot.double(x)_P
  = F_("motor")(t)
  underbrace(- k_s x_P - c_s dot(x)_P, "support restraint")
  underbrace(- F_("restore")(x_("rel")) + c_h dot(x)_("rel"), "reaction from hook") $ <eq-plateeom>

*Hook equation of motion (absolute frame):*
$ m_h dot.double(x)_H = F_("restore")(x_("rel")) - c_h dot(x)_("rel") $ <eq-hookeom>

The sign convention in equation @eq-plateeom reflects Newton's third law:
the bistable spring pulls the plate with force $-F_("restore")$ (equal and
opposite to what it exerts on the hook), and the dashpot exerts $+c_h dot(x)_("rel")$
on the plate.
===== Relative-Coordinate Formulation

Subtracting equation @eq-plateeom (divided by $M_P$) from equation @eq-hookeom
(divided by $m_h$):
$ dot.double(x)_("rel") = dot.double(x)_H - dot.double(x)_P
  = underbrace((F_("restore") - c_h dot(x)_("rel"))/(m_h), "dot.double(x)_H")
  - underbrace((F_("motor") - k_s x_P - c_s dot(x)_P
      - F_("restore") + c_h dot(x)_("rel"))/(M_P), "dot.double(x)_P") $ <eq-relEOM>

This is the governing equation for the bistable snap.
The full state vector is $bold(y) = [x_P, dot(x)_P, x_("rel"), dot(x)_("rel")]^ -> p$.
The support parameters are defined from the plate natural frequency $f_P$:
$ k_s &= M_P (2 pi f_P)^2 $ <eq-ks>

$ c_s &= 2 zeta_P M_P (2 pi f_P) $ <eq-cs>

where $zeta_P$ is the plate structural damping ratio.
== Plate Transfer Function (Decoupled Analysis) <ssec-plateTF>

For small $x_("rel")$ (early in the simulation, well before snap), the
back-reaction of the bistable on the plate is negligible relative to the dominant
terms $F_("motor")$, $k_s x_P$, and $c_s dot(x)_P$. Under this
approximation, the plate behaves as an independent SDOF oscillator:
$ M_P dot.double(x)_P + c_s dot(x)_P + k_s x_P  approx  F_("motor")(t) $ <eq-plateSDOF>

Assuming harmonic excitation at frequency $omega$ with constant amplitude $F_0$,
the steady-state plate displacement amplitude is:
$ |X_P(omega)|
  = (F_0)/(sqrt((k_s - M_P omega^2)^2
    + (c_s omega)^2)) $ <eq-Xplate>

Substituting equation @eq-ks and equation @eq-cs and defining the frequency ratio
$r_P = omega / omega_P$ where $omega_P = 2 pi f_P$:
$ |X_P(omega)|
  = (F_0 / (M_P omega_P^2))/(sqrt((1-r_P^2)^2
    + (2 zeta_P r_P)^2))
  = (F_0 / k_s)/(sqrt((1-r_P^2)^2 + (2 zeta_P r_P)^2)) $ <eq-XplateNorm>

This is the standard frequency-response function of a driven SDOF oscillator.
The amplitude peaks sharply at $r_P = 1$ (plate resonance, $omega = omega_P$),
with the peak width governed by $zeta_P$.
== Effective Excitation Force on the Bistable Hook <ssec-Feff>

In the relative-coordinate frame, the plate's acceleration acts as an inertial
(fictitious) force on the hook. The effective force amplitude at frequency
$omega$ is:
$ F_("eff")(omega)
  = m_h omega^2 |X_P(omega)| $ <eq-Feff-def>

Substituting equation @eq-Xplate:
$ F_("eff")(omega)
  = (m_h F_0 omega^2)/(sqrt((k_s - M_P omega^2)^2
    + (c_s omega)^2)) $ <eq-Feff>

This is the force that the base excitation delivers to the bistable hook at
frequency $omega$. Several observations follow directly:
+ The numerator scales as $omega^2$ (not $omega^4$ as in a
rotating-unbalance source): the force still grows with frequency, but more
gradually.  At low frequencies the inertial coupling is weak; at high
frequencies the hook is increasingly driven by the plate's acceleration.
+ The denominator contains the plate resonance denominator. When $omega$
approaches $omega_P$, the denominator passes through a minimum, and
$F_("eff")$ passes through a maximum.
+ The maximum of $F_("eff")$ occurs near (but not exactly at)
$omega_P$. If $omega_P$ is designed to coincide with $omega_("n,1")$, the
maximum excitation force is delivered precisely when the bistable hook is at
its resonance - a double-resonance condition that amplifies the effective
force by $Q_P  dot  Q_h$ relative to the quasi-static case.

== General Snap-Through Condition <ssec-snapcond-general>

From equation @eq-F0min2 (Optimization section), the minimum force required for
snap-through at resonance $omega = omega_("n,1")$ is:
$ F_("0,min") = 2 zeta_h k_("eff,1") d_s
             = 2 zeta_h m_h omega_("n,1")^2 d_s $ <eq-F0min-recall>

Setting $F_("eff")(omega_("n,1"))  >=  F_("0,min")$ and substituting
equation @eq-Feff:
$ (m_h F_0 omega_("n,1")^2)/(sqrt((k_s - M_P omega_("n,1")^2)^2
    + (c_s omega_("n,1"))^2))
   >= 
  2 zeta_h m_h omega_("n,1")^2 d_s $ <eq-snapineq>

Dividing both sides by $m_h omega_("n,1")^2$ (positive):
$ (F_0)/(sqrt((k_s - M_P omega_("n,1")^2)^2
    + (c_s omega_("n,1"))^2))
   >=  2 zeta_h d_s $ <eq-snapcond-general>

Rearranging, the *general snap condition* on the motor force amplitude is:
$ F_0
   >= 
  2 zeta_h d_s sqrt((k_s - M_P omega_("n,1")^2)^2 + (c_s omega_("n,1"))^2) $ <eq-F0-general>

The right-hand side is entirely determined by known system parameters. Notably,
$m_h$ has cancelled: the snap condition is _independent of hook mass_.
However, $omega_("n,1")$ itself depends on $m_h$ through equation @eq-fn1, so the
dependence is implicit.
== Case 1: Tuned Plate ($f_P = f_("n,1")$) <ssec-case1>

==== Derivation

Set the plate support natural frequency equal to the bistable natural frequency:
$ f_P = f_("n,1")
    <=>  
  omega_P = omega_("n,1")
    <=>  
  k_s = M_P omega_("n,1")^2 $ <eq-tuned-cond>

Substituting into equation @eq-snapcond-general. The term $k_s - M_P omega_("n,1")^2$
vanishes exactly, leaving only the damping term in the denominator:
$ sqrt((M_P omega_("n,1")^2 - M_P omega_("n,1")^2)^2 + (c_s omega_("n,1"))^2)
  = c_s omega_("n,1") $ <eq-denom-tuned>

Substituting $c_s = 2 zeta_P M_P omega_P = 2 zeta_P M_P omega_("n,1")$:
$ c_s omega_("n,1") = 2 zeta_P M_P omega_("n,1")^2 $ <eq-cs-tuned>

The snap condition equation @eq-snapcond-general becomes:
$ (F_0)/(2 zeta_P M_P omega_("n,1")^2)  >=  2 zeta_h d_s $

$ F_0   >=   4 zeta_h zeta_P M_P omega_("n,1")^2 d_s
     "(Case 1: tuned plate)" $ <eq-F0-tuned>

==== Physical Interpretation

Two points are useful for design:
+ *Scales with frequency.*  Unlike a rotating-unbalance source,
$F_0  prop  omega_("n,1")^2$: a higher bistable resonance requires more
actuator force.  The available force should be verified against this threshold
after any change to geometry or mass, using the actuator's rated output at the
chosen coupling arm length.
+ *Doubly discounted by damping.* The factor
$4 zeta_h zeta_P$ means the threshold is $Q_P = 1/(2 zeta_P)$ times smaller
than the off-resonance quasi-static baseline $2 zeta_h M_P omega_("n,1")^2 d_s$.
This $Q_P$ factor is the mechanical quality factor of the plate resonance.
_Physical explanation:_ at plate resonance the plate amplitude is
amplified by $Q_P$ relative to its quasi-static response; the hook then
experiences $Q_P$ times more inertial excitation, so $Q_P$ times less
actuator force is required.

==== Plate Amplitude at Snap Threshold

At the tuned threshold, from equation @eq-Xplate with the denominator reduced to
$c_s omega_("n,1")$:
$ |X_P|_("tuned") = (F_0)/(c_s omega_("n,1"))
  = (4 zeta_h zeta_P M_P omega_("n,1")^2 d_s)/(2 zeta_P M_P omega_("n,1")^2)
  = 2 zeta_h d_s $ <eq-Xplate-tuned>

The plate displacement at threshold is $2 zeta_h d_s$. The hook displacement,
amplified by $Q_h = 1/(2 zeta_h)$ at bistable resonance, reaches $d_s$:
$ |x_("rel")|_("at snap") = Q_h |X_P| = (2 zeta_h d_s)/(2 zeta_h) = d_s $ <eq-xrel-check>

as expected. The two resonances (plate at $f_P$, hook at $f_("n,1")$) in series provide
amplification $Q_P  dot  Q_h = 1/(4 zeta_P zeta_h)$ relative to the quasi-static case.
== Case 2: Stiffness-Controlled Plate ($f_P > f_("n,1")$) <ssec-case2>

==== Regime Identification

When the plate support is stiffer than needed for resonance, $f_P > f_("n,1")$, the
plate operates in the _stiffness-controlled_ regime at the bistable's resonant
frequency. Define the detuning ratio:
$ n  equiv  (f_P)/(f_("n,1")) = (omega_P)/(omega_("n,1")) > 1 $ <eq-n-def>

In this regime $k_s > M_P omega_("n,1")^2$, so the term
$k_s - M_P omega_("n,1")^2 = M_P(omega_P^2 - omega_("n,1")^2) > 0$ is the dominant
contribution to the denominator of equation @eq-snapcond-general.
==== Derivation

Substituting $k_s = M_P omega_P^2 = M_P n^2 omega_("n,1")^2$ and
$c_s omega_("n,1") = 2 zeta_P M_P omega_P omega_("n,1") = 2 zeta_P M_P n omega_("n,1")^2$
into the denominator of equation @eq-F0-general:
$ &sqrt((k_s - M_P omega_("n,1")^2)^2 + (c_s omega_("n,1"))^2) $

$ &= sqrt(M_P^2 omega_("n,1")^4(n^2-1)^2 + M_P^2 omega_("n,1")^4(2 zeta_P n)^2) $

$ &= M_P omega_("n,1")^2 sqrt((n^2-1)^2 + (2 zeta_P n)^2) $ <eq-denom-case2>

Substituting into equation @eq-F0-general:
$ F_0   >=   2 zeta_h M_P omega_("n,1")^2 d_s sqrt((n^2-1)^2 + (2 zeta_P n)^2)
      "(Case 2: "n = f_P/f_("n,1") > 1")" $ <eq-F0-detuned>

For light structural damping ($zeta_P  <<  1$, so $2 zeta_P n  <<  n^2 - 1$):
$ F_0   approx   2 zeta_h M_P omega_("n,1")^2 d_s (n^2 - 1)
    (n > 1, zeta_P  <<  1) $ <eq-F0-detuned-approx>

The threshold grows as $(n^2-1)$, which is approximately $n^2$ for large $n$.
A plate resonance twice as high as the bistable resonance ($n=2$) requires
approximately $3 times$ the off-resonance inertia floor, and $75 times$ the tuned
threshold.
==== Verification: Case 2 Reduces to Case 1 at $n = 1$

Setting $n = 1$ in equation @eq-F0-detuned:
$ F_0  >=  2 zeta_h M_P omega_("n,1")^2 d_s sqrt(0 + (2 zeta_P)^2)
      = 4 zeta_h zeta_P M_P omega_("n,1")^2 d_s $

which recovers equation @eq-F0-tuned.
== Closed-Form Case Comparison <ssec-comparison>

==== Closed-Form Comparison

Define the off-resonance baseline $T_0 = 2 zeta_h M_P omega_("n,1")^2 d_s$. Then:
$ "Case 1 (tuned):" 
    F_0 & >=  4 zeta_h zeta_P M_P omega_("n,1")^2 d_s = 2 zeta_P T_0 $ <eq-F0-C1-norm>

$ "Case 2 (stiffness-controlled):" 
    F_0 & >=  T_0 sqrt((n^2-1)^2 + (2 zeta_P n)^2) $ <eq-F0-C2-norm>

The ratio of Case 2 to Case 1 thresholds is unchanged from the rotating-unbalance
analysis:
$ ("Case 2 threshold")/("Case 1 threshold")
  = (sqrt((n^2-1)^2 + (2 zeta_P n)^2))/(2 zeta_P)
   approx  (n^2 - 1)/(2 zeta_P)
    (n > 1, zeta_P  <<  1) $ <eq-ratio-C2-C1>

For $zeta_P = 0.02$ and $n = 2$: ratio $approx  (4-1)/(0.04) = 75$.
For $n = 1.5$: ratio $approx  (2.25-1)/0.04 = 31$.
For $n = 1.1$: ratio $approx  (1.21-1)/0.04 = 5.3$.
Even a modest $10%$ detuning ($n = 1.1$) increases the required motor force
by more than $5 times$ compared to the tuned case.

#figure(image("figures/sim_excitation/Config_2__Prescribed_Base_Motion.png", width: 100%), caption: [Time-domain snap-through simulation for Configuration 2 (prescribed base motion). The plate displacement $x_P(t)$ and relative hook displacement $x_("rel") (t)$ are plotted; the hook crosses the saddle $d_s$ and latches into State 2 without further motor input.]) <fig-config2-snap>

#figure(image("figures/sim_excitation/Bistable_Excitation__Animation__Config_2_Prescribed_Motion.png", width: 100%), caption: [Animation snapshot of the Configuration 2 bistable excitation (prescribed base motion). The plate oscillates at $f_("n,1")$ and the hook builds resonant amplitude until snap-through occurs.]) <fig-anim-config2>

== Motor Mounted on the Bistable Plate: Coupling Model and Assumptions <ssec-motor-plate>

This subsection addresses the specific physical configuration assumed throughout
the base-excitation model: the motor body is bolted directly to the same rigid
plate that carries the bistable mechanism. It tracks the force path from an
eccentric mass to the plate, shows why shaft angle is not a state variable in
the hook dynamics, and states the assumptions explicitly.
==== Force Transmission Chain <sssec-transmission>

When the motor shaft rotates, the eccentric mass $m_e$ at radius $e$ generates
a centrifugal force.  Before this force reaches the plate it passes through
three mechanical interfaces, each with its own compliance:
+ *Shaft* (stiffness $k_("sh")$, damping $c_("sh")$):
The shaft transmits the eccentric mass force to the inner bearing race.
A rigid shaft ($k_("sh")  ->   infinity$) transmits 100%; a flexible
shaft at speeds near its _critical speed_
$omega_("crit") = sqrt(k_("sh")/m_e)$
can amplify or attenuate the transmitted force.
+ *Bearing* (stiffness $k_b$, damping $c_b$): The rolling-element
bearing connects inner race to outer race (motor housing).  The Hertzian
contact stiffness of a typical deep-groove ball bearing is
$k_b  tilde  10^7$-$10^8 "N/m"$, far above the frequencies of interest
here, so bearing deflection is negligible.
+ *Motor mount bolts and plate*: The motor housing is assumed
rigidly bolted to the plate.  Any compliance in this joint
(bolt stretch, washer deformation) is folded into the effective bearing
stiffness.

The _bearing transmission function_ $T_b(omega)$ quantifies how much of
the raw centrifugal force $m_e e omega^2$ actually reaches the plate at
frequency $omega$:
$ T_b(omega)
  = (k_b + i c_b omega)/((k_b - m_e omega^2)
    + i c_b omega) $ <eq-Tb>

The force entering the plate is therefore:
$ F_("plate")(omega)
  = T_b(omega) dot  m_e e omega^2 $ <eq-Fplate-transmitted>

*Rigid-bearing limit.*
When $k_b  >>  m_e omega^2$ (bearing far from its own resonance):
$ T_b(omega)   ->   1   (k_b  ->   infinity ) $ <eq-Tb-rigid>

All force transfers to the plate.  Rolling-element bearings typically have
Hertzian contact stiffnesses $k_b  tilde  10^7$-$10^8$ N/m.  As long as
$m_e omega^2  <<  k_b$ - satisfied at any bistable resonance frequency well
below the bearing's own resonance - $T_b  approx  1$ and the full centrifugal
force reaches the plate, justifying
$F_("motor") = m_e e omega^2 sin(omega t)$.
==== Single-Axis Projection of the 2-D Rotating Force <sssec-projection>

The centrifugal force is a vector that rotates in the plane of the motor shaft:
$ bold(F)_("cent")(t)
  = m_e e omega^2 [cos(omega t) hat(x) + sin(omega t) hat(y)] $ <eq-Fcent-2d>

Only the component aligned with the bistable stroke direction (take $hat(y)$
as the snap axis) does useful work on the bistable:
$ F_("snap")(t)
  = bold(F)_("cent")(t) dot hat(y)
  = m_e e omega^2 sin(omega t) $ <eq-Fsnap-proj>

The orthogonal component $m_e e omega^2 cos(omega t)$ loads the plate
transversely; it is transmitted to the plate support as a lateral force but
does not couple into the bistable snap direction and is neglected in the
1-DOF model.
*Practical implication.*

Aligning the motor shaft so that the eccentricity plane contains the snap
direction maximises the effective force.  Misalignment by angle $alpha$
reduces $F_("snap")$ by $cos alpha$.
==== Motor Shaft Angle Does Not Appear in the Hook Dynamics <sssec-decoupling>

A key property of the rotating-unbalance architecture is that the
_motor shaft angle_ $theta(t) = omega t + theta_0$ does not appear in
the bistable hook's equation of motion - only the resulting plate
acceleration does.
*Proof.*

The hook EOM in relative coordinates is (equation @eq-relEOM):
$ m_h dot.double(x)_("rel")
  = F_("restore")(x_("rel"))
    - c_h dot(x)_("rel")
    - m_h dot.double(x)_P(t) $ <eq-hook-decoupled>

The right-hand side contains $dot.double(x)_P(t)$ - the plate acceleration.
This quantity depends on $F_("snap")(t) = m_e e omega^2 sin(omega t)$
through the plate dynamics, but the shaft angle $theta$ itself never appears.
Whether the motor has rotated $100$ turns or $1000$ turns is irrelevant; only
the instantaneous force amplitude and frequency matter.
*Contrast with direct drive.*

In a crank-slider or lever-arm direct-drive arrangement, the motor shaft angle
$theta$ sets the hook position geometrically:
$x_("hook") = f(theta)$.  A change in $theta$ immediately moves the
hook.  With rotating unbalance there is no such kinematic constraint: the hook
position is determined entirely by the energy balance between the inertial
excitation and the bistable restoring force.  The motor can spin freely at
constant speed without tracking or position control.
==== Summary of Model Assumptions <sssec-assumptions>

#figure(table(
  columns: 3,
  [p{4.5cm} p{5.0cm}}

*Assumption*],
  [*Mathematical form*],
  [*Validity / break-down*],
  [Rigid bearing],
  [$T_b(omega) = 1$],
  [Valid: $k_b  >>  m_e omega^2$ near the current resonance],
  [Rigid motor shaft],
  [$omega_("crit")  >>  omega$],
  [Valid if shaft critical speed $>> f_("n,1")$],
  [Constant motor speed],
  [$dot(omega) = 0$],
  [Valid if motor torque bandwidth $>>  f_("n,1")$],
  [Single-axis force],
  [$F = m_e e omega^2 sin(omega t)$],
  [Loses factor $cos alpha$ if shaft misaligned],
  [Rigid motor-to-plate joint],
  [$k_("joint")  ->   infinity$],
  [Valid for bolted flange; invalid for press-fit],
  [Plate as rigid body],
  [No plate bending modes],
  [Valid if plate structural modes $>>  f_("n,1")$],
  [Linear plate support],
  [$k_s$, $c_s$ constant],
  [Invalid near plate support nonlinearity],
), caption: [Assumptions in the shared-plate rotating-unbalance model and their validity for the Rolly deployment mechanism.]) <tab-assumptions>

== Motor Housing on the Plate with a Proof Mass on the Output Shaft <ssec-proof-mass>

This subsection treats a distinct configuration: the motor housing is bolted to
the bistable plate as before, but instead of a continuously spinning eccentric
mass, the motor output shaft drives a _proof mass_ $M_("pm")$ that
*oscillates linearly* along the snap axis relative to the plate.  This
is sometimes called a _reaction-mass actuator_ or _proof-mass actuator_ and is used when independent control of force amplitude and frequency
is required.
==== Equations of Motion <sssec-pm-eom>

Define coordinates:
- $x_P(t)$: absolute plate displacement (positive in snap direction)
- $delta(t) = x_("pm") (t) - x_P(t)$: proof mass displacement _relative_ to plate
- $x_("rel") (t)$: hook displacement relative to plate (unchanged from equation @eq-relEOM)

The motor applies an internal force $F_("act")(t)$ between the plate
(housing) and the proof mass.  By Newton's third law the plate receives
$-F_("act")$ and the proof mass receives $+F_("act")$.
*Proof mass (absolute frame):*
$ M_("pm") dot.double(x)_("pm")
  = M_("pm") (dot.double(x)_P + dot.double(delta))
  = F_("act")(t) $ <eq-pm-eom>

*Plate (absolute frame):*
$ M_P dot.double(x)_P
  = -F_("act")(t)
  - k_s x_P - c_s dot(x)_P
  - F_("restore")(x_("rel")) + c_h dot(x)_("rel") $ <eq-plate-pm>

Adding equation @eq-pm-eom and equation @eq-plate-pm to eliminate $F_("act")$:
$ (M_P + M_("pm")) dot.double(x)_P + M_("pm") dot.double(delta)
  = -k_s x_P - c_s dot(x)_P
    - F_("restore")(x_("rel")) + c_h dot(x)_("rel") $ <eq-plate-pm-combined>

The key observation is that $F_("act")$ has been eliminated.  The plate
equation is now driven entirely by the proof mass _relative acceleration_
$dot.double(delta)$.
==== Prescribed Harmonic Proof-Mass Motion

If the motor drives the proof mass to oscillate at amplitude $Delta$ and
frequency $omega$:
$ delta(t) = Delta sin(omega t)
    =>  
  dot.double(delta)(t) = -Delta omega^2 sin(omega t) $ <eq-pm-prescribed>

Substituting into equation @eq-plate-pm-combined:
$ (M_P + M_("pm")) dot.double(x)_P
  = underbrace(M_("pm") Delta omega^2 sin(omega t), "effective driving force")
    - k_s x_P - c_s dot(x)_P
    - F_("restore")(x_("rel")) + c_h dot(x)_("rel") $ <eq-plate-pm-driven>

This has _exactly the same form_ as the rotating-unbalance plate equation
(equation @eq-plateeom), with two substitutions:
$ m_e e   &  ->   M_("pm") Delta
     "(force product)" $ <eq-pm-sub1>

$ M_P      &  ->   M_P + M_("pm")
     "(effective plate mass)" $ <eq-pm-sub2>

All plate-transfer-function and comparison results carry over
with these substitutions.
==== Snap Condition for the Proof-Mass Configuration

Applying the same derivation as the tuned-plate case with substitutions
equation @eq-pm-sub1 and equation @eq-pm-sub2:
*Tuned plate ($f_P = f_("n,1")$):*
$ M_("pm") Delta   >=   4 zeta_h zeta_P (M_P + M_("pm")) d_s $ <eq-pm-snap-tuned>

Defining the proof-mass ratio $mu = M_("pm")/M_P$ and rearranging:
$ Delta   >=   4 zeta_h zeta_P M_P d_s 
    (1 + mu)/(mu)
  = (4 zeta_h zeta_P M_P d_s)/(mu) + 4 zeta_h zeta_P d_s $ <eq-pm-Delta-min>

As $mu  ->   infinity$ (proof mass much heavier than plate), the required stroke
$Delta  ->  4 zeta_h zeta_P d_s$, independent of $M_P$.  As $mu  ->  0$ (very
light proof mass), $Delta  ->   infinity$: a tiny mass must oscillate with
enormous stroke to produce enough force.
==== Comparison with Rotating Unbalance

#figure(table(
  columns: 3,
  [p{4.8cm} p{5.2cm}}

*Property*],
  [*Rotating unbalance*],
  [*Proof-mass oscillator*],
  [Force amplitude],
  [$m_e e omega^2$ (fixed by geometry)],
  [$M_("pm") Delta omega^2$ ($Delta$ controllable)],
  [Force-frequency],
  [Always grows as $omega^2$],
  [Grows as $omega^2$ unless $Delta(omega)$ is adjusted],
  [Effective plate mass],
  [$M_P$],
  [$M_P + M_("pm")$],
  [Motor direction],
  [Continuous rotation (one direction)],
  [Reciprocating (reverses each half-cycle)],
  [Snap product],
  [$m_e e  >=  4 zeta_h zeta_P M_P d_s$],
  [$M_("pm")Delta  >=  4 zeta_h zeta_P(M_P+M_("pm"))d_s$],
  [Shaft angle in EOM?],
  [No],
  [No (same decoupling argument)],
), caption: [Proof-mass oscillator vs. rotating unbalance: key differences for the same bistable plate.]) <tab-pm-vs-ru>

The proof-mass oscillator is strictly more flexible than the rotating unbalance
because $Delta$ can be varied in real time via motor current.  The cost is
that the motor must reverse direction each cycle rather than spinning
continuously, and the heavier effective plate mass $(M_P + M_("pm"))$ raises the
snap threshold.
==== Proof-Mass Resonance

If the shaft connecting motor to proof mass has compliance $k_("pm")$, the proof
mass forms a secondary resonance:
$ omega_("pm") = sqrt((k_("pm"))/(M_("pm"))) $ <eq-pm-resonance>

Driving near $omega_("pm")$ amplifies $Delta$ by $Q_("pm") = 1/(2 zeta_("pm"))$,
reducing the required motor force by the same factor.  This creates a
_triple resonance_ path: plate resonance at $omega_P$, bistable hook
resonance at $omega_("n,1")$, and proof-mass resonance at $omega_("pm")$.
Tuning all three to the same frequency gives amplification
$Q_P  dot  Q_h  dot  Q_("pm")$ but also narrows the bandwidth and makes the
system sensitive to parameter variation.
#figure(image("figures/sim_excitation/412_Proof-Mass__Mass_Actuation_Analysis.png", width: 100%), caption: [Proof-mass actuation analysis: minimum proof mass $M_("pm,min")$ and corresponding snap-through drive frequency as a function of design parameters ($N$ parallel beams, hook mass $m_h$).]) <fig-pm-analysis>

#figure(image("figures/sim_excitation/Bistable_Excitation__Animation__Sec_412_Proof-Mass_Optimal.png", width: 100%), caption: [Animation snapshot for the proof-mass actuator at the optimal proof-mass parameter. The proof mass oscillates on the motor output shaft, driving the plate and hook into snap-through.]) <fig-anim-pm>

= Actuator Limit Used in the Simulation <sec-actuator-limit>

The detailed RS05 motor specifications and CAN notes have been moved to
`rs05_motor_specifications.tex` and `rs05_motor_specifications.typ`.  The main
model only needs the shaft velocity limit used by the optimization.

The RS05 no-load speed is treated as a shaft angular-velocity limit:
$ omega_("max") = 50.3 "rad/s" $

For sinusoidal oscillation at frequency $f$ with angular shaft amplitude
$Theta$:
$ v_("peak") = Theta dot 2 pi f <= omega_("max") $ <eq-vpeak>

Thus the maximum angular amplitude is:
$ Theta_("max")(f) = (omega_("max"))/(2 pi f) $ <eq-Thetamax>

Through a lever arm $r$, the corresponding linear stroke is:
$ delta_("max")(f) = (omega_("max") r)/(2 pi f) $ <eq-deltamax>

For snap-through, the available stroke must reach the saddle. The minimum lever
arm is therefore:
$ r_("min")(f) = (2 pi f d_s)/(omega_("max")) $ <eq-rmin>

= Optimization <sec-opt>

== Snap-Through Threshold <ssec-F0min>

From linear resonance theory, the steady-state hook amplitude at $omega = omega_("n,1")$
is:
$ A_("ss") = (F_0)/(2 zeta_h k_("eff,1")) $ <eq-Ass>

Snap-through occurs when $A_("ss")  >=  d_s$, giving:
$ F_("0,min") = 2 zeta_h k_("eff,1") d_s $ <eq-F0min2>

This is the _minimum force applied directly at the hook_.  For base
excitation (motor on plate), the motor force required is larger by a factor
that depends on the plate dynamics and drive frequency.
== Mass Actuation Force <ssec-mass-actuation>

The central design question for the Rolly deployment mechanism is:
_for a given mass budget added to the robot, what is the maximum force that mass can deliver to the bistable hook?_
==== Proof-Mass Oscillator (proof-mass configuration)

A proof mass $M_("pm")$ oscillating at frequency $omega$ and amplitude $Delta$
delivers inertial force $F = M_("pm") Delta omega^2$.  At the velocity limit
(equation @eq-Thetamax):
$ Delta_("max") (omega) = (omega_("max") r)/(omega) $

The maximum force per unit proof mass is:
$ (F_("max"))/(M_("pm")) = Delta_("max") omega^2
                           = omega_("max") r omega $ <eq-force-per-mass>

This grows _linearly with drive frequency_.  Higher frequency delivers
more force for the same proof mass — up to the limit imposed by the bistable
dynamics (TMD anti-resonance at $omega_("n,1")$; see the tuned-plate case).
The coupled relative-mode resonance at $omega_("rel")$ is the correct
drive frequency:
$ omega_("rel") = sqrt((k_("eff,1"))/(M_("red"))),
   
  M_("red") = (m_h M_P)/(m_h + M_P) $ <eq-omrel>

The minimum proof mass to snap at $omega_("rel")$ is found by binary
search over the nonlinear ODE.
#figure(image("figures/sim_excitation/3-D_M_pmminf_drive_m_hook.png", width: 100%), caption: [$M_("pm,min")$ vs. drive frequency and hook mass $m_h$.]) <fig-3d-pm-mhook>

==== Config 2: Prescribed Base Motion (no added mass)

The motor prescribes the plate displacement $x_P(t) = A sin(omega_("n,1")t)$.
The minimum plate amplitude for resonant snap is:
$ A_("min") = 2 zeta_h d_s $ <eq-Amin-c2>

The required motor torque at resonance ($omega = omega_("n,1")$):
$ tau = m_h A_("min") omega_("n,1")^2 r
       = 2 zeta_h k_("eff,1") d_s r $ <eq-tau-c2>

This is _independent of hook mass_ $m_h$.  No additional hardware
mass is required - the motor already present for locomotion is used.
#figure(image("figures/sim_excitation/3-D_Torquef_drive_m_hook__Config_2.png", width: 100%), caption: [Required torque vs. drive frequency and hook mass $m_h$.]) <fig-3d-tau-mhook>

== Parallel Beam-Count Scaling ($N$ beams) <ssec-ncells>

For the current model, $N$ is the number of parallel bistable beams. The beams
share the same hook displacement, so adding beams increases force and stiffness,
not stroke:
$ d_f &= d_("f,1")  "(stroke unchanged)" $

$ d_s &= d_("s,1")  "(saddle unchanged)" $

$ k_("eff,1") & prop  N  "(stiffness rises)" $

$ f_("n,1") & prop  sqrt(N)  "(natural frequency rises)" $

$ F_("0,min") & prop  N  "(force threshold rises)" $

The old series assumption made the saddle displacement look too large. In the
parallel model, $N=5$ raises the stored energy and stiffness while keeping the
hook travel at the single-beam value.
The post-snap potential energy available to accelerate the hook to State 2:
$ Delta E_2 = V(d_s) - V(d_f) $

grows with $N$ because the force-displacement curve scales upward, not because
the stroke grows. This is the _impact energy_ available when the hook collides
with a target object at State 2.
== Impact Force at State 2 <ssec-impact>

When the hook arrives at $x_("rel") = d_f$ it has kinetic energy
accumulated from two sources:
+ *Kinetic energy at saddle crossing* $K E_("saddle")$
(from resonant build-up).
+ *Potential energy released* by the bistable spring from saddle
to State 2: $Delta E_2$.

The hook velocity at impact (ignoring damping over the short snap transit):
$ v_("impact") = sqrt(v_("saddle")^2 + (2 Delta E_2)/(m_h)) $ <eq-vimpact>

At the minimum snap threshold $v_("saddle")  approx  0$, so
$v_("impact")  approx  sqrt(2 Delta E_2 / m_h)$.
The peak force on the target object (Hertz contact model):
$ F_("peak") = v_("impact") sqrt(k_("contact") m_h) $ <eq-Fpeak>

where $k_("contact")$ is the contact stiffness.  The impact force
scales as $sqrt(k_("contact"))$: steel-on-steel
($k  tilde  10^(10)$ N/m) gives $tilde  3000 times$ more peak force than
rubber-on-rubber ($k  tilde  10^(4)$ N/m) for the same impact velocity.
The impact momentum (relevant for deforming targets):
$ p = m_h v_("impact") = sqrt(2 m_h Delta E_2) $ <eq-pimpact>

grows as $sqrt(m_h)$ - heavier hooks deliver more momentum.

#figure(image("figures/sim_excitation/Comprehensive_Design_Space_vs_n_cells.png", width: 100%), caption: [Comprehensive design space: key performance metrics (natural frequency $f_("n,1")$, energy barrier $Delta E_1$, impact energy $Delta E_2$, minimum required force $F_("0,min")$, and minimum proof mass $M_("pm,min")$) as a function of the number of parallel beams $N$. All curves are normalised to their single-beam values.]) <fig-design-space>

#figure(image("figures/sim_excitation/Impact_Energy__Design_Cost_vs_n_cells.png", width: 100%), caption: [Impact energy $Delta E_2$ and actuation cost metrics as a function of the number of parallel beams $N$. Impact energy grows with $N$ because the force-displacement curve scales upward.]) <fig-impact-energy>


= References

#bibliography("bistable_math_derivation.bib", title: [References])
