%% bistable_curved_beam.m
% Compliant Curved-Beam Bistable Mechanism — Qiu/MIT Patent + Zolfagharian Design 9
%
% Beam geometry from Qiu et al. (US 2003/0029705 A1):
%   y0(xi) = (h/2) * [1 - cos(2*pi*xi/l)]   (cosine profile, 0 <= xi <= l)
%
% Force-displacement calibrated to Zolfagharian Design 9 experimental data
% (ABS, 0.6 mm beam, 5-cell stack, ~27.5 mm total stroke, 3.3 cm in paper).
%
% Two dynamic models:
%   1-DOF: hook mass on bistable element under chirp excitation → snap-through
%   2-DOF: main mass + TMD mass (Zolfagharian Fig. 4) → frequency response
%
% Bistability condition (Qiu et al.):  Q = h/t  >  2.31

clear; clc; close all;

%% ══════════════════════════════════════════════════════════════════════
%%  TUNABLE PARAMETERS
%% ══════════════════════════════════════════════════════════════════════

%% ── Beam geometry (Zolfagharian Design 9 defaults) ───────────────────
E_abs      = 2300e6;   % [Pa]    ABS elastic modulus (from paper ASTM D638 test)
nu_abs     = 0.35;     % [-]     Poisson's ratio (ABS)
rho_abs    = 1040;     % [kg/m³] ABS density
t_beam     = 0.6e-3;   % [m]     Beam thickness  ← key bistability parameter
l_span     = 42e-3;    % [m]     Horizontal beam span  (hinge-to-hinge)
h_apex     = 2.5e-3;   % [m]     Apex height  (set h/t > 2.31 for bistability)
n_cells    = 5;        % [-]     Stacked cells (each adds ~5.5 mm stroke)
% To target a different total stroke, adjust n_cells or h_apex.
% To target a different natural frequency, adjust m_hook or n_cells
% (more cells → lower stiffness → lower freq; fewer cells → higher freq).

%% ── Force calibration (Design 9, Table 2 of Zolfagharian 2025) ───────
F_push_1cell = 12.55;  % [N]  Peak applied force (push, state 1 → saddle)
%  Experimental value: 11.10 N; simulation value: 12.55 N
d_stroke_1cell = 8.25e-3; % [m]  Stroke per cell  (1.5× original 5.5 mm → 41.25 mm total)
d_saddle_frac  = 0.545;  % [-]  Saddle position as fraction of stroke (slightly past mid)
%  Adjusting this shifts the asymmetry of the potential wells.

%% ── 1-DOF hook model ─────────────────────────────────────────────────
m_hook      = 0.200;   % [kg]  Hook / payload mass  (doubled)
zeta_hook   = 0.05;    % [-]   Damping ratio for hook model

%% ── Chirp excitation (1-DOF model) ──────────────────────────────────
F0_chirp    = 1.0;    % [N]   Excitation amplitude (chirp / direct-drive reference)
%  Required minimum: F0 >= 2*zeta*k_eff_s1*d_saddle  (independent of m_hook)
%  For n_cells=5, zeta=0.05: F0_min ~ 7 N at resonance.
%  10 N is achievable from the RobStride O5 (6 N·m / ~50 mm lever arm = 120 N peak).
f_chirp_lo  = 1;       % [Hz]  Chirp start frequency
% f_chirp_hi derived below from f_n1 (= ceil(f_n1 * 1.15)) so it auto-scales
t_end_1dof  = 20.0;    % [s]   Simulation duration (1-DOF)

%% ── Base excitation model (shared plate + rotating unbalance) ────────
% Physical setup: motor with eccentric mass mounts on the same plate as the
% bistable mechanism. Rotating unbalance vibrates the plate; the hook mass
% (which rests on the bistable spring) cannot follow the plate rigidly, so
% the relative displacement x_rel drives snap-through.
%
% Snap conditions (derived analytically):
%   Tuned plate (f_plate_nat = f_n1):   m_unbal*e_unbal >= 4*zeta_hook*zeta_plate*M_plate*d_saddle
%   Off-resonance plate:                m_unbal*e_unbal >= 2*zeta_hook*M_plate*d_saddle
M_plate     = 0.200;   % [kg]  Total plate mass (motor + chassis + bistable assembly)
f_plate_nat = [];      % [Hz]  Plate support natural freq — set [] to auto-use f_n1 (tuned case)
%   Setting f_plate_nat = f_n1 maximises force amplification at the bistable's resonance.
%   Set to a specific Hz value to simulate a detuned (non-resonant) plate.
zeta_plate  = 0.02;    % [-]   Plate structural damping (0.01–0.05 for bolted/bonded structures)
m_unbal     = 0.050;   % [kg]  Rotating unbalance mass on motor shaft
e_unbal     = 0.003;   % [m]   Eccentricity (3 mm)  →  m_e×e = 150 g·mm

%% ── 2-DOF TMD model (Zolfagharian Fig. 4 parameters) ────────────────
m_main      = 0.150;   % [kg]  Main mass (from paper)
m_TMD       = 0.01875; % [kg]  TMD mass (from paper)
% Main spring: k = 3EI/L³ for stainless steel ruler
% E_ruler = 210 GPa, width = 20 mm, thickness = 1.5 mm
% Natural frequency of main system ≈ 6.2 Hz with paper's k_main = 226.8 N/m.
% Back-calculated from experimental 35 Hz peak: k_eff ≈ 7253 N/m.
% Use k_main_mode = 'paper' (226.8 N/m) or 'experiment' (7253 N/m):
k_main_mode = 'experiment';  % 'paper' | 'experiment'
zeta_main   = 0.03;    % [-]   Main-system damping ratio
zeta_TMD    = 0.02;    % [-]   TMD damping ratio
F0_TMD      = 1.0;     % [N]   Harmonic excitation amplitude (2-DOF sweep)
f_TMD_lo    = 2;       % [Hz]  Sweep start (2-DOF)
f_TMD_hi    = 60;      % [Hz]  Sweep end  (2-DOF, matches paper Fig. 10)

ANIMATE     = true;     % true → run animations and save GIFs
DISPLAY_FIGS = false;   % false → figures are invisible (still saved); true → show on screen

%% ══════════════════════════════════════════════════════════════════════
%%  DERIVED QUANTITIES
%% ══════════════════════════════════════════════════════════════════════
Q = h_apex / t_beam;
assert(Q > 2.31, ...
    'Q = h/t = %.3f < 2.31 — increase h_apex or decrease t_beam for bistability.', Q);

I_beam = t_beam^3 / 12;  % [m³/m] Second moment of area (per unit width)

% Per-cell stroke and saddle
d_stroke = n_cells * d_stroke_1cell;   % [m] Total stroke
d_saddle = d_saddle_frac * d_stroke;   % [m] Saddle (unstable equilibrium)
d_push   = 0.27 * d_stroke;            % [m] Location of maximum push force

% Coefficient A for cubic restoring force:
%   F_internal(x) = -A * x * (x - d_saddle) * (x - d_stroke)
%   Calibrated so that peak of applied external force = F_push_1cell.
%   (Stack in series: same force threshold, displacement multiplies by n_cells,
%    so A scales as 1/n_cells^3 relative to per-cell value.)
A_1cell = F_push_1cell / ...
    (d_stroke_1cell * d_saddle_frac * d_stroke_1cell * ...
     (d_saddle_frac * d_stroke_1cell - d_stroke_1cell) * ...
     (0.27 * d_stroke_1cell - d_stroke_1cell));
% Calibrate A_coeff so the peak of the APPLIED external force matches F_push.
% At x=d_push (between State 1 and saddle):
%   d_push > 0, (d_push-d_saddle) < 0, (d_push-d_stroke) < 0  → product > 0
% A_coeff = F_push / denom  → A_coeff > 0
% Internal restoring force: F_restore = -A_coeff*x*(x-ds)*(x-df)
%   For 0 < x < d_saddle: F_restore < 0  (pulls back to State 1)  ← stable
%   For d_saddle < x < d_stroke: F_restore > 0  (pushes to State 2)  ← snap
denom   = d_push * (d_push - d_saddle) * (d_push - d_stroke);  % > 0
A_coeff = F_push_1cell / denom;                                  % > 0

% Restoring force on hook (positive = toward State 2, negative = toward State 1)
F_restore = @(x) -A_coeff .* x .* (x - d_saddle) .* (x - d_stroke);

% Potential energy V(x) = -integral of F_restore from 0 to x  (analytic)
%   F_restore = -A*(x³ - (d_s+d_f)*x² + d_s*d_f*x)
%   V(x) = A*(x⁴/4 - (d_s+d_f)*x³/3 + d_s*d_f*x²/2)
ds = d_saddle; df = d_stroke;
V_fun = @(x) A_coeff .* (x.^4/4 - (ds+df)*x.^3/3 + ds*df*x.^2/2);

% Effective stiffness at each stable state  (k = -dF/dx at equilibrium)
k_eff_s1 = A_coeff * d_saddle * d_stroke;             % at x=0
k_eff_s2 = A_coeff * d_stroke * (d_stroke - d_saddle);% at x=d_stroke

% Hook natural frequencies
omega_n1 = sqrt(k_eff_s1 / m_hook);  f_n1 = omega_n1/(2*pi);
omega_n2 = sqrt(k_eff_s2 / m_hook);  f_n2 = omega_n2/(2*pi);
f_chirp_hi = ceil(f_n1 * 1.15);   % [Hz] 15% headroom above resonance; auto-scales with geometry

% Plate support dynamics (base excitation model)
if isempty(f_plate_nat), f_plate_nat = f_n1; end   % [] → auto-tune to bistable resonance
k_supp = M_plate * (2*pi*f_plate_nat)^2;            % support stiffness
c_supp = 2 * zeta_plate * M_plate * (2*pi*f_plate_nat);  % support damping

c_hook = 2 * zeta_hook * m_hook * omega_n1;  % damping (linearised at state 1)

% 2-DOF TMD parameters
switch lower(k_main_mode)
    case 'paper'
        k_main = 226.8;           % [N/m]  As stated in paper
        f_main = sqrt(k_main/m_main)/(2*pi);
    case 'experiment'
        f_main = 35;              % [Hz]   Experimental resonance peak
        k_main = (2*pi*f_main)^2 * m_main;
    otherwise
        error('k_main_mode must be ''paper'' or ''experiment''.');
end
omega_main = 2*pi*f_main;
c_main  = 2 * zeta_main * m_main * omega_main;
% TMD spring: match natural frequency of TMD to main system
k_TMD   = omega_main^2 * m_TMD;
c_TMD   = 2 * zeta_TMD * m_TMD * omega_main;

% Energy barrier
DeltaE_1 = V_fun(d_saddle) - V_fun(0);          % barrier from State 1
DeltaE_2 = V_fun(d_saddle) - V_fun(d_stroke);   % barrier from State 2

fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('   Curved-Beam Bistable — Derived Parameters\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  Q = h/t = %.3f  (bistability threshold: 2.31)\n', Q);
fprintf('  Total stroke       = %.1f mm  (%d × %.1f mm cells)\n', ...
        d_stroke*1e3, n_cells, d_stroke_1cell*1e3);
fprintf('  Saddle position    = %.2f mm  (%.1f%% of stroke)\n', ...
        d_saddle*1e3, d_saddle_frac*100);
fprintf('  A_coeff            = %.4e N/m³\n', A_coeff);
fprintf('  k_eff (State 1)    = %.2f N/m\n', k_eff_s1);
fprintf('  k_eff (State 2)    = %.2f N/m\n', k_eff_s2);
fprintf('  f_n1 (State 1)     = %.3f Hz  (%.2f rad/s)\n', f_n1, omega_n1);
fprintf('  f_n2 (State 2)     = %.3f Hz  (%.2f rad/s)\n', f_n2, omega_n2);
fprintf('  ΔE₁ (barrier→S1)  = %.4f mJ\n', DeltaE_1*1e3);
fprintf('  ΔE₂ (barrier→S2)  = %.4f mJ\n', DeltaE_2*1e3);
fprintf('  k_main (%s) = %.2f N/m  →  f_main = %.2f Hz\n', k_main_mode, k_main, f_main);
me_product        = m_unbal * e_unbal;
me_thresh_tuned   = 4*zeta_hook*zeta_plate*M_plate*d_saddle;
me_thresh_detuned = 2*zeta_hook*M_plate*d_saddle;
fprintf('  ── Base excitation design margins ─────────────────────────\n');
if abs(f_plate_nat - f_n1) < 0.1
    plate_tune_str = 'TUNED to f_n1 ✓';
else
    plate_tune_str = sprintf('detuned by %.1f Hz', f_plate_nat - f_n1);
end
fprintf('  f_plate_nat        = %.2f Hz  (%s)\n', f_plate_nat, plate_tune_str);
fprintf('  m_e×e product      = %.1f g·mm\n', me_product*1e6);
fprintf('  Tuned-plate thresh = %.1f g·mm  →  margin %.2fx\n', me_thresh_tuned*1e6,   me_product/me_thresh_tuned);
fprintf('  Off-res. threshold = %.1f g·mm  →  margin %.2fx\n', me_thresh_detuned*1e6, me_product/me_thresh_detuned);
fprintf('══════════════════════════════════════════════════════════════\n\n');

%% ══════════════════════════════════════════════════════════════════════
%%  1-DOF SNAP-THROUGH SIMULATION (chirp excitation)
%% ══════════════════════════════════════════════════════════════════════
% Chirp: F(t) = F0 * sin(2*pi*(f_lo + (f_hi-f_lo)*t/(2*t_end)) * t)
f_inst  = @(t) f_chirp_lo + (f_chirp_hi - f_chirp_lo) * t / t_end_1dof;
phi_fun = @(t) 2*pi*(f_chirp_lo*t + (f_chirp_hi - f_chirp_lo)*t.^2/(2*t_end_1dof));
F_chirp = @(t) F0_chirp .* sin(phi_fun(t));

ode_1dof = @(t,y) [y(2); ...
    (F_chirp(t) + F_restore(y(1)) - c_hook*y(2)) / m_hook];

y0_hook  = [0; 0];  % start at State 1 (x=0, v=0)

opts_snap = odeset('Events',  @(t,y) snap_event(t, y, d_saddle), ...
                   'RelTol',  1e-7, 'AbsTol', 1e-10, 'MaxStep', 5e-4);
opts_free = odeset('RelTol',1e-7,'AbsTol',1e-10,'MaxStep',5e-4);

% Free-decay ODE: excitation stops at snap, hook settles to State 2 under restoring + damping only
ode_decay = @(t,y) [y(2); (F_restore(y(1)) - c_hook*y(2)) / m_hook];

[t1, y1, t_snap, y_snap, ~] = ode45(ode_1dof, [0, t_end_1dof], y0_hook, opts_snap);

if isempty(t_snap)
    warning('No snap-through in 1-DOF model. Increase F0_chirp or t_end_1dof.');
    t_snap = NaN;  t_1dof = t1; y_1dof = y1;
else
    f_snap = f_inst(t_snap);
    fprintf('>>> 1-DOF Snap-through at  t = %.3f s  |  f_inst = %.2f Hz\n\n', ...
            t_snap, f_snap);
    [t2, y2] = ode45(ode_decay, [t_snap, t_snap + 2.0], y_snap, opts_free);
    t_1dof = [t1; t2(2:end)];
    y_1dof = [y1; y2(2:end,:)];
end

x_1dof = y_1dof(:,1);
v_1dof = y_1dof(:,2);

%% ══════════════════════════════════════════════════════════════════════
%%  SUB-SIM A: Ramped Sine at f_n1 (fixed resonance frequency)
%% ══════════════════════════════════════════════════════════════════════
fprintf('Running sub-sim A: ramped sine at f_n1 = %.2f Hz …\n', f_n1);
t_ramp_dur = 2.0;   % [s] amplitude ramp-up 0 → F0_chirp
F_ramp_fn  = @(t) F0_chirp .* min(t / t_ramp_dur, 1) .* sin(2*pi*f_n1.*t);
ode_ramp   = @(t,y) [y(2); (F_ramp_fn(t) + F_restore(y(1)) - c_hook*y(2)) / m_hook];

[t1r, y1r, t_snap_r, y_snap_r, ~] = ode45(ode_ramp, [0, t_end_1dof], y0_hook, opts_snap);
if isempty(t_snap_r)
    warning('Sub-sim A: no snap-through. Try increasing F0_chirp.');
    t_snap_r = NaN;  t_ramp = t1r;  y_ramp = y1r;
else
    fprintf('  >>> Sub-sim A snap at t = %.3f s\n', t_snap_r);
    [t2r, y2r] = ode45(ode_decay, [t_snap_r, t_snap_r + 2.0], y_snap_r, opts_free);
    t_ramp = [t1r; t2r(2:end)];  y_ramp = [y1r; y2r(2:end,:)];
end
x_ramp = y_ramp(:,1);  v_ramp = y_ramp(:,2);

%% ══════════════════════════════════════════════════════════════════════
%%  SUB-SIM B: Minimum Amplitude at f_n1 (binary search)
%% ══════════════════════════════════════════════════════════════════════
fprintf('Running sub-sim B: binary search for F0_min at f_n1 = %.2f Hz …\n', f_n1);
F0_lo_bs = 0.001;  F0_hi_bs = F_push_1cell;
for bs_iter = 1:20
    F0_try   = (F0_lo_bs + F0_hi_bs) / 2;
    F_test   = @(t) F0_try .* sin(2*pi*f_n1.*t);
    ode_test = @(t,y) [y(2); (F_test(t) + F_restore(y(1)) - c_hook*y(2)) / m_hook];
    [~, ~, t_snap_test, ~, ~] = ode45(ode_test, [0, t_end_1dof], y0_hook, opts_snap);
    if isempty(t_snap_test)
        F0_lo_bs = F0_try;
    else
        F0_hi_bs = F0_try;
    end
end
F0_min_fn1 = F0_hi_bs;
fprintf('  >>> F0_min at f_n1 = %.4f N\n', F0_min_fn1);

F_minA  = @(t) F0_min_fn1 .* sin(2*pi*f_n1.*t);
ode_minA = @(t,y) [y(2); (F_minA(t) + F_restore(y(1)) - c_hook*y(2)) / m_hook];
[t1m, y1m, t_snap_m, y_snap_m, ~] = ode45(ode_minA, [0, t_end_1dof], y0_hook, opts_snap);
if isempty(t_snap_m)
    warning('Sub-sim B: no snap-through after binary search. Check parameters.');
    t_snap_m = NaN;  t_minA = t1m;  y_minA = y1m;
else
    fprintf('  >>> Sub-sim B snap at t = %.3f s\n', t_snap_m);
    [t2m, y2m] = ode45(ode_decay, [t_snap_m, t_snap_m + 2.0], y_snap_m, opts_free);
    t_minA = [t1m; t2m(2:end)];  y_minA = [y1m; y2m(2:end,:)];
end
x_minA = y_minA(:,1);  v_minA = y_minA(:,2);

%% ══════════════════════════════════════════════════════════════════════
%%  SUB-SIM C: Base Excitation (shared plate + rotating unbalance)
%% ══════════════════════════════════════════════════════════════════════
% Motor spins eccentric mass (m_unbal, e_unbal) on the same plate as the
% bistable mechanism. The plate vibrates; the hook floats on the bistable
% spring and can't follow the plate rigidly, so x_rel (relative to plate)
% drives snap-through.
%
% State: y = [x_P; ẋ_P; x_rel; ẋ_rel]
%   ẍ_P   = (F_motor - k_supp*x_P - c_supp*ẋ_P - F_restore(x_rel) + c_hook*ẋ_rel) / M_plate
%   ẍ_rel = (F_restore(x_rel) - c_hook*ẋ_rel)/m_hook - ẍ_P
fprintf('Running sub-sim C: base excitation (M_plate=%.0fg, f_plate=%.1f Hz, m_e×e=%.1f g·mm) …\n', ...
        M_plate*1e3, f_plate_nat, m_unbal*e_unbal*1e6);

ode_base = @(t,y) base_excit_ode(t, y, M_plate, m_hook, k_supp, c_supp, c_hook, ...
                                  F_restore, f_inst, phi_fun, m_unbal, e_unbal);
y0_base  = [0; 0; 0; 0];   % plate and hook both at rest, hook at State 1

opts_snap_base = odeset('Events', @(t,y) snap_event_base(t, y, d_saddle), ...
                        'RelTol', 1e-7, 'AbsTol', 1e-10, 'MaxStep', 5e-4);

[t1c, y1c, t_snap_c, y_snap_c, ~] = ode45(ode_base, [0, t_end_1dof], y0_base, opts_snap_base);
if isempty(t_snap_c)
    warning(['Sub-sim C: no snap-through. Try: set f_plate_nat=[] (auto-tune), ' ...
             'increase m_unbal*e_unbal, or decrease M_plate.']);
    t_snap_c = NaN;  t_base = t1c;  y_base = y1c;
else
    f_snap_c = f_inst(t_snap_c);
    fprintf('  >>> Sub-sim C snap at  t = %.3f s  |  f_inst = %.2f Hz\n', t_snap_c, f_snap_c);
    % Motor stops at snap; plate and hook oscillate freely to State 2 (m_unbal=0 → F_motor=0)
    ode_base_decay = @(t,y) base_excit_ode(t, y, M_plate, m_hook, k_supp, c_supp, c_hook, ...
                                            F_restore, f_inst, phi_fun, 0, e_unbal);
    [t2c, y2c] = ode45(ode_base_decay, [t_snap_c, t_snap_c + 2.0], y_snap_c, opts_free);
    t_base = [t1c; t2c(2:end)];  y_base = [y1c; y2c(2:end,:)];
end
x_base_rel = y_base(:,3);   % relative hook displacement (what snaps the bistable)
v_base_rel = y_base(:,4);   % relative hook velocity
x_base_plt = y_base(:,1);   % absolute plate displacement (diagnostic)

%% ══════════════════════════════════════════════════════════════════════
%%  2-DOF TMD FREQUENCY RESPONSE (linearized at each bistable state)
%% ══════════════════════════════════════════════════════════════════════
fprintf('Computing 2-DOF TMD frequency response …\n');
omega_vec = 2*pi * linspace(f_TMD_lo, f_TMD_hi, 300);

% Steady-state amplitude via transfer-function approach (linearised TMD spring)
% Main mass equation:  m1*x1dd + c1*x1d + k1*x1 + k_b*(x1-x2) + c_b*(x1d-x2d) = F0*sin
% TMD equation:        m2*x2dd + k_b*(x2-x1) + c_b*(x2d-x1d) = 0
% k_b = effective linearized bistable spring at chosen state
%
% Frequency response amplitude of x1:
amp_noTMD = zeros(size(omega_vec));  % no TMD at all
amp_s1    = zeros(size(omega_vec));  % bistable in State 1
amp_s2    = zeros(size(omega_vec));  % bistable in State 2

for ki = 1:numel(omega_vec)
    om = omega_vec(ki);
    % No TMD: single DOF
    D0 = -m_main*om^2 + k_main + 1i*om*c_main;
    amp_noTMD(ki) = abs(F0_TMD / D0);

    % With TMD (state-dependent k_b)
    % k_b = stiffness of the ruler spring at the TMD mass location.
    % State 1: TMD tuned to main system  (k_b = k_TMD = omega_main^2 * m_TMD).
    % State 2: TMD shifted by d_stroke → assume L_TMD reference = 200 mm;
    %          k scales as (L1/(L1+d_stroke))^3 per the cantilever formula.
    L_TMD_ref  = 0.200;   % [m] approx lever-arm at State 1 (adjustable)
    k_b_state2 = k_TMD * (L_TMD_ref / (L_TMD_ref + d_stroke))^3;
    for state = 1:2
        if state == 1
            k_b = k_TMD;
        else
            k_b = k_b_state2;
        end
        c_b = 2 * zeta_TMD * sqrt(k_b * m_TMD);  % state-dependent TMD damping
        % 2×2 impedance matrix [Z]{X} = {F}
        Z11 = (k_main + k_b - m_main*om^2) + 1i*om*(c_main + c_b);
        Z12 = -(k_b + 1i*om*c_b);
        Z21 = Z12;
        Z22 = (k_b - m_TMD*om^2) + 1i*om*c_b;
        detZ = Z11*Z22 - Z12*Z21;
        X1   = F0_TMD * Z22 / detZ;
        if state == 1, amp_s1(ki) = abs(X1);
        else,          amp_s2(ki) = abs(X1);
        end
    end
end

freq_vec_Hz = omega_vec / (2*pi);

%% ══════════════════════════════════════════════════════════════════════
%%  PRIMARY FIGURE — 5 panels
%% ══════════════════════════════════════════════════════════════════════
if ~DISPLAY_FIGS, set(0,'DefaultFigureVisible','off'); end

fig_main = figure('Name','Bistable Curved Beam — Analysis','NumberTitle','off', ...
                  'Color','w','Position',[50 30 1450 940]);

%% ── Panel 1: Beam geometry ───────────────────────────────────────────
ax1 = subplot(2,3,1);
xi = linspace(0, l_span, 400);
y_state1 =  (h_apex/2) * (1 - cos(2*pi*xi/l_span));   % State 1: arch up
y_state2 = -(h_apex/2) * (1 - cos(2*pi*xi/l_span));   % State 2: arch down (snapped)

hold on;
% Draw wall supports
patch([-1.5, 0, 0, -1.5]*1e3, [-h_apex, -h_apex, h_apex, h_apex]*1.5*1e3, ...
      [0.5 0.5 0.5],'EdgeColor','k','FaceAlpha',0.4,'HandleVisibility','off');
patch([l_span, l_span+1.5e-3, l_span+1.5e-3, l_span]*1e3, ...
      [-h_apex, -h_apex, h_apex, h_apex]*1.5*1e3, ...
      [0.5 0.5 0.5],'EdgeColor','k','FaceAlpha',0.4,'HandleVisibility','off');

fill([xi, fliplr(xi)]*1e3, [y_state1+t_beam/2, fliplr(y_state1-t_beam/2)]*1e3, ...
     [0.4 0.7 1],'EdgeColor','b','LineWidth',1.5,'FaceAlpha',0.7,'DisplayName','State 1 (retracted)');
fill([xi, fliplr(xi)]*1e3, [y_state2+t_beam/2, fliplr(y_state2-t_beam/2)]*1e3, ...
     [0.4 1 0.6],'EdgeColor',[0 0.6 0],'LineWidth',1.5,'FaceAlpha',0.7,'DisplayName','State 2 (deployed)');

% Annotations — consolidated into a single info box in the upper-right corner
% (avoids crowding inside the small beam area)
info_str = sprintf('h = %.1f mm\nt = %.1f mm\nl = %.0f mm\nQ = %.2f  ✓', ...
                   h_apex*1e3, t_beam*1e3, l_span*1e3, Q);
text(0.97, 0.97, info_str, 'Units','normalized', ...
     'HorizontalAlignment','right','VerticalAlignment','top', ...
     'FontSize',8,'FontName','Monospaced', ...
     'BackgroundColor','w','EdgeColor',[0.6 0.6 0.6],'Margin',3);

xlabel('Beam span  (mm)','FontSize',9);
ylabel('Transverse disp.  (mm)','FontSize',9);
title('Beam Geometry  |  Qiu cosine profile','FontWeight','bold');
legend('Location','northeast','FontSize',8);
grid on; box on; set(ax1,'FontSize',8);
ylim([-h_apex*2.5e3, h_apex*2.5e3]);
axis equal; xlim([-2, (l_span+2e-3)*1e3]);

%% ── Panel 2: Force-Displacement ──────────────────────────────────────
ax2 = subplot(2,3,2);
x_fd = linspace(-0.01*d_stroke, 1.05*d_stroke, 800);
% Applied external force (compression test convention): F_ext = -F_restore
F_ext_vec = -F_restore(x_fd);

% Shade push and pull regions
mask_push = F_ext_vec >= 0;
mask_pull = F_ext_vec <  0;
if any(mask_push)
    patch([x_fd(mask_push), fliplr(x_fd(mask_push))]*1e3, ...
          [F_ext_vec(mask_push), zeros(1,sum(mask_push))], ...
          [0.8 1 0.8],'EdgeColor','none','FaceAlpha',0.6); hold on;
end
if any(mask_pull)
    patch([x_fd(mask_pull), fliplr(x_fd(mask_pull))]*1e3, ...
          [F_ext_vec(mask_pull), zeros(1,sum(mask_pull))], ...
          [1 0.8 0.8],'EdgeColor','none','FaceAlpha',0.6);
end
hold on;
plot(x_fd*1e3, F_ext_vec,'b-','LineWidth',2.2);
yline(0,'k--','LineWidth',0.9);
% Vertical lines without labels — annotated by text at mid-height
xline(0,           'g:','LineWidth',1.2,'HandleVisibility','off');
xline(d_stroke*1e3,'g:','LineWidth',1.2,'HandleVisibility','off');
xline(d_saddle*1e3,'r:','LineWidth',1.2,'HandleVisibility','off');

% Peak markers
[Fp_max, ipm] = max(F_ext_vec);
[Fp_min, ipl] = min(F_ext_vec);
plot(x_fd(ipm)*1e3, Fp_max,'ms','MarkerFaceColor','m','MarkerSize',9);
plot(x_fd(ipl)*1e3, Fp_min,'cs','MarkerFaceColor','c','MarkerSize',9);
text(x_fd(ipm)*1e3, Fp_max*0.6, sprintf('F_{push}=%.1f N', Fp_max), ...
     'FontSize',7,'Color','m','HorizontalAlignment','center');
text(x_fd(ipl)*1e3, Fp_min*0.6, sprintf('F_{pop}=%.1f N', Fp_min), ...
     'FontSize',7,'Color','c','HorizontalAlignment','center');
% Vertical line labels at consistent height inside the plot
text(0,             Fp_max*0.85, 'State 1','FontSize',7,'Color',[0 0.5 0],'HorizontalAlignment','center');
text(d_saddle*1e3,  Fp_max*0.85, 'Saddle', 'FontSize',7,'Color',[0.6 0 0],'HorizontalAlignment','center');
text(d_stroke*1e3,  Fp_max*0.85, 'State 2','FontSize',7,'Color',[0 0.5 0],'HorizontalAlignment','center');

xlabel('Displacement  (mm)','FontSize',9);
ylabel('Applied force  (N)','FontSize',9);
title('Force–Displacement (cubic fit, Design 9)','FontWeight','bold');
grid on; box on; set(ax2,'FontSize',8);

%% ── Panel 3: Potential Energy ────────────────────────────────────────
ax3 = subplot(2,3,3);
x_v = linspace(-0.01*d_stroke, 1.05*d_stroke, 800);
V_vec = V_fun(x_v);
V_s1  = V_fun(0);
V_s2  = V_fun(d_stroke);
V_bar = V_fun(d_saddle);

fill([x_v, fliplr(x_v)]*1e3, [V_vec, zeros(1,numel(x_v))]*1e3, ...
     [0.88 0.93 1],'EdgeColor','none','FaceAlpha',0.5,'HandleVisibility','off'); hold on;
plot(x_v*1e3, V_vec*1e3,'b-','LineWidth',2.2);
yline(0,'k--','LineWidth',0.8);
plot([0, d_stroke]*1e3, [V_s1, V_s2]*1e3,'go','MarkerFaceColor','g','MarkerSize',11,'DisplayName','Stable eq.');
plot(d_saddle*1e3, V_bar*1e3,'rv','MarkerFaceColor','r','MarkerSize',11,'DisplayName','Saddle (unstable)');

% Barrier arrows
plot([0 0]*1e3,         [V_s1 V_bar]*1e3,'m-','LineWidth',2,'HandleVisibility','off');
plot([d_stroke d_stroke]*1e3, [V_s2 V_bar]*1e3,'c-','LineWidth',2,'HandleVisibility','off');
text(0.5e3, (V_s1+V_bar)/2*1e3, sprintf(' ΔE₁=%.2f mJ', DeltaE_1*1e3),'Color','m','FontSize',8,'FontWeight','bold');
text((d_stroke-1e-3)*1e3, (V_s2+V_bar)/2*1e3, sprintf('ΔE₂=%.2f mJ ', DeltaE_2*1e3),'Color','c','FontSize',8,'FontWeight','bold','HorizontalAlignment','right');

xlabel('Displacement  (mm)','FontSize',9);
ylabel('V(x)  (mJ)','FontSize',9);
title('Potential Energy Double Well','FontWeight','bold');
legend('Location','northeast','FontSize',8);
grid on; box on; set(ax3,'FontSize',8);

%% ── Panel 4: 1-DOF Time History (minimum-amplitude snap-through) ────
ax4 = subplot(2,3,4);
hold on;
if ~isnan(t_snap_m)
    xline(t_snap_m, 'r:', 'LineWidth', 1.8, ...
          'Label', sprintf('snap  t=%.2f s', t_snap_m), 'FontSize', 8);
end
skip_p4 = max(1, round(numel(t_minA)/120));
idx_p4  = 1:skip_p4:numel(t_minA);
plot(t_minA(idx_p4), x_minA(idx_p4)*1e3, '-', 'Color', [0 0.45 0.74], 'LineWidth', 1.5);
yline(0,            'b--', 'LineWidth', 1.3, ...
      'Label', 'State 1  (0 mm)', 'LabelHorizontalAlignment', 'left', 'FontSize', 8);
yline(d_stroke*1e3, 'g--', 'LineWidth', 1.3, ...
      'Label', sprintf('State 2  (%.1f mm)', d_stroke*1e3), ...
      'LabelHorizontalAlignment', 'left', 'FontSize', 8);
yline(d_saddle*1e3, 'k:',  'LineWidth', 1.0, ...
      'Label', 'Saddle', 'LabelHorizontalAlignment', 'right', 'FontSize', 8);
xlabel('Time  (s)', 'FontSize', 9);
ylabel('Hook displacement  (mm)', 'FontSize', 9);
title(sprintf('Displacement vs. Time  (F_0 = %.3f N  at  f_{n1} = %.1f Hz)', ...
              F0_min_fn1, f_n1), 'FontWeight', 'bold');
grid on; box on; set(ax4, 'FontSize', 8);
xlim([0, t_minA(end)]);
ylim([-0.12*d_stroke*1e3, 1.18*d_stroke*1e3]);

%% ── Panel 5: Phase Portrait (1-DOF) ─────────────────────────────────
ax5 = subplot(2,3,5);
% Separatrix (E = V(saddle))
x_s_vec  = linspace(0, d_stroke, 1000);
E_sep    = V_fun(d_saddle);
vsq_sep  = max(0, 2*(E_sep - V_fun(x_s_vec)) / m_hook);
v_sep    = sqrt(vsq_sep);
     
     hold on;
plot(x_s_vec*1e3,  v_sep*1e3,'r--','LineWidth',1.8,'DisplayName','Separatrix');
plot(x_s_vec*1e3, -v_sep*1e3,'r--','LineWidth',1.8,'HandleVisibility','off');

% Downsample to ≤2000 segments before plotting (avoids 100k+ plot() calls)
N_p  = numel(t_minA);
ds   = max(1, floor(N_p / 2000));
idx  = 1:ds:N_p;
cmap = parula(numel(idx));
for k = 1:numel(idx)-1
    plot(x_minA(idx(k):idx(k+1))*1e3, v_minA(idx(k):idx(k+1))*1e3,'-','Color',cmap(k,:),'LineWidth',0.9,'HandleVisibility','off');
end
plot(nan, nan, '-', 'Color',[0 0.4 0.8], 'LineWidth',2, 'DisplayName','Trajectory (time \rightarrow)');
if ~isnan(t_snap_m)
    [~, is] = min(abs(t_minA - t_snap_m));
    plot(x_minA(is)*1e3, v_minA(is)*1e3,'kp','MarkerSize',14,'MarkerFaceColor','y','DisplayName','Snap-through');
end
plot([0, d_stroke]*1e3, [0,0],'go','MarkerFaceColor','g','MarkerSize',10,'DisplayName','Stable eq.');

cb = colorbar; cb.Label.String = 'Time →'; colormap(ax5,parula); clim([0 t_minA(end)]);
xlabel('x  (mm)','FontSize',9); ylabel('Velocity  (mm/s)','FontSize',9);
title('Phase Portrait — 1-DOF Hook','FontWeight','bold');
legend('Location','northeast','FontSize',7); grid on; box on; set(ax5,'FontSize',8);

%% ── Panel 6: TMD Frequency Response ─────────────────────────────────
ax6 = subplot(2,3,6);
hold on;
plot(freq_vec_Hz, amp_s1*1e3,   'r--','LineWidth',2,'DisplayName',sprintf('State 1 (k_b=%.0f N/m)',k_eff_s1));
plot(freq_vec_Hz, amp_s2*1e3,   'g:','LineWidth',2,'DisplayName',sprintf('State 2 (k_b=%.0f N/m)',k_eff_s2));
xline(f_main,'k--','LineWidth',1,'Label',sprintf('f_{main}=%.1f Hz',f_main),'FontSize',8);
xlabel('Excitation frequency  (Hz)','FontSize',9);
ylabel('|X_1|  (mm)','FontSize',9);
title(sprintf('Frequency Response — %s params', k_main_mode),'FontWeight','bold');
legend('Location','northwest','FontSize',8); grid on; box on; set(ax6,'FontSize',8);

sgtitle(sprintf(['Curved-Beam Bistable (Design 9)  |  h=%.1f mm, t=%.1f mm, Q=%.2f, ' ...
                 '%d cells, stroke=%.1f mm'], ...
        h_apex*1e3, t_beam*1e3, Q, n_cells, d_stroke*1e3), ...
        'FontSize',12,'FontWeight','bold');

%% ══════════════════════════════════════════════════════════════════════
%%  EXCITATION COMPARISON FIGURE
%%  Shows how much force each strategy delivers at each frequency,
%%  versus the minimum force needed to snap at resonance.
%% ══════════════════════════════════════════════════════════════════════
fig_excit = figure('Name','Bistable Curved Beam — Excitation Comparison', ...
                   'NumberTitle','off','Color','w','Position',[80 80 950 530]);

f_comp   = linspace(0.5, f_chirp_hi*1.5, 600);
om_comp  = 2*pi*f_comp;
F_thresh = 2*zeta_hook*k_eff_s1*d_saddle;   % snap threshold (linear resonance)

% Base excitation: plate Bode magnitude × inertial loading on hook
X_plate_amp = (m_unbal*e_unbal .* om_comp.^2) ./ ...
              sqrt((k_supp - M_plate.*om_comp.^2).^2 + (c_supp.*om_comp).^2);
F_eff_base  = m_hook .* om_comp.^2 .* X_plate_amp;   % effective force on hook [N]

% Raw centrifugal force (what you'd get if the unbalance pushed the hook directly)
F_raw_unbal = m_unbal * e_unbal .* om_comp.^2;

% Direct sinusoidal drive at F0_chirp (flat line for reference)
F_direct_ref = F0_chirp * ones(size(f_comp));

ax_ec = axes; hold on;
fill([f_comp, fliplr(f_comp)], [F_eff_base, zeros(1,numel(f_comp))], ...
     [1 0.82 0.5], 'EdgeColor','none', 'FaceAlpha',0.45, 'HandleVisibility','off');
plot(f_comp, F_eff_base,    '-',  'Color',[0.85 0.40 0], 'LineWidth',2.5, ...
     'DisplayName', sprintf('Base excitation (M_{plate}=%.0fg, \\zeta_{plate}=%.2f, f_{plate}=%.1fHz)', ...
                            M_plate*1e3, zeta_plate, f_plate_nat));
plot(f_comp, F_raw_unbal,   'b--', 'LineWidth',1.5, ...
     'DisplayName', sprintf('Raw centrifugal (direct, m_e×e=%.1f g·mm)', m_unbal*e_unbal*1e6));
plot(f_comp, F_direct_ref,  'k:', 'LineWidth',1.5, ...
     'DisplayName', sprintf('Direct sinusoidal F_0 = %.1f N', F0_chirp));
yline(F_thresh, 'r-', 'LineWidth',2.2, ...
      'Label', sprintf('Snap threshold  %.2f N', F_thresh), ...
      'LabelHorizontalAlignment','right', 'FontSize',9);
xline(f_n1, 'k--', 'LineWidth',1.2, ...
      'Label', sprintf('f_{n1}=%.1f Hz', f_n1), 'FontSize',8, 'HandleVisibility','off');
if abs(f_plate_nat - f_n1) > 0.5
    xline(f_plate_nat, 'm--', 'LineWidth',1.2, ...
          'Label', sprintf('f_{plate}=%.1f Hz', f_plate_nat), 'FontSize',8, ...
          'HandleVisibility','off');
end

xlabel('Excitation frequency  (Hz)', 'FontSize',11);
ylabel('Force amplitude at hook  (N)', 'FontSize',11);
title(sprintf('Excitation Strategies  |  m_e×e = %.1f g·mm', m_unbal*e_unbal*1e6), ...
      'FontWeight','bold', 'FontSize',11);
legend('Location','northwest', 'FontSize',9);
grid on; box on; set(ax_ec,'FontSize',9);
xlim([0, f_chirp_hi*1.5]);
y_top = max([max(F_eff_base), max(F_raw_unbal), F_thresh*2]);
ylim([0, y_top*1.15]);

annotation('textbox',[0.13 0.76 0.35 0.14],'String', ...
    {sprintf('Tuned-plate snap margin:  %.2fx (threshold %.1f g·mm)', ...
             me_product/me_thresh_tuned, me_thresh_tuned*1e6), ...
     sprintf('Off-res. snap margin:  %.2fx (threshold %.1f g·mm)', ...
             me_product/me_thresh_detuned, me_thresh_detuned*1e6)}, ...
    'BackgroundColor','w','EdgeColor',[0.6 0.6 0.6],'FontSize',8.5,'FitBoxToText','on');

%% ══════════════════════════════════════════════════════════════════════
%%  SECONDARY FIGURE — Animation
%% ══════════════════════════════════════════════════════════════════════
if ANIMATE
    % Minimum amplitude at f_n1 — exact threshold snap + free decay to State 2
    run_beam_animation(t_minA, x_minA, v_minA, d_stroke, d_saddle, ...
                       V_fun, t_snap_m, t_minA(end), m_hook, ...
                       l_span, h_apex, t_beam, @(t) f_n1, ' — Min Amplitude fn1');
end

%% ══════════════════════════════════════════════════════════════════════
%%  AUTO-SAVE ALL FIGURES
%% ══════════════════════════════════════════════════════════════════════
save_dir = fileparts(mfilename('fullpath'));
if isempty(save_dir), save_dir = pwd; end
fig_dir  = fullfile(save_dir, 'figures', 'sim2_curved_beam');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

fig_list = findobj('Type','figure');
for fi = 1:numel(fig_list)
    fh  = fig_list(fi);
    nm  = fh.Name;
    nm  = regexprep(nm, '[^a-zA-Z0-9_\- ]', '');
    nm  = strtrim(strrep(nm, ' ', '_'));
    out = fullfile(fig_dir, [nm '.png']);
    exportgraphics(fh, out, 'Resolution', 150);
    fprintf('Saved: %s\n', out);
end

set(0,'DefaultFigureVisible','on');   % restore default before closing
close all;

%% ══════════════════════════════════════════════════════════════════════
%%  LOCAL FUNCTIONS
%% ══════════════════════════════════════════════════════════════════════

function [val, isterm, dir] = snap_event(~, y, d_saddle)
%SNAP_EVENT  Detect hook displacement crossing the saddle point.
    val    = y(1) - d_saddle;
    isterm = 1;
    dir    = 1;   % positive-crossing only (State 1 → State 2)
end

function [val, isterm, dir] = snap_event_base(~, y, d_saddle)
%SNAP_EVENT_BASE  Snap detection for base excitation state [x_P; ẋ_P; x_rel; ẋ_rel].
%  Monitors y(3) = x_rel (hook displacement relative to plate).
    val    = y(3) - d_saddle;
    isterm = 1;
    dir    = 1;
end

function dydt = base_excit_ode(t, y, M_plate, m_hook, k_supp, c_supp, c_hook, ...
                                F_restore, f_inst, phi_fun, m_unbal, e_unbal)
%BASE_EXCIT_ODE  2-DOF base excitation model: rigid plate + hook on bistable spring.
%
%  State: y = [x_P; ẋ_P; x_rel; ẋ_rel]
%    x_P   = absolute plate displacement [m]
%    x_rel = hook displacement relative to plate [m]  ← drives snap-through
%
%  Physics:
%    M_plate*ẍ_P   = F_motor − k_supp*x_P − c_supp*ẋ_P − F_rest + c_hook*ẋ_rel
%    m_hook *ẍ_H   = F_rest − c_hook*ẋ_rel
%    ẍ_rel = ẍ_H − ẍ_P
%
%  F_motor = m_e*e*ω(t)²*sin(φ(t))  — rotating unbalance centrifugal force

    x_P    = y(1);  xd_P   = y(2);
    x_rel  = y(3);  xd_rel = y(4);
    omega  = 2*pi * f_inst(t);
    F_mot  = m_unbal * e_unbal * omega^2 * sin(phi_fun(t));
    F_rest = F_restore(x_rel);
    xdd_P  = (F_mot - k_supp*x_P - c_supp*xd_P - F_rest + c_hook*xd_rel) / M_plate;
    xdd_H  = (F_rest - c_hook*xd_rel) / m_hook;
    dydt   = [xd_P; xdd_P; xd_rel; xdd_H - xdd_P];
end

function run_beam_animation(t_all, x_all, v_all, d_stroke, d_saddle, ...
                             V_fun, t_snap, ~, m_hook, l_span, h_apex, t_beam, f_inst, fig_label)
%RUN_BEAM_ANIMATION  Three-panel animation: beam shape, potential well, phase portrait.
    if nargin < 14 || isempty(fig_label), fig_label = ''; end
    fig_name = ['Curved-Beam Bistable — Animation' fig_label];
    fig = figure('Name', fig_name, 'NumberTitle','off', ...
                 'Color','w','Position',[500 120 1300 520]);

    xi  = linspace(0, l_span, 300);
    x_v = linspace(0, d_stroke, 500);
    V_v = V_fun(x_v);
    E_sep = V_fun(d_saddle);
    vsq  = max(0, 2*(E_sep - V_fun(x_v)) / m_hook);
    v_sep = sqrt(vsq);

    % ── Left: beam shape ──
    ax_B = subplot(1,3,1);
    ys1  = (h_apex/2)*(1-cos(2*pi*xi/l_span));
    ys2  = -ys1;
    hold on;
    hbeam = fill([xi, fliplr(xi)]*1e3, [ys1+t_beam/2, fliplr(ys1-t_beam/2)]*1e3, ...
                 [0.4 0.7 1],'EdgeColor','b','LineWidth',1.5,'FaceAlpha',0.7);
    patch([-1,0,0,-1]*1e3,  [-h_apex,-h_apex,h_apex,h_apex]*1.5*1e3,[0.5 0.5 0.5],'EdgeColor','k');
    patch([l_span,(l_span+1e-3),(l_span+1e-3),l_span]*1e3, [-h_apex,-h_apex,h_apex,h_apex]*1.5*1e3,[0.5 0.5 0.5],'EdgeColor','k');
    htb = title('t=0.000 s  State 1','FontSize',10); xlabel('mm'); ylabel('mm');
    axis equal; xlim([-2,(l_span+2e-3)*1e3]); ylim([-h_apex*2.5e3,h_apex*2.5e3]);
    grid on; box on;

    % ── Center: hook displacement vs. time (live history) ──
    ax_F = subplot(1,3,2);
    hold on;
    % State and saddle reference lines
    yline(0,            'b--', 'LineWidth', 1.3, ...
          'Label', 'State 1  (0 mm)', 'LabelHorizontalAlignment', 'left', 'FontSize', 8);
    yline(d_stroke*1e3, 'g--', 'LineWidth', 1.3, ...
          'Label', sprintf('State 2  (%.1f mm)', d_stroke*1e3), ...
          'LabelHorizontalAlignment', 'left', 'FontSize', 8);
    yline(d_saddle*1e3, 'k:',  'LineWidth', 1.0, ...
          'Label', 'Saddle', 'LabelHorizontalAlignment', 'right', 'FontSize', 8);
    if ~isnan(t_snap)
        xline(t_snap, 'r:', 'LineWidth', 1.8, ...
              'Label', sprintf('snap  t=%.2f s', t_snap), 'FontSize', 8);
    end
    % Animated trace — builds up progressively as animation plays
    % All ODE points between frames are flushed per drawnow, so no aliasing
    h_xtrace = animatedline('Color', [0 0.45 0.74], 'LineWidth', 1.5);
    % Current-position marker
    h_xcur = plot(t_all(1), x_all(1)*1e3, 'o', ...
                  'Color', [0.85 0.33 0.1], 'MarkerFaceColor', [0.85 0.33 0.1], ...
                  'MarkerSize', 9, 'HandleVisibility', 'off');
    xlabel('Time  (s)', 'FontSize', 9);
    ylabel('Hook displacement  (mm)', 'FontSize', 9);
    title('Displacement vs. Time', 'FontWeight', 'bold');
    xlim([0, t_all(end)]);
    ylim([-0.12*d_stroke*1e3, 1.18*d_stroke*1e3]);
    grid on; box on;

    % ── Right: phase portrait ──
    ax_P = subplot(1,3,3);
    fill([x_v,fliplr(x_v)]*1e3,[v_sep,-fliplr(v_sep)]*1e3,[1 0.9 0.9],'EdgeColor','none','FaceAlpha',0.4); hold on;
    plot(x_v*1e3, v_sep*1e3,'r--','LineWidth',1.5);
    plot(x_v*1e3,-v_sep*1e3,'r--','LineWidth',1.5);
    hph = animatedline('Color','b','LineWidth',1.2);
    hpc = plot(0,0,'ko','MarkerFaceColor','r','MarkerSize',9);
    xlabel('x (mm)'); ylabel('Velocity (mm/s)'); title('Phase Portrait'); grid on; box on;
    xlim([-0.05*d_stroke, 1.1*d_stroke]*1e3);
    v_range = max(abs(v_all)) * 1.3e3;
    v_range = max(v_range, max(v_sep)*1e3 * 0.15);   % never collapse below 15% of separatrix
    ylim([-v_range, v_range]);

    % GIF setup
    gif_dir   = fileparts(mfilename('fullpath'));
    if isempty(gif_dir), gif_dir = pwd; end
    gif_dir   = fullfile(gif_dir, 'figures', 'sim2_curved_beam');
    if ~exist(gif_dir, 'dir'), mkdir(gif_dir); end
    gif_suffix = strrep(strtrim(fig_label), ' ', '_');
    gif_path  = fullfile(gif_dir, ['Bistable_Curved_Beam_Animation' gif_suffix '.gif']);
    gif_delay = 0.06;   % [s] per frame — adjust for playback speed
    has_ipt   = license('test','image_toolbox') && exist('rgb2ind','file') == 2;
    if ~has_ipt
        warning('Image Processing Toolbox not found — animation will play but GIF will not be saved.');
    end
    first_gif = true;

    skip = max(1, round(numel(t_all)/120));   % ~120 frames (GIF + display)
    if has_ipt
        fprintf('  Rendering animation (%d frames) -> %s\n', ceil(numel(t_all)/skip), gif_path);
    else
        fprintf('  Rendering animation (%d frames) — display only\n', ceil(numel(t_all)/skip));
    end

    for k = 1:skip:numel(t_all)
        xk = x_all(k); vk = v_all(k); tk = t_all(k);
        frac = min(1, max(0, xk/d_stroke));
        col  = (1-frac)*[0.4 0.7 1] + frac*[0.4 1 0.6];
        deployed = ~isnan(t_snap) && tk >= t_snap;
        bc   = [0 0.65 0]*deployed + [0.85 0 0]*(~deployed);

        % update beam shape (interpolate between state1 and state2)
        y_interp = (1-frac)*((h_apex/2)*(1-cos(2*pi*xi/l_span))) + ...
                   frac*   (-(h_apex/2)*(1-cos(2*pi*xi/l_span)));
        set(hbeam,'XData',[xi,fliplr(xi)]*1e3, ...
                  'YData',[y_interp+t_beam/2,fliplr(y_interp-t_beam/2)]*1e3, ...
                  'FaceColor',col,'EdgeColor',col*0.6);

        % Displacement-vs-time: one point per frame (sparse/jagged look)
        addpoints(h_xtrace, tk, xk*1e3);
        set(h_xcur, 'XData', tk, 'YData', xk*1e3);
        fi = f_inst(tk);   % still needed for title

        set(hpc,  'XData',xk*1e3,'YData',vk*1e3,'MarkerFaceColor',bc);
        addpoints(hph, xk*1e3, vk*1e3);
        state_str = 'State 2 (Deployed)';
        if ~deployed, state_str = 'State 1 (Retracted)'; end
        set(htb,'String',sprintf('t=%.3f s | %.1f Hz | %s', tk, fi, state_str));
        drawnow;   % full render so getframe captures a complete image

        % Capture and append GIF frame (requires Image Processing Toolbox)
        if has_ipt
            fr = getframe(fig);
            im = frame2im(fr);
            [imind, cm] = rgb2ind(im, 256);
            if first_gif
                imwrite(imind, cm, gif_path, 'gif', 'Loopcount', inf, 'DelayTime', gif_delay);
                first_gif = false;
            else
                imwrite(imind, cm, gif_path, 'gif', 'WriteMode', 'append', 'DelayTime', gif_delay);
            end
        end
    end
    if has_ipt, fprintf('  GIF saved: %s\n', gif_path); end
end
