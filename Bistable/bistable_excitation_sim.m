%% bistable_excitation_sim.m
% Mass Actuation Force Optimization for Bistable Hook Deployment
%
% Physical chain (Zolfagharian Design 9):
%
%   ROBOT CHASSIS
%       │  k_supp, c_supp  ← spring isolators
%       │
%   RIGID ASSEMBLY  (M_plate = motor + plate + external blocks)
%       │              external blocks = beam end pinned supports
%       │
%     BEAM PACK          ← parallel bistable curved beams (Qiu cosine profile)
%       └──────┬────────┘
%              │
%           HOOK  (m_hook = reference mass at beam apex)
%
% Two excitation configurations are optimised:
%
%   §4.12 — Proof-Mass Oscillator
%     Motor on rigid assembly, drives proof mass M_pm at stroke Δ.
%     Velocity-limited: Δ = ω_shaft_max / ω  (480 RPM = 50.3 rad/s limit).
%     Optimise: find minimum M_pm at the best drive frequency.
%
%   Config 2 — Prescribed Base Motion
%     Motor off the plate, drives plate+external blocks via bearing/lever.
%     Motor prescribes x_P(t) = A × sin(ω_n1 × t).
%     No added mass — optimise for minimum motor torque (drive at f_n1).
%
% Key analysis outputs:
%   Figure 1: §4.12 — 1-D frequency sweep + simulation at optimal point
%   Figure 2: Config 2 — torque vs frequency + simulation at f_n1
%   Figure 3: 3-D M_pm_min(f_drive, m_hook) surface
%   Figure 4: 3-D torque(f_drive, m_hook) surface (analytical, Config 2)

clear; clc; close all;

%% ══════════════════════════════════════════════════════════════════════
%%  TUNABLE PARAMETERS
%% ══════════════════════════════════════════════════════════════════════

%% ── Beam geometry ────────────────────────────────────────────────────
E_beam  = 2300e6;   % [Pa]  Young's modulus (stiffer polymer — PEEK ~4 GPa, CF-nylon ~6 GPa)
%                           Set to 2300 MPa (ABS baseline); increase for stiffer material
t_beam  = 0.6e-3;   % [m]  Beam thickness (out-of-plane is b_beam)
l_span  = 42e-3;    % [m]  Beam span (hinge-to-hinge)
h_apex  = 3.5e-3;   % [m]  Apex height  (modest increase from 2.5mm — avoids f_rel leaving sweep)
b_beam_scale = 1.5; % [-]  Width multiplier vs original Design 9
%   Q = h_apex/t_beam must be >= 2.35 for bistability

%% ── Bistable calibration — ANALYTICAL (Qiu 2003 energy model) ───────
% Back-calculate original beam width b from Design 9 paper value.
% Then compute F_push_1cell for any (h_apex, b_beam_scale) analytically.
%
% Formula (clamped cosine arch, mode-2 suppressed):
%   F_push = (E × I × π⁴ × h / l³) × (Q²-1/3) × sqrt(3Q²-1) / (3Q)
%   where I = b × t³/12,  Q = h/t
%
h_orig      = 2.5e-3;     % [m]  Original Design 9 apex height
F_push_orig = 12.55;      % [N]  Measured value from Zolfagharian 2025 (Design 9)
Q_orig      = h_orig / t_beam;
I_per_b_orig = t_beam^3 / 12;   % I per unit width [m³]
F_per_b_orig = E_beam * I_per_b_orig * pi^4 * h_orig / l_span^3 ...
               * (Q_orig^2 - 1/3) * sqrt(3*Q_orig^2-1) / (3*Q_orig);
b_beam      = F_push_orig / F_per_b_orig;      % [m]  back-calculated width
b_beam      = b_beam * b_beam_scale;           % [m]  scaled width

% Compute F_push_1cell for current geometry
Q_beam      = h_apex / t_beam;
Q_min       = 2.35;
assert(Q_beam >= Q_min, 'Q = %.3f < %.2f — increase h_apex or reduce t_beam', Q_beam, Q_min);
I_beam      = b_beam * t_beam^3 / 12;
F_push_1cell = E_beam * I_beam * pi^4 * h_apex / l_span^3 ...
               * (Q_beam^2 - 1/3) * sqrt(3*Q_beam^2-1) / (3*Q_beam);

% Use the d/h ≈ 2.2 ratio observed in Design 9 (accounts for clamped BCs).
% In the robot layout the beams act in parallel: displacement stays at the
% single-beam stroke, while force and stiffness scale with beam count.
d_stroke_1cell = h_apex * 2.2;  % [m]  single-beam stroke
d_saddle_frac  = 0.545;         % [-]  saddle fraction of stroke
n_cells        = 5;             % [-]  parallel beam count

%% ── Hook (reference mass at beam apex) ──────────────────────────────
m_hook     = 0.200;   % [kg]  Hook = reference mass
zeta_hook  = 0.02;    % [-]   Bistable damping ratio  (stiffer polymer, e.g. PEEK/CF-nylon)

%% ── Rigid assembly (plate + motor + external blocks) ─────────────────
M_plate    = 0.045;   % [kg]  Motor + plate + external blocks  (lightweight design)
zeta_plate = 0.02;    % [-]   Isolator damping

%% ── RS05 motor constraints (RS05 User Manual Rev.260428) ─────────────
% 480 RPM = max SHAFT ANGULAR VELOCITY = 50.3 rad/s (velocity limit).
% For oscillation: peak velocity = amplitude × omega ≤ 50.3 rad/s.
% For continuous rotation (not used here): max 8 Hz.
omega_shaft_max = 2*pi * (480/60);   % [rad/s]  50.3 rad/s (velocity limit)
f_max_rotation  = 480/60;            % [Hz]  8 Hz — continuous rotation limit (§4.11 only)

%% ── §4.12: Proof-mass oscillator ─────────────────────────────────────
M_pm_nominal = 0.020;   % [kg]  Nominal proof mass (used in 1-D sweep)
lever_arm    = 0.050;   % [m]   Motor shaft-to-plate contact

%% ── Config 2: Prescribed base motion ────────────────────────────────
t_end_c2 = 20.0;   % [s]  Max sim duration

%% ── Sweep parameters ─────────────────────────────────────────────────
f_sweep_1d  = linspace(20, 650, 50);  % [Hz]  1-D frequency sweep for §4.12
t_end_sweep = 2.0;                    % [s]   ODE time limit in sweep

n_cells_vec = 3:2:11;                 % [3 5 7 9 11]  parallel beam count sweep

% 3-D grid parameters
f_grid_vec     = linspace(20, 650, 18);    % [Hz]  frequency axis
mhook_grid_vec = linspace(0.020, 0.250, 14);  % [kg]  hook mass axis

ANIMATE      = true;
DISPLAY_FIGS = false;

%% ══════════════════════════════════════════════════════════════════════
%%  DERIVED BISTABLE QUANTITIES
%% ══════════════════════════════════════════════════════════════════════
d_stroke = d_stroke_1cell;
d_saddle = d_saddle_frac * d_stroke;
d_push   = 0.27 * d_stroke;
F_push_total = n_cells * F_push_1cell;

denom    = d_push * (d_push - d_saddle) * (d_push - d_stroke);
A_coeff  = F_push_total / denom;

F_restore = @(x) -A_coeff .* x .* (x - d_saddle) .* (x - d_stroke);
ds_v = d_saddle; df_v = d_stroke;
V_fun = @(x) A_coeff .* (x.^4/4 - (ds_v+df_v)*x.^3/3 + ds_v*df_v*x.^2/2);

k_eff_s1 = A_coeff * d_saddle * d_stroke;
k_eff_s2 = A_coeff * d_stroke * (d_stroke - d_saddle);

omega_n1 = sqrt(k_eff_s1 / m_hook);  f_n1 = omega_n1/(2*pi);
omega_n2 = sqrt(k_eff_s2 / m_hook);  f_n2 = omega_n2/(2*pi);
c_hook   = 2 * zeta_hook * m_hook * omega_n1;

F0_min_snap = 2 * zeta_hook * k_eff_s1 * d_saddle;

% Coupled relative-mode resonance (§4.12 optimal drive frequency)
M_red   = m_hook * M_plate / (m_hook + M_plate);
f_rel   = sqrt(k_eff_s1 / M_red) / (2*pi);

% Config 2 analytical minimum torque (at resonance, independent of m_hook)
tau_resonance = 2 * zeta_hook * k_eff_s1 * d_saddle * lever_arm;

fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('   Beam Geometry (Qiu 2003 analytical model)\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  Back-calculated beam width  b = %.2f mm  (%.1f× original)\n', b_beam*1e3, b_beam_scale);
fprintf('  Q = h/t = %.2f  (bistability threshold %.2f ✓)\n', Q_beam, Q_min);
fprintf('  F_push_1cell (analytical) = %.2f N\n', F_push_1cell);
fprintf('  F_push total (N=%d parallel) = %.2f N\n', n_cells, F_push_total);
fprintf('  d_stroke_1cell            = %.2f mm\n', d_stroke_1cell*1e3);
fprintf('  d_stroke mechanism        = %.2f mm  (parallel beams do not add stroke)\n', d_stroke*1e3);
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('   Mass Actuation Force Optimisation — Parameters\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  k_eff_s1 = %.1f N/m  |  d_saddle = %.2f mm  |  d_stroke = %.1f mm\n', ...
        k_eff_s1, d_saddle*1e3, d_stroke*1e3);
fprintf('  f_n1 = %.2f Hz  |  f_rel = %.2f Hz  |  F0_min = %.3f N\n', ...
        f_n1, f_rel, F0_min_snap);
fprintf('  omega_shaft_max = %.1f rad/s  (480 RPM velocity limit)\n', omega_shaft_max);
fprintf('  Config 2 resonant torque = %.4f N·m  (%.1f%% of RS05 rated 1.6 N·m)\n', ...
        tau_resonance, tau_resonance/1.6*100);
fprintf('══════════════════════════════════════════════════════════════\n\n');

%% ══════════════════════════════════════════════════════════════════════
%%  ODE OPTIONS
%% ══════════════════════════════════════════════════════════════════════
opts_snap4 = odeset('Events', @(t,y) snap_event_base(t,y,d_saddle), ...
                    'RelTol',1e-7,'AbsTol',1e-10,'MaxStep',5e-4);
opts_snap2 = odeset('Events', @(t,y) snap_event_2state(t,y,d_saddle), ...
                    'RelTol',1e-7,'AbsTol',1e-10,'MaxStep',5e-4);
opts_free  = odeset('RelTol',1e-7,'AbsTol',1e-10,'MaxStep',5e-4);
y0_base    = [0;0;0;0];

%% ══════════════════════════════════════════════════════════════════════
%%  §4.12 — 1-D FREQUENCY SWEEP  (find M_pm_min at each frequency)
%% ══════════════════════════════════════════════════════════════════════
fprintf('Computing §4.12 1-D frequency sweep (%d points) …\n', numel(f_sweep_1d));
M_pm_min_1d  = zeros(size(f_sweep_1d));
Delta_vlim_1d = zeros(size(f_sweep_1d));
tau_1d       = zeros(size(f_sweep_1d));

for fi = 1:numel(f_sweep_1d)
    omega_i  = 2*pi*f_sweep_1d(fi);
    Delta_i  = lever_arm * omega_shaft_max / omega_i;  % [m] linear stroke = r × θ_max
    % Binary search: minimum M_pm to snap at this frequency
    Mpm_lo = 0;  Mpm_hi = 5.0;                    % [kg] search range
    for bs = 1:15
        Mpm_try   = (Mpm_lo + Mpm_hi)/2;
        M_eff_i   = M_plate + Mpm_try;
        k_s_i     = M_eff_i * omega_i^2;
        c_s_i     = 2*zeta_plate*M_eff_i*omega_i;
        ode_i     = @(t,y) excit_pm_ode(t,y,M_eff_i,m_hook,k_s_i,c_s_i,c_hook, ...
                                          F_restore,Mpm_try,Delta_i,omega_i);
        [~,~,ts,~,~] = ode45(ode_i,[0,t_end_sweep],y0_base,opts_snap4);
        if isempty(ts), Mpm_lo = Mpm_try; else, Mpm_hi = Mpm_try; end
    end
    M_pm_min_1d(fi)   = Mpm_hi;
    Delta_vlim_1d(fi) = Delta_i;
    % M_eff used only to compute tau_1d; M_eff_opt is computed separately below
    tau_1d(fi)        = Mpm_hi * Delta_i * omega_i^2 * lever_arm;
end

[M_pm_opt_1d, idx_opt] = min(M_pm_min_1d);
f_opt     = f_sweep_1d(idx_opt);
omega_opt = 2*pi*f_opt;
Delta_opt = lever_arm * omega_shaft_max / omega_opt;   % [m]
fprintf('  Optimal: f_opt = %.1f Hz | M_pm_min = %.1f g | Δ = %.2f mm | τ = %.3f N·m\n', ...
        f_opt, M_pm_opt_1d*1e3, Delta_opt*1e3, tau_1d(idx_opt));

%% §4.12 — Simulation at optimal point
fprintf('Running §4.12 at optimal point (f = %.1f Hz, M_pm = %.1f g) …\n', ...
        f_opt, M_pm_opt_1d*1e3);
M_eff_opt = M_plate + M_pm_opt_1d;
k_s_opt   = M_eff_opt * omega_opt^2;
c_s_opt   = 2*zeta_plate*M_eff_opt*omega_opt;
ode_pm    = @(t,y) excit_pm_ode(t,y,M_eff_opt,m_hook,k_s_opt,c_s_opt,c_hook, ...
                                  F_restore,M_pm_opt_1d,Delta_opt,omega_opt);
[t1p,y1p,t_snap_p,y_snap_p,~] = ode45(ode_pm,[0,30],y0_base,opts_snap4);
if isempty(t_snap_p)
    warning('§4.12: no snap at optimal point — increase t_end or refine sweep.');
    t_snap_p = NaN;  t_pm = t1p;  y_pm = y1p;
else
    fprintf('  >>> Snap at t = %.3f s\n', t_snap_p);
    ode_pm_d = @(t,y) excit_pm_ode(t,y,M_eff_opt,m_hook,k_s_opt,c_s_opt,c_hook, ...
                                     F_restore,0,Delta_opt,omega_opt);
    [t2p,y2p] = ode45(ode_pm_d,[t_snap_p,t_snap_p+2],y_snap_p,opts_free);
    t_pm = [t1p;t2p(2:end)];  y_pm = [y1p;y2p(2:end,:)];
end
x_rel_p = y_pm(:,3);  v_rel_p = y_pm(:,4);

%% ══════════════════════════════════════════════════════════════════════
%%  CONFIG 2 — PRESCRIBED BASE MOTION at f_n1
%% ══════════════════════════════════════════════════════════════════════
fprintf('Running Config 2 at f_n1 = %.2f Hz …\n', f_n1);
% Analytical minimum plate amplitude: x_rel_ss = A/(2*zeta_h) >= d_saddle
A_min_c2 = 2 * zeta_hook * d_saddle;
tau_c2_analytic = m_hook * A_min_c2 * omega_n1^2 * lever_arm;
vpeak_c2 = (A_min_c2/lever_arm) * omega_n1;
fprintf('  A_min = %.3f mm | τ = %.4f N·m (%.1f%% rated) | v_peak = %.2f rad/s\n', ...
        A_min_c2*1e3, tau_c2_analytic, tau_c2_analytic/1.6*100, vpeak_c2);

% Binary search to confirm numerical minimum
A_lo=0; A_hi=d_saddle*3;
for bs=1:20
    A_try = (A_lo+A_hi)/2;
    ode_t = @(t,y) excit_prescribed_ode(t,y,m_hook,c_hook,F_restore,A_try,omega_n1);
    [~,~,ts,~,~] = ode45(ode_t,[0,t_end_c2],[0;0],opts_snap2);
    if isempty(ts), A_lo=A_try; else, A_hi=A_try; end
end
A_min_c2_num = A_hi;
tau_c2_num   = m_hook * A_min_c2_num * omega_n1^2 * lever_arm;
fprintf('  Numerical A_min = %.3f mm  (analytical = %.3f mm)\n', ...
        A_min_c2_num*1e3, A_min_c2*1e3);
fprintf('  Numerical torque = %.4f N·m  (%.1f%% rated)\n', ...
        tau_c2_num, tau_c2_num/1.6*100);

ode_c2 = @(t,y) excit_prescribed_ode(t,y,m_hook,c_hook,F_restore,A_min_c2_num,omega_n1);
[t1c,y1c,t_snap_c,y_snap_c,~] = ode45(ode_c2,[0,t_end_c2],[0;0],opts_snap2);
if isempty(t_snap_c)
    warning('Config 2: no snap. Check parameters.');
    t_snap_c=NaN; t_c2=t1c; y_c2=y1c;
else
    fprintf('  >>> Snap at t = %.3f s\n', t_snap_c);
    ode_c2d = @(t,y) excit_prescribed_ode(t,y,m_hook,c_hook,F_restore,0,omega_n1);
    [t2c,y2c] = ode45(ode_c2d,[t_snap_c,t_snap_c+2],y_snap_c,opts_free);
    t_c2=[t1c;t2c(2:end)]; y_c2=[y1c;y2c(2:end,:)];
end
x_rel_c2=y_c2(:,1); v_rel_c2=y_c2(:,2);

%% ══════════════════════════════════════════════════════════════════════
%%  3-D GRID: M_pm_min(f_drive, m_hook)  — Figure 3
%% ══════════════════════════════════════════════════════════════════════
fprintf('Computing 3-D M_pm_min surface (%d × %d = %d grid points) …\n', ...
        numel(f_grid_vec), numel(mhook_grid_vec), numel(f_grid_vec)*numel(mhook_grid_vec));

Nf = numel(f_grid_vec);
Nm = numel(mhook_grid_vec);
surf_Mpm   = zeros(Nm, Nf);   % Z: M_pm_min [g]
surf_fn1   = zeros(1,  Nm);   % resonance curve: f_n1(m_hook) [Hz]
surf_frel  = zeros(1,  Nm);

for mi = 1:Nm
    mh_i        = mhook_grid_vec(mi);
    omega_n1_i  = sqrt(k_eff_s1/mh_i);
    c_hook_i    = 2*zeta_hook*mh_i*omega_n1_i;
    surf_fn1(mi) = omega_n1_i/(2*pi);
    Mred_i       = mh_i*M_plate/(mh_i+M_plate);
    surf_frel(mi)= sqrt(k_eff_s1/Mred_i)/(2*pi);
    for fi = 1:Nf
        omega_i = 2*pi*f_grid_vec(fi);
        Delta_i = lever_arm * omega_shaft_max / omega_i;   % [m] linear stroke = r × θ_max
        Mpm_lo=0; Mpm_hi=5.0;
        for bs=1:12
            Mpm_t  = (Mpm_lo+Mpm_hi)/2;
            Meff_t = M_plate+Mpm_t;
            ks_t   = Meff_t*omega_i^2;
            cs_t   = 2*zeta_plate*Meff_t*omega_i;
            ode_t  = @(t,y) excit_pm_ode(t,y,Meff_t,mh_i,ks_t,cs_t,c_hook_i, ...
                                           F_restore,Mpm_t,Delta_i,omega_i);
            [~,~,ts,~,~] = ode45(ode_t,[0,t_end_sweep],y0_base,opts_snap4);
            if isempty(ts), Mpm_lo=Mpm_t; else, Mpm_hi=Mpm_t; end
        end
        surf_Mpm(mi,fi) = Mpm_hi * 1e3;   % convert to g
    end
    fprintf('  m_hook = %.0f g done\n', mh_i*1e3);
end

%% ══════════════════════════════════════════════════════════════════════
%%  3-D GRID: torque(f_drive, m_hook)  — Figure 4  (ANALYTICAL)
%% ══════════════════════════════════════════════════════════════════════
fprintf('Computing 3-D torque surface (analytical) …\n');
surf_tau   = zeros(Nm, Nf);

for mi = 1:Nm
    mh_i       = mhook_grid_vec(mi);
    omega_n1_i = sqrt(k_eff_s1/mh_i);
    c_hook_i   = 2*zeta_hook*mh_i*omega_n1_i;
    for fi = 1:Nf
        omega_i = 2*pi*f_grid_vec(fi);
        % Analytical: A_min = d_saddle / |H(omega)|
        % H(omega) = m_hook*omega^2 / sqrt((k_eff-m_hook*omega^2)^2 + (c_hook*omega)^2)
        denom_tf = sqrt((k_eff_s1 - mh_i*omega_i^2)^2 + (c_hook_i*omega_i)^2);
        H_mag    = mh_i * omega_i^2 / denom_tf;
        A_min_i  = d_saddle / H_mag;
        % Torque = m_hook * A_min * omega^2 * lever_arm
        surf_tau(mi,fi) = mh_i * A_min_i * omega_i^2 * lever_arm;
    end
end

%% ══════════════════════════════════════════════════════════════════════
%%  N_CELLS SWEEP — M_pm_min(f_drive, n_cells) and torque(f_drive, n_cells)
%% ══════════════════════════════════════════════════════════════════════
Nnc = numel(n_cells_vec);
Nf1 = numel(f_sweep_1d);
M_pm_min_nc  = zeros(Nnc, Nf1);
tau_c2_nc    = zeros(Nnc, Nf1);
fn1_nc       = zeros(1, Nnc);
frel_nc      = zeros(1, Nnc);
DeltaE2_nc   = zeros(1, Nnc);
KE_impact_nc = zeros(1, Nnc);

fprintf('Computing n_cells sweep (%d beam counts × %d frequencies) …\n', Nnc, Nf1);

for nci = 1:Nnc
    nc_i = n_cells_vec(nci);
    B = bistable_params(nc_i, d_stroke_1cell, d_saddle_frac, F_push_1cell, m_hook, zeta_hook);
    fn1_nc(nci)       = B.f_n1;
    frel_nc(nci)      = B.f_rel;
    DeltaE2_nc(nci)   = B.DeltaE2;
    KE_impact_nc(nci) = B.DeltaE2;   % = 0.5*m_hook*v^2 when v=sqrt(2*DE2/m)
    for fi = 1:Nf1
        omega_i = 2*pi*f_sweep_1d(fi);
        Delta_i = lever_arm * omega_shaft_max / omega_i;   % [m] linear stroke = r × θ_max
        Mpm_lo=0; Mpm_hi=5.0;
        for bs=1:15
            Mpm_t  = (Mpm_lo+Mpm_hi)/2;
            Meff_t = M_plate+Mpm_t;
            ks_t   = Meff_t*omega_i^2;
            cs_t   = 2*zeta_plate*Meff_t*omega_i;
            ode_t  = @(t,y) excit_pm_ode(t,y,Meff_t,m_hook,ks_t,cs_t,B.c_hook, ...
                                           B.F_restore,Mpm_t,Delta_i,omega_i);
            [~,~,ts,~,~]=ode45(ode_t,[0,t_end_sweep],y0_base,opts_snap4);
            if isempty(ts), Mpm_lo=Mpm_t; else, Mpm_hi=Mpm_t; end
        end
        M_pm_min_nc(nci,fi) = Mpm_hi*1e3;
        denom_tf = sqrt((B.k_eff_s1 - m_hook*omega_i^2)^2 + (B.c_hook*omega_i)^2);
        H_i      = m_hook*omega_i^2 / denom_tf;
        tau_c2_nc(nci,fi) = m_hook*(B.d_saddle/H_i)*omega_i^2*lever_arm;
    end
    fprintf('  N_parallel=%d: f_n1=%.1f Hz  f_rel=%.1f Hz  KE=%.2f mJ  stroke=%.1f mm\n', ...
            nc_i, B.f_n1, B.f_rel, DeltaE2_nc(nci)*1e3, B.d_stroke*1e3);
end

%% ══════════════════════════════════════════════════════════════════════
%%  PLOTTING
%% ══════════════════════════════════════════════════════════════════════
if ~DISPLAY_FIGS, set(0,'DefaultFigureVisible','off'); end

% Shared beam/FD/PE data
xi_beam  = linspace(0,l_span,400);
y_state1 = (h_apex/2)*(1-cos(2*pi*xi_beam/l_span));
y_state2 = -y_state1;
x_fd     = linspace(-0.01*d_stroke,1.05*d_stroke,800);
F_ext    = -F_restore(x_fd);
[Fp_max,ipm]=max(F_ext); [Fp_min,ipl]=min(F_ext);
mask_push = F_ext>=0; mask_pull = F_ext<0;
x_v   = linspace(-0.01*d_stroke,1.05*d_stroke,800);
V_vec = V_fun(x_v);
V_s1=V_fun(0); V_s2=V_fun(d_stroke); V_bar=V_fun(d_saddle);
DeltaE_1=V_bar-V_s1; DeltaE_2=V_bar-V_s2;
x_sv  = linspace(0,d_stroke,600);
v_sep = sqrt(max(0,2*(V_bar-V_fun(x_sv))/m_hook));
info_str = sprintf('h = %.1f mm\nt = %.1f mm\nl = %.0f mm\nQ = %.2f', ...
                   h_apex*1e3,t_beam*1e3,l_span*1e3,h_apex/t_beam);
y_lim_t = [-0.12*d_stroke,1.2*d_stroke]*1e3;

%% ── Figure 1: §4.12 — 1-D sweep + simulation ────────────────────────
fig1 = figure('Name','§4.12 Proof-Mass — Mass Actuation Analysis', ...
              'NumberTitle','off','Color','w','Position',[50 30 1500 940]);

subplot(2,3,1); hold on;   % beam geometry
patch([-1.5,0,0,-1.5]*1e3,[-h_apex,-h_apex,h_apex,h_apex]*1.5*1e3,[.5 .5 .5],'EdgeColor','k','FaceAlpha',.4,'HandleVisibility','off');
patch([l_span,l_span+1.5e-3,l_span+1.5e-3,l_span]*1e3,[-h_apex,-h_apex,h_apex,h_apex]*1.5*1e3,[.5 .5 .5],'EdgeColor','k','FaceAlpha',.4,'HandleVisibility','off');
fill([xi_beam,fliplr(xi_beam)]*1e3,[y_state1+t_beam/2,fliplr(y_state1-t_beam/2)]*1e3,[.4 .7 1],'EdgeColor','b','LineWidth',1.5,'FaceAlpha',.7,'DisplayName','State 1');
fill([xi_beam,fliplr(xi_beam)]*1e3,[y_state2+t_beam/2,fliplr(y_state2-t_beam/2)]*1e3,[.4 1 .6],'EdgeColor',[0 .6 0],'LineWidth',1.5,'FaceAlpha',.7,'DisplayName','State 2');
text(.97,.97,info_str,'Units','normalized','HorizontalAlignment','right','VerticalAlignment','top','FontSize',8,'FontName','Monospaced','BackgroundColor','w','EdgeColor',[.6 .6 .6],'Margin',3);
xlabel('Beam span (mm)','FontSize',9); ylabel('Disp. (mm)','FontSize',9);
title('Beam Geometry | Qiu cosine profile','FontWeight','bold');
legend('Location','northeast','FontSize',8); grid on; box on;
ylim([-h_apex*2.5e3,h_apex*2.5e3]); axis equal; xlim([-2,(l_span+2e-3)*1e3]);

subplot(2,3,2); hold on;   % F-D
if any(mask_push), patch([x_fd(mask_push),fliplr(x_fd(mask_push))]*1e3,[F_ext(mask_push),zeros(1,sum(mask_push))],[.8 1 .8],'EdgeColor','none','FaceAlpha',.6); end
if any(mask_pull), patch([x_fd(mask_pull),fliplr(x_fd(mask_pull))]*1e3,[F_ext(mask_pull),zeros(1,sum(mask_pull))],[1 .8 .8],'EdgeColor','none','FaceAlpha',.6); end
plot(x_fd*1e3,F_ext,'b-','LineWidth',2.2); yline(0,'k--','LineWidth',.9);
xline(0,'g:','LineWidth',1.2,'HandleVisibility','off'); xline(d_stroke*1e3,'g:','LineWidth',1.2,'HandleVisibility','off'); xline(d_saddle*1e3,'r:','LineWidth',1.2,'HandleVisibility','off');
plot(x_fd(ipm)*1e3,Fp_max,'ms','MarkerFaceColor','m','MarkerSize',9); plot(x_fd(ipl)*1e3,Fp_min,'cs','MarkerFaceColor','c','MarkerSize',9);
text(x_fd(ipm)*1e3,Fp_max*.6,sprintf('F_{push}=%.1f N',Fp_max),'FontSize',7,'Color','m','HorizontalAlignment','center');
text(x_fd(ipl)*1e3,Fp_min*.6,sprintf('F_{pop}=%.1f N',Fp_min),'FontSize',7,'Color','c','HorizontalAlignment','center');
text(0,Fp_max*.85,'State 1','FontSize',7,'Color',[0 .5 0],'HorizontalAlignment','center');
text(d_saddle*1e3,Fp_max*.85,'Saddle','FontSize',7,'Color',[.6 0 0],'HorizontalAlignment','center');
text(d_stroke*1e3,Fp_max*.85,'State 2','FontSize',7,'Color',[0 .5 0],'HorizontalAlignment','center');
xlabel('Displacement (mm)','FontSize',9); ylabel('Applied force (N)','FontSize',9);
title('Force–Displacement (Design 9)','FontWeight','bold'); grid on; box on;

subplot(2,3,3); hold on;   % potential energy
fill([x_v,fliplr(x_v)]*1e3,[V_vec,zeros(1,numel(x_v))]*1e3,[.88 .93 1],'EdgeColor','none','FaceAlpha',.5,'HandleVisibility','off');
plot(x_v*1e3,V_vec*1e3,'b-','LineWidth',2.2); yline(0,'k--','LineWidth',.8);
plot([0,d_stroke]*1e3,[V_s1,V_s2]*1e3,'go','MarkerFaceColor','g','MarkerSize',11,'DisplayName','Stable eq.');
plot(d_saddle*1e3,V_bar*1e3,'rv','MarkerFaceColor','r','MarkerSize',11,'DisplayName','Saddle');
plot([0 0]*1e3,[V_s1 V_bar]*1e3,'m-','LineWidth',2,'HandleVisibility','off');
plot([d_stroke d_stroke]*1e3,[V_s2 V_bar]*1e3,'c-','LineWidth',2,'HandleVisibility','off');
text(.5e3,(V_s1+V_bar)/2*1e3,sprintf(' ΔE₁=%.2f mJ',DeltaE_1*1e3),'Color','m','FontSize',8,'FontWeight','bold');
text((d_stroke-1e-3)*1e3,(V_s2+V_bar)/2*1e3,sprintf('ΔE₂=%.2f mJ ',DeltaE_2*1e3),'Color','c','FontSize',8,'FontWeight','bold','HorizontalAlignment','right');
xlabel('Displacement (mm)','FontSize',9); ylabel('V(x) (mJ)','FontSize',9);
title('Potential Energy Double Well','FontWeight','bold');
legend('Location','northeast','FontSize',8); grid on; box on;

ax1_4 = subplot(2,3,4); hold on;   % M_pm_min vs frequency — one curve per n_cells
nc_colors = lines(Nnc);
for nci = 1:Nnc
    nc_i = n_cells_vec(nci);
    B_i  = bistable_params(nc_i, d_stroke_1cell, d_saddle_frac, F_push_1cell, m_hook, zeta_hook);
    [Mopt_i, iopt_i] = min(M_pm_min_nc(nci,:));
    plot(f_sweep_1d, M_pm_min_nc(nci,:), '-', 'Color', nc_colors(nci,:), 'LineWidth', 2.0, ...
         'DisplayName', sprintf('N=%d  (f_{n1}=%.1f Hz)', nc_i, B_i.f_n1));
    plot(f_sweep_1d(iopt_i), Mopt_i, '*', 'Color', nc_colors(nci,:), 'MarkerSize', 10, 'LineWidth', 2, ...
         'HandleVisibility','off');
end
% Highlight current n_cells (index in n_cells_vec matching n_cells=5)
[~,nc_cur_idx] = min(abs(n_cells_vec - n_cells));
plot(f_sweep_1d, M_pm_min_1d*1e3,'w-','LineWidth',3.5,'HandleVisibility','off');  % white highlight
plot(f_sweep_1d, M_pm_min_1d*1e3,'-','Color',nc_colors(nc_cur_idx,:),'LineWidth',2.5,'HandleVisibility','off');
xline(f_n1,'k--','LineWidth',1.2,'Label',sprintf('f_{n1}=%.1f Hz',f_n1),'FontSize',8,'HandleVisibility','off');
xlabel('Drive frequency (Hz)','FontSize',9); ylabel('Min proof mass M_{pm}  (g)','FontSize',9);
title('M_{pm,min} vs Frequency — Parallel Beam Count','FontWeight','bold');
legend('Location','northeast','FontSize',7); grid on; box on;

subplot(2,3,5); hold on;   % time history
if ~isnan(t_snap_p)
    xline(t_snap_p,'r:','LineWidth',1.8,'Label',sprintf('snap t=%.2f s',t_snap_p),'FontSize',8);
end
skip_p=max(1,round(numel(t_pm)/120)); idx_p=1:skip_p:numel(t_pm);
plot(t_pm(idx_p),x_rel_p(idx_p)*1e3,'-','Color',[0 .45 .74],'LineWidth',1.5);
yline(0,'b--','LineWidth',1.2,'Label','State 1','LabelHorizontalAlignment','left','FontSize',8);
yline(d_stroke*1e3,'g--','LineWidth',1.2,'Label','State 2','LabelHorizontalAlignment','left','FontSize',8);
yline(d_saddle*1e3,'k:','LineWidth',.9,'Label','Saddle','LabelHorizontalAlignment','right','FontSize',8);
xlabel('Time (s)','FontSize',9); ylabel('Hook displacement (mm)','FontSize',9);
title(sprintf('Displacement vs Time  (f_{opt}=%.1f Hz, M_{pm}=%.0f g)',f_opt,M_pm_opt_1d*1e3),'FontWeight','bold');
grid on; box on; xlim([0,t_pm(end)]); ylim(y_lim_t);

ax1_6=subplot(2,3,6); hold on;   % phase portrait
fill([x_sv,fliplr(x_sv)]*1e3,[v_sep,-fliplr(v_sep)]*1e3,[1 .9 .9],'EdgeColor','none','FaceAlpha',.4);
plot(x_sv*1e3,v_sep*1e3,'r--','LineWidth',1.8,'DisplayName','Separatrix');
plot(x_sv*1e3,-v_sep*1e3,'r--','LineWidth',1.8,'HandleVisibility','off');
N_pu=numel(t_pm); ds_u=max(1,floor(N_pu/2000)); idx_pu=1:ds_u:N_pu;
cmap_u=parula(numel(idx_pu));
for k=1:numel(idx_pu)-1
    plot(x_rel_p(idx_pu(k):idx_pu(k+1))*1e3,v_rel_p(idx_pu(k):idx_pu(k+1))*1e3,'-','Color',cmap_u(k,:),'LineWidth',.9,'HandleVisibility','off');
end
if ~isnan(t_snap_p), [~,is]=min(abs(t_pm-t_snap_p)); plot(x_rel_p(is)*1e3,v_rel_p(is)*1e3,'kp','MarkerSize',14,'MarkerFaceColor','y','DisplayName','Snap'); end
plot([0,d_stroke]*1e3,[0,0],'go','MarkerFaceColor','g','MarkerSize',10,'DisplayName','Stable eq.');
cb1=colorbar; cb1.Label.String='Time →'; colormap(ax1_6,parula); clim([0,t_pm(end)]);
xlabel('x_{rel} (mm)','FontSize',9); ylabel('Velocity (mm/s)','FontSize',9);
title('Phase Portrait','FontWeight','bold');
legend('Location','northeast','FontSize',7); grid on; box on;
v_rng=max(abs(v_rel_p))*1.2e3; v_rng=max(v_rng,max(v_sep)*1e3*.2); ylim(ax1_6,[-v_rng,v_rng]);

sgtitle(sprintf('§4.12 Proof-Mass  |  M_{pm,opt} = %.0f g  |  f_{opt} = %.1f Hz  |  Δ = %.2f mm', ...
        M_pm_opt_1d*1e3, f_opt, Delta_opt*1e3),'FontSize',11,'FontWeight','bold');

%% ── Figure 2: Config 2 ───────────────────────────────────────────────
fig2 = figure('Name','Config 2 — Prescribed Base Motion', ...
              'NumberTitle','off','Color','w','Position',[80 50 1500 940]);

subplot(2,3,1); hold on;
patch([-1.5,0,0,-1.5]*1e3,[-h_apex,-h_apex,h_apex,h_apex]*1.5*1e3,[.5 .5 .5],'EdgeColor','k','FaceAlpha',.4,'HandleVisibility','off');
patch([l_span,l_span+1.5e-3,l_span+1.5e-3,l_span]*1e3,[-h_apex,-h_apex,h_apex,h_apex]*1.5*1e3,[.5 .5 .5],'EdgeColor','k','FaceAlpha',.4,'HandleVisibility','off');
fill([xi_beam,fliplr(xi_beam)]*1e3,[y_state1+t_beam/2,fliplr(y_state1-t_beam/2)]*1e3,[.4 .7 1],'EdgeColor','b','LineWidth',1.5,'FaceAlpha',.7,'DisplayName','State 1');
fill([xi_beam,fliplr(xi_beam)]*1e3,[y_state2+t_beam/2,fliplr(y_state2-t_beam/2)]*1e3,[.4 1 .6],'EdgeColor',[0 .6 0],'LineWidth',1.5,'FaceAlpha',.7,'DisplayName','State 2');
text(.97,.97,info_str,'Units','normalized','HorizontalAlignment','right','VerticalAlignment','top','FontSize',8,'FontName','Monospaced','BackgroundColor','w','EdgeColor',[.6 .6 .6],'Margin',3);
xlabel('Beam span (mm)','FontSize',9); ylabel('Disp. (mm)','FontSize',9);
title('Beam Geometry | Qiu cosine profile','FontWeight','bold');
legend('Location','northeast','FontSize',8); grid on; box on;
ylim([-h_apex*2.5e3,h_apex*2.5e3]); axis equal; xlim([-2,(l_span+2e-3)*1e3]);

subplot(2,3,2); hold on;
if any(mask_push), patch([x_fd(mask_push),fliplr(x_fd(mask_push))]*1e3,[F_ext(mask_push),zeros(1,sum(mask_push))],[.8 1 .8],'EdgeColor','none','FaceAlpha',.6); end
if any(mask_pull), patch([x_fd(mask_pull),fliplr(x_fd(mask_pull))]*1e3,[F_ext(mask_pull),zeros(1,sum(mask_pull))],[1 .8 .8],'EdgeColor','none','FaceAlpha',.6); end
plot(x_fd*1e3,F_ext,'b-','LineWidth',2.2); yline(0,'k--','LineWidth',.9);
xline(0,'g:','LineWidth',1.2,'HandleVisibility','off'); xline(d_stroke*1e3,'g:','LineWidth',1.2,'HandleVisibility','off'); xline(d_saddle*1e3,'r:','LineWidth',1.2,'HandleVisibility','off');
plot(x_fd(ipm)*1e3,Fp_max,'ms','MarkerFaceColor','m','MarkerSize',9); plot(x_fd(ipl)*1e3,Fp_min,'cs','MarkerFaceColor','c','MarkerSize',9);
text(x_fd(ipm)*1e3,Fp_max*.6,sprintf('F_{push}=%.1f N',Fp_max),'FontSize',7,'Color','m','HorizontalAlignment','center');
text(x_fd(ipl)*1e3,Fp_min*.6,sprintf('F_{pop}=%.1f N',Fp_min),'FontSize',7,'Color','c','HorizontalAlignment','center');
text(0,Fp_max*.85,'State 1','FontSize',7,'Color',[0 .5 0],'HorizontalAlignment','center');
text(d_saddle*1e3,Fp_max*.85,'Saddle','FontSize',7,'Color',[.6 0 0],'HorizontalAlignment','center');
text(d_stroke*1e3,Fp_max*.85,'State 2','FontSize',7,'Color',[0 .5 0],'HorizontalAlignment','center');
xlabel('Displacement (mm)','FontSize',9); ylabel('Applied force (N)','FontSize',9);
title('Force–Displacement (Design 9)','FontWeight','bold'); grid on; box on;

subplot(2,3,3); hold on;
fill([x_v,fliplr(x_v)]*1e3,[V_vec,zeros(1,numel(x_v))]*1e3,[.88 .93 1],'EdgeColor','none','FaceAlpha',.5,'HandleVisibility','off');
plot(x_v*1e3,V_vec*1e3,'b-','LineWidth',2.2); yline(0,'k--','LineWidth',.8);
plot([0,d_stroke]*1e3,[V_s1,V_s2]*1e3,'go','MarkerFaceColor','g','MarkerSize',11,'DisplayName','Stable eq.');
plot(d_saddle*1e3,V_bar*1e3,'rv','MarkerFaceColor','r','MarkerSize',11,'DisplayName','Saddle');
plot([0 0]*1e3,[V_s1 V_bar]*1e3,'m-','LineWidth',2,'HandleVisibility','off');
plot([d_stroke d_stroke]*1e3,[V_s2 V_bar]*1e3,'c-','LineWidth',2,'HandleVisibility','off');
text(.5e3,(V_s1+V_bar)/2*1e3,sprintf(' ΔE₁=%.2f mJ',DeltaE_1*1e3),'Color','m','FontSize',8,'FontWeight','bold');
text((d_stroke-1e-3)*1e3,(V_s2+V_bar)/2*1e3,sprintf('ΔE₂=%.2f mJ ',DeltaE_2*1e3),'Color','c','FontSize',8,'FontWeight','bold','HorizontalAlignment','right');
xlabel('Displacement (mm)','FontSize',9); ylabel('V(x) (mJ)','FontSize',9);
title('Potential Energy Double Well','FontWeight','bold');
legend('Location','northeast','FontSize',8); grid on; box on;

% Panel 4: torque vs frequency (analytical)
f_tau_vec = linspace(20,350,350);
omega_tau  = 2*pi*f_tau_vec;
c_hook_tau = 2*zeta_hook*m_hook*omega_n1;
denom_tau  = sqrt((k_eff_s1 - m_hook.*omega_tau.^2).^2 + (c_hook_tau.*omega_tau).^2);
H_tau      = m_hook.*omega_tau.^2 ./ denom_tau;
A_min_tau  = d_saddle ./ H_tau;
tau_curve  = m_hook .* A_min_tau .* omega_tau.^2 .* lever_arm;
subplot(2,3,4); hold on;
semilogy(f_tau_vec, tau_curve,'b-','LineWidth',2.5,'DisplayName','Required torque');
yline(1.6,'r--','LineWidth',1.8,'Label','RS05 rated 1.6 N·m','LabelHorizontalAlignment','right','FontSize',8,'DisplayName','RS05 rated');
yline(5.5,'m--','LineWidth',1.2,'Label','RS05 peak 5.5 N·m','LabelHorizontalAlignment','right','FontSize',8,'DisplayName','RS05 peak');
xline(f_n1,'k--','LineWidth',1.2,'Label',sprintf('f_{n1}=%.1f Hz',f_n1),'FontSize',8,'HandleVisibility','off');
plot(f_n1, tau_c2_num,'r*','MarkerSize',14,'LineWidth',2.5,'DisplayName',sprintf('Config 2 sim: τ=%.3f N·m',tau_c2_num));
xlabel('Drive frequency (Hz)','FontSize',9); ylabel('Required torque (N·m)','FontSize',9);
title('Config 2: Torque vs Frequency','FontWeight','bold');
legend('Location','northwest','FontSize',8); grid on; box on; xlim([20,350]);

subplot(2,3,5); hold on;   % time history
if ~isnan(t_snap_c)
    xline(t_snap_c,'r:','LineWidth',1.8,'Label',sprintf('snap t=%.2f s',t_snap_c),'FontSize',8);
end
skip_c=max(1,round(numel(t_c2)/120)); idx_c=1:skip_c:numel(t_c2);
plot(t_c2(idx_c),x_rel_c2(idx_c)*1e3,'-','Color',[.2 .6 .2],'LineWidth',1.5);
yline(0,'b--','LineWidth',1.2,'Label','State 1','LabelHorizontalAlignment','left','FontSize',8);
yline(d_stroke*1e3,'g--','LineWidth',1.2,'Label','State 2','LabelHorizontalAlignment','left','FontSize',8);
yline(d_saddle*1e3,'k:','LineWidth',.9,'Label','Saddle','LabelHorizontalAlignment','right','FontSize',8);
xlabel('Time (s)','FontSize',9); ylabel('Hook displacement (mm)','FontSize',9);
title(sprintf('Displacement vs Time  (f_{n1}=%.1f Hz, A_{min}=%.2f mm)',f_n1,A_min_c2_num*1e3),'FontWeight','bold');
grid on; box on; xlim([0,t_c2(end)]); ylim(y_lim_t);

ax2_6=subplot(2,3,6); hold on;   % phase portrait
fill([x_sv,fliplr(x_sv)]*1e3,[v_sep,-fliplr(v_sep)]*1e3,[1 .9 .9],'EdgeColor','none','FaceAlpha',.4);
plot(x_sv*1e3,v_sep*1e3,'r--','LineWidth',1.8,'DisplayName','Separatrix');
plot(x_sv*1e3,-v_sep*1e3,'r--','LineWidth',1.8,'HandleVisibility','off');
N_c2=numel(t_c2); ds_c2=max(1,floor(N_c2/2000)); idx_c2p=1:ds_c2:N_c2;
cmap_c2=parula(numel(idx_c2p));
for k=1:numel(idx_c2p)-1
    plot(x_rel_c2(idx_c2p(k):idx_c2p(k+1))*1e3,v_rel_c2(idx_c2p(k):idx_c2p(k+1))*1e3,'-','Color',cmap_c2(k,:),'LineWidth',.9,'HandleVisibility','off');
end
if ~isnan(t_snap_c), [~,is]=min(abs(t_c2-t_snap_c)); plot(x_rel_c2(is)*1e3,v_rel_c2(is)*1e3,'kp','MarkerSize',14,'MarkerFaceColor','y','DisplayName','Snap'); end
plot([0,d_stroke]*1e3,[0,0],'go','MarkerFaceColor','g','MarkerSize',10,'DisplayName','Stable eq.');
cb2=colorbar; cb2.Label.String='Time →'; colormap(ax2_6,parula); clim([0,t_c2(end)]);
xlabel('x_{rel} (mm)','FontSize',9); ylabel('Velocity (mm/s)','FontSize',9);
title('Phase Portrait','FontWeight','bold');
legend('Location','northeast','FontSize',7); grid on; box on;
v_rng2=max(abs(v_rel_c2))*1.2e3; v_rng2=max(v_rng2,max(v_sep)*1e3*.2); ylim(ax2_6,[-v_rng2,v_rng2]);

sgtitle(sprintf('Config 2 — Prescribed Base Motion  |  f_{n1}=%.1f Hz  |  A_{min,num}=%.2f mm  |  τ_{num}=%.4f N·m', ...
        f_n1, A_min_c2_num*1e3, tau_c2_num),'FontSize',11,'FontWeight','bold');

%% ── Figure 3: 3-D M_pm_min surface ──────────────────────────────────
fig3 = figure('Name','3-D M_{pm,min}(f_{drive}, m_{hook})','NumberTitle','off', ...
              'Color','w','Position',[120 80 900 680]);
ax3 = axes;
surf(f_grid_vec, mhook_grid_vec*1e3, surf_Mpm, 'EdgeColor','none','FaceAlpha',.85);
hold on;
% Resonance curve f_rel(m_hook) as 3D line on the surface
Z_frel = zeros(1,Nm);
for mi=1:Nm, [~,fci]=min(abs(f_grid_vec-surf_frel(mi))); Z_frel(mi)=surf_Mpm(mi,fci); end
plot3(surf_frel, mhook_grid_vec*1e3, Z_frel+0.5,'w-','LineWidth',3,'DisplayName','f_{rel}(m_{hook})');
% Current design point
[~,fc_now]=min(abs(f_grid_vec-f_opt)); [~,mc_now]=min(abs(mhook_grid_vec-m_hook));
plot3(f_opt, m_hook*1e3, surf_Mpm(mc_now,fc_now)+1,'r*','MarkerSize',16,'LineWidth',2.5,'DisplayName','Current design');
xlabel('Drive frequency  (Hz)','FontSize',12);
ylabel('Hook mass  m_{hook}  (g)','FontSize',12);
zlabel('Min proof mass  M_{pm,min}  (g)','FontSize',12);
title('Mass Actuation: Minimum Proof Mass for Snap','FontWeight','bold','FontSize',13);
colormap(ax3,parula); cb3=colorbar; cb3.Label.String='M_{pm,min}  (g)'; cb3.FontSize=10;
shading interp; lighting gouraud; view(45,30);
legend('Location','northeast','FontSize',10);
grid on; box on;

%% ── Figure 4: 3-D torque surface (Config 2, analytical) ─────────────
fig4 = figure('Name','3-D Torque(f_{drive}, m_{hook}) — Config 2','NumberTitle','off', ...
              'Color','w','Position',[150 100 900 680]);
ax4 = axes;
% Cap torque values for visualisation (hide extreme off-resonance values)
tau_plot = min(surf_tau, 20);
surf(f_grid_vec, mhook_grid_vec*1e3, tau_plot,'EdgeColor','none','FaceAlpha',.85);
hold on;
% Resonance curve f_n1(m_hook)
Z_fn1 = zeros(1,Nm);
for mi=1:Nm, [~,fci]=min(abs(f_grid_vec-surf_fn1(mi))); Z_fn1(mi)=surf_tau(mi,fci); end
plot3(surf_fn1, mhook_grid_vec*1e3, Z_fn1+0.02,'w-','LineWidth',3,'DisplayName','f_{n1}(m_{hook}) — resonance valley');
% RS05 rated torque plane
[Fgrid,Mgrid]=meshgrid(f_grid_vec, mhook_grid_vec*1e3);
surf(Fgrid, Mgrid, 1.6*ones(size(Fgrid)),'FaceAlpha',.15,'EdgeColor','none','FaceColor','r','DisplayName','RS05 rated 1.6 N·m');
% Current design point
[~,fc_now]=min(abs(f_grid_vec-f_n1)); [~,mc_now]=min(abs(mhook_grid_vec-m_hook));
plot3(f_n1, m_hook*1e3, surf_tau(mc_now,fc_now)+0.05,'r*','MarkerSize',16,'LineWidth',2.5,'DisplayName','Current design');
xlabel('Drive frequency  (Hz)','FontSize',12);
ylabel('Hook mass  m_{hook}  (g)','FontSize',12);
zlabel('Required torque  (N·m)','FontSize',12);
title('Config 2: Motor Torque Required for Snap','FontWeight','bold','FontSize',13);
colormap(ax4,parula); cb4=colorbar; cb4.Label.String='Torque  (N·m)'; cb4.FontSize=10;
shading interp; lighting gouraud; view(45,30);
legend('Location','northeast','FontSize',10);
grid on; box on;

%% ── Figure 5: 3-D M_pm_min(f_drive, N_parallel) ─────────────────────
fig5 = figure('Name','3-D M_{pm,min}(f_{drive}, N_{parallel})','NumberTitle','off', ...
              'Color','w','Position',[180 120 900 680]);
ax5  = axes;
[F5grid, N5grid] = meshgrid(f_sweep_1d, n_cells_vec);
surf(F5grid, N5grid, M_pm_min_nc,'EdgeColor','none','FaceAlpha',.85); hold on;
% Resonance ridge: f_rel(N_parallel)
Z5_frel = zeros(1,Nnc);
for nci=1:Nnc, [~,fi_r]=min(abs(f_sweep_1d-frel_nc(nci))); Z5_frel(nci)=M_pm_min_nc(nci,fi_r); end
plot3(frel_nc, n_cells_vec, Z5_frel+0.5,'w-','LineWidth',3,'DisplayName','f_{rel}(N_{parallel}) ridge');
% Current design point
[~,nc_now]=min(abs(n_cells_vec-n_cells)); [~,fi_now]=min(abs(f_sweep_1d-f_opt));
plot3(f_opt, n_cells, M_pm_min_nc(nc_now,fi_now)+1,'r*','MarkerSize',16,'LineWidth',2.5,'DisplayName','Current design');
xlabel('Drive frequency  (Hz)','FontSize',12);
ylabel('Parallel beam count  N','FontSize',12);
zlabel('Min proof mass  M_{pm,min}  (g)','FontSize',12);
title('§4.12: Min Proof Mass vs Frequency & Parallel Beam Count','FontWeight','bold','FontSize',13);
colormap(ax5,parula); cb5=colorbar; cb5.Label.String='M_{pm,min}  (g)'; cb5.FontSize=10;
shading interp; view(45,30); legend('Location','northeast','FontSize',10);
grid on; box on; set(gca,'YTick',n_cells_vec);

%% ── Figure 6: 3-D Torque(f_drive, N_parallel) — Config 2 (analytical) ─
fig6 = figure('Name','3-D Torque(f_{drive}, N_{parallel}) — Config 2','NumberTitle','off', ...
              'Color','w','Position',[200 140 900 680]);
ax6  = axes;
tau_plot6 = min(tau_c2_nc, 20);   % cap for display
surf(F5grid, N5grid, tau_plot6,'EdgeColor','none','FaceAlpha',.85); hold on;
% Resonance valley: f_n1(N_parallel)
Z6_fn1 = zeros(1,Nnc);
for nci=1:Nnc, [~,fi_n]=min(abs(f_sweep_1d-fn1_nc(nci))); Z6_fn1(nci)=tau_c2_nc(nci,fi_n); end
plot3(fn1_nc, n_cells_vec, Z6_fn1+0.02,'w-','LineWidth',3,'DisplayName','f_{n1}(N_{parallel}) valley');
[F6g,N6g]=meshgrid(f_sweep_1d,n_cells_vec);
surf(F6g,N6g,1.6*ones(size(F6g)),'FaceAlpha',.12,'EdgeColor','none','FaceColor','r','DisplayName','RS05 rated 1.6 N·m');
[~,nc_now6]=min(abs(n_cells_vec-n_cells)); [~,fi_n1]=min(abs(f_sweep_1d-fn1_nc(nc_now6)));
plot3(fn1_nc(nc_now6),n_cells,tau_c2_nc(nc_now6,fi_n1)+0.05,'r*','MarkerSize',16,'LineWidth',2.5,'DisplayName','Current design');
xlabel('Drive frequency  (Hz)','FontSize',12);
ylabel('Parallel beam count  N','FontSize',12);
zlabel('Required torque  (N·m)','FontSize',12);
title('Config 2: Motor Torque vs Frequency & Parallel Beam Count','FontWeight','bold','FontSize',13);
colormap(ax6,parula); cb6=colorbar; cb6.Label.String='Torque  (N·m)'; cb6.FontSize=10;
shading interp; view(45,30); legend('Location','northeast','FontSize',10);
grid on; box on; set(gca,'YTick',n_cells_vec);

%% ── Figure 7: Comprehensive design optimisation vs N_parallel ───────
% Precompute per-n_cells derived quantities
[Mpm_opt_nc, idx_opt_nc] = min(M_pm_min_nc, [], 2);
f_opt_nc        = f_sweep_1d(idx_opt_nc);
tau_pm_opt_nc   = zeros(1, Nnc);   % §4.12 torque at optimal frequency
tau_c2_fn1_nc   = zeros(1, Nnc);   % Config 2 torque at f_n1
d_stroke_nc     = zeros(1, Nnc);
for nci = 1:Nnc
    omega_opt_i        = 2*pi * f_opt_nc(nci);
    Delta_opt_i        = lever_arm * omega_shaft_max / omega_opt_i;   % [m]
    tau_pm_opt_nc(nci) = (Mpm_opt_nc(nci)/1e3) * Delta_opt_i * omega_opt_i^2 * lever_arm;
    [~, fi_n]          = min(abs(f_sweep_1d - fn1_nc(nci)));
    tau_c2_fn1_nc(nci) = tau_c2_nc(nci, fi_n);
    B_tmp              = bistable_params(n_cells_vec(nci), d_stroke_1cell, d_saddle_frac, F_push_1cell, m_hook, zeta_hook);
    d_stroke_nc(nci)   = B_tmp.d_stroke * 1e3;   % mm
end

fig7 = figure('Name','Comprehensive Design Space vs N_{parallel}','NumberTitle','off', ...
              'Color','w','Position',[220 160 1200 800]);

nc_col = lines(Nnc);

%% Panel 1: released spring energy
subplot(2,2,1); hold on;
b1 = bar(n_cells_vec, KE_impact_nc*1e3, 0.5, 'FaceAlpha', 0.8);
b1.CData = nc_col;  b1.FaceColor = 'flat';
for nci=1:Nnc
    text(n_cells_vec(nci), KE_impact_nc(nci)*1e3*1.06, ...
         sprintf('%.1f mm', d_stroke_nc(nci)), ...
         'HorizontalAlignment','center','FontSize',8,'FontWeight','bold');
end
xlabel('N_{parallel}','FontSize',10); ylabel('Released energy \DeltaE_2  (mJ)','FontSize',10);
title('Stored Energy Released After Snap','FontWeight','bold');
xticks(n_cells_vec); grid on; box on;

%% Panel 2: Actuator requirements — §4.12 proof mass
subplot(2,2,2); hold on;
yyaxis left;
bar(n_cells_vec, Mpm_opt_nc, 0.5, 'FaceColor',[0.85 0.33 0.1],'FaceAlpha',0.75);
ylabel('Min proof mass  M_{pm}  (g)','FontSize',10,'Color',[0.85 0.33 0.1]);
ax72 = gca; ax72.YAxis(1).Color = [0.85 0.33 0.1];
yyaxis right;
plot(n_cells_vec, tau_pm_opt_nc, 'b-o','LineWidth',2.2,'MarkerSize',8,'MarkerFaceColor','b');
yline(1.6,'b--','LineWidth',1.2,'Label','Rated 1.6 N·m','LabelHorizontalAlignment','right','FontSize',8);
yline(5.5,'b:','LineWidth',1.2,'Label','Peak 5.5 N·m','LabelHorizontalAlignment','right','FontSize',8);
ylabel('Motor torque  (N·m)','FontSize',10,'Color','b');
ax72.YAxis(2).Color = 'b';
xlabel('N_{parallel}','FontSize',10);
title('§4.12 Proof Mass: M_{pm,min} & Torque vs N','FontWeight','bold');
xticks(n_cells_vec); grid on; box on;

%% Panel 3: Key frequencies vs parallel beam count
subplot(2,2,3); hold on;
plot(n_cells_vec, fn1_nc,  'b-s','LineWidth',2.2,'MarkerSize',9,'MarkerFaceColor','b',  'DisplayName','f_{n1} (bistable resonance)');
plot(n_cells_vec, frel_nc, 'm-^','LineWidth',2.2,'MarkerSize',9,'MarkerFaceColor','m',  'DisplayName','f_{rel} (coupled resonance)');
plot(n_cells_vec, f_opt_nc,'r-o','LineWidth',2.2,'MarkerSize',9,'MarkerFaceColor','r',  'DisplayName','f_{opt} §4.12 (min M_{pm})');
yline(f_max_rotation,'k--','LineWidth',1.5,'Label',sprintf('RS05 rot. limit %.0f Hz',f_max_rotation),'LabelHorizontalAlignment','right','FontSize',8,'HandleVisibility','off');
xlabel('N_{parallel}','FontSize',10); ylabel('Frequency  (Hz)','FontSize',10);
title('Key Frequencies vs N','FontWeight','bold');
xticks(n_cells_vec); legend('Location','northeast','FontSize',9); grid on; box on;

%% Panel 4: Torque comparison — §4.12 vs Config 2
subplot(2,2,4); hold on;
plot(n_cells_vec, tau_pm_opt_nc, 'b-s','LineWidth',2.2,'MarkerSize',9,'MarkerFaceColor','b','DisplayName','§4.12 at f_{opt}');
plot(n_cells_vec, tau_c2_fn1_nc,'g-o','LineWidth',2.2,'MarkerSize',9,'MarkerFaceColor','g','DisplayName','Config 2 at f_{n1}');
yline(1.6,'r--','LineWidth',1.8,'Label','RS05 rated 1.6 N·m','LabelHorizontalAlignment','right','FontSize',8,'HandleVisibility','off');
yline(5.5,'m--','LineWidth',1.2,'Label','RS05 peak 5.5 N·m', 'LabelHorizontalAlignment','right','FontSize',8,'HandleVisibility','off');
fill([n_cells_vec(1)-0.5, n_cells_vec(end)+0.5, n_cells_vec(end)+0.5, n_cells_vec(1)-0.5], ...
     [0, 0, 1.6, 1.6],[0 0.8 0],'FaceAlpha',0.08,'EdgeColor','none','HandleVisibility','off');
xlabel('N_{parallel}','FontSize',10); ylabel('Required torque  (N·m)','FontSize',10);
title('Motor Torque Required: §4.12 vs Config 2','FontWeight','bold');
xticks(n_cells_vec); legend('Location','northwest','FontSize',9); grid on; box on;

sgtitle(sprintf('Comprehensive Design Space vs N_{parallel}  |  m_{hook}=%.0fg  |  d_{beam}=%.2fmm  |  \\zeta_h=%.2f', ...
        m_hook*1e3, d_stroke_1cell*1e3, zeta_hook), 'FontSize',12,'FontWeight','bold');

%% ══════════════════════════════════════════════════════════════════════
%%  ANIMATIONS
%% ══════════════════════════════════════════════════════════════════════
if ANIMATE
    run_excit_animation(t_pm, x_rel_p, v_rel_p, d_stroke, d_saddle, ...
                        V_fun, t_snap_p, m_hook, l_span, h_apex, t_beam, ...
                        @(t) f_opt, ' — Sec 4.12 Proof-Mass Optimal', ...
                        sprintf('§4.12 Proof-Mass  |  M_{pm}=%.0f g  |  f_{opt}=%.1f Hz  |  Δ=%.2f mm', ...
                                M_pm_opt_1d*1e3, f_opt, Delta_opt*1e3));
    run_excit_animation(t_c2, x_rel_c2, v_rel_c2, d_stroke, d_saddle, ...
                        V_fun, t_snap_c, m_hook, l_span, h_apex, t_beam, ...
                        @(t) f_n1, ' — Config 2 Prescribed Motion', ...
                        sprintf('Config 2 — Prescribed Base Motion  |  f_{n1}=%.1f Hz  |  A_{min}=%.2f mm', ...
                                f_n1, A_min_c2_num*1e3));
end

%% ══════════════════════════════════════════════════════════════════════
%%  AUTO-SAVE
%% ══════════════════════════════════════════════════════════════════════
save_dir = fileparts(mfilename('fullpath'));
if isempty(save_dir), save_dir=pwd; end
fig_dir = fullfile(save_dir,'figures','sim_excitation');
if ~exist(fig_dir,'dir'), mkdir(fig_dir); end

for fh = findobj('Type','figure')'
    nm = regexprep(fh.Name,'[^a-zA-Z0-9_\- ]','');
    nm = strtrim(strrep(nm,' ','_'));
    if isempty(nm), continue; end
    exportgraphics(fh,fullfile(fig_dir,[nm '.png']),'Resolution',150);
    fprintf('Saved: %s\n', nm);
end

set(0,'DefaultFigureVisible','on'); close all;

%% ══════════════════════════════════════════════════════════════════════
%%  LOCAL FUNCTIONS
%% ══════════════════════════════════════════════════════════════════════

function B = bistable_params(n_cells, d_stroke_1cell, d_saddle_frac, F_push_1cell, m_hook, zeta_hook)
%BISTABLE_PARAMS  Recompute all bistable quantities for a given n_cells.
%  Returns struct B with fields: d_stroke, d_saddle, k_eff_s1, k_eff_s2,
%  omega_n1, f_n1, omega_n2, f_n2, f_rel, c_hook, F_restore, V_fun, DeltaE2.
    M_plate_default = 0.045;
    B.d_stroke   = d_stroke_1cell;
    B.d_saddle   = d_saddle_frac * B.d_stroke;
    B.F_push     = n_cells * F_push_1cell;
    d_push       = 0.27 * B.d_stroke;
    denom        = d_push * (d_push - B.d_saddle) * (d_push - B.d_stroke);
    B.A_coeff    = B.F_push / denom;
    ds           = B.d_saddle;  df = B.d_stroke;
    B.F_restore  = @(x) -B.A_coeff .* x .* (x - ds) .* (x - df);
    B.V_fun      = @(x) B.A_coeff .* (x.^4/4 - (ds+df)*x.^3/3 + ds*df*x.^2/2);
    B.k_eff_s1   = B.A_coeff * B.d_saddle * B.d_stroke;
    B.k_eff_s2   = B.A_coeff * B.d_stroke * (B.d_stroke - B.d_saddle);
    B.omega_n1   = sqrt(B.k_eff_s1 / m_hook);
    B.f_n1       = B.omega_n1 / (2*pi);
    B.omega_n2   = sqrt(B.k_eff_s2 / m_hook);
    B.f_n2       = B.omega_n2 / (2*pi);
    B.c_hook     = 2 * zeta_hook * m_hook * B.omega_n1;
    B.DeltaE2    = B.V_fun(B.d_saddle) - B.V_fun(B.d_stroke);
    M_red        = m_hook * M_plate_default / (m_hook + M_plate_default);
    B.f_rel      = sqrt(B.k_eff_s1 / M_red) / (2*pi);
end

function [val,isterm,dir] = snap_event_base(~,y,d_saddle)
    val=y(3)-d_saddle; isterm=1; dir=1;
end

function [val,isterm,dir] = snap_event_2state(~,y,d_saddle)
    val=y(1)-d_saddle; isterm=1; dir=1;
end

function dydt = excit_pm_ode(t,y,M_eff,m_hook,k_supp,c_supp,c_hook,F_restore,M_pm,Delta,omega)
%EXCIT_PM_ODE  §4.12 proof-mass oscillator. State: [x_P;xd_P;x_rel;xd_rel]
    xP=y(1); xdP=y(2); xr=y(3); xdr=y(4);
    Fm   = M_pm*Delta*omega^2*sin(omega*t);
    Fr   = F_restore(xr);
    xddP = (Fm - k_supp*xP - c_supp*xdP - Fr + c_hook*xdr)/M_eff;
    xddH = (Fr - c_hook*xdr)/m_hook;
    dydt = [xdP; xddP; xdr; xddH-xddP];
end

function dydt = excit_prescribed_ode(t,y,m_hook,c_hook,F_restore,A_plate,omega)
%EXCIT_PRESCRIBED_ODE  Config 2 prescribed plate motion. State: [x_rel;xd_rel]
    xr=y(1); xdr=y(2);
    xddP  = -A_plate*omega^2*sin(omega*t);
    Fr    = F_restore(xr);
    xddr  = (Fr - c_hook*xdr)/m_hook - xddP;
    dydt  = [xdr; xddr];
end

function run_excit_animation(t_all,x_all,v_all,d_stroke,d_saddle,V_fun,t_snap, ...
                              m_hook,l_span,h_apex,t_beam,f_inst,fig_label,fig_title)
    if nargin<13||isempty(fig_label), fig_label=''; end
    if nargin<14||isempty(fig_title), fig_title=fig_label; end
    fig=figure('Name',['Bistable Excitation — Animation' fig_label], ...
               'NumberTitle','off','Color','w','Position',[500 120 1300 520]);
    xi=linspace(0,l_span,300);
    x_v=linspace(0,d_stroke,500);
    v_sep=sqrt(max(0,2*(V_fun(d_saddle)-V_fun(x_v))/m_hook));
    % Left: beam
    subplot(1,3,1); ys1=(h_apex/2)*(1-cos(2*pi*xi/l_span)); hold on;
    hbeam=fill([xi,fliplr(xi)]*1e3,[ys1+t_beam/2,fliplr(ys1-t_beam/2)]*1e3,[.4 .7 1],'EdgeColor','b','LineWidth',1.5,'FaceAlpha',.7);
    patch([-1,0,0,-1]*1e3,[-h_apex,-h_apex,h_apex,h_apex]*1.5*1e3,[.5 .5 .5],'EdgeColor','k');
    patch([l_span,(l_span+1e-3),(l_span+1e-3),l_span]*1e3,[-h_apex,-h_apex,h_apex,h_apex]*1.5*1e3,[.5 .5 .5],'EdgeColor','k');
    htb=title('t=0.000 s  State 1','FontSize',10); xlabel('mm'); ylabel('mm');
    axis equal; xlim([-2,(l_span+2e-3)*1e3]); ylim([-h_apex*2.5e3,h_apex*2.5e3]); grid on; box on;
    % Centre: displacement
    subplot(1,3,2); hold on;
    if ~isnan(t_snap), xline(t_snap,'r:','LineWidth',1.8,'Label',sprintf('snap t=%.2f s',t_snap),'FontSize',8); end
    yline(0,'b--','LineWidth',1.2,'Label','State 1','LabelHorizontalAlignment','left','FontSize',8);
    yline(d_stroke*1e3,'g--','LineWidth',1.2,'Label','State 2','LabelHorizontalAlignment','left','FontSize',8);
    yline(d_saddle*1e3,'k:','LineWidth',.9,'Label','Saddle','LabelHorizontalAlignment','right','FontSize',8);
    h_trace=animatedline('Color',[0 .45 .74],'LineWidth',1.5);
    h_cur=plot(t_all(1),x_all(1)*1e3,'o','Color',[.85 .33 .1],'MarkerFaceColor',[.85 .33 .1],'MarkerSize',9,'HandleVisibility','off');
    xlabel('Time (s)','FontSize',9); ylabel('Hook disp. (mm)','FontSize',9); title('Displacement vs Time','FontWeight','bold');
    xlim([0,t_all(end)]); ylim([-0.12*d_stroke*1e3,1.18*d_stroke*1e3]); grid on; box on;
    % Right: phase portrait
    subplot(1,3,3);
    fill([x_v,fliplr(x_v)]*1e3,[v_sep,-fliplr(v_sep)]*1e3,[1 .9 .9],'EdgeColor','none','FaceAlpha',.4); hold on;
    plot(x_v*1e3,v_sep*1e3,'r--','LineWidth',1.5); plot(x_v*1e3,-v_sep*1e3,'r--','LineWidth',1.5);
    hph=animatedline('Color','b','LineWidth',1.2); hpc=plot(0,0,'ko','MarkerFaceColor','r','MarkerSize',9);
    xlabel('x_{rel} (mm)'); ylabel('Velocity (mm/s)'); title('Phase Portrait'); grid on; box on;
    xlim([-.05*d_stroke,1.1*d_stroke]*1e3);
    v_rng=max(abs(v_all))*1.3e3; v_rng=max(v_rng,max(v_sep)*1e3*.15); ylim([-v_rng,v_rng]);
    sgtitle(fig_title,'FontSize',10,'FontWeight','bold');
    % GIF
    gif_dir=fileparts(mfilename('fullpath')); if isempty(gif_dir),gif_dir=pwd; end
    gif_dir=fullfile(gif_dir,'figures','sim_excitation'); if ~exist(gif_dir,'dir'),mkdir(gif_dir); end
    gif_path=fullfile(gif_dir,['Bistable_Excitation_Animation' strrep(strtrim(fig_label),' ','_') '.gif']);
    has_ipt=license('test','image_toolbox')&&exist('rgb2ind','file')==2;
    first_gif=true; skip=max(1,round(numel(t_all)/120)); prev_k=1;
    for k=1:skip:numel(t_all)
        xk=x_all(k); vk=v_all(k); tk=t_all(k);
        frac=min(1,max(0,xk/d_stroke));
        col=(1-frac)*[.4 .7 1]+frac*[.4 1 .6];
        deployed=~isnan(t_snap)&&tk>=t_snap;
        bc=[0 .65 0]*deployed+[.85 0 0]*(~deployed);
        y_interp=(1-frac)*((h_apex/2)*(1-cos(2*pi*xi/l_span)))+frac*(-(h_apex/2)*(1-cos(2*pi*xi/l_span)));
        set(hbeam,'XData',[xi,fliplr(xi)]*1e3,'YData',[y_interp+t_beam/2,fliplr(y_interp-t_beam/2)]*1e3,'FaceColor',col,'EdgeColor',col*.6);
        addpoints(h_trace,t_all(prev_k:k),x_all(prev_k:k)*1e3); prev_k=k+1;
        set(h_cur,'XData',tk,'YData',xk*1e3);
        set(hpc,'XData',xk*1e3,'YData',vk*1e3,'MarkerFaceColor',bc);
        addpoints(hph,xk*1e3,vk*1e3);
        state_str='State 1 (Retracted)'; if deployed,state_str='State 2 (Deployed)'; end
        set(htb,'String',sprintf('t=%.3f s | %.1f Hz | %s',tk,f_inst(tk),state_str));
        drawnow;
        if has_ipt
            fr=getframe(fig); im=frame2im(fr); [imind,cm]=rgb2ind(im,256);
            if first_gif, imwrite(imind,cm,gif_path,'gif','Loopcount',inf,'DelayTime',.06); first_gif=false;
            else, imwrite(imind,cm,gif_path,'gif','WriteMode','append','DelayTime',.06); end
        end
    end
    if has_ipt,fprintf('  GIF: %s\n',gif_path); end
end
