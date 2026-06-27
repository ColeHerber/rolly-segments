%% optimize_displacement_sim2.m
% Optimization for the Curved-Beam Bistable Mechanism (Simulation 2).
%
% Finds:
%   1. omega_opt — frequency maximizing pre-snap displacement amplitude in State 1
%   2. F0_min(omega) — minimum excitation amplitude for snap-through at each frequency
%   3. Sensitivity sweep — how snap threshold changes with h_apex, t_beam,
%      and parallel beam count n_cells
%
% Usage:
%   [omega_opt, F0_min_curve, sweep] = optimize_displacement_sim2(params)
%
% Call with no arguments to run with Design 9 defaults and plot everything.

function [omega_opt, F0_min_curve, sweep] = optimize_displacement_sim2(params)

if nargin == 0
    %% ── Default parameters (match bistable_curved_beam.m Design 9) ──────
    params.F_push_1cell    = 52.40;    % [N] current 3.5 mm beam analytical value
    params.d_stroke_1cell  = 7.7e-3;   % [m] single-beam stroke
    params.d_saddle_frac   = 0.545;    % [-]
    params.n_cells         = 5;        % [-] parallel beam count
    params.m_hook          = 0.200;    % [kg]
    params.zeta_hook       = 0.02;     % [-]
    params.t_end           = 25.0;     % [s]  search window
    params.F0_lo           = 0.01;     % [N]
    params.F0_hi           = 10.0;     % [N]
    params.n_freq          = 50;       % frequency test points
    params.n_bisect        = 20;       % binary-search steps
    params.f_lo            = 1;        % [Hz] sweep start
    params.f_hi            = 500;      % [Hz] sweep end (no cap)

    % Sensitivity sweep parameters
    params.h_apex_range    = linspace(1.5e-3, 4.5e-3, 6);   % [m]
    params.t_beam_range    = linspace(0.4e-3, 1.0e-3, 5);   % [m]
    params.n_cells_range   = 1:8;

    params.plot_results = true;
end

% Build force model from params
[F_restore, V_fun, d_stroke, d_saddle, k_eff_s1, k_eff_s2, ~] = ...
    build_force_model(params);

m    = params.m_hook;
zeta = params.zeta_hook;
omega_n1 = sqrt(k_eff_s1 / m);  f_n1 = omega_n1/(2*pi);
omega_n2 = sqrt(k_eff_s2 / m);  f_n2 = omega_n2/(2*pi);
c    = 2*zeta*m*omega_n1;

DeltaE = V_fun(d_saddle) - V_fun(0);

fprintf('\n══════════════════════════════════════════════════════════\n');
fprintf('  Optimization: Sim 2 — Curved-Beam Bistable\n');
fprintf('  d_stroke = %.1f mm | d_saddle = %.1f mm\n', d_stroke*1e3, d_saddle*1e3);
fprintf('  f_n1 (State 1) = %.3f Hz | f_n2 (State 2) = %.3f Hz\n', f_n1, f_n2);
fprintf('  ΔE (barrier)   = %.4f mJ\n', DeltaE*1e3);
fprintf('  Searching %d freq × %d bisect steps\n', params.n_freq, params.n_bisect);
fprintf('══════════════════════════════════════════════════════════\n');

%% ── 1: F0_min(omega) sweep ───────────────────────────────────────────────
omega_vec = 2*pi * logspace(log10(params.f_lo), log10(params.f_hi), params.n_freq);
F0_min_vec   = NaN(size(omega_vec));
t_snap_vec   = NaN(size(omega_vec));
amp_presnapvec = NaN(size(omega_vec));

y0   = [0; 0];  % State 1
opts = odeset('Events', @(t,y) snap_event(t,y,d_saddle), ...
              'RelTol',1e-6,'AbsTol',1e-8,'MaxStep',5e-4);
opts_free = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',5e-4);

for ki = 1:numel(omega_vec)
    om = omega_vec(ki);

    % Binary search on F0
    lo = params.F0_lo;  hi = params.F0_hi;
    F0_found = NaN;  t_found = NaN;
    for bi = 1:params.n_bisect
        F0_try = (lo+hi)/2;
        rhs = @(t,y)[y(2); (F0_try*sin(om*t)+F_restore(y(1))-c*y(2))/m];
        [~,~,te,~,~] = ode45(rhs,[0,params.t_end],y0,opts);
        if ~isempty(te)
            hi = F0_try;  F0_found = F0_try;  t_found = te(1);
        else
            lo = F0_try;
        end
    end
    F0_min_vec(ki) = F0_found;
    t_snap_vec(ki) = t_found;

    % Pre-snap peak displacement (at 90% of F0_min, no snap)
    if ~isnan(F0_found)
        F0_sub = F0_found * 0.9;
        rhs_s  = @(t,y)[y(2);(F0_sub*sin(om*t)+F_restore(y(1))-c*y(2))/m];
        [~,ys] = ode45(rhs_s,[0,params.t_end],y0,opts_free);
        amp_presnapvec(ki) = max(ys(:,1));
    end

    if mod(ki,10)==0
        fprintf('  [%3d/%d]  f = %6.2f Hz  F0_min = %.4f N\n', ...
                ki, numel(omega_vec), om/(2*pi), F0_found);
    end
end

% Optimal frequency (minimum F0_min)
valid = ~isnan(F0_min_vec);
[F0_min, idx_opt] = min(F0_min_vec(valid));
fv = omega_vec(valid)/(2*pi);
omega_opt = fv(idx_opt) * 2*pi;
t_snap_opt= t_snap_vec(valid);  t_snap_opt = t_snap_opt(idx_opt);

fprintf('\n══ RESULT ══════════════════════════════════════════════\n');
fprintf('  Optimal frequency  f_opt  = %.3f Hz\n', omega_opt/(2*pi));
fprintf('  Ratio to f_n1             = %.3f\n', (omega_opt/(2*pi))/f_n1);
fprintf('  Minimum amplitude  F0_min = %.5f N\n', F0_min);
fprintf('  Snap time          t_snap = %.3f s\n', t_snap_opt);
fprintf('════════════════════════════════════════════════════════\n\n');

F0_min_curve.omega = omega_vec;
F0_min_curve.F0    = F0_min_vec;
F0_min_curve.t_snap= t_snap_vec;
F0_min_curve.amp   = amp_presnapvec;

%% ── 2: Sensitivity sweep ─────────────────────────────────────────────────
fprintf('Running sensitivity sweep …\n');
sweep = run_sensitivity(params, omega_opt);

%% ── Plots ────────────────────────────────────────────────────────────────
if isfield(params,'plot_results') && params.plot_results
    plot_results_sim2(F0_min_curve, sweep, omega_opt, F0_min, f_n1, f_n2, ...
                      d_stroke, d_saddle, params);
end

end % main function

%% ══════════════════════════════════════════════════════════════════════════
%%  SENSITIVITY SWEEP
%% ══════════════════════════════════════════════════════════════════════════
function sweep = run_sensitivity(params, ~)

sweep.h_apex  = struct();
sweep.t_beam  = struct();
sweep.n_cells = struct();

% vary h_apex (keep t_beam = 0.6mm, n_cells = 5 parallel beams)
p2 = params;
p2.t_beam_val  = 0.6e-3;
p2.n_cells     = params.n_cells;
hv = params.h_apex_range;
sweep.h_apex.vals    = hv;
sweep.h_apex.Q       = hv / 0.6e-3;
sweep.h_apex.f_n1    = NaN(size(hv));
sweep.h_apex.F0_min  = NaN(size(hv));
for i = 1:numel(hv)
    p2.h_apex_val = hv(i);
    [~,~,~,~,k1s,~,~] = build_force_model_raw(params.F_push_1cell, ...
        params.d_stroke_1cell, params.d_saddle_frac, params.n_cells, ...
        hv(i), 0.6e-3);
    sweep.h_apex.f_n1(i) = sqrt(k1s/params.m_hook)/(2*pi);
    sweep.h_apex.F0_min(i) = params.n_cells * params.F_push_1cell;
end

% vary t_beam (keep h_apex = 2.5mm, n_cells = 5 parallel beams)
tv = params.t_beam_range;
sweep.t_beam.vals   = tv;
sweep.t_beam.Q      = 2.5e-3 ./ tv;
sweep.t_beam.f_n1   = NaN(size(tv));
    sweep.t_beam.bistable = tv <= 2.5e-3 / 2.35;   % Q >= 2.35 condition
for i = 1:numel(tv)
    [~,~,~,~,k1s,~,~] = build_force_model_raw(params.F_push_1cell, ...
        params.d_stroke_1cell, params.d_saddle_frac, params.n_cells, ...
        2.5e-3, tv(i));
    sweep.t_beam.f_n1(i) = sqrt(k1s/params.m_hook)/(2*pi);
end

% vary parallel beam count (keep h_apex=2.5mm, t_beam=0.6mm)
nv = params.n_cells_range;
sweep.n_cells.vals  = nv;
sweep.n_cells.f_n1  = NaN(size(nv));
sweep.n_cells.stroke= NaN(size(nv));
for i = 1:numel(nv)
    [~,~,dstk,~,k1s,~,~] = build_force_model_raw(params.F_push_1cell, ...
        params.d_stroke_1cell, params.d_saddle_frac, nv(i), 2.5e-3, 0.6e-3);
    sweep.n_cells.f_n1(i)  = sqrt(k1s/params.m_hook)/(2*pi);
    sweep.n_cells.stroke(i) = dstk*1e3;
end

end

%% ══════════════════════════════════════════════════════════════════════════
%%  PLOTTING
%% ══════════════════════════════════════════════════════════════════════════
function plot_results_sim2(fc, sw, omega_opt, F0_min_opt, f_n1, f_n2, ~, ~, params)

f_opt = omega_opt/(2*pi);
freq_hz = fc.omega/(2*pi);
valid   = ~isnan(fc.F0);

figure('Name','Optimize Sim 2 — Curved Beam','NumberTitle','off', ...
       'Color','w','Position',[80 60 1400 900]);

%% Panel 1: F0_min vs frequency
subplot(2,3,1);
plot(freq_hz(valid), fc.F0(valid),'b-o','LineWidth',2,'MarkerSize',4); hold on;
plot(f_opt, F0_min_opt,'rp','MarkerFaceColor','r','MarkerSize',14, ...
     'DisplayName',sprintf('Optimum: %.2f Hz, F_0=%.4f N',f_opt,F0_min_opt));
xline(f_n1,'k--','LineWidth',1.5,'Label',sprintf('f_{n1}=%.1f Hz',f_n1),'FontSize',8);
xline(f_n2,'m--','LineWidth',1.5,'Label',sprintf('f_{n2}=%.1f Hz',f_n2),'FontSize',8);
set(gca,'XScale','log');
xlabel('Drive frequency  (Hz)','FontSize',10);
ylabel('F_{0,min}  (N)','FontSize',10);
title('Minimum Amplitude for Snap-Through','FontWeight','bold');
legend('F_{0,min}','Optimum','Location','northeast','FontSize',8);
grid on; box on;

%% Panel 2: Snap time vs frequency
subplot(2,3,2);
plot(freq_hz(valid), fc.t_snap(valid),'r-o','LineWidth',2,'MarkerSize',4); hold on;
xline(f_n1,'k--','LineWidth',1.5,'Label',sprintf('f_{n1}=%.1f Hz',f_n1));
set(gca,'XScale','log');
xlabel('Drive frequency  (Hz)','FontSize',10);
ylabel('t_{snap}  (s)','FontSize',10);
title('Snap-Through Time vs Frequency','FontWeight','bold');
grid on; box on;

%% Panel 3: Snap map  (frequency × F0 heatmap)
ax3 = subplot(2,3,3);
fv = freq_hz(valid);
Fv = fc.F0(valid);
n_F0s = 25;
F0_axis = linspace(params.F0_lo, min(params.F0_hi, max(Fv)*2), n_F0s);
snap_map = zeros(n_F0s, numel(fv));
for fi = 1:numel(fv)
    for Fi = 1:n_F0s
        snap_map(Fi,fi) = double(F0_axis(Fi) >= Fv(fi));
    end
end
imagesc(fv, F0_axis, snap_map); axis xy;
colormap(ax3,[1 0.8 0.8; 0.85 1 0.85]);
hold on;
plot(fv, Fv,'k-','LineWidth',2,'DisplayName','F_{0,min}');
plot(f_opt, F0_min_opt,'rp','MarkerFaceColor','r','MarkerSize',14,'DisplayName','Optimum');
set(gca,'XScale','log');
xlabel('Drive frequency  (Hz)','FontSize',10);
ylabel('F_0  (N)','FontSize',10);
title('Snap Map  (red=no snap, green=snap)','FontWeight','bold');
legend('FontSize',8,'Location','northwest'); box on;

%% Panel 4: Sensitivity — n_cells
subplot(2,3,4);
yyaxis left
plot(sw.n_cells.vals, sw.n_cells.f_n1,'b-o','LineWidth',2,'MarkerSize',7,...
     'DisplayName','f_{n1}  (State 1 freq)');
ylabel('f_{n1}  (Hz)','FontSize',10);
yyaxis right
plot(sw.n_cells.vals, sw.n_cells.stroke,'r-s','LineWidth',2,'MarkerSize',7,...
     'DisplayName','Mechanism stroke  (mm)');
ylabel('Mechanism stroke  (mm)','FontSize',10);
xlabel('Parallel beam count','FontSize',10);
title('Sensitivity: Parallel Beam Count','FontWeight','bold');
legend('Location','northeast','FontSize',8);
grid on; box on;

%% Panel 5: Sensitivity — h_apex
subplot(2,3,5);
Q_vals = sw.h_apex.Q;
yyaxis left
plot(Q_vals, sw.h_apex.f_n1,'b-o','LineWidth',2,'MarkerSize',7,'DisplayName','f_{n1}');
ylabel('f_{n1}  (Hz)','FontSize',10);
    xline(2.35,'r--','LineWidth',1.5,'Label','Q=2.35 (threshold)','FontSize',9);
yyaxis right
plot(Q_vals, sw.h_apex.vals*1e3,'g-^','LineWidth',2,'MarkerSize',7,'DisplayName','h_{apex} (mm)');
ylabel('h_{apex}  (mm)','FontSize',10);
xlabel('Q = h/t','FontSize',10);
title('Sensitivity: Apex Height (Q ratio)','FontWeight','bold');
legend('Location','northeast','FontSize',8);
grid on; box on;

%% Panel 6: Sensitivity — t_beam
subplot(2,3,6);
tv_mm = sw.t_beam.vals*1e3;
yyaxis left
plot(tv_mm, sw.t_beam.f_n1,'b-o','LineWidth',2,'MarkerSize',7);
ylabel('f_{n1}  (Hz)','FontSize',10);
yyaxis right
plot(tv_mm, sw.t_beam.Q,'m-s','LineWidth',2,'MarkerSize',7);
    yline(2.35,'r--','LineWidth',1.5,'Label','Q_{min}=2.35');
ylabel('Q = h/t','FontSize',10);
xlabel('Beam thickness  (mm)','FontSize',10);
title('Sensitivity: Beam Thickness','FontWeight','bold');
grid on; box on;

sgtitle(sprintf(['Optimization — Curved-Beam Bistable  |  Design 9 defaults  |  ' ...
                 'f_{n1}=%.2f Hz, ΔE=%.3f mJ'], f_n1, (params.n_cells * params.F_push_1cell * params.d_stroke_1cell)/4*1e3), ...
        'FontSize',12,'FontWeight','bold');
end

%% ══════════════════════════════════════════════════════════════════════════
%%  HELPER: build restoring-force model from params struct
%% ══════════════════════════════════════════════════════════════════════════
function [F_restore, V_fun, d_stroke, d_saddle, k_eff_s1, k_eff_s2, A_coeff] = ...
    build_force_model(params)

[F_restore, V_fun, d_stroke, d_saddle, k_eff_s1, k_eff_s2, A_coeff] = ...
    build_force_model_raw(params.F_push_1cell, params.d_stroke_1cell, ...
                          params.d_saddle_frac, params.n_cells, ...
                          2.5e-3, 0.6e-3);  % h_apex, t_beam not used in force model
end

function [F_restore, V_fun, d_stroke, d_saddle, k_eff_s1, k_eff_s2, A_coeff] = ...
    build_force_model_raw(F_push_1cell, d_stroke_1cell, d_saddle_frac, n_cells, ~, ~)

d_stroke = d_stroke_1cell;
d_saddle = d_saddle_frac * d_stroke;
d_push   = 0.27 * d_stroke;
F_push_total = n_cells * F_push_1cell;

% denom = (+)(-)(-)  > 0  →  A_coeff > 0  →  F_restore = -A*x*(x-ds)*(x-df)
% gives negative restoring force for 0 < x < d_saddle  (State 1 stable) ✓
denom   = d_push * (d_push - d_saddle) * (d_push - d_stroke);
A_coeff = F_push_total / denom;

F_restore = @(x) -A_coeff .* x .* (x - d_saddle) .* (x - d_stroke);

ds = d_saddle; df = d_stroke;
V_fun = @(x) A_coeff .* (x.^4/4 - (ds+df)*x.^3/3 + ds*df*x.^2/2);

k_eff_s1 = A_coeff * d_saddle * d_stroke;
k_eff_s2 = A_coeff * d_stroke * (d_stroke - d_saddle);
end

%% ── Event function ────────────────────────────────────────────────────────
function [val, isterm, dir] = snap_event(~, y, d_saddle)
    val    = y(1) - d_saddle;
    isterm = 1;
    dir    = 1;
end
