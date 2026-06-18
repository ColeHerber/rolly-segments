%% optimize_displacement_sim1.m
% Optimization for the Classical Bistable MSD (Duffing oscillator).
%
% Finds the optimal driving frequency omega_opt and minimum excitation
% amplitude F0_min that causes snap-through within t_end seconds.
%
% Usage:
%   [omega_opt, F0_min, results] = optimize_displacement_sim1(params)
%
% If called with no arguments, uses default parameters and plots results.

function [omega_opt, F0_min, results] = optimize_displacement_sim1(params)

if nargin == 0
    % ── Default parameters (match bistable_classic_msd.m defaults) ──────
    params.m      = 0.010;    % [kg]
    params.k1     = 20;       % [N/m]
    params.k3     = 88889;    % [N/m³]
    params.zeta   = 0.05;     % [-]
    params.t_end  = 15.0;     % [s]  search window
    params.t_ramp = 0;        % [s]  no ramp for optimization (instant amplitude)
    params.n_omega = 60;      % number of frequency test points
    params.n_F0    = 25;      % binary-search steps per frequency
    params.F0_lo   = 0.001;   % [N] minimum F0 to search
    params.F0_hi   = 5.0;     % [N] maximum F0 to search
    params.omega_range_factor = [0.3, 3.5]; % search [factor_lo, factor_hi] × omega_n
    params.plot_results = true;
end

% Unpack
m     = params.m;
k1    = params.k1;
k3    = params.k3;
zeta  = params.zeta;
t_end = params.t_end;

x_eq    = sqrt(k1/k3);
omega_n = sqrt(2*k1/m);
f_n     = omega_n/(2*pi);
c       = 2*zeta*m*omega_n;
DeltaE  = k1^2/(4*k3);

F_restore = @(x) k1*x - k3*x.^3;

fprintf('\n══════════════════════════════════════════════════════\n');
fprintf('  Optimization: Sim 1 — Duffing Bistable MSD\n');
fprintf('  f_n = %.3f Hz | x_eq = %.2f mm | ΔE = %.3f mJ\n', ...
        f_n, x_eq*1e3, DeltaE*1e3);
fprintf('  Searching %d frequency points × %d F0 binary steps\n', ...
        params.n_omega, params.n_F0);
fprintf('══════════════════════════════════════════════════════\n');

% Frequency grid (covers sub-harmonic, fundamental, super-harmonic)
omega_lo  = params.omega_range_factor(1) * omega_n;
omega_hi  = params.omega_range_factor(2) * omega_n;
omega_vec = linspace(omega_lo, omega_hi, params.n_omega);

F0_min_vec   = NaN(size(omega_vec));
t_snap_vec   = NaN(size(omega_vec));
max_disp_vec = NaN(size(omega_vec));

opts = odeset('Events', @snap_event, 'RelTol',1e-6,'AbsTol',1e-8,'MaxStep',5e-4);
y0 = [-x_eq; 0];

for ki = 1:params.n_omega
    om = omega_vec(ki);

    % Binary search for minimum F0 that causes snap
    lo = params.F0_lo;  hi = params.F0_hi;
    F0_found = NaN;  t_found = NaN;

    for bi = 1:params.n_F0
        F0_try = (lo + hi) / 2;
        rhs = @(t,y) [y(2); (F0_try*sin(om*t) + F_restore(y(1)) - c*y(2))/m];
        [~, ~, te, ~, ~] = ode45(rhs, [0, t_end], y0, opts);
        if ~isempty(te)
            hi = F0_try;
            F0_found = F0_try;
            t_found  = te(1);
        else
            lo = F0_try;
        end
    end

    F0_min_vec(ki) = F0_found;
    t_snap_vec(ki) = t_found;

    % Maximum displacement at this F0 (sub-threshold, F0 just below)
    if ~isnan(F0_found)
        F0_sub = F0_found * 0.92;
        rhs_sub = @(t,y) [y(2); (F0_sub*sin(om*t) + F_restore(y(1)) - c*y(2))/m];
        [~, ysub] = ode45(rhs_sub, [0, t_end], y0, ...
                          odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',5e-4));
        max_disp_vec(ki) = max(abs(ysub(:,1)));
    end

    if mod(ki, 10) == 0
        fprintf('  [%3d/%d]  f = %.2f Hz  →  F0_min = %.4f N  t_snap = %.2f s\n', ...
                ki, params.n_omega, om/(2*pi), F0_found, t_found);
    end
end

% Find optimum: minimum F0_min over all tested frequencies
valid = ~isnan(F0_min_vec);
[F0_min, idx_opt] = min(F0_min_vec(valid));
omega_valid = omega_vec(valid);
omega_opt   = omega_valid(idx_opt);
f_opt       = omega_opt / (2*pi);
t_snap_opt  = t_snap_vec(valid);
t_snap_opt  = t_snap_opt(idx_opt);

fprintf('\n══ RESULT ══════════════════════════════════════════\n');
fprintf('  Optimal frequency  f_opt  = %.3f Hz\n', f_opt);
fprintf('  Ratio to f_n              = %.3f\n', f_opt/f_n);
fprintf('  Minimum amplitude  F0_min = %.5f N\n', F0_min);
fprintf('  Snap-through time  t_snap = %.3f s\n', t_snap_opt);
fprintf('════════════════════════════════════════════════════\n\n');

% Package results
results.omega_vec   = omega_vec;
results.F0_min_vec  = F0_min_vec;
results.t_snap_vec  = t_snap_vec;
results.max_disp_vec= max_disp_vec;
results.omega_opt   = omega_opt;
results.F0_min      = F0_min;
results.t_snap_opt  = t_snap_opt;
results.f_n         = f_n;
results.omega_n     = omega_n;

if isfield(params,'plot_results') && params.plot_results
    plot_optimization_results(results, params);
end

end % function

%% ── Plotting ─────────────────────────────────────────────────────────────
function plot_optimization_results(r, params)

f_vec = r.omega_vec / (2*pi);
f_n   = r.f_n;

figure('Name','Optimize Sim 1 — Duffing Bistable MSD','NumberTitle','off', ...
       'Color','w','Position',[80 80 1100 800]);

%% Panel 1: F0_min vs frequency
ax1 = subplot(2,2,1);
valid = ~isnan(r.F0_min_vec);
plot(f_vec(valid), r.F0_min_vec(valid),'b-o','LineWidth',2,'MarkerSize',5); hold on;
[~,io] = min(r.F0_min_vec(valid));
fv = f_vec(valid);
plot(fv(io), r.F0_min_vec(valid & ~isnan(r.F0_min_vec)), ...
     'rs','MarkerFaceColor','r','MarkerSize',10,'HandleVisibility','off');
scatter(r.omega_opt/(2*pi), r.F0_min, 120,'rp','filled','DisplayName', ...
        sprintf('Optimum: %.2f Hz, F_0=%.4f N', r.omega_opt/(2*pi), r.F0_min));
xline(f_n,'k--','LineWidth',1.5,'Label','f_n','FontSize',9);
xlabel('Drive frequency  (Hz)','FontSize',10);
ylabel('F_{0,min}  (N)','FontSize',10);
title('Minimum Amplitude to Cause Snap-Through','FontWeight','bold');
legend('Location','northwest','FontSize',8);
grid on; box on;

%% Panel 2: Snap-through time vs frequency
ax2 = subplot(2,2,2);
plot(f_vec(valid), r.t_snap_vec(valid),'r-o','LineWidth',2,'MarkerSize',5); hold on;
xline(f_n,'k--','LineWidth',1.5,'Label','f_n');
xlabel('Drive frequency  (Hz)','FontSize',10);
ylabel('t_{snap}  (s)','FontSize',10);
title('Snap-Through Time vs Drive Frequency','FontWeight','bold');
grid on; box on;

%% Panel 3: Optimal trajectory simulation
ax3 = subplot(2,2,3);
m   = params.m; k1 = params.k1; k3 = params.k3;
zeta= params.zeta;
omega_n = r.omega_n; c = 2*zeta*m*omega_n;
x_eq = sqrt(k1/k3);
F_restore = @(x) k1*x - k3*x.^3;

om_opt = r.omega_opt;
rhs_opt = @(t,y)[y(2);(r.F0_min*sin(om_opt*t)+F_restore(y(1))-c*y(2))/m];
[t_tr, y_tr] = ode45(rhs_opt, [0, min(r.t_snap_opt*2, params.t_end)], [-x_eq;0], ...
                     odeset('RelTol',1e-7,'AbsTol',1e-10,'MaxStep',5e-4));

plot(t_tr, y_tr(:,1)*1e3,'b-','LineWidth',1.8); hold on;
yline( x_eq*1e3,'g--','LineWidth',1.2,'Label','State 2');
yline(-x_eq*1e3,'r--','LineWidth',1.2,'Label','State 1');
yline(0,'k:','LineWidth',0.8);
xline(r.t_snap_opt,'k--','LineWidth',1.5,'Label',sprintf('Snap t=%.2f s',r.t_snap_opt));
xlabel('Time  (s)','FontSize',10); ylabel('x  (mm)','FontSize',10);
title(sprintf('Optimal Trajectory  (f=%.2f Hz, F_0=%.4f N)',r.omega_opt/(2*pi),r.F0_min),'FontWeight','bold');
grid on; box on;

%% Panel 4: Snap probability heatmap (F0 vs frequency)
ax4 = subplot(2,2,4);
n_F0s = 20;
F0_range = linspace(params.F0_lo, min(params.F0_hi, max(r.F0_min_vec(valid))*2), n_F0s);
snap_map = zeros(n_F0s, sum(valid));
f_vec_v  = f_vec(valid);
F0min_v  = r.F0_min_vec(valid);
for fi = 1:numel(f_vec_v)
    for Fi = 1:n_F0s
        snap_map(Fi, fi) = double(F0_range(Fi) >= F0min_v(fi));
    end
end
imagesc(f_vec_v, F0_range, snap_map); axis xy;
colormap(ax4, [1 0.8 0.8; 0.8 1 0.8]);
hold on;
plot(f_vec_v, F0min_v,'k-','LineWidth',2,'DisplayName','F_{0,min} boundary');
plot(r.omega_opt/(2*pi), r.F0_min,'rp','MarkerFaceColor','r','MarkerSize',14,'DisplayName','Optimum');
legend('FontSize',8,'Location','northwest');
xlabel('Drive frequency  (Hz)','FontSize',10);
ylabel('F_0  (N)','FontSize',10);
title('Snap-Through Map  (red=no snap, green=snap)','FontWeight','bold');
box on;

sgtitle(sprintf('Optimization — Duffing MSD  |  m=%.0fg, k_1=%.0f N/m, ζ=%.3f, f_n=%.2f Hz', ...
        params.m*1e3, params.k1, params.zeta, r.f_n), ...
        'FontSize',12,'FontWeight','bold');
end

%% ── Event function ────────────────────────────────────────────────────────
function [val, isterm, dir] = snap_event(~, y)
    val    = y(1);
    isterm = 1;
    dir    = 1;
end
