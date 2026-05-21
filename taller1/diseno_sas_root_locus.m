function sas_data = diseno_sas_root_locus(channels, cfg, export_figures)
%DISENO_SAS_ROOT_LOCUS Disena el SAS desde los canales de velocidad angular.
%
% La ley usada en simulacion es:
%   u_e = q_cmd - D_q*q
%   u_a = p_cmd - D_p*p
%
% Por eso el lazo interno que se revisa por root locus es G/(1 + D*G).

if nargin < 3
    export_figures = true;
end

ensure_figures_dir(cfg);

sas_data.theta = design_axis(channels.q, ...
    [-0.05 -0.10 -0.15 -0.20 -0.30 -0.50], -0.20, ...
    'q/elevator', 'D_q');
sas_data.phi = design_axis(channels.p, ...
    [-0.08 -0.05 0.02 0.05 0.08 0.10], 0.05, ...
    'p/aileron', 'D_p');
sas_data.yaw = design_axis(channels.r, ...
    [-0.50 -0.20 -0.10 -0.05], -cfg.pid.yaw_damper_gain, ...
    'r/rudder', 'D_r');

sas_data.D_q = sas_data.theta.selected_gain;
sas_data.D_p = sas_data.phi.selected_gain;
sas_data.D_r = sas_data.yaw.selected_gain;
sas_data.notes = ['SAS elegido por root locus: D_q negativo amortigua ', ...
    'el modo de q; D_p positivo conserva estable el lazo interno de p; ', ...
    'yaw se mantiene como damper con washout.'];

if export_figures
    plot_axis_root_locus(channels.q, sas_data.theta, cfg, ...
        'root_locus_sas_q.png', 'SAS pitch: q/elevator');
    plot_axis_root_locus(channels.p, sas_data.phi, cfg, ...
        'root_locus_sas_p.png', 'SAS roll: p/aileron');
    plot_axis_root_locus(channels.r, sas_data.yaw, cfg, ...
        'root_locus_sas_r.png', 'Yaw damper: r/rudder');
end
end

function axis = design_axis(G, candidates, selected_gain, label, gain_name)
axis.label = label;
axis.gain_name = gain_name;
axis.candidates = evaluate_candidates(G, candidates);
axis.selected_gain = selected_gain;
axis.selected = evaluate_candidates(G, selected_gain);
axis.inner_loop = minreal(feedback(G, selected_gain), 1e-6);
axis.inner_loop_stable = isstable(axis.inner_loop);
axis.open_loop_dcgain = safe_dcgain(G);
axis.inner_loop_dcgain = safe_dcgain(axis.inner_loop);
end

function rows = evaluate_candidates(G, candidates)
for k = 1:numel(candidates)
    D = candidates(k);
    cl = minreal(feedback(G, D), 1e-6);
    p = pole(cl);
    rows(k).gain = D; %#ok<AGROW>
    rows(k).stable = isstable(cl); %#ok<AGROW>
    rows(k).max_real_pole = max(real(p)); %#ok<AGROW>
    rows(k).min_damping = min_damping_ratio(p); %#ok<AGROW>
    rows(k).dcgain = safe_dcgain(cl); %#ok<AGROW>
end
end

function zeta = min_damping_ratio(p)
finite_poles = p(isfinite(p) & abs(p) > 1e-8);
if isempty(finite_poles)
    zeta = NaN;
    return;
end
zeta_values = -real(finite_poles)./abs(finite_poles);
zeta = min(zeta_values);
end

function g = safe_dcgain(sys)
try
    g = dcgain(sys);
catch
    g = NaN;
end
end

function plot_axis_root_locus(G, axis, cfg, filename, title_text)
fig = figure('Name', title_text, 'Visible', 'off', 'Color', 'w');
tiledlayout(1, 2);

nexttile;
rlocus(G);
hold on;
plot(real(pole(axis.inner_loop)), imag(pole(axis.inner_loop)), ...
    'rx', 'MarkerSize', 8, 'LineWidth', 1.6);
title([title_text ' con D positivo']);
grid on;

nexttile;
rlocus(-G);
hold on;
plot(real(pole(axis.inner_loop)), imag(pole(axis.inner_loop)), ...
    'rx', 'MarkerSize', 8, 'LineWidth', 1.6);
title([title_text ' con D negativo']);
grid on;

sgtitle(sprintf('%s seleccionado = %.4g', axis.gain_name, ...
    axis.selected_gain));
exportgraphics(fig, fullfile(cfg.figures_dir, filename), ...
    'BackgroundColor', 'white', 'Resolution', 200);
close(fig);
end

function ensure_figures_dir(cfg)
if ~exist(cfg.figures_dir, 'dir')
    mkdir(cfg.figures_dir);
end
end
