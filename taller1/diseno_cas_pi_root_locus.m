function cas_data = diseno_cas_pi_root_locus(channels, sas_data, cfg, export_figures)
%DISENO_CAS_PI_ROOT_LOCUS Disena el CAS externo como PI sobre planta con SAS.

if nargin < 4
    export_figures = true;
end

ensure_figures_dir(cfg);
s = tf('s');

cas_data.theta.plant_sas = minreal(channels.theta/(1 + ...
    sas_data.D_q*channels.q), cfg.minreal_tol);
cas_data.phi.plant_sas = minreal(channels.phi/(1 + ...
    sas_data.D_p*channels.p), cfg.minreal_tol);

cas_data.theta.Kp = cfg.pid.kp_theta;
cas_data.theta.Ki = cfg.pid.ki_theta;
cas_data.phi.Kp = cfg.pid.kp_phi;
cas_data.phi.Ki = cfg.pid.ki_phi;

cas_data.theta.PI = minreal(cas_data.theta.Kp + cas_data.theta.Ki/s, ...
    cfg.minreal_tol);
cas_data.phi.PI = minreal(cas_data.phi.Kp + cas_data.phi.Ki/s, ...
    cfg.minreal_tol);

cas_data.theta.loop = minreal(cas_data.theta.PI*cas_data.theta.plant_sas, ...
    cfg.minreal_tol);
cas_data.phi.loop = minreal(cas_data.phi.PI*cas_data.phi.plant_sas, ...
    cfg.minreal_tol);
cas_data.theta.closed_loop = feedback(cas_data.theta.loop, 1);
cas_data.phi.closed_loop = feedback(cas_data.phi.loop, 1);

cas_data.theta.metrics = loop_summary(cas_data.theta.loop, ...
    cas_data.theta.closed_loop);
cas_data.phi.metrics = loop_summary(cas_data.phi.loop, ...
    cas_data.phi.closed_loop);

cas_data.notes = ['El CAS se sintoniza como PI sobre la planta ya ', ...
    'amortiguada por SAS. Las ganancias finales se validan luego en ', ...
    'linmodel acoplado con saturacion y anti-windup.'];

if export_figures
    plot_cas_root_locus(cas_data.theta.loop, cas_data.theta.closed_loop, ...
        cfg, 'root_locus_cas_theta.png', 'CAS PI theta');
    plot_cas_root_locus(cas_data.phi.loop, cas_data.phi.closed_loop, ...
        cfg, 'root_locus_cas_phi.png', 'CAS PI phi');
end
end

function metrics = loop_summary(open_loop, closed_loop)
metrics.stable = isstable(closed_loop);
metrics.bandwidth = bandwidth(closed_loop);
try
    [gm, pm, wcg, wcp] = margin(open_loop);
catch
    gm = NaN; pm = NaN; wcg = NaN; wcp = NaN;
end
metrics.gain_margin = gm;
metrics.phase_margin_deg = pm;
metrics.gain_cross_frequency = wcg;
metrics.phase_cross_frequency = wcp;
metrics.max_real_pole = max(real(pole(closed_loop)));
end

function plot_cas_root_locus(open_loop, closed_loop, cfg, filename, title_text)
fig = figure('Name', title_text, 'Visible', 'off', 'Color', 'w');
tiledlayout(1, 2);

nexttile;
rlocus(open_loop);
hold on;
plot(real(pole(closed_loop)), imag(pole(closed_loop)), ...
    'rx', 'MarkerSize', 8, 'LineWidth', 1.6);
title([title_text ' root locus']);
grid on;

nexttile;
step(closed_loop, cfg.sim.t_final);
title([title_text ' escalon unitario']);
grid on;

exportgraphics(fig, fullfile(cfg.figures_dir, filename), ...
    'BackgroundColor', 'white', 'Resolution', 200);
close(fig);
end

function ensure_figures_dir(cfg)
if ~exist(cfg.figures_dir, 'dir')
    mkdir(cfg.figures_dir);
end
end
