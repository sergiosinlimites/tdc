function crear_graficas_taller1(channels, sens, sim_results, cfg)
%CREAR_GRAFICAS_TALLER1 Exporta figuras principales del taller.
%
% Este archivo concentra la salida visual del taller. Todas las figuras se
% generan con fondo blanco, colores contrastados, rejilla y referencias en
% negro punteado para que no se confundan con el fondo ni con otras senales.

% Paso 1: crear la carpeta de figuras si aun no existe.
if ~exist(cfg.figures_dir, 'dir')
    mkdir(cfg.figures_dir);
end

% Paso 2: exportar graficas frecuenciales de planta y sensibilidad.
plot_sigma_plants(channels, cfg);
plot_sensitivities(sens, cfg);

% Paso 3: exportar comparaciones temporales PID vs H_inf.
plot_time_comparisons(sim_results, cfg);
plot_final_comparisons(sim_results, cfg);
end

function plot_sigma_plants(channels, cfg)
% Grafica los valores singulares de las plantas usadas para diseno.
w = logspace(-2, 3, 600);
fig = make_white_figure('Valores singulares de plantas');
tiledlayout(1, 2);
nexttile;
plot_sigma_response(channels.theta, 'b-', 'theta/elevator', w);
hold on;
plot_sigma_response(channels.phi, 'r--', 'phi/aileron', w);
style_axes();
legend('theta/elevator', 'phi/aileron', 'Location', 'best');
title('Canales SISO');
nexttile;
plot_sigma_response(channels.angle_mimo, 'k-', 'MIMO theta-phi', w);
style_axes();
title('MIMO theta-phi');
export_png(fig, fullfile(cfg.figures_dir, 'planta_sigma.png'));
close(fig);
end

function plot_sensitivities(sens, cfg)
% Crea figuras separadas por eje y una figura resumida comparativa.
plot_axis_sens('theta', sens.theta, weights_for_axis(sens.weights, ...
    'theta'), cfg);
plot_axis_sens('phi', sens.phi, weights_for_axis(sens.weights, 'phi'), cfg);

fig = make_white_figure('Comparacion S T KS PID Hinf');
tiledlayout(3, 2);
plot_compare_tile(sens.theta.pid.S, sens.theta.hinf.S, 'S theta');
plot_compare_tile(sens.phi.pid.S, sens.phi.hinf.S, 'S phi');
plot_compare_tile(sens.theta.pid.T, sens.theta.hinf.T, 'T theta');
plot_compare_tile(sens.phi.pid.T, sens.phi.hinf.T, 'T phi');
plot_compare_tile(sens.theta.pid.KS, sens.theta.hinf.KS, 'KS theta');
plot_compare_tile(sens.phi.pid.KS, sens.phi.hinf.KS, 'KS phi');
export_png(fig, fullfile(cfg.figures_dir, 'comparacion_sensibilidades.png'));
close(fig);
end

function axis_weights = weights_for_axis(weights, axis_name)
%WEIGHTS_FOR_AXIS Mantiene compatibilidad con resultados sin pesos por eje.
if isfield(weights, axis_name)
    axis_weights = weights.(axis_name);
else
    axis_weights = weights;
end
end

function plot_axis_sens(axis_name, axis_sens, weights, cfg)
% Compara PID, H_inf y la cota inversa del peso para un eje.
w = logspace(-2, 3, 600);
fig = make_white_figure(['Sensibilidades ' axis_name]);
tiledlayout(3, 1);
nexttile;
plot_sigma_response(axis_sens.pid.S, 'b-', 'PID', w);
hold on;
plot_sigma_response(axis_sens.hinf.S, 'r--', 'Hinf', w);
plot_sigma_response(1/weights.W1, 'k:', '1/W1', w);
style_axes();
legend('PID', 'Hinf', '1/W1', 'Location', 'best');
title(['S - ' axis_name]);
nexttile;
plot_sigma_response(axis_sens.pid.T, 'b-', 'PID', w);
hold on;
plot_sigma_response(axis_sens.hinf.T, 'r--', 'Hinf', w);
plot_sigma_response(1/weights.W3, 'k:', '1/W3', w);
style_axes();
legend('PID', 'Hinf', '1/W3', 'Location', 'best');
title(['T - ' axis_name]);
nexttile;
plot_sigma_response(axis_sens.pid.KS, 'b-', 'PID', w);
hold on;
plot_sigma_response(axis_sens.hinf.KS, 'r--', 'Hinf', w);
plot_sigma_response(inv_weight(weights.W2), 'k:', '1/W2', w);
style_axes();
legend('PID', 'Hinf', '1/W2', 'Location', 'best');
title(['K*S - ' axis_name]);
export_png(fig, fullfile(cfg.figures_dir, ...
    ['sensibilidades_' axis_name '.png']));
close(fig);
end

function plot_compare_tile(pid_sys, hinf_sys, title_text)
% Tile auxiliar para comparar dos respuestas frecuenciales.
w = logspace(-2, 3, 600);
nexttile;
plot_sigma_response(pid_sys, 'b-', 'PID', w);
hold on;
plot_sigma_response(hinf_sys, 'r--', 'Hinf', w);
style_axes();
legend('PID', 'Hinf', 'Location', 'best');
title(title_text);
end

function plot_time_comparisons(sim_results, cfg)
% Selecciona los escenarios mas informativos para guardar como figuras.
names = {sim_results.pid.name};
selected = {'theta_10', 'phi_10', 'theta_phi_10', 'theta_30', ...
    'phi_30', 'theta_phi_30', 'theta_40', 'phi_40', ...
    'noise_disturbance'};

for i = 1:numel(selected)
    idx = find(strcmp(names, selected{i}), 1);
    if isempty(idx)
        continue;
    end
    plot_case_pair(sim_results.pid(idx), sim_results.hinf(idx), cfg);
end
end

function plot_case_pair(pid_sim, hinf_sim, cfg)
% Grafica referencia, respuesta y accion de control para un escenario.
fig = make_white_figure(['Simulacion ' pid_sim.name]);
tiledlayout(4, 1);

nexttile;
plot(pid_sim.t, pid_sim.theta_ref_deg, 'k:', 'LineWidth', 1.6); hold on;
plot(pid_sim.t, pid_sim.theta_deg, 'b-', 'LineWidth', 1.3);
plot(hinf_sim.t, hinf_sim.theta_deg, 'r--', 'LineWidth', 1.3);
style_axes();
ylabel('theta [deg]');
legend('ref', 'SAS/CAS', 'Hinf', 'Location', 'best');

nexttile;
plot(pid_sim.t, pid_sim.phi_ref_deg, 'k:', 'LineWidth', 1.6); hold on;
plot(pid_sim.t, pid_sim.phi_deg, 'b-', 'LineWidth', 1.3);
plot(hinf_sim.t, hinf_sim.phi_deg, 'r--', 'LineWidth', 1.3);
style_axes();
ylabel('phi [deg]');

nexttile;
plot(pid_sim.t, pid_sim.elevator_deg, 'b-', 'LineWidth', 1.3); hold on;
plot(hinf_sim.t, hinf_sim.elevator_deg, 'r--', 'LineWidth', 1.3);
yline(cfg.spec.control_limit_deg, 'Color', [0.25 0.25 0.25], ...
    'LineStyle', ':', 'LineWidth', 1.2);
yline(-cfg.spec.control_limit_deg, 'Color', [0.25 0.25 0.25], ...
    'LineStyle', ':', 'LineWidth', 1.2);
style_axes();
ylabel('elevator [deg]');

nexttile;
plot(pid_sim.t, pid_sim.aileron_deg, 'b-', 'LineWidth', 1.3); hold on;
plot(hinf_sim.t, hinf_sim.aileron_deg, 'r--', 'LineWidth', 1.3);
yline(cfg.spec.control_limit_deg, 'Color', [0.25 0.25 0.25], ...
    'LineStyle', ':', 'LineWidth', 1.2);
yline(-cfg.spec.control_limit_deg, 'Color', [0.25 0.25 0.25], ...
    'LineStyle', ':', 'LineWidth', 1.2);
style_axes();
ylabel('aileron [deg]');
xlabel('t [s]');

export_png(fig, fullfile(cfg.figures_dir, ...
    ['sim_' pid_sim.name '.png']));
close(fig);
end

function fig = make_white_figure(name)
% Crea una figura invisible con fondo blanco explicito.
fig = figure('Name', name, 'Visible', 'off', 'Color', 'w');
end

function plot_sigma_response(sys, line_spec, display_name, w)
% Calcula sigma numericamente y lo dibuja en dB con estilo controlado.
% Asi evitamos depender del objeto grafico SigmaPlot de versiones recientes.
[sv, wout] = sigma(sys, w);
sv = squeeze(sv);

if isvector(sv)
    mag = sv(:).';
else
    mag = max(sv, [], 1);
end

semilogx(wout, 20*log10(max(mag, eps)), line_spec, ...
    'LineWidth', 1.4, 'DisplayName', display_name);
ylabel('Magnitud [dB]');
xlabel('Frecuencia [rad/s]');
end

function style_axes()
% Aplica estilo comun: fondo blanco, rejilla legible y caja de ejes.
axes_list = findall(gcf, 'Type', 'axes');

for k = 1:numel(axes_list)
    ax = axes_list(k);
    ax.Color = 'w';
    ax.XColor = [0 0 0];
    ax.YColor = [0 0 0];
    ax.GridColor = [0.65 0.65 0.65];
    ax.MinorGridColor = [0.80 0.80 0.80];
    ax.GridAlpha = 0.35;
    ax.MinorGridAlpha = 0.18;
    grid(ax, 'on');
    box(ax, 'on');
end
end

function export_png(fig, filename)
% Exporta siempre con fondo blanco para evitar depender del tema de MATLAB.
exportgraphics(fig, filename, 'BackgroundColor', 'white', ...
    'Resolution', 200);
end

function W_inv = inv_weight(W)
if isa(W, 'DynamicSystem')
    W_inv = 1/W;
else
    W_inv = tf(1/W);
end
end

function plot_final_comparisons(sim_results, cfg)
% Figuras resumidas para defender conclusiones sin revisar cada png.
plot_rms_summary(sim_results, cfg);
plot_saturation_summary(sim_results, cfg);
plot_gamma_summary(cfg);
end

function plot_rms_summary(sim_results, cfg)
selected = {'theta_10', 'phi_10', 'theta_phi_10', 'theta_30', ...
    'phi_30', 'theta_phi_30', 'noise_disturbance'};
[labels, pid_theta, pid_phi, hinf_theta, hinf_phi] = ...
    collect_rms(sim_results, selected);

fig = make_white_figure('Comparacion temporal final');
tiledlayout(2, 1);
nexttile;
bar(categorical(labels), [pid_theta(:), hinf_theta(:)]);
ylabel('RMS theta [deg]');
legend('SAS/CAS', 'Hinf', 'Location', 'best');
style_axes();
nexttile;
bar(categorical(labels), [pid_phi(:), hinf_phi(:)]);
ylabel('RMS phi [deg]');
legend('SAS/CAS', 'Hinf', 'Location', 'best');
style_axes();
export_png(fig, fullfile(cfg.figures_dir, 'comparacion_temporal_final.png'));
close(fig);
end

function plot_saturation_summary(sim_results, cfg)
selected = {'theta_10', 'phi_10', 'theta_phi_10', 'theta_30', ...
    'phi_30', 'theta_phi_30', 'theta_40', 'phi_40', ...
    'noise_disturbance'};
labels = selected;
pid_sat = zeros(size(selected));
hinf_sat = zeros(size(selected));
for k = 1:numel(selected)
    idx_pid = find(strcmp({sim_results.pid.name}, selected{k}), 1);
    idx_hinf = find(strcmp({sim_results.hinf.name}, selected{k}), 1);
    pid_sat(k) = 100*sim_results.pid(idx_pid).metrics.sat_fraction;
    hinf_sat(k) = 100*sim_results.hinf(idx_hinf).metrics.sat_fraction;
end

fig = make_white_figure('Comparacion saturacion final');
bar(categorical(labels), [pid_sat(:), hinf_sat(:)]);
ylabel('Tiempo saturado [%]');
legend('SAS/CAS', 'Hinf', 'Location', 'best');
style_axes();
export_png(fig, fullfile(cfg.figures_dir, 'comparacion_saturacion_final.png'));
close(fig);
end

function plot_gamma_summary(cfg)
current_file = fullfile(cfg.results_dir, 'taller1_results.mat');
baseline_file = fullfile(cfg.results_dir, ...
    'taller1_results_baseline_actual.mat');
if ~isfile(current_file)
    return;
end

current = load(current_file, 'hinf_data');
labels = {'actual theta', 'actual phi'};
values = [current.hinf_data.theta.gamma, current.hinf_data.phi.gamma];
if isfile(baseline_file)
    baseline = load(baseline_file, 'hinf_data');
    labels = {'baseline theta', 'baseline phi', 'actual theta', ...
        'actual phi'};
    values = [baseline.hinf_data.theta.gamma, ...
        baseline.hinf_data.phi.gamma, values];
end

fig = make_white_figure('Comparacion gamma Hinf');
bar(categorical(labels), values);
ylabel('gamma mixsyn');
style_axes();
export_png(fig, fullfile(cfg.figures_dir, 'comparacion_gamma_hinf.png'));
close(fig);
end

function [labels, pid_theta, pid_phi, hinf_theta, hinf_phi] = ...
    collect_rms(sim_results, selected)
labels = selected;
pid_theta = zeros(size(selected));
pid_phi = zeros(size(selected));
hinf_theta = zeros(size(selected));
hinf_phi = zeros(size(selected));
for k = 1:numel(selected)
    idx_pid = find(strcmp({sim_results.pid.name}, selected{k}), 1);
    idx_hinf = find(strcmp({sim_results.hinf.name}, selected{k}), 1);
    pid_theta(k) = sim_results.pid(idx_pid).metrics.theta_rms_error_deg;
    pid_phi(k) = sim_results.pid(idx_pid).metrics.phi_rms_error_deg;
    hinf_theta(k) = sim_results.hinf(idx_hinf).metrics.theta_rms_error_deg;
    hinf_phi(k) = sim_results.hinf(idx_hinf).metrics.phi_rms_error_deg;
end
end
