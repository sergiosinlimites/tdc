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
plot_axis_sens('theta', sens.theta, sens.weights, cfg);
plot_axis_sens('phi', sens.phi, sens.weights, cfg);

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
plot_sigma_response(tf(1/weights.W2), 'k:', '1/W2', w);
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
selected = {'theta_10', 'phi_10', 'theta_phi_10', 'theta_40', ...
    'phi_40', 'noise_disturbance'};

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
legend('ref', 'PID', 'Hinf', 'Location', 'best');

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
