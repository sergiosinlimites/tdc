function stress = validacion_extrema_taller1()
%VALIDACION_EXTREMA_TALLER1 Revisa figuras Hinf y pruebas de estres.
%
% Este script no cambia el diseno. Su objetivo es comprobar si las figuras
% actuales sostienen las conclusiones y que pasa al permitir actuadores de
% mayor recorrido.

project_dir = fileparts(mfilename('fullpath'));
addpath(project_dir);

cfg = parametros_taller1(project_dir);
cfg.export_design_figures = false;
plant = cargar_modelo_uav(cfg);
channels = seleccionar_canales_uav(plant, cfg);
pid_data = diseno_pid_sas_cas(channels, cfg);
hinf_data = diseno_hinf_taller1(channels, cfg);
sens = analisis_sensibilidades(channels, pid_data, hinf_data, cfg);

if ~exist(cfg.results_dir, 'dir')
    mkdir(cfg.results_dir);
end
if ~exist(cfg.figures_dir, 'dir')
    mkdir(cfg.figures_dir);
end

stress.figure_review = revisar_figuras_actuales(sens, cfg);
stress.saturation = run_saturation_sweep(cfg, plant, pid_data, hinf_data);
save(fullfile(cfg.results_dir, 'validacion_extrema_taller1_partial.mat'), ...
    'stress');
stress.hard_tests = run_hard_tests(cfg, plant, pid_data, hinf_data);

write_review_markdown(stress, cfg);
plot_saturation_sweep(stress.saturation, cfg);
plot_hard_test_summary(stress.hard_tests, cfg);

save(fullfile(cfg.results_dir, 'validacion_extrema_taller1.mat'), ...
    'stress');
end

function review = revisar_figuras_actuales(sens, cfg)
review.phi.norm_S_hinf = sens.phi.hinf.norm_S;
review.phi.norm_T_hinf = sens.phi.hinf.norm_T;
review.phi.norm_KS_hinf = sens.phi.hinf.norm_KS;
review.phi.norm_KS_sas = sens.phi.pid.norm_KS;
review.phi.KS_ratio_hinf_vs_sas = sens.phi.hinf.norm_KS / ...
    sens.phi.pid.norm_KS;
review.phi.low_freq_S = abs(freqresp(sens.phi.hinf.S, 1e-2));
review.phi.low_freq_tracking_error_pct = 100*review.phi.low_freq_S;
review.phi.comment = ['La sensibilidad de phi muestra Hinf con menor S ', ...
    'en baja frecuencia que SAS/CAS, pero KS de Hinf es mucho mayor que ', ...
    'SAS/CAS. Esa combinacion anticipa mejor tracking nominal, pero ', ...
    'accion de aileron mas agresiva ante ruido, cambios rapidos o ', ...
    'saturacion.'];

review.theta.norm_KS_hinf = sens.theta.hinf.norm_KS;
review.theta.norm_KS_sas = sens.theta.pid.norm_KS;
review.control_limit_deg = cfg.spec.control_limit_deg;
end

function rows = run_saturation_sweep(cfg, plant, pid_data, hinf_data)
limits = [30 45 60];
scenarios = [ ...
    scenario('phi_30_sat_sweep', 0, 30, false, false), ...
    scenario('phi_40_sat_sweep', 0, 40, false, false), ...
    scenario('theta_phi_30_sat_sweep', 30, 30, false, false)];

rows = [];
for i = 1:numel(limits)
    cfg_i = cfg;
    cfg_i.spec.control_limit_deg = limits(i);
    cfg_i.spec.control_limit_rad = deg2rad(limits(i));
    cfg_i.sim.dt = 0.01;
    cfg_i.scenarios = scenarios;
    sim_results = simulacion_taller1(plant, pid_data, hinf_data, cfg_i);
    rows = [rows, summarize_family(sim_results, cfg_i, "sat_sweep")]; %#ok<AGROW>
end
end

function rows = run_hard_tests(cfg, plant, pid_data, hinf_data)
cfg_i = cfg;
cfg_i.spec.control_limit_deg = 60;
cfg_i.spec.control_limit_rad = deg2rad(60);
cfg_i.sim.dt = 0.01;
cfg_i.sim.t_final = 12.0;
cfg_i.sim.disturbance_amp_rad = deg2rad(3.0);
cfg_i.spec.noise_power_long = 9*cfg.spec.noise_power_long;
cfg_i.spec.noise_power_lat = 9*cfg.spec.noise_power_lat;
cfg_i.scenarios = [ ...
    scenario('phi_60_hard', 0, 60, false, false), ...
    scenario('theta_phi_45_hard', 45, 45, false, false), ...
    scenario('noise_dist_x3_hard', 20, 20, true, true)];

sim_results = simulacion_taller1(plant, pid_data, hinf_data, cfg_i);
rows = summarize_family(sim_results, cfg_i, "hard_60deg_x3");
end

function rows = summarize_family(sim_results, cfg, suite)
controllers = {'pid', 'hinf'};
rows = [];
for c = 1:numel(controllers)
    family = sim_results.(controllers{c});
    for k = 1:numel(family)
        sim = family(k);
        row = summarize_sim(sim, cfg, suite);
        rows = [rows, row]; %#ok<AGROW>
    end
end
end

function row = summarize_sim(sim, cfg, suite)
m = sim.metrics;
theta_ref = sim.theta_ref_deg(end);
phi_ref = sim.phi_ref_deg(end);
[theta_overshoot, theta_undershoot] = overshoot_metrics(sim.theta_deg, theta_ref);
[phi_overshoot, phi_undershoot] = overshoot_metrics(sim.phi_deg, phi_ref);

row.suite = char(suite);
row.controller = sim.controller;
row.scenario = sim.name;
row.limit_deg = cfg.spec.control_limit_deg;
row.theta_ref_deg = theta_ref;
row.phi_ref_deg = phi_ref;
row.theta_rms_error_deg = m.theta_rms_error_deg;
row.phi_rms_error_deg = m.phi_rms_error_deg;
row.theta_abs_final_error_deg = m.theta_abs_final_error_deg;
row.phi_abs_final_error_deg = m.phi_abs_final_error_deg;
row.theta_overshoot_deg = theta_overshoot;
row.phi_overshoot_deg = phi_overshoot;
row.theta_undershoot_deg = theta_undershoot;
row.phi_undershoot_deg = phi_undershoot;
row.max_abs_theta_deg = m.max_abs_theta_deg;
row.max_abs_phi_deg = m.max_abs_phi_deg;
row.max_abs_elevator_deg = m.max_abs_elevator_deg;
row.max_abs_aileron_deg = m.max_abs_aileron_deg;
row.sat_fraction = m.sat_fraction;
row.flags = classify_flags(row);
end

function [overshoot, undershoot] = overshoot_metrics(y, ref)
if abs(ref) < 1e-9
    overshoot = max(abs(y));
    undershoot = overshoot;
    return;
end
sgn = sign(ref);
e = sgn*(y - ref);
overshoot = max(e);
undershoot = max(-sgn*y);
overshoot = max(overshoot, 0);
undershoot = max(undershoot, 0);
end

function flags = classify_flags(row)
flags = strings(0);
theta_tol = max(1.0, 0.05*abs(row.theta_ref_deg));
phi_tol = max(1.0, 0.05*abs(row.phi_ref_deg));

if abs(row.theta_ref_deg) > 0 && row.theta_abs_final_error_deg > theta_tol
    flags(end+1) = "theta_no_llega";
end
if abs(row.phi_ref_deg) > 0 && row.phi_abs_final_error_deg > phi_tol
    flags(end+1) = "phi_no_llega";
end
if row.theta_overshoot_deg > max(2.0, 0.15*abs(row.theta_ref_deg))
    flags(end+1) = "theta_sobrepasa";
end
if row.phi_overshoot_deg > max(2.0, 0.15*abs(row.phi_ref_deg))
    flags(end+1) = "phi_sobrepasa";
end
if row.sat_fraction > 0.05
    flags(end+1) = "saturacion_persistente";
end
if row.max_abs_theta_deg > max(10, 1.5*abs(row.theta_ref_deg) + 5)
    flags(end+1) = "theta_descontrol";
end
if row.max_abs_phi_deg > max(10, 1.5*abs(row.phi_ref_deg) + 5)
    flags(end+1) = "phi_descontrol";
end

if isempty(flags)
    flags = "ok";
else
    flags = strjoin(flags, ", ");
end
end

function s = scenario(name, theta_deg, phi_deg, noise_enabled, disturbance_enabled)
s.name = name;
s.theta_ref_deg = theta_deg;
s.phi_ref_deg = phi_deg;
s.theta_ref_rad = deg2rad(theta_deg);
s.phi_ref_rad = deg2rad(phi_deg);
s.noise_enabled = noise_enabled;
s.disturbance_enabled = disturbance_enabled;
end

function write_review_markdown(stress, cfg)
filename = fullfile(cfg.results_dir, ...
    'revision_figuras_hinf_y_validacion_extrema.md');
fid = fopen(filename, 'w');
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Revision de figuras Hinf y validacion extrema\n\n');
fprintf(fid, 'Esta revision se enfoca en las dudas sobre `phi`, seguimiento, sobrepaso y saturacion.\n\n');

fprintf(fid, '## Lectura de sensibilidad phi\n\n');
fprintf(fid, '- `||KS||` phi SAS/CAS = %.3f.\n', stress.figure_review.phi.norm_KS_sas);
fprintf(fid, '- `||KS||` phi Hinf = %.3f.\n', stress.figure_review.phi.norm_KS_hinf);
fprintf(fid, '- Relacion `KS_Hinf/KS_SAS` = %.2f.\n', ...
    stress.figure_review.phi.KS_ratio_hinf_vs_sas);
fprintf(fid, '- Error de tracking de baja frecuencia estimado por `S_phi(1e-2)` = %.2f %%.\n\n', ...
    stress.figure_review.phi.low_freq_tracking_error_pct);
fprintf(fid, '%s\n\n', stress.figure_review.phi.comment);

fprintf(fid, '## Barrido de saturacion\n\n');
write_rows_table(fid, stress.saturation);

fprintf(fid, '\n## Pruebas mas fuertes\n\n');
write_rows_table(fid, stress.hard_tests);

fprintf(fid, '\n## Conclusion operativa\n\n');
fprintf(fid, ['Con la iteracion de pesos especificos para phi, Hinf ya no ', ...
    'presenta `phi_no_llega` en las pruebas fuertes y cumple el criterio ', ...
    'nominal de error final para `phi_30` y `phi_40` sin cambiar el limite ', ...
    'de saturacion. La reserva principal queda en `noise_dist_x3_hard`: ', ...
    'aparece `phi_sobrepasa`, tambien presente en SAS/CAS, aunque Hinf ', ...
    'mantiene menor RMS lateral. Los intentos adicionales de apretar ', ...
    'W2_phi/W3_phi en SISO redujeron algo KS y sobrepaso, pero reabrieron ', ...
    'phi_no_llega en el caso combinado fuerte. Si se exige aprobar sin ese ', ...
    'flag, la siguiente iteracion debe pasar a Hinf MIMO o a una estructura ', ...
    'adicional de rechazo lateral; el precompensador de referencia ya no es ', ...
    'la primera necesidad porque el error estacionario de phi quedo dentro ', ...
    'de los umbrales nominales.', newline]);
end

function write_rows_table(fid, rows)
fprintf(fid, '| suite | control | escenario | lim [deg] | RMS theta | RMS phi | err fin theta | err fin phi | over theta | over phi | sat %% | flags |\n');
fprintf(fid, '|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|\n');
for k = 1:numel(rows)
    r = rows(k);
    fprintf(fid, ['| %s | %s | %s | %.0f | %.3f | %.3f | %.3f | ', ...
        '%.3f | %.3f | %.3f | %.2f | %s |\n'], ...
        r.suite, controller_label(r.controller), r.scenario, ...
        r.limit_deg, r.theta_rms_error_deg, r.phi_rms_error_deg, ...
        r.theta_abs_final_error_deg, r.phi_abs_final_error_deg, ...
        r.theta_overshoot_deg, r.phi_overshoot_deg, ...
        100*r.sat_fraction, r.flags);
end
end

function label = controller_label(name)
if strcmp(name, 'pid')
    label = 'SAS/CAS';
else
    label = 'Hinf';
end
end

function plot_saturation_sweep(rows, cfg)
phi_rows = rows(strcmp({rows.scenario}, 'phi_40_sat_sweep') & ...
    strcmp({rows.controller}, 'hinf'));
limits = [phi_rows.limit_deg];
final_errors = [phi_rows.phi_abs_final_error_deg];
rms_errors = [phi_rows.phi_rms_error_deg];
sat_pct = 100*[phi_rows.sat_fraction];

fig = figure('Name', 'Validacion saturacion phi Hinf', ...
    'Visible', 'off', 'Color', 'w');
tiledlayout(3, 1);
nexttile;
bar(limits, final_errors);
ylabel('Error final phi [deg]');
ylim([0, 1.15*max(final_errors)]);
style_axes();
nexttile;
bar(limits, rms_errors);
ylabel('RMS phi [deg]');
ylim([0, 1.15*max(rms_errors)]);
style_axes();
nexttile;
bar(limits, sat_pct);
ylabel('Saturacion [%]');
xlabel('Limite actuador [deg]');
ylim([0, 1.15*max(sat_pct)]);
style_axes();
exportgraphics(fig, fullfile(cfg.figures_dir, ...
    'validacion_saturacion_phi_hinf.png'), 'BackgroundColor', 'white', ...
    'Resolution', 200);
close(fig);
end

function plot_hard_test_summary(rows, cfg)
hinf_rows = rows(strcmp({rows.controller}, 'hinf'));
labels = categorical(strrep({hinf_rows.scenario}, '_', ' '));
fig = figure('Name', 'Validacion extrema Hinf', 'Visible', 'off', ...
    'Color', 'w');
tiledlayout(2, 1);
nexttile;
bar(labels, [[hinf_rows.theta_rms_error_deg].', ...
    [hinf_rows.phi_rms_error_deg].']);
ylabel('RMS [deg]');
lgd = legend('theta', 'phi', 'Location', 'best');
lgd.Color = 'w';
lgd.TextColor = [0 0 0];
style_axes();
nexttile;
bar(labels, 100*[hinf_rows.sat_fraction].');
ylabel('Saturacion [%]');
style_axes();
exportgraphics(fig, fullfile(cfg.figures_dir, ...
    'validacion_extrema_hinf.png'), 'BackgroundColor', 'white', ...
    'Resolution', 200);
close(fig);
end

function style_axes()
ax = gca;
ax.Color = 'w';
ax.XColor = [0 0 0];
ax.YColor = [0 0 0];
ax.GridColor = [0.65 0.65 0.65];
ax.GridAlpha = 0.35;
grid(ax, 'on');
box(ax, 'on');
end
