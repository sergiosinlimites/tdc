function sweep = barrido_suave_hinf_siso_phi(max_candidates, grid_mode)
%BARRIDO_SUAVE_HINF_SISO_PHI Ultima ronda SISO suave para el eje phi.
%
% Evalua el baseline Hinf SISO actual y una grilla acotada de pesos
% moderados antes de pasar a una arquitectura MIMO o rechazo lateral.

if nargin < 1 || isempty(max_candidates)
    max_candidates = Inf;
end
if nargin < 2 || isempty(grid_mode)
    grid_mode = "mini";
end

project_dir = fileparts(mfilename('fullpath'));
addpath(project_dir);

cfg = parametros_taller1(project_dir);
cfg.export_design_figures = false;
plant = cargar_modelo_uav(cfg);
channels = seleccionar_canales_uav(plant, cfg);
pid_data = diseno_pid_sas_cas(channels, cfg);

candidates = candidate_grid(cfg, grid_mode);
if isfinite(max_candidates)
    candidates = candidates(1:min(max_candidates, numel(candidates)));
end
rows = repmat(empty_row(), numel(candidates), 1);

fprintf('Barrido suave Hinf SISO phi: %d candidatos\n', numel(candidates));
for k = 1:numel(candidates)
    cfg_k = apply_candidate(cfg, candidates(k));
    try
        hinf_data = diseno_hinf_taller1(channels, cfg_k);
        sens = analisis_sensibilidades(channels, pid_data, hinf_data, cfg_k);
        nominal = run_nominal_phi(cfg_k, plant, pid_data, hinf_data);
        hard = run_hard_phi(cfg_k, plant, pid_data, hinf_data);
        rows(k) = summarize_candidate(k, candidates(k), hinf_data, sens, ...
            nominal, hard);
        fprintf(['%3d/%3d %-16s KS=%.3f err30=%.3f err40=%.3f ', ...
            'combo=%.3f over_noise=%.3f flags=%s\n'], ...
            k, numel(candidates), rows(k).tag, rows(k).KS_phi, ...
            rows(k).phi30_final_error, rows(k).phi40_final_error, ...
            rows(k).theta_phi_45_final_error, rows(k).noise_phi_overshoot, ...
            rows(k).noise_flags);
    catch ME
        rows(k) = empty_row();
        rows(k).candidate = k;
        rows(k).tag = candidates(k).tag;
        rows(k).is_baseline = candidates(k).is_baseline;
        rows(k).notes = string(ME.message);
        fprintf('%3d/%3d %-16s ERROR: %s\n', k, numel(candidates), ...
            rows(k).tag, ME.message);
    end
end

rows = classify_acceptance(rows);
sweep.rows = rows;
sweep.baseline = rows(1);
sweep.accepted = rows([rows.accepted]);
sweep.best = select_best(rows);
sweep.selection_rule = ['Aceptar solo si: phi_30 < 1.5 deg, ', ...
    'phi_40 < 2.0 deg, theta_phi_45_hard sin phi_no_llega, ', ...
    'KS_phi <= baseline/3.080 y noise_dist_x3_hard con menor ', ...
    'sobrepaso que el baseline.'];

if ~exist(cfg.results_dir, 'dir')
    mkdir(cfg.results_dir);
end
save(fullfile(cfg.results_dir, 'hinf_siso_suave_phi_sweep.mat'), 'sweep');
write_markdown(sweep, fullfile(cfg.results_dir, ...
    'hinf_siso_suave_phi_sweep.md'));
fprintf('\nResumen guardado en %s\n', fullfile(cfg.results_dir, ...
    'hinf_siso_suave_phi_sweep.md'));
end

function candidates = candidate_grid(cfg, grid_mode)
wp = cfg.spec.wp;
baseline = struct('tag', "baseline", 'is_baseline', true, ...
    'W1_phi_low_gain', cfg.hinf.phi.W1_low_gain, ...
    'W2_phi_low_gain', cfg.hinf.phi.W2_low_gain, ...
    'W2_phi_high_gain', cfg.hinf.phi.W2_high_gain, ...
    'W2_phi_cross_frequency', cfg.hinf.phi.W2_cross_frequency, ...
    'W3_phi_high_gain', cfg.hinf.phi.W3_high_gain, ...
    'W3_phi_cross_frequency', cfg.hinf.phi.W3_cross_frequency);

if strcmpi(char(grid_mode), 'full')
    candidates = full_factorial_grid(cfg, baseline);
    return;
end

% Modo normal: mini-ronda de bajo riesgo. Incluye variaciones una-a-una
% alrededor del baseline y unas mezclas suaves que usan todos los valores
% propuestos sin abrir una busqueda factorial larga.
values = [ ...
    180 0.80 3.20 wp 15; ...
    200 0.80 3.20 wp 15; ...
    220 0.70 3.20 wp 15; ...
    220 0.90 3.20 wp 15; ...
    220 0.80 2.40 wp 15; ...
    220 0.80 2.80 wp 15; ...
    220 0.80 3.20 45 15; ...
    220 0.80 3.20 wp 12; ...
    220 0.80 3.20 wp 18; ...
    220 0.80 3.20 wp 20; ...
    180 0.70 2.40 wp 12; ...
    200 0.80 2.80 wp 15; ...
    200 0.80 2.80 45 15; ...
    220 0.90 3.20 wp 18; ...
    220 0.90 3.20 45 20; ...
    220 0.80 2.80 45 20; ...
    180 0.90 3.20 45 20; ...
    200 0.90 3.20 45 20];

candidates = baseline;
for k = 1:size(values, 1)
    c = candidate_from_values(baseline, values(k, :), sprintf('soft_%03d', k));
    if same_weights(c, baseline)
        continue;
    end
    candidates(end+1) = c; %#ok<AGROW>
end
end

function candidates = full_factorial_grid(cfg, baseline)
wp = cfg.spec.wp;
W1_vals = [180 200 220];
W2_low_vals = [0.70 0.80 0.90];
W2_high_vals = [2.40 2.80 3.20];
W2_cross_vals = [wp 45];
W3_high_vals = [12 15 18 20];

candidates = baseline;
idx = 1;
for i1 = 1:numel(W1_vals)
    for i2 = 1:numel(W2_low_vals)
        for i3 = 1:numel(W2_high_vals)
            for i4 = 1:numel(W2_cross_vals)
                for i5 = 1:numel(W3_high_vals)
                    c = baseline;
                    c.is_baseline = false;
                    c.W1_phi_low_gain = W1_vals(i1);
                    c.W2_phi_low_gain = W2_low_vals(i2);
                    c.W2_phi_high_gain = W2_high_vals(i3);
                    c.W2_phi_cross_frequency = W2_cross_vals(i4);
                    c.W3_phi_high_gain = W3_high_vals(i5);
                    c.W3_phi_cross_frequency = wp;
                    c.tag = sprintf('soft_%03d', idx);
                    idx = idx + 1;
                    if same_weights(c, baseline)
                        continue;
                    end
                    candidates(end+1) = c; %#ok<AGROW>
                end
            end
        end
    end
end
end

function c = candidate_from_values(baseline, values, tag)
c = baseline;
c.is_baseline = false;
c.W1_phi_low_gain = values(1);
c.W2_phi_low_gain = values(2);
c.W2_phi_high_gain = values(3);
c.W2_phi_cross_frequency = values(4);
c.W3_phi_high_gain = values(5);
c.W3_phi_cross_frequency = baseline.W3_phi_cross_frequency;
c.tag = tag;
end

function tf = same_weights(a, b)
tol = 1e-9;
tf = abs(a.W1_phi_low_gain - b.W1_phi_low_gain) < tol && ...
    abs(a.W2_phi_low_gain - b.W2_phi_low_gain) < tol && ...
    abs(a.W2_phi_high_gain - b.W2_phi_high_gain) < tol && ...
    abs(a.W2_phi_cross_frequency - b.W2_phi_cross_frequency) < tol && ...
    abs(a.W3_phi_high_gain - b.W3_phi_high_gain) < tol && ...
    abs(a.W3_phi_cross_frequency - b.W3_phi_cross_frequency) < tol;
end

function cfg = apply_candidate(cfg, c)
cfg.hinf.phi.W1_low_gain = c.W1_phi_low_gain;
cfg.hinf.phi.W2_gain = c.W2_phi_low_gain;
cfg.hinf.phi.W2_low_gain = c.W2_phi_low_gain;
cfg.hinf.phi.W2_high_gain = c.W2_phi_high_gain;
cfg.hinf.phi.W2_cross_frequency = c.W2_phi_cross_frequency;
cfg.hinf.phi.W3_high_gain = c.W3_phi_high_gain;
cfg.hinf.phi.W3_cross_frequency = c.W3_phi_cross_frequency;
end

function rows = run_nominal_phi(cfg, plant, pid_data, hinf_data)
cfg_i = cfg;
cfg_i.sim.dt = 0.01;
cfg_i.scenarios = [ ...
    scenario('phi_30', 0, 30, false, false), ...
    scenario('phi_40', 0, 40, false, false)];
sim_results = simulacion_taller1(plant, pid_data, hinf_data, cfg_i);
rows = summarize_hinf_rows(sim_results.hinf, cfg_i, "nominal");
end

function rows = run_hard_phi(cfg, plant, pid_data, hinf_data)
cfg_i = cfg;
cfg_i.spec.control_limit_deg = 60;
cfg_i.spec.control_limit_rad = deg2rad(60);
cfg_i.sim.dt = 0.01;
cfg_i.sim.t_final = 12.0;
cfg_i.sim.disturbance_amp_rad = deg2rad(3.0);
cfg_i.spec.noise_power_long = 9*cfg.spec.noise_power_long;
cfg_i.spec.noise_power_lat = 9*cfg.spec.noise_power_lat;
cfg_i.scenarios = [ ...
    scenario('theta_phi_45_hard', 45, 45, false, false), ...
    scenario('noise_dist_x3_hard', 20, 20, true, true)];
sim_results = simulacion_taller1(plant, pid_data, hinf_data, cfg_i);
rows = summarize_hinf_rows(sim_results.hinf, cfg_i, "hard_60deg_x3");
end

function rows = summarize_hinf_rows(family, cfg, suite)
rows = repmat(empty_case_row(), numel(family), 1);
for k = 1:numel(family)
    rows(k) = summarize_sim(family(k), cfg, suite);
end
end

function row = summarize_sim(sim, cfg, suite)
m = sim.metrics;
theta_ref = sim.theta_ref_deg(end);
phi_ref = sim.phi_ref_deg(end);
[theta_overshoot, theta_undershoot] = overshoot_metrics(sim.theta_deg, theta_ref);
[phi_overshoot, phi_undershoot] = overshoot_metrics(sim.phi_deg, phi_ref);

row = empty_case_row();
row.suite = char(suite);
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

if isempty(flags)
    flags = "ok";
else
    flags = strjoin(flags, ", ");
end
flags = char(flags);
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

function row = summarize_candidate(k, c, hinf_data, sens, nominal, hard)
row = empty_row();
row.candidate = k;
row.tag = c.tag;
row.is_baseline = c.is_baseline;
row.W1_phi_low_gain = c.W1_phi_low_gain;
row.W2_phi_low_gain = c.W2_phi_low_gain;
row.W2_phi_high_gain = c.W2_phi_high_gain;
row.W2_phi_cross_frequency = c.W2_phi_cross_frequency;
row.W3_phi_high_gain = c.W3_phi_high_gain;
row.W3_phi_cross_frequency = c.W3_phi_cross_frequency;
row.gamma_phi = hinf_data.phi.gamma;
row.order_phi = hinf_data.phi.order;
row.KS_phi = sens.phi.hinf.norm_KS;

phi30 = nominal(strcmp({nominal.scenario}, 'phi_30'));
phi40 = nominal(strcmp({nominal.scenario}, 'phi_40'));
combo = hard(strcmp({hard.scenario}, 'theta_phi_45_hard'));
noise = hard(strcmp({hard.scenario}, 'noise_dist_x3_hard'));

row.phi30_final_error = phi30.phi_abs_final_error_deg;
row.phi40_final_error = phi40.phi_abs_final_error_deg;
row.theta_phi_45_final_error = combo.phi_abs_final_error_deg;
row.theta_phi_45_phi_overshoot = combo.phi_overshoot_deg;
row.theta_phi_45_flags = combo.flags;
row.noise_final_error = noise.phi_abs_final_error_deg;
row.noise_phi_overshoot = noise.phi_overshoot_deg;
row.noise_flags = noise.flags;
row.sat_fraction_max = max([nominal.sat_fraction, hard.sat_fraction]);
end

function rows = classify_acceptance(rows)
baseline = rows(1);
ks_limit = min(3.080, baseline.KS_phi) + 1e-3;
noise_limit = baseline.noise_phi_overshoot - 1e-3;

for k = 1:numel(rows)
    rows(k).core_ok = rows(k).phi30_final_error < 1.5 && ...
        rows(k).phi40_final_error < 2.0 && ...
        rows(k).KS_phi <= ks_limit && ...
        ~contains(rows(k).theta_phi_45_flags, 'phi_no_llega');
    rows(k).noise_improved = rows(k).noise_phi_overshoot < noise_limit;
    rows(k).accepted = rows(k).core_ok && rows(k).noise_improved;
    rows(k).score = score_row(rows(k));
end
end

function score = score_row(r)
penalty = 0;
penalty = penalty + 1000*max(0, r.phi30_final_error - 1.5);
penalty = penalty + 1000*max(0, r.phi40_final_error - 2.0);
penalty = penalty + 1000*contains(r.theta_phi_45_flags, 'phi_no_llega');
penalty = penalty + 1000*max(0, r.KS_phi - 3.080);
score = penalty + r.noise_phi_overshoot + 0.1*r.KS_phi + ...
    0.25*r.theta_phi_45_final_error;
end

function best = select_best(rows)
accepted = rows([rows.accepted]);
if ~isempty(accepted)
    [~, idx] = min([accepted.score]);
    best = accepted(idx);
    return;
end

core_rows = rows([rows.core_ok]);
if ~isempty(core_rows)
    [~, idx] = min([core_rows.score]);
    best = core_rows(idx);
    return;
end

[~, idx] = min([rows.score]);
best = rows(idx);
end

function row = empty_row()
row = struct('candidate', NaN, 'tag', "", 'is_baseline', false, ...
    'W1_phi_low_gain', NaN, 'W2_phi_low_gain', NaN, ...
    'W2_phi_high_gain', NaN, 'W2_phi_cross_frequency', NaN, ...
    'W3_phi_high_gain', NaN, 'W3_phi_cross_frequency', NaN, ...
    'gamma_phi', NaN, 'order_phi', NaN, 'KS_phi', NaN, ...
    'phi30_final_error', NaN, 'phi40_final_error', NaN, ...
    'theta_phi_45_final_error', NaN, ...
    'theta_phi_45_phi_overshoot', NaN, 'theta_phi_45_flags', "", ...
    'noise_final_error', NaN, 'noise_phi_overshoot', NaN, ...
    'noise_flags', "", 'sat_fraction_max', NaN, 'core_ok', false, ...
    'noise_improved', false, 'accepted', false, 'score', Inf, ...
    'notes', "");
end

function row = empty_case_row()
row = struct('suite', '', 'scenario', '', 'limit_deg', NaN, ...
    'theta_ref_deg', NaN, 'phi_ref_deg', NaN, ...
    'theta_rms_error_deg', NaN, 'phi_rms_error_deg', NaN, ...
    'theta_abs_final_error_deg', NaN, 'phi_abs_final_error_deg', NaN, ...
    'theta_overshoot_deg', NaN, 'phi_overshoot_deg', NaN, ...
    'theta_undershoot_deg', NaN, 'phi_undershoot_deg', NaN, ...
    'sat_fraction', NaN, 'flags', '');
end

function write_markdown(sweep, filename)
fid = fopen(filename, 'w');
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Barrido suave Hinf SISO phi\n\n');
fprintf(fid, 'Regla: %s\n\n', sweep.selection_rule);

b = sweep.baseline;
fprintf(fid, '## Baseline\n\n');
write_row_detail(fid, b);

fprintf(fid, '\n## Decision\n\n');
if isempty(sweep.accepted)
    fprintf(fid, ['Ningun candidato suave mejoro `noise_dist_x3_hard` ', ...
        'sin romper los criterios de aceptacion. Se conserva el baseline ', ...
        'SISO y la siguiente ruta recomendada es Hinf MIMO theta-phi o ', ...
        'rechazo lateral adicional.\n\n']);
else
    fprintf(fid, 'Candidatos aceptados: %d. Mejor candidato: `%s`.\n\n', ...
        numel(sweep.accepted), sweep.best.tag);
    write_row_detail(fid, sweep.best);
    fprintf(fid, ['\nLectura: el candidato aceptado es una micro-mejora ', ...
        'formal, pero no elimina `phi_sobrepasa`. Si el objetivo es ', ...
        'quitar ese flag, esta ronda confirma que conviene cerrar SISO ', ...
        'y pasar a Hinf MIMO theta-phi o rechazo lateral adicional.\n\n']);
end

fprintf(fid, '## Top por score\n\n');
write_top_table(fid, sweep.rows, 15);

fprintf(fid, '\n## Todos los candidatos\n\n');
write_table(fid, sweep.rows);
end

function write_row_detail(fid, r)
fprintf(fid, '- tag: `%s`\n', r.tag);
fprintf(fid, '- pesos: W1 %.0f, W2 %.2f -> %.2f, W2 cross %.3g, W3 high %.0f\n', ...
    r.W1_phi_low_gain, r.W2_phi_low_gain, r.W2_phi_high_gain, ...
    r.W2_phi_cross_frequency, r.W3_phi_high_gain);
fprintf(fid, '- gamma phi: %.3f; KS phi: %.3f\n', r.gamma_phi, r.KS_phi);
fprintf(fid, '- phi_30 error final: %.3f deg; phi_40 error final: %.3f deg\n', ...
    r.phi30_final_error, r.phi40_final_error);
fprintf(fid, '- theta_phi_45_hard error final phi: %.3f deg; flags: `%s`\n', ...
    r.theta_phi_45_final_error, r.theta_phi_45_flags);
fprintf(fid, '- noise_dist_x3_hard sobrepaso phi: %.3f deg; flags: `%s`\n', ...
    r.noise_phi_overshoot, r.noise_flags);
fprintf(fid, '- core ok: %d; ruido mejora: %d; aceptado: %d\n', ...
    r.core_ok, r.noise_improved, r.accepted);
end

function write_top_table(fid, rows, n)
[~, idx] = sort([rows.score], 'ascend');
idx = idx(1:min(n, numel(idx)));
write_table(fid, rows(idx));
end

function write_table(fid, rows)
fprintf(fid, '| cand | tag | W1 | W2 low | W2 high | W2 cross | W3 high | gamma phi | KS phi | err phi30 | err phi40 | err combo | over noise | combo flags | noise flags | core | mejora ruido | ok |\n');
fprintf(fid, '|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|:---:|:---:|:---:|\n');
for k = 1:numel(rows)
    r = rows(k);
    fprintf(fid, ['| %d | `%s` | %.0f | %.2f | %.2f | %.3g | %.0f | ', ...
        '%.3f | %.3f | %.3f | %.3f | %.3f | %.3f | `%s` | `%s` | ', ...
        '%d | %d | %d |\n'], ...
        r.candidate, r.tag, r.W1_phi_low_gain, r.W2_phi_low_gain, ...
        r.W2_phi_high_gain, r.W2_phi_cross_frequency, ...
        r.W3_phi_high_gain, r.gamma_phi, r.KS_phi, ...
        r.phi30_final_error, r.phi40_final_error, ...
        r.theta_phi_45_final_error, r.noise_phi_overshoot, ...
        r.theta_phi_45_flags, r.noise_flags, r.core_ok, ...
        r.noise_improved, r.accepted);
end
end
