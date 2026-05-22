function sweep = optimizar_pesos_hinf(channels, plant, pid_data, cfg)
%OPTIMIZAR_PESOS_HINF Barrido reproducible de pesos H_inf.
%
% Uso desde el taller:
%   cfg = parametros_taller1(pwd);
%   plant = cargar_modelo_uav(cfg);
%   channels = seleccionar_canales_uav(plant,cfg);
%   pid_data = diseno_pid_sas_cas(channels,cfg);
%   sweep = optimizar_pesos_hinf(channels,plant,pid_data,cfg);

if nargin < 4
    cfg = parametros_taller1(fileparts(mfilename('fullpath')));
    plant = cargar_modelo_uav(cfg);
    channels = seleccionar_canales_uav(plant, cfg);
    cfg.export_design_figures = false;
    pid_data = diseno_pid_sas_cas(channels, cfg);
end

grid = candidate_grid(cfg);
rows = repmat(empty_row(), numel(grid), 1);

for k = 1:numel(grid)
    cfg_k = apply_candidate(cfg, grid(k));
    cfg_k.export_design_figures = false;
    try
        hinf_data = diseno_hinf_taller1(channels, cfg_k);
        sens = analisis_sensibilidades(channels, pid_data, hinf_data, cfg_k);
        cfg_k.scenarios = escenarios_optimizacion(cfg_k.scenarios);
        sim_results = simulacion_taller1(plant, pid_data, hinf_data, cfg_k);
        rows(k) = summarize_candidate(k, grid(k), hinf_data, sens, ...
            sim_results);
    catch ME
        rows(k) = empty_row();
        rows(k).candidate = k;
        rows(k).accepted = false;
        rows(k).notes = string(ME.message);
    end
end

[~, best_idx] = min([rows.score]);
sweep.rows = rows;
sweep.best = rows(best_idx);
    sweep.selection_rule = ['Se minimiza error final de phi, RMS lateral, ', ...
    'saturacion y KS; theta se mantiene con los pesos base.'];

if ~exist(cfg.results_dir, 'dir')
    mkdir(cfg.results_dir);
end
save(fullfile(cfg.results_dir, 'hinf_weight_sweep.mat'), 'sweep');
write_sweep_markdown(sweep, fullfile(cfg.results_dir, ...
    'hinf_weight_sweep_resumen.md'));
end

function scenarios = escenarios_optimizacion(scenarios)
names = {scenarios.name};
keep = {'theta_10', 'phi_10', 'theta_phi_10', 'phi_30', ...
    'theta_40', 'phi_40', 'noise_disturbance'};
mask = false(size(names));
for k = 1:numel(keep)
    mask = mask | strcmp(names, keep{k});
end
scenarios = scenarios(mask);
end

function grid = candidate_grid(cfg)
% Columnas: W1_phi_low, W2_phi_low, W2_phi_high, W2_phi_cross.
% El primer candidato conserva el Hinf SISO redisenado previo para comparar.
base = [ ...
    80 1.00 1.00 8; ...
    100 0.80 1.80 cfg.spec.wp; ...
    120 0.80 2.00 cfg.spec.wp; ...
    120 0.60 2.20 cfg.spec.wp; ...
    140 0.70 2.40 cfg.spec.wp; ...
    160 0.80 2.50 cfg.spec.wp; ...
    180 0.70 2.80 cfg.spec.wp; ...
    200 0.60 3.00 cfg.spec.wp; ...
    160 1.00 3.00 cfg.spec.wp; ...
    220 0.80 3.20 cfg.spec.wp];

for k = 1:size(base, 1)
    grid(k).W1_phi_low_gain = base(k, 1); %#ok<AGROW>
    grid(k).W2_phi_low_gain = base(k, 2); %#ok<AGROW>
    grid(k).W2_phi_high_gain = base(k, 3); %#ok<AGROW>
    grid(k).W2_phi_cross_frequency = base(k, 4); %#ok<AGROW>
end
end

function cfg = apply_candidate(cfg, c)
if ~isfield(cfg.hinf, 'phi')
    cfg.hinf.phi = cfg.hinf;
end

cfg.hinf.phi.W1_low_gain = c.W1_phi_low_gain;
cfg.hinf.phi.W2_low_gain = c.W2_phi_low_gain;
cfg.hinf.phi.W2_high_gain = c.W2_phi_high_gain;
cfg.hinf.phi.W2_gain = c.W2_phi_low_gain;
cfg.hinf.phi.W2_cross_frequency = c.W2_phi_cross_frequency;
end

function row = summarize_candidate(k, candidate, hinf_data, sens, sim_results)
rows = sim_results.summary(strcmp({sim_results.summary.controller}, 'hinf'));
theta_rms = mean([rows.theta_rms_error_deg]);
phi_rms = mean([rows.phi_rms_error_deg]);
sat_fraction_max = max([rows.sat_fraction]);
phi30 = find(strcmp({rows.scenario}, 'phi_30'), 1);
phi40 = find(strcmp({rows.scenario}, 'phi_40'), 1);
noise = find(strcmp({rows.scenario}, 'noise_disturbance'), 1);
phi30_final = abs(rows(phi30).phi_final_error_deg);
phi40_final = abs(rows(phi40).phi_final_error_deg);
noise_phi_final = abs(rows(noise).phi_final_error_deg);
score = 0.25*theta_rms + phi_rms + 1.5*phi30_final + ...
    2.0*phi40_final + 0.8*noise_phi_final + ...
    20*sat_fraction_max + 0.08*sens.phi.hinf.norm_KS;

row = empty_row();
row.candidate = k;
row.W1_phi_low_gain = candidate.W1_phi_low_gain;
row.W2_phi_low_gain = candidate.W2_phi_low_gain;
row.W2_phi_high_gain = candidate.W2_phi_high_gain;
row.W2_phi_cross_frequency = candidate.W2_phi_cross_frequency;
row.gamma_theta = hinf_data.theta.gamma;
row.gamma_phi = hinf_data.phi.gamma;
row.order_theta = hinf_data.theta.order;
row.order_phi = hinf_data.phi.order;
row.norm_KS_theta = sens.theta.hinf.norm_KS;
row.norm_KS_phi = sens.phi.hinf.norm_KS;
row.theta_rms_mean = theta_rms;
row.phi_rms_mean = phi_rms;
row.phi30_final_error = phi30_final;
row.phi40_final_error = phi40_final;
row.noise_phi_final_error = noise_phi_final;
row.sat_fraction_max = sat_fraction_max;
row.score = score;
row.accepted = hinf_data.theta.closed_loop_stable && ...
    hinf_data.phi.closed_loop_stable && sat_fraction_max < 0.05 && ...
    phi30_final < 1.5 && phi40_final < 2.0;
row.notes = "";
end

function row = empty_row()
row = struct('candidate', NaN, 'W1_phi_low_gain', NaN, ...
    'W2_phi_low_gain', NaN, 'W2_phi_high_gain', NaN, ...
    'W2_phi_cross_frequency', NaN, 'gamma_theta', NaN, 'gamma_phi', NaN, ...
    'order_theta', NaN, 'order_phi', NaN, 'norm_KS_theta', NaN, ...
    'norm_KS_phi', NaN, 'theta_rms_mean', NaN, 'phi_rms_mean', NaN, ...
    'phi30_final_error', NaN, 'phi40_final_error', NaN, ...
    'noise_phi_final_error', NaN, ...
    'sat_fraction_max', NaN, 'score', Inf, 'accepted', false, ...
    'notes', "");
end

function write_sweep_markdown(sweep, filename)
fid = fopen(filename, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# Barrido de pesos Hinf\n\n');
fprintf(fid, 'Regla: %s\n\n', sweep.selection_rule);
fprintf(fid, '| cand | W1 phi low | W2 phi low | W2 phi high | W2 cross | gamma theta | gamma phi | KS theta | KS phi | RMS theta | RMS phi | err phi30 | err phi40 | err noise | sat max %% | score | ok |\n');
fprintf(fid, '|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|\n');
for k = 1:numel(sweep.rows)
    r = sweep.rows(k);
    ok_text = 'no';
    if r.accepted
        ok_text = 'si';
    end
    fprintf(fid, ['| %d | %.3g | %.3g | %.3g | %.3g | %.3f | ', ...
        '%.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | ', ...
        '%.3f | %.2f | %.3f | %s |\n'], ...
        r.candidate, r.W1_phi_low_gain, r.W2_phi_low_gain, ...
        r.W2_phi_high_gain, r.W2_phi_cross_frequency, ...
        r.gamma_theta, r.gamma_phi, ...
        r.norm_KS_theta, r.norm_KS_phi, r.theta_rms_mean, ...
        r.phi_rms_mean, r.phi30_final_error, r.phi40_final_error, ...
        r.noise_phi_final_error, 100*r.sat_fraction_max, r.score, ok_text);
end
fprintf(fid, '\nMejor candidato: %d.\n', sweep.best.candidate);
end
