%% Taller 1: Control robusto H_inf en UAV
% Flujo reproducible:
% 1) carga y analisis de planta,
% 2) diseno PID/SAS-CAS,
% 3) diseno H_inf por sensibilidad mixta,
% 4) analisis S, T, K*S,
% 5) simulacion final sobre linmodel acoplado.

clear; clc; close all;

% Paso 1: registrar carpeta del taller en el path y leer configuracion.
project_dir = fileparts(mfilename('fullpath'));
addpath(project_dir);

cfg = parametros_taller1(project_dir);
if ~exist(cfg.results_dir, 'dir')
    mkdir(cfg.results_dir);
end


% Paso 2: cargar modelos del UAV y extraer canales de diseno.
plant = cargar_modelo_uav(cfg);
channels = seleccionar_canales_uav(plant, cfg);

% Se deja en base workspace como comodidad para inspeccion interactiva.
assignin('base', 'taller1_channels', channels);

% Paso 3: analizar planta, disenar PID y sintetizar H_inf.
analysis = analisis_planta_uav(plant, channels, cfg);
pid_data = diseno_pid_sas_cas(channels, cfg);
hinf_data = diseno_hinf_taller1(channels, cfg);

% Paso 4: calcular sensibilidad y simular el modelo acoplado.
sens = analisis_sensibilidades(channels, pid_data, hinf_data, cfg);
sim_results = simulacion_taller1(plant, pid_data, hinf_data, cfg);

% Paso 5: exportar figuras con fondo blanco y guardar resultados completos.
crear_graficas_taller1(channels, sens, sim_results, cfg);

save(fullfile(cfg.results_dir, 'taller1_results.mat'), 'cfg', 'plant', ...
    'channels', 'analysis', 'pid_data', 'hinf_data', 'sens', ...
    'sim_results');

% Paso 6: imprimir resumen numerico para revision rapida en consola.
fprintf('\n=== Taller 1 UAV H_inf ===\n');
fprintf('Modelo completo: %d estados, %d entradas, %d salidas\n', ...
    order(plant.full), size(plant.full, 2), size(plant.full, 1));
fprintf('PID theta estable: %d | PID phi estable: %d\n', ...
    pid_data.theta_stable, pid_data.phi_stable);
fprintf('Hinf theta gamma: %.4f | orden K: %d\n', ...
    hinf_data.theta.gamma, hinf_data.theta.order);
fprintf('Hinf phi gamma: %.4f | orden K: %d\n', ...
    hinf_data.phi.gamma, hinf_data.phi.order);

fprintf('\n=== Normas H_inf aproximadas ===\n');
fprintf('theta PID : ||S||=%.3f ||T||=%.3f ||KS||=%.3f\n', ...
    sens.theta.pid.norm_S, sens.theta.pid.norm_T, sens.theta.pid.norm_KS);
fprintf('theta Hinf: ||S||=%.3f ||T||=%.3f ||KS||=%.3f\n', ...
    sens.theta.hinf.norm_S, sens.theta.hinf.norm_T, sens.theta.hinf.norm_KS);
fprintf('phi PID   : ||S||=%.3f ||T||=%.3f ||KS||=%.3f\n', ...
    sens.phi.pid.norm_S, sens.phi.pid.norm_T, sens.phi.pid.norm_KS);
fprintf('phi Hinf  : ||S||=%.3f ||T||=%.3f ||KS||=%.3f\n', ...
    sens.phi.hinf.norm_S, sens.phi.hinf.norm_T, sens.phi.hinf.norm_KS);

fprintf('\n=== Resumen simulacion acoplada ===\n');
for k = 1:numel(sim_results.summary)
    row = sim_results.summary(k);
    fprintf('%-4s %-18s RMS theta=%7.3f deg RMS phi=%7.3f deg sat=%5.1f%%\n', ...
        row.controller, row.scenario, row.theta_rms_error_deg, ...
        row.phi_rms_error_deg, 100*row.sat_fraction);
end

fprintf('\nResultados guardados en: %s\n', cfg.results_dir);
fprintf('Figuras guardadas en: %s\n', cfg.figures_dir);
fprintf('Para generar el modelo Simulink: build_taller1_simulink\n');
