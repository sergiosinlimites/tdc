%% Taller 1 — Control robusto H-inf en UAV (archivo unificado)
% Flujo reproducible de principio a fin:
%   0) Limpieza y configuracion
%   1) Carga del modelo UAV
%   2) Extraccion de canales de diseno
%   3) Analisis de la planta
%   4) Diseno SAS por root locus
%   5) Diseno CAS PI sobre planta amortiguada
%   6) Controlador PID/SAS-CAS equivalente
%   7) Diseno H-inf por sensibilidad mixta
%   8) Analisis de sensibilidades S, T, KS
%   9) Simulacion temporal sobre linmodel acoplado
%  10) Generacion de figuras
%  11) Resumen y conclusiones

%% 0. Limpieza y configuracion
clear; clc; close all;

project_dir = fileparts(mfilename('fullpath'));
repo_dir    = fileparts(project_dir);
figures_dir = fullfile(project_dir, 'figures');
if ~exist(figures_dir, 'dir'), mkdir(figures_dir); end

model_path = fullfile(repo_dir, 'drive', 'TDC', '02. TAREAS', ...
    'T1', 'modelo_lin.mat');

% Especificaciones del enunciado
ref_max_deg       = 40;
control_limit_deg = 30;
control_limit_rad = deg2rad(control_limit_deg);
bandwidth_hz      = 8;
perturbation_hz   = 6;
wb = 2*pi*bandwidth_hz;
wp = 2*pi*perturbation_hz;
noise_power_long  = 1e-4;
noise_power_lat   = 1e-3;
minreal_tol       = 1e-6;

% Nombres del modelo completo
full_inputs  = {'throttle','elevator','rudder','l_aileron','r_aileron', ...
    'l_flap','r_flap','aileron'};
full_outputs = {'V','beta','alpha','h','phi','theta','psi','p','q','r', ...
    'gamma','ax','ay','az'};

% Ganancias SAS/CAS
kp_theta = -1.00;  ki_theta = -0.30;  kd_theta = -0.20;
kp_phi   = -0.35;  ki_phi   = -0.18;  kd_phi   =  0.05;
derivative_tau = 0.03;
antiwindup_gain = 8.0;
yaw_damper_gain = 0.065;
yaw_damper_pole = -2.0;

% Parametros de simulacion
t_final          = 12.0;
dt               = 0.005;
t_step           = 1.0;
disturbance_start = 2.0;
disturbance_amp_rad = deg2rad(1.0);
rng_seed         = 7;

s = tf('s');

%% 1. Carga del modelo UAV
if ~isfile(model_path)
    error('No se encontro modelo_lin.mat en: %s', model_path);
end
data = load(model_path);
plant_full = ss(data.linmodel);
plant_lat  = ss(data.latmod);
plant_long = ss(data.longmod);
plant_full.InputName  = full_inputs;
plant_full.OutputName = full_outputs;

fprintf('Modelo completo: %d estados, %d entradas, %d salidas\n', ...
    order(plant_full), size(plant_full,2), size(plant_full,1));

%% 2. Extraccion de canales de diseno
G_theta = minreal(plant_long('theta','elevator'), minreal_tol);
G_theta.InputName = {'elevator'}; G_theta.OutputName = {'theta'};

G_q = minreal(plant_long('q','elevator'), minreal_tol);
G_q.InputName = {'elevator'}; G_q.OutputName = {'q'};

G_phi = minreal(plant_lat('phi','aileron'), minreal_tol);
G_phi.InputName = {'aileron'}; G_phi.OutputName = {'phi'};

G_p = minreal(plant_lat('p','aileron'), minreal_tol);
G_p.InputName = {'aileron'}; G_p.OutputName = {'p'};

G_r = minreal(plant_lat('r','rudder'), minreal_tol);
G_r.InputName = {'rudder'}; G_r.OutputName = {'r'};

G_mimo = minreal(plant_full({'theta','phi'}, {'elevator','aileron'}), ...
    minreal_tol);
G_mimo.InputName = {'elevator','aileron'};
G_mimo.OutputName = {'theta','phi'};

G_lat_mimo = minreal(plant_lat({'phi','r'}, {'aileron','rudder'}), ...
    minreal_tol);
G_lat_mimo.InputName = {'aileron','rudder'};
G_lat_mimo.OutputName = {'phi','r'};

%% 3. Analisis de la planta
fprintf('\n=== Analisis de la planta ===\n');

poles_full = pole(plant_full);
poles_lat  = pole(plant_lat);
poles_long = pole(plant_long);

fprintf('Polos planta completa: %d  (max Re = %.4f)\n', ...
    numel(poles_full), max(real(poles_full)));
fprintf('Polos lat: %d | Polos long: %d\n', ...
    numel(poles_lat), numel(poles_long));

tzeros_theta = tzero(G_theta);
tzeros_phi   = tzero(G_phi);
dc_theta = dcgain(G_theta);
dc_phi   = dcgain(G_phi);
fprintf('DC gain theta/elev = %.4f | DC gain phi/ail = %.4f\n', ...
    dc_theta, dc_phi);

ctrl_rank_full = rank(ctrb(plant_full.A, plant_full.B));
obsv_rank_full = rank(obsv(plant_full.A, plant_full.C));
n_full = size(plant_full.A,1);
fprintf('Controlabilidad: %d/%d | Observabilidad: %d/%d\n', ...
    ctrl_rank_full, n_full, obsv_rank_full, n_full);

%% 4. Diseno SAS por root locus
fprintf('\n=== Diseno SAS ===\n');

% Pitch: q/elevator con D_q negativo para amortiguar
D_q = kd_theta;
G_q_inner = minreal(feedback(G_q, D_q), minreal_tol);
fprintf('SAS pitch D_q = %.4f, lazo interno estable: %d\n', ...
    D_q, isstable(G_q_inner));

% Roll: p/aileron con D_p positivo para conservar estabilidad
D_p = kd_phi;
G_p_inner = minreal(feedback(G_p, D_p), minreal_tol);
fprintf('SAS roll  D_p = %.4f, lazo interno estable: %d\n', ...
    D_p, isstable(G_p_inner));

% Yaw damper con washout
D_r = -yaw_damper_gain;
G_r_inner = minreal(feedback(G_r, D_r), minreal_tol);
fprintf('Yaw damper D_r = %.4f, lazo interno estable: %d\n', ...
    D_r, isstable(G_r_inner));

%% 5. Diseno CAS PI sobre planta amortiguada
fprintf('\n=== Diseno CAS ===\n');

% Planta externa: G_angle / (1 + D*G_rate)
G_theta_sas = minreal(G_theta/(1 + D_q*G_q), minreal_tol);
G_phi_sas   = minreal(G_phi/(1 + D_p*G_p), minreal_tol);

% Controladores PI
PI_theta = minreal(kp_theta + ki_theta/s, minreal_tol);
PI_phi   = minreal(kp_phi   + ki_phi/s,   minreal_tol);

% Lazos abiertos y cerrados del CAS
L_cas_theta = minreal(PI_theta*G_theta_sas, minreal_tol);
L_cas_phi   = minreal(PI_phi*G_phi_sas, minreal_tol);
T_cas_theta = feedback(L_cas_theta, 1);
T_cas_phi   = feedback(L_cas_phi, 1);

% Margenes
[gm_theta, pm_theta] = margin(L_cas_theta);
[gm_phi,   pm_phi]   = margin(L_cas_phi);
fprintf('CAS theta: estable=%d, GM=%.2f dB, PM=%.1f deg\n', ...
    isstable(T_cas_theta), 20*log10(gm_theta), pm_theta);
fprintf('CAS phi:   estable=%d, GM=%.2f dB, PM=%.1f deg\n', ...
    isstable(T_cas_phi), 20*log10(gm_phi), pm_phi);

%% 6. Controlador PID/SAS-CAS equivalente
K_theta_pid = minreal(kp_theta + ki_theta/s + ...
    kd_theta*s/(derivative_tau*s + 1), minreal_tol);
K_phi_pid = minreal(kp_phi + ki_phi/s + ...
    kd_phi*s/(derivative_tau*s + 1), minreal_tol);
K_theta_pid.InputName = {'e_theta'};
K_theta_pid.OutputName = {'elevator'};
K_phi_pid.InputName = {'e_phi'};
K_phi_pid.OutputName = {'aileron'};

% Verificacion de estabilidad nominal SISO
theta_loop_pid = feedback(G_theta*K_theta_pid, 1);
phi_loop_pid   = feedback(G_phi*K_phi_pid, 1);
fprintf('\nPID theta estable: %d | PID phi estable: %d\n', ...
    isstable(theta_loop_pid), isstable(phi_loop_pid));

%% 7. Diseno H-inf por sensibilidad mixta
fprintf('\n=== Diseno H-inf ===\n');

% 7.1 Pesos para theta
W1_theta = makeweight(80, wb, 0.05);
W2_theta = 1.0;
W3_theta = makeweight(0.005, wp, 15);

% 7.2 Pesos para phi (iteracion aceptada del barrido)
W1_phi = makeweight(220, wb, 0.05);
wz_phi = wp;
wp_phi = wz_phi * 3.20/0.80;
W2_phi = 0.80*(s/wz_phi + 1)/(s/wp_phi + 1);
W2_phi = minreal(W2_phi, minreal_tol);
W3_phi = makeweight(0.005, wp, 15);

% 7.3 Sintesis theta
[K_theta_hinf, CL_theta, gamma_theta, info_theta] = ...
    mixsyn(G_theta, W1_theta, W2_theta, W3_theta);
K_theta_hinf = minreal(ss(K_theta_hinf), minreal_tol);
CL_theta     = minreal(ss(CL_theta), minreal_tol);
fprintf('Theta: gamma = %.4f, orden K = %d, K estable = %d, CL estable = %d\n', ...
    gamma_theta, order(K_theta_hinf), isstable(K_theta_hinf), ...
    isstable(CL_theta));

% 7.4 Sintesis phi
[K_phi_hinf, CL_phi, gamma_phi, info_phi] = ...
    mixsyn(G_phi, W1_phi, W2_phi, W3_phi);
K_phi_hinf = minreal(ss(K_phi_hinf), minreal_tol);
CL_phi     = minreal(ss(CL_phi), minreal_tol);
fprintf('Phi:   gamma = %.4f, orden K = %d, K estable = %d, CL estable = %d\n', ...
    gamma_phi, order(K_phi_hinf), isstable(K_phi_hinf), ...
    isstable(CL_phi));

%% 8. Analisis de sensibilidades S, T, KS
fprintf('\n=== Sensibilidades ===\n');

% SAS/CAS
[S_pid_theta, T_pid_theta, KS_pid_theta] = loop_sens(G_theta, K_theta_pid, minreal_tol);
[S_pid_phi,   T_pid_phi,   KS_pid_phi]   = loop_sens(G_phi,   K_phi_pid,   minreal_tol);

% H-inf
[S_hinf_theta, T_hinf_theta, KS_hinf_theta] = loop_sens(G_theta, K_theta_hinf, minreal_tol);
[S_hinf_phi,   T_hinf_phi,   KS_hinf_phi]   = loop_sens(G_phi,   K_phi_hinf,   minreal_tol);

% Normas H-inf aproximadas
nS_pid_theta  = safe_norminf(S_pid_theta);
nT_pid_theta  = safe_norminf(T_pid_theta);
nKS_pid_theta = safe_norminf(KS_pid_theta);
nS_pid_phi    = safe_norminf(S_pid_phi);
nT_pid_phi    = safe_norminf(T_pid_phi);
nKS_pid_phi   = safe_norminf(KS_pid_phi);

nS_hinf_theta  = safe_norminf(S_hinf_theta);
nT_hinf_theta  = safe_norminf(T_hinf_theta);
nKS_hinf_theta = safe_norminf(KS_hinf_theta);
nS_hinf_phi    = safe_norminf(S_hinf_phi);
nT_hinf_phi    = safe_norminf(T_hinf_phi);
nKS_hinf_phi   = safe_norminf(KS_hinf_phi);

fprintf('theta SAS : ||S||=%.3f ||T||=%.3f ||KS||=%.3f\n', ...
    nS_pid_theta, nT_pid_theta, nKS_pid_theta);
fprintf('theta Hinf: ||S||=%.3f ||T||=%.3f ||KS||=%.3f\n', ...
    nS_hinf_theta, nT_hinf_theta, nKS_hinf_theta);
fprintf('phi   SAS : ||S||=%.3f ||T||=%.3f ||KS||=%.3f\n', ...
    nS_pid_phi, nT_pid_phi, nKS_pid_phi);
fprintf('phi   Hinf: ||S||=%.3f ||T||=%.3f ||KS||=%.3f\n', ...
    nS_hinf_phi, nT_hinf_phi, nKS_hinf_phi);

%% 9. Simulacion temporal sobre linmodel acoplado
fprintf('\n=== Simulacion temporal ===\n');

scenarios = make_scenarios();
[A_plant, B_plant, C_plant, ~] = ssdata(plant_full);
n_plant = size(A_plant,1);
idx = make_indices(plant_full);

% 9.1 Preparar controladores en forma de estados
pid_ctrl  = make_pid_controller(kp_theta, ki_theta, kd_theta, ...
    kp_phi, ki_phi, kd_phi, derivative_tau, antiwindup_gain, ...
    yaw_damper_gain, yaw_damper_pole);
hinf_ctrl = make_hinf_controller(K_theta_hinf, K_phi_hinf, ...
    yaw_damper_gain, yaw_damper_pole);

% 9.2 Simular todos los escenarios
rng(rng_seed);
for k = 1:numel(scenarios)
    sim_pid(k) = simulate_case('pid', A_plant, B_plant, C_plant, ...
        n_plant, idx, pid_ctrl, scenarios(k), ...
        t_final, dt, t_step, control_limit_rad, ...
        noise_power_long, noise_power_lat, ...
        disturbance_start, disturbance_amp_rad, perturbation_hz); %#ok<SAGROW>
end

rng(rng_seed);
for k = 1:numel(scenarios)
    sim_hinf(k) = simulate_case('hinf', A_plant, B_plant, C_plant, ...
        n_plant, idx, hinf_ctrl, scenarios(k), ...
        t_final, dt, t_step, control_limit_rad, ...
        noise_power_long, noise_power_lat, ...
        disturbance_start, disturbance_amp_rad, perturbation_hz); %#ok<SAGROW>
end

% 9.3 Resumen en consola
fprintf('\n%-4s %-18s %10s %10s %7s\n', ...
    'Ctrl', 'Escenario', 'RMS_th[d]', 'RMS_ph[d]', 'Sat[%]');
fprintf('%s\n', repmat('-', 1, 55));
for k = 1:numel(scenarios)
    m = sim_pid(k).metrics;
    fprintf('%-4s %-18s %10.3f %10.3f %7.1f\n', ...
        'SAS', sim_pid(k).name, m.theta_rms_error_deg, ...
        m.phi_rms_error_deg, 100*m.sat_fraction);
end
for k = 1:numel(scenarios)
    m = sim_hinf(k).metrics;
    fprintf('%-4s %-18s %10.3f %10.3f %7.1f\n', ...
        'Hinf', sim_hinf(k).name, m.theta_rms_error_deg, ...
        m.phi_rms_error_deg, 100*m.sat_fraction);
end

%% 10. Generacion de figuras
fprintf('\n=== Generando figuras ===\n');

% 10.1 Valores singulares de plantas SISO
w_plot = logspace(-2, 3, 600);
fig = make_white_figure('Valores singulares de plantas');
tiledlayout(1,2);
nexttile;
plot_sigma_response(G_theta, 'b-', 'theta/elevator', w_plot);
hold on;
plot_sigma_response(G_phi, 'r--', 'phi/aileron', w_plot);
style_axes(); legend('theta/elevator','phi/aileron','Location','best');
title('Canales SISO');
nexttile;
plot_sigma_response(G_mimo, 'k-', 'MIMO theta-phi', w_plot);
style_axes(); title('MIMO theta-phi');
export_png(fig, fullfile(figures_dir, 'planta_sigma.png'));
close(fig);

% 10.2 Root locus SAS
plot_sas_rl(G_q, D_q, G_q_inner, 'q/elevator', 'D_q', ...
    fullfile(figures_dir, 'root_locus_sas_q.png'));
plot_sas_rl(G_p, D_p, G_p_inner, 'p/aileron', 'D_p', ...
    fullfile(figures_dir, 'root_locus_sas_p.png'));
plot_sas_rl(G_r, D_r, G_r_inner, 'r/rudder', 'D_r', ...
    fullfile(figures_dir, 'root_locus_sas_r.png'));

% 10.3 Root locus CAS
plot_cas_rl(L_cas_theta, T_cas_theta, t_final, ...
    'CAS PI theta', fullfile(figures_dir, 'root_locus_cas_theta.png'));
plot_cas_rl(L_cas_phi, T_cas_phi, t_final, ...
    'CAS PI phi', fullfile(figures_dir, 'root_locus_cas_phi.png'));

% 10.4 Sensibilidades por eje
plot_axis_sensitivities('theta', ...
    S_pid_theta, T_pid_theta, KS_pid_theta, ...
    S_hinf_theta, T_hinf_theta, KS_hinf_theta, ...
    W1_theta, W2_theta, W3_theta, w_plot, ...
    fullfile(figures_dir, 'sensibilidades_theta.png'));
plot_axis_sensitivities('phi', ...
    S_pid_phi, T_pid_phi, KS_pid_phi, ...
    S_hinf_phi, T_hinf_phi, KS_hinf_phi, ...
    W1_phi, W2_phi, W3_phi, w_plot, ...
    fullfile(figures_dir, 'sensibilidades_phi.png'));

% 10.5 Comparacion de sensibilidades 3x2
fig = make_white_figure('Comparacion S T KS');
tiledlayout(3,2);
plot_compare_tile(S_pid_theta,  S_hinf_theta,  'S theta',  w_plot);
plot_compare_tile(S_pid_phi,    S_hinf_phi,    'S phi',    w_plot);
plot_compare_tile(T_pid_theta,  T_hinf_theta,  'T theta',  w_plot);
plot_compare_tile(T_pid_phi,    T_hinf_phi,    'T phi',    w_plot);
plot_compare_tile(KS_pid_theta, KS_hinf_theta, 'KS theta', w_plot);
plot_compare_tile(KS_pid_phi,   KS_hinf_phi,   'KS phi',   w_plot);
export_png(fig, fullfile(figures_dir, 'comparacion_sensibilidades.png'));
close(fig);

% 10.6 Simulaciones temporales seleccionadas
selected_sims = {'theta_10','phi_10','theta_phi_10','theta_30', ...
    'phi_30','theta_phi_30','theta_40','phi_40','noise_disturbance'};
names_pid = {sim_pid.name};
for i = 1:numel(selected_sims)
    jj = find(strcmp(names_pid, selected_sims{i}), 1);
    if ~isempty(jj)
        plot_case_pair(sim_pid(jj), sim_hinf(jj), control_limit_deg, ...
            fullfile(figures_dir, ['sim_' selected_sims{i} '.png']));
    end
end

% 10.7 Barras comparativas RMS y saturacion
plot_rms_bars(sim_pid, sim_hinf, figures_dir);
plot_sat_bars(sim_pid, sim_hinf, figures_dir);

fprintf('Figuras guardadas en: %s\n', figures_dir);

%% 11. Resumen y conclusiones
fprintf('\n========================================\n');
fprintf('  RESUMEN FINAL - Taller 1 UAV H-inf\n');
fprintf('========================================\n');
fprintf('\nGanancias SAS/CAS:\n');
fprintf('  D_q = %.2f | D_p = %.2f\n', D_q, D_p);
fprintf('  Kp_theta = %.2f | Ki_theta = %.2f\n', kp_theta, ki_theta);
fprintf('  Kp_phi   = %.2f | Ki_phi   = %.2f\n', kp_phi, ki_phi);
fprintf('\nResultados H-inf:\n');
fprintf('  theta: gamma = %.3f, orden K = %d\n', ...
    gamma_theta, order(K_theta_hinf));
fprintf('  phi:   gamma = %.3f, orden K = %d\n', ...
    gamma_phi, order(K_phi_hinf));
fprintf('\nSensibilidades comparadas:\n');
fprintf('  %-14s %7s %7s %7s\n', 'Lazo', '||S||', '||T||', '||KS||');
fprintf('  %-14s %7.3f %7.3f %7.3f\n', 'theta SAS/CAS', ...
    nS_pid_theta, nT_pid_theta, nKS_pid_theta);
fprintf('  %-14s %7.3f %7.3f %7.3f\n', 'theta H-inf', ...
    nS_hinf_theta, nT_hinf_theta, nKS_hinf_theta);
fprintf('  %-14s %7.3f %7.3f %7.3f\n', 'phi SAS/CAS', ...
    nS_pid_phi, nT_pid_phi, nKS_pid_phi);
fprintf('  %-14s %7.3f %7.3f %7.3f\n', 'phi H-inf', ...
    nS_hinf_phi, nT_hinf_phi, nKS_hinf_phi);
fprintf('\nConclusiones:\n');
fprintf('  - SAS/CAS: simple, poco esfuerzo de actuador, tracking directo.\n');
fprintf('  - H-inf: mejor RMS global, T baja en alta frecuencia.\n');
fprintf('  - H-inf SISO phi: KS lateral mayor, flag residual en pruebas extremas.\n');
fprintf('  - Extension natural: H-inf MIMO para manejar acoplamiento.\n');
fprintf('\n=== Taller 1 completado ===\n');

%% ========================================================================
%  FUNCIONES LOCALES
%  ========================================================================

function [S, T, KS] = loop_sens(G, K, tol)
    L  = minreal(G*K, tol);
    S  = minreal(feedback(1, L), tol);
    T  = minreal(feedback(L, 1), tol);
    KS = minreal(K*S, tol);
end

function n = safe_norminf(sys)
    try
        n = norm(sys, inf);
    catch
        n = NaN;
    end
end

function scenarios = make_scenarios()
    scenarios(1)  = make_scenario('theta_10',        10,  0, false, false);
    scenarios(2)  = make_scenario('theta_minus_10', -10,  0, false, false);
    scenarios(3)  = make_scenario('phi_10',           0, 10, false, false);
    scenarios(4)  = make_scenario('phi_minus_10',     0,-10, false, false);
    scenarios(5)  = make_scenario('theta_phi_10',    10, 10, false, false);
    scenarios(6)  = make_scenario('theta_30',        30,  0, false, false);
    scenarios(7)  = make_scenario('phi_30',           0, 30, false, false);
    scenarios(8)  = make_scenario('theta_phi_30',    30, 30, false, false);
    scenarios(9)  = make_scenario('theta_40',        40,  0, false, false);
    scenarios(10) = make_scenario('phi_40',           0, 40, false, false);
    scenarios(11) = make_scenario('noise_disturbance',10,10, true,  true);
end

function sc = make_scenario(name, theta_deg, phi_deg, noise_on, dist_on)
    sc.name = name;
    sc.theta_ref_deg = theta_deg;
    sc.phi_ref_deg   = phi_deg;
    sc.theta_ref_rad = deg2rad(theta_deg);
    sc.phi_ref_rad   = deg2rad(phi_deg);
    sc.noise_enabled = noise_on;
    sc.disturbance_enabled = dist_on;
end

function idx = make_indices(sys)
    idx.theta = find(strcmp(sys.OutputName, 'theta'));
    idx.phi   = find(strcmp(sys.OutputName, 'phi'));
    idx.q     = find(strcmp(sys.OutputName, 'q'));
    idx.p     = find(strcmp(sys.OutputName, 'p'));
    idx.r     = find(strcmp(sys.OutputName, 'r'));
    idx.u_elevator = find(strcmp(sys.InputName, 'elevator'));
    idx.u_rudder   = find(strcmp(sys.InputName, 'rudder'));
    idx.u_aileron  = find(strcmp(sys.InputName, 'aileron'));
end

function ctrl = make_pid_controller(kp_th, ki_th, kd_th, ...
        kp_ph, ki_ph, kd_ph, ~, aw, yd_gain, yd_pole)
    ctrl.type = 'pid';
    ctrl.pid.kp_theta = kp_th;
    ctrl.pid.ki_theta = ki_th;
    ctrl.pid.kd_theta = kd_th;
    ctrl.pid.kp_phi   = kp_ph;
    ctrl.pid.ki_phi   = ki_ph;
    ctrl.pid.kd_phi   = kd_ph;
    ctrl.pid.antiwindup = aw;
    yaw_tf = zpk(0, yd_pole, yd_gain);
    [ctrl.yaw.A, ctrl.yaw.B, ctrl.yaw.C, ctrl.yaw.D] = ssdata(ss(yaw_tf));
    ctrl.nx = 3;
end

function ctrl = make_hinf_controller(K_theta, K_phi, yd_gain, yd_pole)
    ctrl.type = 'hinf';
    [ctrl.theta.A, ctrl.theta.B, ctrl.theta.C, ctrl.theta.D] = ...
        ssdata(ss(K_theta));
    [ctrl.phi.A, ctrl.phi.B, ctrl.phi.C, ctrl.phi.D] = ...
        ssdata(ss(K_phi));
    yaw_tf = zpk(0, yd_pole, yd_gain);
    [ctrl.yaw.A, ctrl.yaw.B, ctrl.yaw.C, ctrl.yaw.D] = ssdata(ss(yaw_tf));
    ctrl.n_theta = size(ctrl.theta.A, 1);
    ctrl.n_phi   = size(ctrl.phi.A, 1);
    ctrl.nx = ctrl.n_theta + ctrl.n_phi + 1;
end

function sim = simulate_case(ctrl_name, A, B, C, n_plant, idx, ctrl, ...
        scenario, t_final, dt, t_step, ulim, ...
        np_long, np_lat, dist_start, dist_amp, pert_hz)
    t_grid = (0:dt:t_final).';
    noise = gen_noise(t_grid, np_long, np_lat, scenario);
    z0 = zeros(n_plant + ctrl.nx, 1);

    ode_fun = @(t, z) cl_ode(t, z, A, B, C, n_plant, idx, ctrl, ...
        scenario, t_grid, noise, t_step, ulim, dist_start, dist_amp, pert_hz);
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [t, z] = ode45(ode_fun, t_grid, z0, opts);

    sim = reconstruct_sim(t, z, A, B, C, n_plant, idx, ctrl, ...
        scenario, t_grid, noise, ctrl_name, t_step, ulim, ...
        dist_start, dist_amp, pert_hz, t_final);
end

function dz = cl_ode(t, z, A, B, C, np, idx, ctrl, sc, tg, noise, ...
        t_step, ulim, ds, da, ph)
    x  = z(1:np);
    xc = z(np+1:end);
    y  = C*x;
    n  = interp_noise(t, tg, noise);
    sig = build_signals(t, y, n, idx, sc, t_step);
    [u_sat, ~, xcdot] = ctrl_output(sig, xc, ctrl, ulim);
    u_plant = build_full_input(u_sat, t, sc, ds, da, ph);
    xdot = A*x + B*u_plant;
    dz = [xdot; xcdot];
end

function sim = reconstruct_sim(t, z, A, B, C, np, idx, ctrl, sc, ...
        tg, noise, ctrl_name, t_step, ulim, ds, da, ph, t_final)
    nt = numel(t);
    x  = z(:, 1:np);
    xc = z(:, np+1:end);
    ny = size(C, 1);
    nu = size(B, 2);
    y       = zeros(nt, ny);
    u_sat   = zeros(nt, 3);
    u_raw   = zeros(nt, 3);
    u_full  = zeros(nt, nu);
    refs    = zeros(nt, 2);
    err_out = zeros(nt, 2);

    for k = 1:nt
        y(k,:) = (C*x(k,:).').';
        n = interp_noise(t(k), tg, noise);
        sig = build_signals(t(k), y(k,:).', n, idx, sc, t_step);
        [us, ur, ~] = ctrl_output(sig, xc(k,:).', ctrl, ulim);
        u_sat(k,:) = us.';
        u_raw(k,:) = ur.';
        u_full(k,:) = build_full_input(us, t(k), sc, ds, da, ph).';
        refs(k,:) = [sig.theta_ref, sig.phi_ref];
        err_out(k,:) = [sig.e_theta, sig.e_phi];
    end

    sim.name       = sc.name;
    sim.controller = ctrl_name;
    sim.t          = t;
    sim.theta_deg     = rad2deg(y(:, idx.theta));
    sim.phi_deg       = rad2deg(y(:, idx.phi));
    sim.theta_ref_deg = rad2deg(refs(:,1));
    sim.phi_ref_deg   = rad2deg(refs(:,2));
    sim.elevator_deg  = rad2deg(u_sat(:,1));
    sim.aileron_deg   = rad2deg(u_sat(:,2));
    sim.rudder_deg    = rad2deg(u_sat(:,3));
    sim.u_sat = u_sat;
    sim.u_raw = u_raw;

    limit_rad = ulim;
    limit_deg = rad2deg(ulim);
    post = t >= t_step;
    tail = t >= max(t_final - 2, t_step);
    theta_err = sim.theta_ref_deg - sim.theta_deg;
    phi_err   = sim.phi_ref_deg   - sim.phi_deg;
    sat_mask  = any(abs(u_sat) >= limit_rad*0.999, 2);

    m.theta_rms_error_deg   = rms(theta_err(post));
    m.phi_rms_error_deg     = rms(phi_err(post));
    m.theta_final_error_deg = mean(theta_err(tail));
    m.phi_final_error_deg   = mean(phi_err(tail));
    m.max_abs_elevator_deg  = max(abs(sim.elevator_deg));
    m.max_abs_aileron_deg   = max(abs(sim.aileron_deg));
    m.max_abs_rudder_deg    = max(abs(sim.rudder_deg));
    m.sat_fraction          = mean(sat_mask);
    m.sat_time_s            = trapz(t, double(sat_mask));
    m.limit_deg             = limit_deg;
    sim.metrics = m;
end

function noise = gen_noise(t, np_long, np_lat, sc)
    nt = numel(t);
    if sc.noise_enabled
        noise.theta = sqrt(np_long)*randn(nt,1);
        noise.q     = sqrt(np_long)*randn(nt,1);
        noise.phi   = sqrt(np_lat)*randn(nt,1);
        noise.p     = sqrt(np_lat)*randn(nt,1);
        noise.r     = sqrt(np_lat)*randn(nt,1);
    else
        noise.theta = zeros(nt,1);
        noise.q     = zeros(nt,1);
        noise.phi   = zeros(nt,1);
        noise.p     = zeros(nt,1);
        noise.r     = zeros(nt,1);
    end
end

function n = interp_noise(t, tg, noise)
    k = min(max(1, floor((t - tg(1))/(tg(2)-tg(1))) + 1), numel(tg));
    n.theta = noise.theta(k);
    n.q     = noise.q(k);
    n.phi   = noise.phi(k);
    n.p     = noise.p(k);
    n.r     = noise.r(k);
end

function sig = build_signals(t, y, n, idx, sc, t_step)
    if t >= t_step
        sig.theta_ref = sc.theta_ref_rad;
        sig.phi_ref   = sc.phi_ref_rad;
    else
        sig.theta_ref = 0;
        sig.phi_ref   = 0;
    end
    sig.theta      = y(idx.theta);
    sig.phi        = y(idx.phi);
    sig.q          = y(idx.q);
    sig.p          = y(idx.p);
    sig.r          = y(idx.r);
    sig.theta_meas = sig.theta + n.theta;
    sig.phi_meas   = sig.phi   + n.phi;
    sig.q_meas     = sig.q     + n.q;
    sig.p_meas     = sig.p     + n.p;
    sig.r_meas     = sig.r     + n.r;
    sig.e_theta    = sig.theta_ref - sig.theta_meas;
    sig.e_phi      = sig.phi_ref   - sig.phi_meas;
end

function [u_sat, u_raw, xcdot] = ctrl_output(sig, xc, ctrl, ulim)
    switch ctrl.type
        case 'pid'
            xi_th = xc(1);
            xi_ph = xc(2);
            x_yaw = xc(3);

            u_elev = ctrl.pid.kp_theta*sig.e_theta + ...
                     ctrl.pid.ki_theta*xi_th - ctrl.pid.kd_theta*sig.q_meas;
            u_ail  = ctrl.pid.kp_phi*sig.e_phi + ...
                     ctrl.pid.ki_phi*xi_ph - ctrl.pid.kd_phi*sig.p_meas;
            u_rud  = ctrl.yaw.C*x_yaw + ctrl.yaw.D*sig.r_meas;

            u_raw = [u_elev; u_ail; u_rud];
            u_sat = min(max(u_raw, -ulim), ulim);

            xi_th_dot = sig.e_theta + ctrl.pid.antiwindup*(u_sat(1)-u_raw(1));
            xi_ph_dot = sig.e_phi   + ctrl.pid.antiwindup*(u_sat(2)-u_raw(2));
            x_yaw_dot = ctrl.yaw.A*x_yaw + ctrl.yaw.B*sig.r_meas;
            xcdot = [xi_th_dot; xi_ph_dot; x_yaw_dot];

        case 'hinf'
            xt  = xc(1:ctrl.n_theta);
            xp  = xc(ctrl.n_theta+1:ctrl.n_theta+ctrl.n_phi);
            xy  = xc(end);

            u_elev = ctrl.theta.C*xt + ctrl.theta.D*sig.e_theta;
            u_ail  = ctrl.phi.C*xp  + ctrl.phi.D*sig.e_phi;
            u_rud  = ctrl.yaw.C*xy  + ctrl.yaw.D*sig.r_meas;

            u_raw = [u_elev; u_ail; u_rud];
            u_sat = min(max(u_raw, -ulim), ulim);

            xtdot = ctrl.theta.A*xt + ctrl.theta.B*sig.e_theta;
            xpdot = ctrl.phi.A*xp  + ctrl.phi.B*sig.e_phi;
            xydot = ctrl.yaw.A*xy  + ctrl.yaw.B*sig.r_meas;
            xcdot = [xtdot; xpdot; xydot];
    end
end

function u = build_full_input(u_sat, t, sc, ds, da, ph)
    u = zeros(8,1);
    d = zeros(3,1);
    if sc.disturbance_enabled && t >= ds
        w = 2*pi*ph;
        d = da*[sin(w*t); sin(w*t+pi/4); sin(w*t+pi/2)];
    end
    u(2) = u_sat(1) + d(1);
    u(3) = u_sat(3) + d(3);
    u(8) = u_sat(2) + d(2);
end

% --- Funciones de graficacion ---

function fig = make_white_figure(name)
    fig = figure('Name', name, 'Visible', 'off', 'Color', 'w');
end

function plot_sigma_response(sys, ls, dn, w)
    [sv, wout] = sigma(sys, w);
    sv = squeeze(sv);
    if isvector(sv)
        mag = sv(:).';
    else
        mag = max(sv, [], 1);
    end
    semilogx(wout, 20*log10(max(mag, eps)), ls, ...
        'LineWidth', 1.4, 'DisplayName', dn);
    ylabel('Magnitud [dB]');
    xlabel('Frecuencia [rad/s]');
end

function style_axes()
    axl = findall(gcf, 'Type', 'axes');
    for k = 1:numel(axl)
        ax = axl(k);
        ax.Color = 'w';
        ax.XColor = [0 0 0];
        ax.YColor = [0 0 0];
        ax.GridColor = [0.65 0.65 0.65];
        ax.GridAlpha = 0.35;
        grid(ax, 'on');
        box(ax, 'on');
    end
end

function export_png(fig, filename)
    exportgraphics(fig, filename, 'BackgroundColor', 'white', ...
        'Resolution', 200);
end

function plot_sas_rl(G, D_gain, inner_cl, label, gain_name, filepath)
    fig = make_white_figure(['SAS ' label]);
    tiledlayout(1,2);
    nexttile;
    rlocus(G); hold on;
    plot(real(pole(inner_cl)), imag(pole(inner_cl)), ...
        'rx', 'MarkerSize', 8, 'LineWidth', 1.6);
    title([label ' D positivo']); grid on;
    nexttile;
    rlocus(-G); hold on;
    plot(real(pole(inner_cl)), imag(pole(inner_cl)), ...
        'rx', 'MarkerSize', 8, 'LineWidth', 1.6);
    title([label ' D negativo']); grid on;
    sgtitle(sprintf('%s = %.4g', gain_name, D_gain));
    export_png(fig, filepath);
    close(fig);
end

function plot_cas_rl(L_open, T_cl, tf_sim, ttl, filepath)
    fig = make_white_figure(ttl);
    tiledlayout(1,2);
    nexttile;
    rlocus(L_open); hold on;
    plot(real(pole(T_cl)), imag(pole(T_cl)), ...
        'rx', 'MarkerSize', 8, 'LineWidth', 1.6);
    title([ttl ' root locus']); grid on;
    nexttile;
    step(T_cl, tf_sim);
    title([ttl ' escalon unitario']); grid on;
    export_png(fig, filepath);
    close(fig);
end

function plot_axis_sensitivities(axis_name, ...
        S_pid, T_pid, KS_pid, S_hinf, T_hinf, KS_hinf, ...
        W1, W2, W3, w, filepath)
    fig = make_white_figure(['Sensibilidades ' axis_name]);
    tiledlayout(3,1);

    nexttile;
    plot_sigma_response(S_pid, 'b-', 'SAS/CAS', w); hold on;
    plot_sigma_response(S_hinf, 'r--', 'Hinf', w);
    plot_sigma_response(1/W1, 'k:', '1/W1', w);
    style_axes();
    legend('SAS/CAS','Hinf','1/W1','Location','best');
    title(['S — ' axis_name]);

    nexttile;
    plot_sigma_response(T_pid, 'b-', 'SAS/CAS', w); hold on;
    plot_sigma_response(T_hinf, 'r--', 'Hinf', w);
    plot_sigma_response(1/W3, 'k:', '1/W3', w);
    style_axes();
    legend('SAS/CAS','Hinf','1/W3','Location','best');
    title(['T — ' axis_name]);

    nexttile;
    plot_sigma_response(KS_pid, 'b-', 'SAS/CAS', w); hold on;
    plot_sigma_response(KS_hinf, 'r--', 'Hinf', w);
    if isa(W2, 'DynamicSystem')
        plot_sigma_response(1/W2, 'k:', '1/W2', w);
    else
        yline(20*log10(1/W2), 'k:', '1/W2', 'LineWidth', 1.2);
    end
    style_axes();
    legend('SAS/CAS','Hinf','1/W2','Location','best');
    title(['KS — ' axis_name]);

    export_png(fig, filepath);
    close(fig);
end

function plot_compare_tile(pid_sys, hinf_sys, ttl, w)
    nexttile;
    plot_sigma_response(pid_sys, 'b-', 'SAS/CAS', w); hold on;
    plot_sigma_response(hinf_sys, 'r--', 'Hinf', w);
    style_axes();
    legend('SAS/CAS','Hinf','Location','best');
    title(ttl);
end

function plot_case_pair(ps, hs, clim_deg, filepath)
    fig = make_white_figure(['Sim ' ps.name]);
    tiledlayout(4,1);

    nexttile;
    plot(ps.t, ps.theta_ref_deg, 'k:', 'LineWidth', 1.6); hold on;
    plot(ps.t, ps.theta_deg, 'b-', 'LineWidth', 1.3);
    plot(hs.t, hs.theta_deg, 'r--', 'LineWidth', 1.3);
    style_axes(); ylabel('theta [deg]');
    legend('ref','SAS/CAS','Hinf','Location','best');

    nexttile;
    plot(ps.t, ps.phi_ref_deg, 'k:', 'LineWidth', 1.6); hold on;
    plot(ps.t, ps.phi_deg, 'b-', 'LineWidth', 1.3);
    plot(hs.t, hs.phi_deg, 'r--', 'LineWidth', 1.3);
    style_axes(); ylabel('phi [deg]');

    nexttile;
    plot(ps.t, ps.elevator_deg, 'b-', 'LineWidth', 1.3); hold on;
    plot(hs.t, hs.elevator_deg, 'r--', 'LineWidth', 1.3);
    yline(clim_deg, 'Color', [.25 .25 .25], 'LineStyle', ':', 'LineWidth', 1.2);
    yline(-clim_deg, 'Color', [.25 .25 .25], 'LineStyle', ':', 'LineWidth', 1.2);
    style_axes(); ylabel('elevator [deg]');

    nexttile;
    plot(ps.t, ps.aileron_deg, 'b-', 'LineWidth', 1.3); hold on;
    plot(hs.t, hs.aileron_deg, 'r--', 'LineWidth', 1.3);
    yline(clim_deg, 'Color', [.25 .25 .25], 'LineStyle', ':', 'LineWidth', 1.2);
    yline(-clim_deg, 'Color', [.25 .25 .25], 'LineStyle', ':', 'LineWidth', 1.2);
    style_axes(); ylabel('aileron [deg]');
    xlabel('t [s]');

    export_png(fig, filepath);
    close(fig);
end

function plot_rms_bars(sp, sh, fdir)
    sel = {'theta_10','phi_10','theta_phi_10','theta_30', ...
           'phi_30','theta_phi_30','noise_disturbance'};
    np = {sp.name}; nh = {sh.name};
    pt = zeros(size(sel)); pp = pt; ht = pt; hp = pt;
    for k = 1:numel(sel)
        j1 = find(strcmp(np, sel{k}),1);
        j2 = find(strcmp(nh, sel{k}),1);
        pt(k) = sp(j1).metrics.theta_rms_error_deg;
        pp(k) = sp(j1).metrics.phi_rms_error_deg;
        ht(k) = sh(j2).metrics.theta_rms_error_deg;
        hp(k) = sh(j2).metrics.phi_rms_error_deg;
    end
    fig = make_white_figure('Comparacion temporal final');
    tiledlayout(2,1);
    nexttile;
    bar(categorical(sel), [pt(:), ht(:)]);
    ylabel('RMS theta [deg]'); legend('SAS/CAS','Hinf','Location','best');
    style_axes();
    nexttile;
    bar(categorical(sel), [pp(:), hp(:)]);
    ylabel('RMS phi [deg]'); legend('SAS/CAS','Hinf','Location','best');
    style_axes();
    export_png(fig, fullfile(fdir, 'comparacion_temporal_final.png'));
    close(fig);
end

function plot_sat_bars(sp, sh, fdir)
    sel = {'theta_10','phi_10','theta_phi_10','theta_30', ...
           'phi_30','theta_phi_30','theta_40','phi_40', ...
           'noise_disturbance'};
    np = {sp.name}; nh = {sh.name};
    ps = zeros(size(sel)); hs = ps;
    for k = 1:numel(sel)
        j1 = find(strcmp(np, sel{k}),1);
        j2 = find(strcmp(nh, sel{k}),1);
        ps(k) = 100*sp(j1).metrics.sat_fraction;
        hs(k) = 100*sh(j2).metrics.sat_fraction;
    end
    fig = make_white_figure('Comparacion saturacion final');
    bar(categorical(sel), [ps(:), hs(:)]);
    ylabel('Tiempo saturado [%]');
    legend('SAS/CAS','Hinf','Location','best');
    style_axes();
    export_png(fig, fullfile(fdir, 'comparacion_saturacion_final.png'));
    close(fig);
end
