%% Inicializacion de variables para taller1_uav.slx

% Paso 1: ubicar el taller y dejar sus funciones disponibles.
project_dir = fileparts(mfilename('fullpath'));
addpath(project_dir);

% Paso 2: reconstruir la misma configuracion usada por main_taller1.
cfg = parametros_taller1(project_dir);
cfg.export_design_figures = false;
plant = cargar_modelo_uav(cfg);
channels = seleccionar_canales_uav(plant, cfg);
pid_data = diseno_pid_sas_cas(channels, cfg);
hinf_data = diseno_hinf_taller1(channels, cfg);

% Paso 3: extraer matrices de la planta. En Simulink solo se entregan las
% salidas que usa el controlador para evitar lazos algebraicos innecesarios.
[A_full, B_full, C_full, D_full] = ssdata(plant.full);
idx_theta = find(strcmp(plant.full.OutputName, 'theta'));
idx_phi = find(strcmp(plant.full.OutputName, 'phi'));
idx_p = find(strcmp(plant.full.OutputName, 'p'));
idx_q = find(strcmp(plant.full.OutputName, 'q'));
idx_r = find(strcmp(plant.full.OutputName, 'r'));
C_sim = C_full([idx_theta idx_phi idx_p idx_q idx_r], :);
D_sim = zeros(5, size(B_full, 2));

% Paso 4: extraer matrices de los controladores H_inf dinamicos.
[Ktheta_A, Ktheta_B, Ktheta_C, Ktheta_D] = ssdata(ss(hinf_data.K_theta));
[Kphi_A, Kphi_B, Kphi_C, Kphi_D] = ssdata(ss(hinf_data.K_phi));

% Paso 5: extraer matrices del amortiguador de yaw.
yaw_tf = zpk(0, cfg.pid.yaw_damper_pole, cfg.pid.yaw_damper_gain);
[Kyaw_A, Kyaw_B, Kyaw_C, Kyaw_D] = ssdata(ss(yaw_tf));

% Paso 6: convertir CAS PI y SAS D a variables explicitas de Simulink.
s = tf('s');
CAS_PI_theta = cfg.pid.kp_theta + cfg.pid.ki_theta/s;
CAS_PI_phi = cfg.pid.kp_phi + cfg.pid.ki_phi/s;
[cas_pi_theta_num, cas_pi_theta_den] = tfdata(CAS_PI_theta, 'v');
[cas_pi_phi_num, cas_pi_phi_den] = tfdata(CAS_PI_phi, 'v');
sas_D_q = cfg.pid.kd_theta;
sas_D_p = cfg.pid.kd_phi;

% Paso 7: definir referencias, saturacion, ruido y perturbacion del modelo.
theta_ref_step = deg2rad(10);
phi_ref_step = deg2rad(10);
t_step = cfg.sim.t_step;
t_final = cfg.sim.t_final;
umax = cfg.spec.control_limit_rad;

noise_var_theta = cfg.spec.noise_power_long;
noise_var_phi = cfg.spec.noise_power_lat;
noise_sample_time = cfg.sim.dt;
dist_amp = cfg.sim.disturbance_amp_rad;
dist_freq = 2*pi*cfg.spec.perturbation_hz;

% 1 selecciona H_inf, 0 selecciona PID.
control_mode = 1;

% Paso 8: publicar estructuras utiles para inspeccion interactiva.
assignin('base', 'cfg', cfg);
assignin('base', 'plant', plant);
assignin('base', 'channels', channels);
assignin('base', 'pid_data', pid_data);
assignin('base', 'hinf_data', hinf_data);
