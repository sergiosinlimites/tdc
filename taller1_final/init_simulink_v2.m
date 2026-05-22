%% Inicializacion de variables para taller1_simulink_v2.slx
% Carga plantas SISO, controladores y parametros para los 4 lazos
% independientes del modelo v2.

project_dir = fileparts(mfilename('fullpath'));
repo_dir    = fileparts(project_dir);

model_path = fullfile(repo_dir, 'drive', 'TDC', '02. TAREAS', ...
    'T1', 'modelo_lin.mat');
if ~isfile(model_path)
    error('No se encontro modelo_lin.mat en: %s', model_path);
end

%% Especificaciones
control_limit_deg = 30;
umax = deg2rad(control_limit_deg);
wb   = 2*pi*8;
wp   = 2*pi*6;
minreal_tol = 1e-6;

%% Ganancias SAS/CAS
kp_theta = -1.00;  ki_theta = -0.30;  kd_theta = -0.20;
kp_phi   = -0.35;  ki_phi   = -0.18;  kd_phi   =  0.05;
yaw_damper_gain = 0.065;
yaw_damper_pole = -2.0;

%% Cargar modelo
data = load(model_path);
plant_full = ss(data.linmodel);
plant_lat  = ss(data.latmod);
plant_long = ss(data.longmod);

full_inputs  = {'throttle','elevator','rudder','l_aileron','r_aileron', ...
    'l_flap','r_flap','aileron'};
full_outputs = {'V','beta','alpha','h','phi','theta','psi','p','q','r', ...
    'gamma','ax','ay','az'};
plant_full.InputName  = full_inputs;
plant_full.OutputName = full_outputs;

%% Canales SISO para los lazos independientes
G_theta = minreal(plant_long('theta','elevator'), minreal_tol);
G_phi   = minreal(plant_lat('phi','aileron'), minreal_tol);
G_q     = minreal(plant_long('q','elevator'), minreal_tol);
G_p     = minreal(plant_lat('p','aileron'), minreal_tol);

[Gtheta_A, Gtheta_B, Gtheta_C, Gtheta_D] = ssdata(G_theta);
[Gphi_A,   Gphi_B,   Gphi_C,   Gphi_D]   = ssdata(G_phi);
[Gq_A,     Gq_B,     Gq_C,     Gq_D]     = ssdata(G_q);
[Gp_A,     Gp_B,     Gp_C,     Gp_D]     = ssdata(G_p);

n_theta = size(Gtheta_A, 1);
n_phi   = size(Gphi_A, 1);
n_q     = size(Gq_A, 1);
n_p     = size(Gp_A, 1);

%% Controladores H-inf
s = tf('s');

W1_theta_low_gain = 80;
W2_theta_gain = 5.0;
W1_theta = makeweight(W1_theta_low_gain, wb, 0.05);
W2_theta = W2_theta_gain;
W3_theta = makeweight(0.005, wp, 15);

W1_phi_low_gain = 220;
W1_phi = makeweight(W1_phi_low_gain, wb, 0.05);
W2_phi_low_gain = 4.0;
W2_phi_high_gain = 12.0;
W2_phi_cross = wp;
wz_phi = W2_phi_cross;
wp_phi = W2_phi_cross*W2_phi_high_gain/W2_phi_low_gain;
W2_phi = W2_phi_low_gain*(s/wz_phi + 1)/(s/wp_phi + 1);
W2_phi = minreal(W2_phi, minreal_tol);
W3_phi = makeweight(0.005, wp, 15);

[K_theta_hinf, ~, ~, ~] = mixsyn(G_theta, W1_theta, W2_theta, W3_theta);
K_theta_hinf = minreal(ss(K_theta_hinf), minreal_tol);
[K_phi_hinf, ~, ~, ~] = mixsyn(G_phi, W1_phi, W2_phi, W3_phi);
K_phi_hinf = minreal(ss(K_phi_hinf), minreal_tol);

[Ktheta_A, Ktheta_B, Ktheta_C, Ktheta_D] = ssdata(K_theta_hinf);
[Kphi_A,   Kphi_B,   Kphi_C,   Kphi_D]   = ssdata(K_phi_hinf);

n_Ktheta = size(Ktheta_A, 1);
n_Kphi   = size(Kphi_A, 1);

%% Yaw damper (washout)
yaw_tf = zpk(0, yaw_damper_pole, yaw_damper_gain);
[Kyaw_A, Kyaw_B, Kyaw_C, Kyaw_D] = ssdata(ss(yaw_tf));

%% PI como transfer function (para PID blocks)
CAS_PI_theta = kp_theta + ki_theta/s;
CAS_PI_phi   = kp_phi   + ki_phi/s;
[cas_pi_theta_num, cas_pi_theta_den] = tfdata(CAS_PI_theta, 'v');
[cas_pi_phi_num,   cas_pi_phi_den]   = tfdata(CAS_PI_phi, 'v');
sas_D_q = kd_theta;
sas_D_p = kd_phi;

%% Parametros de escenario (configurables por el usuario antes de simular)
theta_ref_amp = deg2rad(10);
phi_ref_amp   = deg2rad(10);
t_step  = 1.0;
t_final = 12.0;

noise_power_long  = 1e-4;
noise_power_lat   = 1e-3;
noise_sample_time = 0.005;
noise_enabled     = 1;

dist_amp     = deg2rad(1.0);
dist_freq    = 2*pi*6;
dist_enabled = 1;

fprintf('init_simulink_v2.m: variables cargadas para taller1_simulink_v2.slx\n');
fprintf('  Plantas: G_theta(%d), G_phi(%d), G_q(%d), G_p(%d)\n', ...
    n_theta, n_phi, n_q, n_p);
fprintf('  Controladores Hinf: K_theta(%d), K_phi(%d)\n', n_Ktheta, n_Kphi);
fprintf('  Parametros: umax=%.1f deg, ref_theta=%.1f deg, ref_phi=%.1f deg\n', ...
    rad2deg(umax), rad2deg(theta_ref_amp), rad2deg(phi_ref_amp));
