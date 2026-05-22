%% Inicializacion de variables para taller1_simulink.slx
% Carga todo lo necesario en el base workspace para que el modelo Simulink
% pueda ejecutarse directamente.

project_dir = fileparts(mfilename('fullpath'));
repo_dir    = fileparts(project_dir);

model_path = fullfile(repo_dir, 'drive', 'TDC', '02. TAREAS', ...
    'T1', 'modelo_lin.mat');
if ~isfile(model_path)
    error('No se encontro modelo_lin.mat en: %s', model_path);
end

% --- Especificaciones ---
control_limit_deg = 30;
umax = deg2rad(control_limit_deg);
wb   = 2*pi*8;
wp   = 2*pi*6;
noise_power_long = 1e-4;
noise_power_lat  = 1e-3;
minreal_tol = 1e-6;

% --- Ganancias SAS/CAS ---
kp_theta = -1.00;  ki_theta = -0.30;  kd_theta = -0.20;
kp_phi   = -0.35;  ki_phi   = -0.18;  kd_phi   =  0.05;
yaw_damper_gain = 0.065;
yaw_damper_pole = -2.0;

% --- Cargar y preparar la planta ---
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

% Matrices de la planta completa
[A_full, B_full, C_full, ~] = ssdata(plant_full);
idx_theta = find(strcmp(full_outputs, 'theta'));
idx_phi   = find(strcmp(full_outputs, 'phi'));
idx_p     = find(strcmp(full_outputs, 'p'));
idx_q     = find(strcmp(full_outputs, 'q'));
idx_r     = find(strcmp(full_outputs, 'r'));
C_sim = C_full([idx_theta idx_phi idx_p idx_q idx_r], :);
D_sim = zeros(5, size(B_full, 2));

% --- Canales SISO ---
G_theta = minreal(plant_long('theta','elevator'), minreal_tol);
G_phi   = minreal(plant_lat('phi','aileron'), minreal_tol);

% --- Controladores H-inf ---
s = tf('s');

W1_theta = makeweight(80, wb, 0.05);
W2_theta = 1.0;
W3_theta = makeweight(0.005, wp, 15);

W1_phi = makeweight(220, wb, 0.05);
wz_phi = wp; wp_phi = wz_phi*3.20/0.80;
W2_phi = 0.80*(s/wz_phi + 1)/(s/wp_phi + 1);
W2_phi = minreal(W2_phi, minreal_tol);
W3_phi = makeweight(0.005, wp, 15);

[K_theta_hinf, ~, ~, ~] = mixsyn(G_theta, W1_theta, W2_theta, W3_theta);
K_theta_hinf = minreal(ss(K_theta_hinf), minreal_tol);
[K_phi_hinf, ~, ~, ~] = mixsyn(G_phi, W1_phi, W2_phi, W3_phi);
K_phi_hinf = minreal(ss(K_phi_hinf), minreal_tol);

[Ktheta_A, Ktheta_B, Ktheta_C, Ktheta_D] = ssdata(K_theta_hinf);
[Kphi_A, Kphi_B, Kphi_C, Kphi_D]         = ssdata(K_phi_hinf);

% --- Yaw damper ---
yaw_tf = zpk(0, yaw_damper_pole, yaw_damper_gain);
[Kyaw_A, Kyaw_B, Kyaw_C, Kyaw_D] = ssdata(ss(yaw_tf));

% --- CAS PI como funcion de transferencia ---
CAS_PI_theta = kp_theta + ki_theta/s;
CAS_PI_phi   = kp_phi   + ki_phi/s;
[cas_pi_theta_num, cas_pi_theta_den] = tfdata(CAS_PI_theta, 'v');
[cas_pi_phi_num,   cas_pi_phi_den]   = tfdata(CAS_PI_phi, 'v');
sas_D_q = kd_theta;
sas_D_p = kd_phi;

% --- Parametros de simulacion ---
theta_ref_step = deg2rad(10);
phi_ref_step   = deg2rad(10);
t_step  = 1.0;
t_final = 12.0;
noise_var_theta    = noise_power_long;
noise_var_phi      = noise_power_lat;
noise_sample_time  = 0.005;
dist_amp  = deg2rad(1.0);
dist_freq = 2*pi*6;

% 1 = H-inf, 0 = SAS/CAS
control_mode = 1;

fprintf('init_simulink.m: variables cargadas para taller1_simulink.slx\n');
