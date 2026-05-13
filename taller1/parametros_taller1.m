function cfg = parametros_taller1(project_dir)
%PARAMETROS_TALLER1 Configuracion reproducible para el Taller 1.
%
% Todas las magnitudes angulares internas estan en radianes. En el informe y
% las graficas se convierten a grados cuando sea mas claro.

if nargin < 1 || isempty(project_dir)
    project_dir = fileparts(mfilename('fullpath'));
end

% Paso 1: ubicar carpetas del proyecto, resultados y figuras.
repo_dir = fileparts(project_dir);

cfg.project_dir = project_dir;
cfg.repo_dir = repo_dir;
cfg.data.model_path = fullfile(repo_dir, 'drive', 'TDC', '02. TAREAS', ...
    'T1', 'modelo_lin.mat');
cfg.figures_dir = fullfile(project_dir, 'figures');
cfg.results_dir = fullfile(project_dir, 'results');

cfg.minreal_tol = 1e-6;

% Paso 2: declarar las especificaciones del enunciado. MATLAB trabaja las
% frecuencias continuas en rad/s; por eso 8 Hz y 6 Hz se convierten con
% omega = 2*pi*f.
cfg.spec.ref_max_deg = 40;
cfg.spec.control_limit_deg = 30;
cfg.spec.control_limit_rad = deg2rad(cfg.spec.control_limit_deg);
cfg.spec.bandwidth_hz = 8;
cfg.spec.perturbation_hz = 6;
cfg.spec.wb = 2*pi*cfg.spec.bandwidth_hz;
cfg.spec.wp = 2*pi*cfg.spec.perturbation_hz;
cfg.spec.noise_power_long = 1e-4;
cfg.spec.noise_power_lat = 1e-3;

% Paso 3: fijar nombres de entradas y salidas del modelo completo para que
% los indices no dependan del orden visual del archivo .mat.
cfg.signals.full_inputs = {'throttle', 'elevator', 'rudder', ...
    'l_aileron', 'r_aileron', 'l_flap', 'r_flap', 'aileron'};
cfg.signals.full_outputs = {'V', 'beta', 'alpha', 'h', 'phi', 'theta', ...
    'psi', 'p', 'q', 'r', 'gamma', 'ax', 'ay', 'az'};

% Paso 4: cargar las ganancias clasicas del controlador baseline del paquete
% UAV_SIM_AEM. La fuente es:
% drive/TDC/04. Otros Recursos/UAV_SIM_AEM/Simulation_Lin/Controllers/
% baseline_gains.m
% En ese archivo aparecen como kp_PT, ki_PT, kp_PD para pitch/theta y
% kp_RT, ki_RT, kp_RD para roll/phi. Aqui se renombran para que coincidan
% con las variables del taller.
cfg.pid.kp_theta = -0.84;
cfg.pid.ki_theta = -0.23;
cfg.pid.kd_theta = -0.08;
cfg.pid.kp_phi = -0.52;
cfg.pid.ki_phi = -0.20;
cfg.pid.kd_phi = -0.07;
cfg.pid.derivative_tau = 0.03;
cfg.pid.antiwindup = 8.0;

cfg.pid.yaw_damper_gain = 0.065;
cfg.pid.yaw_damper_pole = -2.0;

% Paso 5: pesos iniciales H_inf. Se escogen suaves para obtener una sintesis
% factible y luego se revisan contra las especificaciones.
cfg.hinf.W1_low_gain = 5;
cfg.hinf.W1_high_gain = 0.05;
cfg.hinf.W2_gain = 0.15;
cfg.hinf.W3_low_gain = 0.02;
cfg.hinf.W3_high_gain = 5;

% Paso 4: parametros de simulacion temporal y escenarios de referencia.
cfg.sim.t_final = 12.0;
cfg.sim.dt = 0.005;
cfg.sim.t_step = 1.0;
cfg.sim.disturbance_start = 2.0;
cfg.sim.disturbance_amp_rad = deg2rad(1.0);
cfg.sim.rng_seed = 7;

cfg.scenarios = make_scenarios();
end

function scenarios = make_scenarios()
%MAKE_SCENARIOS Define los casos que se simulan para PID y H_inf.
base_noise = false;
base_dist = false;

scenarios(1) = scenario('theta_10', 10, 0, base_noise, base_dist);
scenarios(2) = scenario('theta_minus_10', -10, 0, base_noise, base_dist);
scenarios(3) = scenario('phi_10', 0, 10, base_noise, base_dist);
scenarios(4) = scenario('phi_minus_10', 0, -10, base_noise, base_dist);
scenarios(5) = scenario('theta_phi_10', 10, 10, base_noise, base_dist);
scenarios(6) = scenario('theta_40', 40, 0, base_noise, base_dist);
scenarios(7) = scenario('phi_40', 0, 40, base_noise, base_dist);
scenarios(8) = scenario('noise_disturbance', 10, 10, true, true);
end

function s = scenario(name, theta_deg, phi_deg, noise_enabled, disturbance_enabled)
%SCENARIO Empaqueta una referencia y sus banderas de ruido/perturbacion.
s.name = name;
s.theta_ref_deg = theta_deg;
s.phi_ref_deg = phi_deg;
s.theta_ref_rad = deg2rad(theta_deg);
s.phi_ref_rad = deg2rad(phi_deg);
s.noise_enabled = noise_enabled;
s.disturbance_enabled = disturbance_enabled;
end
