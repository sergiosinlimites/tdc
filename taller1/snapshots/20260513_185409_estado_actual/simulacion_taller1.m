function sim_results = simulacion_taller1(plant, pid_data, hinf_data, cfg)
%SIMULACION_TALLER1 Simula PID y H_inf sobre la planta acoplada linmodel.

% Paso 1: fijar semilla para que el ruido sea reproducible.
rng(cfg.sim.rng_seed);

% Paso 2: correr todos los escenarios con PID.
sim_results.pid = run_family('pid', plant, pid_data, hinf_data, cfg);

% Paso 3: reiniciar semilla para que H_inf vea el mismo ruido que PID.
rng(cfg.sim.rng_seed);
sim_results.hinf = run_family('hinf', plant, pid_data, hinf_data, cfg);

% Paso 4: condensar resultados en una tabla estructurada.
sim_results.summary = summarize_results(sim_results, cfg);
end

function family = run_family(controller_name, plant, pid_data, hinf_data, cfg)
%RUN_FAMILY Ejecuta todos los escenarios para un controlador.
for k = 1:numel(cfg.scenarios)
    family(k) = simulate_case(controller_name, plant, pid_data, ...
        hinf_data, cfg, cfg.scenarios(k)); %#ok<AGROW>
end
end

function sim = simulate_case(controller_name, plant, pid_data, hinf_data, cfg, scenario)
%SIMULATE_CASE Prepara y resuelve una simulacion de lazo cerrado.

% Paso 1: extraer matrices de la planta acoplada.
[A, B, C, ~] = ssdata(plant.full);
nplant = size(A, 1);

% Paso 2: preparar indices de senales, tiempo y ruido del escenario.
idx = make_indices(plant.full);
t_grid = (0:cfg.sim.dt:cfg.sim.t_final).';
noise = make_noise(t_grid, cfg, scenario);

% Paso 3: convertir el controlador elegido a una forma de estados.
ctrl = make_controller(controller_name, pid_data, hinf_data, cfg);
nctrl = ctrl.nx;
z0 = zeros(nplant + nctrl, 1);

% Paso 4: integrar planta y controlador juntos con ode45.
ode = @(t, z) closed_loop_ode(t, z, A, B, C, idx, ctrl, cfg, scenario, ...
    t_grid, noise);
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t, z] = ode45(ode, t_grid, z0, opts);

% Paso 5: reconstruir salidas, controles, errores y metricas.
sim = reconstruct_simulation(t, z, A, B, C, idx, ctrl, cfg, scenario, ...
    t_grid, noise, controller_name);
end

function dz = closed_loop_ode(t, z, A, B, C, idx, ctrl, cfg, scenario, ...
    t_grid, noise)
%CLOSED_LOOP_ODE Define la dinamica conjunta planta-controlador.
nplant = size(A, 1);
x = z(1:nplant);
xc = z(nplant+1:end);

% Paso 1: medir la planta y contaminar mediciones si el escenario lo pide.
y = C*x;
n = sample_noise(t, t_grid, noise);
signals = measured_signals(t, y, n, idx, cfg, scenario);

% Paso 2: calcular control, saturarlo y armar entrada completa de linmodel.
[u_sat, ~, xcdot] = controller_output(signals, xc, ctrl, cfg);
u_plant = full_input_vector(u_sat, t, cfg, scenario);

% Paso 3: propagar estados de planta y controlador.
xdot = A*x + B*u_plant;
dz = [xdot; xcdot];
end

function sim = reconstruct_simulation(t, z, A, B, C, idx, ctrl, cfg, ...
    scenario, t_grid, noise, controller_name)
%RECONSTRUCT_SIMULATION Convierte estados integrados en senales utiles.
nplant = size(A, 1);
nt = numel(t);
x = z(:, 1:nplant);
xc = z(:, nplant+1:end);

y = zeros(nt, size(C, 1));
y_meas = y;
u_sat = zeros(nt, 3);
u_raw = zeros(nt, 3);
u_full = zeros(nt, size(B, 2));
refs = zeros(nt, 2);
err = zeros(nt, 2);
noise_used = zeros(nt, 5);

% Paso 1: recorrer la solucion para recalcular salidas, errores y control.
for k = 1:nt
    y(k, :) = (C*x(k, :).').';
    n = sample_noise(t(k), t_grid, noise);
    signals = measured_signals(t(k), y(k, :).', n, idx, cfg, scenario);
    [u_sat_k, u_raw_k, ~] = controller_output(signals, ...
        xc(k, :).', ctrl, cfg);
    u_sat(k, :) = u_sat_k.';
    u_raw(k, :) = u_raw_k.';
    u_full(k, :) = full_input_vector(u_sat(k, :).', t(k), cfg, scenario).';
    refs(k, :) = [signals.theta_ref, signals.phi_ref];
    err(k, :) = [signals.e_theta, signals.e_phi];
    y_meas(k, :) = y(k, :);
    y_meas(k, idx.theta) = signals.theta_meas;
    y_meas(k, idx.phi) = signals.phi_meas;
    y_meas(k, idx.q) = signals.q_meas;
    y_meas(k, idx.p) = signals.p_meas;
    y_meas(k, idx.r) = signals.r_meas;
    noise_used(k, :) = [n.theta, n.phi, n.q, n.p, n.r];
end

% Paso 2: extraer salidas principales y convertir a grados para graficas.
theta = y(:, idx.theta);
phi = y(:, idx.phi);

% Paso 3: empaquetar toda la informacion de la simulacion.
sim.name = scenario.name;
sim.controller = controller_name;
sim.t = t;
sim.x = x;
sim.controller_state = xc;
sim.y = y;
sim.y_meas = y_meas;
sim.refs = refs;
sim.error = err;
sim.u_sat = u_sat;
sim.u_raw = u_raw;
sim.u_full = u_full;
sim.noise = noise_used;
sim.theta_deg = rad2deg(theta);
sim.phi_deg = rad2deg(phi);
sim.theta_ref_deg = rad2deg(refs(:, 1));
sim.phi_ref_deg = rad2deg(refs(:, 2));
sim.elevator_deg = rad2deg(u_sat(:, 1));
sim.aileron_deg = rad2deg(u_sat(:, 2));
sim.rudder_deg = rad2deg(u_sat(:, 3));
sim.metrics = metrics_for_case(sim, cfg);
end

function idx = make_indices(sys)
%MAKE_INDICES Encuentra indices por nombre para evitar numeros magicos.
idx.theta = find(strcmp(sys.OutputName, 'theta'));
idx.phi = find(strcmp(sys.OutputName, 'phi'));
idx.q = find(strcmp(sys.OutputName, 'q'));
idx.p = find(strcmp(sys.OutputName, 'p'));
idx.r = find(strcmp(sys.OutputName, 'r'));

idx.u_throttle = find(strcmp(sys.InputName, 'throttle'));
idx.u_elevator = find(strcmp(sys.InputName, 'elevator'));
idx.u_rudder = find(strcmp(sys.InputName, 'rudder'));
idx.u_aileron = find(strcmp(sys.InputName, 'aileron'));
end

function noise = make_noise(t, cfg, scenario)
%MAKE_NOISE Genera ruido blanco discreto con la potencia del enunciado.
nt = numel(t);
if scenario.noise_enabled
    noise.theta = sqrt(cfg.spec.noise_power_long)*randn(nt, 1);
    noise.q = sqrt(cfg.spec.noise_power_long)*randn(nt, 1);
    noise.phi = sqrt(cfg.spec.noise_power_lat)*randn(nt, 1);
    noise.p = sqrt(cfg.spec.noise_power_lat)*randn(nt, 1);
    noise.r = sqrt(cfg.spec.noise_power_lat)*randn(nt, 1);
else
    noise.theta = zeros(nt, 1);
    noise.q = zeros(nt, 1);
    noise.phi = zeros(nt, 1);
    noise.p = zeros(nt, 1);
    noise.r = zeros(nt, 1);
end
end

function n = sample_noise(t, t_grid, noise)
%SAMPLE_NOISE Toma una muestra de ruido compatible con el tiempo continuo.
k = min(max(1, floor((t - t_grid(1))/(t_grid(2) - t_grid(1))) + 1), ...
    numel(t_grid));
n.theta = noise.theta(k);
n.q = noise.q(k);
n.phi = noise.phi(k);
n.p = noise.p(k);
n.r = noise.r(k);
end

function signals = measured_signals(t, y, n, idx, cfg, scenario)
%MEASURED_SIGNALS Construye referencias, mediciones y errores.
if t >= cfg.sim.t_step
    theta_ref = scenario.theta_ref_rad;
    phi_ref = scenario.phi_ref_rad;
else
    theta_ref = 0;
    phi_ref = 0;
end

signals.theta = y(idx.theta);
signals.phi = y(idx.phi);
signals.q = y(idx.q);
signals.p = y(idx.p);
signals.r = y(idx.r);

signals.theta_meas = signals.theta + n.theta;
signals.phi_meas = signals.phi + n.phi;
signals.q_meas = signals.q + n.q;
signals.p_meas = signals.p + n.p;
signals.r_meas = signals.r + n.r;

signals.theta_ref = theta_ref;
signals.phi_ref = phi_ref;
signals.e_theta = theta_ref - signals.theta_meas;
signals.e_phi = phi_ref - signals.phi_meas;
end

function ctrl = make_controller(controller_name, pid_data, hinf_data, cfg)
%MAKE_CONTROLLER Convierte PID o H_inf a matrices listas para simular.
switch controller_name
    case 'pid'
        % PID: dos integradores CAS y un estado para el washout de yaw.
        ctrl.type = 'pid';
        ctrl.nx = 3;
        ctrl.pid = pid_data.gains;
        yaw_tf = zpk(0, cfg.pid.yaw_damper_pole, ...
            cfg.pid.yaw_damper_gain);
        [ctrl.yaw.A, ctrl.yaw.B, ctrl.yaw.C, ctrl.yaw.D] = ...
            ssdata(ss(yaw_tf));
    case 'hinf'
        % H_inf: estados dinamicos de K_theta, K_phi y yaw damper.
        ctrl.type = 'hinf';
        [ctrl.theta.A, ctrl.theta.B, ctrl.theta.C, ctrl.theta.D] = ...
            ssdata(ss(hinf_data.K_theta));
        [ctrl.phi.A, ctrl.phi.B, ctrl.phi.C, ctrl.phi.D] = ...
            ssdata(ss(hinf_data.K_phi));
        yaw_tf = zpk(0, cfg.pid.yaw_damper_pole, ...
            cfg.pid.yaw_damper_gain);
        [ctrl.yaw.A, ctrl.yaw.B, ctrl.yaw.C, ctrl.yaw.D] = ...
            ssdata(ss(yaw_tf));
        ctrl.n_theta = size(ctrl.theta.A, 1);
        ctrl.n_phi = size(ctrl.phi.A, 1);
        ctrl.nx = ctrl.n_theta + ctrl.n_phi + 1;
    otherwise
        error('Controlador no soportado: %s', controller_name);
end
end

function [u_sat, u_raw, xcdot] = controller_output(signals, xc, ctrl, cfg)
%CONTROLLER_OUTPUT Calcula comando crudo, saturado y dinamica interna.
limit = cfg.spec.control_limit_rad;

switch ctrl.type
    case 'pid'
        % Caso clasico: PI de angulo con anti-windup y amortiguamiento.
        xi_theta = xc(1);
        xi_phi = xc(2);
        x_yaw = xc(3);

        u_elev_raw = ctrl.pid.kp_theta*signals.e_theta + ...
            ctrl.pid.ki_theta*xi_theta - ctrl.pid.kd_theta*signals.q_meas;
        u_ail_raw = ctrl.pid.kp_phi*signals.e_phi + ...
            ctrl.pid.ki_phi*xi_phi - ctrl.pid.kd_phi*signals.p_meas;
        u_rud_raw = ctrl.yaw.C*x_yaw + ctrl.yaw.D*signals.r_meas;

        u_raw = [u_elev_raw; u_ail_raw; u_rud_raw];
        u_sat = saturate(u_raw, limit);

        xi_theta_dot = signals.e_theta + ...
            ctrl.pid.antiwindup*(u_sat(1) - u_raw(1));
        xi_phi_dot = signals.e_phi + ...
            ctrl.pid.antiwindup*(u_sat(2) - u_raw(2));
        x_yaw_dot = ctrl.yaw.A*x_yaw + ctrl.yaw.B*signals.r_meas;
        xcdot = [xi_theta_dot; xi_phi_dot; x_yaw_dot];

    case 'hinf'
        % Caso robusto: salida de los controladores dinamicos H_inf.
        xt = xc(1:ctrl.n_theta);
        xp = xc(ctrl.n_theta+1:ctrl.n_theta+ctrl.n_phi);
        xy = xc(end);

        u_elev_raw = ctrl.theta.C*xt + ctrl.theta.D*signals.e_theta;
        u_ail_raw = ctrl.phi.C*xp + ctrl.phi.D*signals.e_phi;
        u_rud_raw = ctrl.yaw.C*xy + ctrl.yaw.D*signals.r_meas;

        u_raw = [u_elev_raw; u_ail_raw; u_rud_raw];
        u_sat = saturate(u_raw, limit);

        xtdot = ctrl.theta.A*xt + ctrl.theta.B*signals.e_theta;
        xpdot = ctrl.phi.A*xp + ctrl.phi.B*signals.e_phi;
        xydot = ctrl.yaw.A*xy + ctrl.yaw.B*signals.r_meas;
        xcdot = [xtdot; xpdot; xydot];
end
end

function u_sat = saturate(u, limit)
%SATURATE Aplica limite simetrico de actuador en radianes.
u_sat = min(max(u, -limit), limit);
end

function u = full_input_vector(u_sat, t, cfg, scenario)
%FULL_INPUT_VECTOR Inserta elevator, rudder y aileron en las 8 entradas.
u = zeros(8, 1);
dist = input_disturbance(t, cfg, scenario);
u(2) = u_sat(1) + dist(1);
u(3) = u_sat(3) + dist(3);
u(8) = u_sat(2) + dist(2);
end

function d = input_disturbance(t, cfg, scenario)
%INPUT_DISTURBANCE Perturbacion sinusoidal de entrada hasta 6 Hz.
d = zeros(3, 1);
if scenario.disturbance_enabled && t >= cfg.sim.disturbance_start
    w = 2*pi*cfg.spec.perturbation_hz;
    a = cfg.sim.disturbance_amp_rad;
    d = a*[sin(w*t); sin(w*t + pi/4); sin(w*t + pi/2)];
end
end

function metrics = metrics_for_case(sim, cfg)
%METRICS_FOR_CASE Calcula errores RMS, maximos y fraccion de saturacion.
limit = cfg.spec.control_limit_deg;
post = sim.t >= cfg.sim.t_step;
tail = sim.t >= max(cfg.sim.t_final - 2, cfg.sim.t_step);

metrics.max_abs_theta_deg = max(abs(sim.theta_deg));
metrics.max_abs_phi_deg = max(abs(sim.phi_deg));
metrics.max_abs_elevator_deg = max(abs(sim.elevator_deg));
metrics.max_abs_aileron_deg = max(abs(sim.aileron_deg));
metrics.max_abs_rudder_deg = max(abs(sim.rudder_deg));
metrics.theta_final_error_deg = mean(sim.theta_ref_deg(tail) - sim.theta_deg(tail));
metrics.phi_final_error_deg = mean(sim.phi_ref_deg(tail) - sim.phi_deg(tail));
metrics.theta_rms_error_deg = rms(sim.theta_ref_deg(post) - sim.theta_deg(post));
metrics.phi_rms_error_deg = rms(sim.phi_ref_deg(post) - sim.phi_deg(post));
metrics.sat_fraction = mean(any(abs(sim.u_sat) >= deg2rad(limit)*0.999, 2));
end

function summary = summarize_results(sim_results, cfg)
%SUMMARIZE_RESULTS Arma un resumen compacto para imprimir en consola.
controllers = {'pid', 'hinf'};
row = 0;
for c = 1:numel(controllers)
    family = sim_results.(controllers{c});
    for k = 1:numel(family)
        row = row + 1;
        summary(row).controller = controllers{c}; %#ok<AGROW>
        summary(row).scenario = family(k).name; %#ok<AGROW>
        summary(row).theta_rms_error_deg = family(k).metrics.theta_rms_error_deg; %#ok<AGROW>
        summary(row).phi_rms_error_deg = family(k).metrics.phi_rms_error_deg; %#ok<AGROW>
        summary(row).max_abs_elevator_deg = family(k).metrics.max_abs_elevator_deg; %#ok<AGROW>
        summary(row).max_abs_aileron_deg = family(k).metrics.max_abs_aileron_deg; %#ok<AGROW>
        summary(row).sat_fraction = family(k).metrics.sat_fraction; %#ok<AGROW>
    end
end

if isempty(cfg.results_dir)
    return;
end
end
