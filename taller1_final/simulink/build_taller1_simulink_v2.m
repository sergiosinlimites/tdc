%% Construye taller1_simulink_v2.slx desde cero
% Modelo con 4 lazos independientes (PID theta, PID phi, Hinf theta, Hinf phi),
% scopes estrategicos y escenarios configurables (saturacion, ruido, perturbaciones).

clear; clc;

project_dir = fileparts(mfilename('fullpath'));
addpath(project_dir);
run(fullfile(project_dir, 'init_simulink_v2.m'));

mdl = 'taller1_simulink_v2';
slx_path = fullfile(project_dir, [mdl '.slx']);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
if exist(slx_path, 'file')
    delete(slx_path);
end

new_system(mdl);
open_system(mdl);
set_param(mdl, 'InitFcn', ['run(''' fullfile(project_dir, ...
    'init_simulink_v2.m') ''')']);
set_param(mdl, 'StopTime', 't_final', 'Solver', 'ode45', ...
    'SaveFormat', 'Dataset', 'SignalLogging', 'on');

%% ========================================================================
%  SUBSISTEMA 1: Lazo_PID_theta
%  ========================================================================
sub_pid_theta = [mdl '/Lazo_PID_theta'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_pid_theta);
delete_line(sub_pid_theta, 'In1/1', 'Out1/1');
delete_block([sub_pid_theta '/In1']);
delete_block([sub_pid_theta '/Out1']);

build_pid_loop(sub_pid_theta, 'theta', ...
    'Gtheta_A', 'Gtheta_B', 'Gtheta_C', 'Gtheta_D', 'n_theta', ...
    'Gq_A', 'Gq_B', 'Gq_C', 'Gq_D', 'n_q', ...
    'cas_pi_theta_num', 'cas_pi_theta_den', 'sas_D_q', ...
    'theta_ref_amp', 'umax');

%% ========================================================================
%  SUBSISTEMA 2: Lazo_PID_phi
%  ========================================================================
sub_pid_phi = [mdl '/Lazo_PID_phi'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_pid_phi);
delete_line(sub_pid_phi, 'In1/1', 'Out1/1');
delete_block([sub_pid_phi '/In1']);
delete_block([sub_pid_phi '/Out1']);

build_pid_loop(sub_pid_phi, 'phi', ...
    'Gphi_A', 'Gphi_B', 'Gphi_C', 'Gphi_D', 'n_phi', ...
    'Gp_A', 'Gp_B', 'Gp_C', 'Gp_D', 'n_p', ...
    'cas_pi_phi_num', 'cas_pi_phi_den', 'sas_D_p', ...
    'phi_ref_amp', 'umax');

add_yaw_damper(sub_pid_phi, 'PID');

%% ========================================================================
%  SUBSISTEMA 3: Lazo_Hinf_theta
%  ========================================================================
sub_hinf_theta = [mdl '/Lazo_Hinf_theta'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_hinf_theta);
delete_line(sub_hinf_theta, 'In1/1', 'Out1/1');
delete_block([sub_hinf_theta '/In1']);
delete_block([sub_hinf_theta '/Out1']);

build_hinf_loop(sub_hinf_theta, 'theta', ...
    'Gtheta_A', 'Gtheta_B', 'Gtheta_C', 'Gtheta_D', 'n_theta', ...
    'Ktheta_A', 'Ktheta_B', 'Ktheta_C', 'Ktheta_D', 'n_Ktheta', ...
    'theta_ref_amp', 'umax');

%% ========================================================================
%  SUBSISTEMA 4: Lazo_Hinf_phi
%  ========================================================================
sub_hinf_phi = [mdl '/Lazo_Hinf_phi'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_hinf_phi);
delete_line(sub_hinf_phi, 'In1/1', 'Out1/1');
delete_block([sub_hinf_phi '/In1']);
delete_block([sub_hinf_phi '/Out1']);

build_hinf_loop(sub_hinf_phi, 'phi', ...
    'Gphi_A', 'Gphi_B', 'Gphi_C', 'Gphi_D', 'n_phi', ...
    'Kphi_A', 'Kphi_B', 'Kphi_C', 'Kphi_D', 'n_Kphi', ...
    'phi_ref_amp', 'umax');

add_yaw_damper(sub_hinf_phi, 'Hinf');

%% ========================================================================
%  SCOPES DE COMPARACION (top-level)
%  ========================================================================

% Scopes comparativos en el nivel raiz
add_block('simulink/Sinks/Scope', [mdl '/Comparar_theta'], ...
    'NumInputPorts', '3', 'Position', [650 30 700 80]);
add_block('simulink/Sinks/Scope', [mdl '/Comparar_phi'], ...
    'NumInputPorts', '3', 'Position', [650 130 700 180]);
add_block('simulink/Sinks/Scope', [mdl '/Comparar_control'], ...
    'NumInputPorts', '4', 'Position', [650 230 700 290]);

% Referencia compartida para comparacion
add_block('simulink/Sources/Step', [mdl '/theta_ref_cmp'], ...
    'Time', 't_step', 'Before', '0', 'After', 'theta_ref_amp', ...
    'Position', [500 35 540 55]);
add_block('simulink/Sources/Step', [mdl '/phi_ref_cmp'], ...
    'Time', 't_step', 'Before', '0', 'After', 'phi_ref_amp', ...
    'Position', [500 135 540 155]);

% Conexiones de comparacion
add_line(mdl, 'theta_ref_cmp/1', 'Comparar_theta/1', 'autorouting', 'on');
add_line(mdl, 'Lazo_PID_theta/1', 'Comparar_theta/2', 'autorouting', 'on');
add_line(mdl, 'Lazo_Hinf_theta/1', 'Comparar_theta/3', 'autorouting', 'on');

add_line(mdl, 'phi_ref_cmp/1', 'Comparar_phi/1', 'autorouting', 'on');
add_line(mdl, 'Lazo_PID_phi/1', 'Comparar_phi/2', 'autorouting', 'on');
add_line(mdl, 'Lazo_Hinf_phi/1', 'Comparar_phi/3', 'autorouting', 'on');

add_line(mdl, 'Lazo_PID_theta/2', 'Comparar_control/1', 'autorouting', 'on');
add_line(mdl, 'Lazo_Hinf_theta/2', 'Comparar_control/2', 'autorouting', 'on');
add_line(mdl, 'Lazo_PID_phi/2', 'Comparar_control/3', 'autorouting', 'on');
add_line(mdl, 'Lazo_Hinf_phi/2', 'Comparar_control/4', 'autorouting', 'on');

%% ========================================================================
%  GUARDAR
%  ========================================================================
save_system(mdl, slx_path);
fprintf('\n=== Modelo guardado: %s ===\n', slx_path);
fprintf('Subsistemas: Lazo_PID_theta, Lazo_PID_phi, Lazo_Hinf_theta, Lazo_Hinf_phi\n');
fprintf('Scopes comparativos: Comparar_theta, Comparar_phi, Comparar_control\n');

%% ========================================================================
%  FUNCIONES DE CONSTRUCCION
%  ========================================================================

function build_pid_loop(sub, axis_name, ...
        G_A, G_B, G_C, G_D, n_G, ...
        Grate_A, Grate_B, Grate_C, Grate_D, n_Grate, ...
        pi_num, pi_den, sas_gain, ...
        ref_amp, sat_limit)

    p = @(x,y,w,h) [x y x+w y+h];
    if strcmp(axis_name, 'theta')
        noise_var = 'noise_power_long*noise_enabled';
    else
        noise_var = 'noise_power_lat*noise_enabled';
    end

    % --- Referencia ---
    add_block('simulink/Sources/Step', [sub '/ref'], ...
        'Time', 't_step', 'Before', '0', 'After', ref_amp, ...
        'Position', p(30, 80, 40, 30));

    % --- Suma error ---
    add_block('simulink/Math Operations/Sum', [sub '/error_sum'], ...
        'Inputs', '+-', 'Position', p(120, 85, 25, 25));

    % --- PI ---
    add_block('simulink/Continuous/Transfer Fcn', [sub '/PI'], ...
        'Numerator', pi_num, 'Denominator', pi_den, ...
        'Position', p(180, 80, 80, 40));

    % --- Planta rate (para SAS) ---
    add_block('simulink/Continuous/State-Space', [sub '/G_rate'], ...
        'A', Grate_A, 'B', Grate_B, 'C', Grate_C, 'D', Grate_D, ...
        'InitialCondition', ['zeros(' n_Grate ',1)'], ...
        'Position', p(580, 200, 80, 40));

    % --- Ganancia D (SAS) ---
    add_block('simulink/Math Operations/Gain', [sub '/D_sas'], ...
        'Gain', sas_gain, 'Position', p(700, 205, 50, 30));

    % --- Suma PI - D ---
    add_block('simulink/Math Operations/Sum', [sub '/pid_sum'], ...
        'Inputs', '+-', 'Position', p(300, 85, 25, 25));

    % --- Saturacion ---
    add_block('simulink/Discontinuities/Saturation', [sub '/sat'], ...
        'UpperLimit', sat_limit, 'LowerLimit', ['-' sat_limit], ...
        'Position', p(360, 82, 50, 30));

    % --- Perturbacion de entrada ---
    add_block('simulink/Sources/Sine Wave', [sub '/dist_input'], ...
        'Amplitude', 'dist_amp*dist_enabled', ...
        'Frequency', 'dist_freq', ...
        'Position', p(360, 145, 50, 30));
    add_block('simulink/Math Operations/Sum', [sub '/sum_dist_in'], ...
        'Inputs', '++', 'Position', p(440, 85, 25, 25));

    % --- Planta principal ---
    add_block('simulink/Continuous/State-Space', [sub '/G_plant'], ...
        'A', G_A, 'B', G_B, 'C', G_C, 'D', G_D, ...
        'InitialCondition', ['zeros(' n_G ',1)'], ...
        'Position', p(500, 78, 80, 40));

    % --- Perturbacion de salida ---
    add_block('simulink/Sources/Sine Wave', [sub '/dist_output'], ...
        'Amplitude', 'dist_amp*dist_enabled*0.5', ...
        'Frequency', 'dist_freq*1.5', ...
        'Position', p(600, 145, 50, 30));
    add_block('simulink/Math Operations/Sum', [sub '/sum_dist_out'], ...
        'Inputs', '++', 'Position', p(620, 85, 25, 25));

    % --- Ruido de medicion ---
    add_block('simulink/Sources/Random Number', [sub '/noise'], ...
        'Mean', '0', ...
        'Variance', noise_var, ...
        'SampleTime', 'noise_sample_time', 'Seed', '11', ...
        'Position', p(680, 145, 50, 30));
    add_block('simulink/Math Operations/Sum', [sub '/sum_noise'], ...
        'Inputs', '++', 'Position', p(720, 85, 25, 25));

    % --- Conexiones del lazo ---
    add_line(sub, 'ref/1', 'error_sum/1', 'autorouting', 'on');
    add_line(sub, 'error_sum/1', 'PI/1', 'autorouting', 'on');
    add_line(sub, 'PI/1', 'pid_sum/1', 'autorouting', 'on');
    add_line(sub, 'D_sas/1', 'pid_sum/2', 'autorouting', 'on');
    add_line(sub, 'pid_sum/1', 'sat/1', 'autorouting', 'on');
    add_line(sub, 'sat/1', 'sum_dist_in/1', 'autorouting', 'on');
    add_line(sub, 'dist_input/1', 'sum_dist_in/2', 'autorouting', 'on');
    add_line(sub, 'sum_dist_in/1', 'G_plant/1', 'autorouting', 'on');
    add_line(sub, 'sum_dist_in/1', 'G_rate/1', 'autorouting', 'on');
    add_line(sub, 'G_rate/1', 'D_sas/1', 'autorouting', 'on');
    add_line(sub, 'G_plant/1', 'sum_dist_out/1', 'autorouting', 'on');
    add_line(sub, 'dist_output/1', 'sum_dist_out/2', 'autorouting', 'on');
    add_line(sub, 'sum_dist_out/1', 'sum_noise/1', 'autorouting', 'on');
    add_line(sub, 'noise/1', 'sum_noise/2', 'autorouting', 'on');
    add_line(sub, 'sum_noise/1', 'error_sum/2', 'autorouting', 'on');

    % --- Outports para comparacion (top-level) ---
    add_block('simulink/Ports & Subsystems/Out1', [sub '/y_out'], ...
        'Position', p(900, 82, 30, 14));
    add_block('simulink/Ports & Subsystems/Out1', [sub '/u_out'], ...
        'Position', p(900, 130, 30, 14));
    add_line(sub, 'G_plant/1', 'y_out/1', 'autorouting', 'on');
    add_line(sub, 'sat/1', 'u_out/1', 'autorouting', 'on');

    % --- SCOPES ---
    add_block('simulink/Sinks/Scope', [sub '/Scope_ref_vs_y'], ...
        'NumInputPorts', '2', 'Position', p(820, 50, 40, 40));
    add_line(sub, 'ref/1', 'Scope_ref_vs_y/1', 'autorouting', 'on');
    add_line(sub, 'G_plant/1', 'Scope_ref_vs_y/2', 'autorouting', 'on');

    add_block('simulink/Sinks/Scope', [sub '/Scope_error'], ...
        'NumInputPorts', '1', 'Position', p(820, 110, 40, 40));
    add_line(sub, 'error_sum/1', 'Scope_error/1', 'autorouting', 'on');

    add_block('simulink/Sinks/Scope', [sub '/Scope_control'], ...
        'NumInputPorts', '2', 'Position', p(820, 170, 40, 40));
    add_line(sub, 'pid_sum/1', 'Scope_control/1', 'autorouting', 'on');
    add_line(sub, 'sat/1', 'Scope_control/2', 'autorouting', 'on');

    add_block('simulink/Sinks/Scope', [sub '/Scope_PI_vs_D'], ...
        'NumInputPorts', '2', 'Position', p(820, 230, 40, 40));
    add_line(sub, 'PI/1', 'Scope_PI_vs_D/1', 'autorouting', 'on');
    add_line(sub, 'D_sas/1', 'Scope_PI_vs_D/2', 'autorouting', 'on');

    add_block('simulink/Sinks/Scope', [sub '/Scope_perturbaciones'], ...
        'NumInputPorts', '3', 'Position', p(820, 290, 40, 40));
    add_line(sub, 'dist_input/1', 'Scope_perturbaciones/1', 'autorouting', 'on');
    add_line(sub, 'dist_output/1', 'Scope_perturbaciones/2', 'autorouting', 'on');
    add_line(sub, 'noise/1', 'Scope_perturbaciones/3', 'autorouting', 'on');

    add_block('simulink/Sinks/Scope', [sub '/Scope_rate'], ...
        'NumInputPorts', '1', 'Position', p(820, 350, 40, 40));
    add_line(sub, 'G_rate/1', 'Scope_rate/1', 'autorouting', 'on');
end

function build_hinf_loop(sub, axis_name, ...
        G_A, G_B, G_C, G_D, n_G, ...
        K_A, K_B, K_C, K_D, n_K, ...
        ref_amp, sat_limit)

    p = @(x,y,w,h) [x y x+w y+h];
    if strcmp(axis_name, 'theta')
        noise_var = 'noise_power_long*noise_enabled';
    else
        noise_var = 'noise_power_lat*noise_enabled';
    end

    % --- Referencia ---
    add_block('simulink/Sources/Step', [sub '/ref'], ...
        'Time', 't_step', 'Before', '0', 'After', ref_amp, ...
        'Position', p(30, 80, 40, 30));

    % --- Suma error ---
    add_block('simulink/Math Operations/Sum', [sub '/error_sum'], ...
        'Inputs', '+-', 'Position', p(120, 85, 25, 25));

    % --- Controlador H-inf ---
    add_block('simulink/Continuous/State-Space', [sub '/K_hinf'], ...
        'A', K_A, 'B', K_B, 'C', K_C, 'D', K_D, ...
        'InitialCondition', ['zeros(' n_K ',1)'], ...
        'Position', p(200, 78, 80, 40));

    % --- Saturacion ---
    add_block('simulink/Discontinuities/Saturation', [sub '/sat'], ...
        'UpperLimit', sat_limit, 'LowerLimit', ['-' sat_limit], ...
        'Position', p(320, 82, 50, 30));

    % --- Perturbacion de entrada ---
    add_block('simulink/Sources/Sine Wave', [sub '/dist_input'], ...
        'Amplitude', 'dist_amp*dist_enabled', ...
        'Frequency', 'dist_freq', ...
        'Position', p(320, 145, 50, 30));
    add_block('simulink/Math Operations/Sum', [sub '/sum_dist_in'], ...
        'Inputs', '++', 'Position', p(400, 85, 25, 25));

    % --- Planta ---
    add_block('simulink/Continuous/State-Space', [sub '/G_plant'], ...
        'A', G_A, 'B', G_B, 'C', G_C, 'D', G_D, ...
        'InitialCondition', ['zeros(' n_G ',1)'], ...
        'Position', p(460, 78, 80, 40));

    % --- Perturbacion de salida ---
    add_block('simulink/Sources/Sine Wave', [sub '/dist_output'], ...
        'Amplitude', 'dist_amp*dist_enabled*0.5', ...
        'Frequency', 'dist_freq*1.5', ...
        'Position', p(560, 145, 50, 30));
    add_block('simulink/Math Operations/Sum', [sub '/sum_dist_out'], ...
        'Inputs', '++', 'Position', p(580, 85, 25, 25));

    % --- Ruido de medicion ---
    add_block('simulink/Sources/Random Number', [sub '/noise'], ...
        'Mean', '0', ...
        'Variance', noise_var, ...
        'SampleTime', 'noise_sample_time', 'Seed', '13', ...
        'Position', p(640, 145, 50, 30));
    add_block('simulink/Math Operations/Sum', [sub '/sum_noise'], ...
        'Inputs', '++', 'Position', p(680, 85, 25, 25));

    % --- Conexiones ---
    add_line(sub, 'ref/1', 'error_sum/1', 'autorouting', 'on');
    add_line(sub, 'error_sum/1', 'K_hinf/1', 'autorouting', 'on');
    add_line(sub, 'K_hinf/1', 'sat/1', 'autorouting', 'on');
    add_line(sub, 'sat/1', 'sum_dist_in/1', 'autorouting', 'on');
    add_line(sub, 'dist_input/1', 'sum_dist_in/2', 'autorouting', 'on');
    add_line(sub, 'sum_dist_in/1', 'G_plant/1', 'autorouting', 'on');
    add_line(sub, 'G_plant/1', 'sum_dist_out/1', 'autorouting', 'on');
    add_line(sub, 'dist_output/1', 'sum_dist_out/2', 'autorouting', 'on');
    add_line(sub, 'sum_dist_out/1', 'sum_noise/1', 'autorouting', 'on');
    add_line(sub, 'noise/1', 'sum_noise/2', 'autorouting', 'on');
    add_line(sub, 'sum_noise/1', 'error_sum/2', 'autorouting', 'on');

    % --- Outports para comparacion (top-level) ---
    add_block('simulink/Ports & Subsystems/Out1', [sub '/y_out'], ...
        'Position', p(780, 82, 30, 14));
    add_block('simulink/Ports & Subsystems/Out1', [sub '/u_out'], ...
        'Position', p(780, 130, 30, 14));
    add_line(sub, 'G_plant/1', 'y_out/1', 'autorouting', 'on');
    add_line(sub, 'sat/1', 'u_out/1', 'autorouting', 'on');

    % --- SCOPES ---
    add_block('simulink/Sinks/Scope', [sub '/Scope_ref_vs_y'], ...
        'NumInputPorts', '2', 'Position', p(850, 50, 40, 40));
    add_line(sub, 'ref/1', 'Scope_ref_vs_y/1', 'autorouting', 'on');
    add_line(sub, 'G_plant/1', 'Scope_ref_vs_y/2', 'autorouting', 'on');

    add_block('simulink/Sinks/Scope', [sub '/Scope_error'], ...
        'NumInputPorts', '1', 'Position', p(850, 110, 40, 40));
    add_line(sub, 'error_sum/1', 'Scope_error/1', 'autorouting', 'on');

    add_block('simulink/Sinks/Scope', [sub '/Scope_control'], ...
        'NumInputPorts', '2', 'Position', p(850, 170, 40, 40));
    add_line(sub, 'K_hinf/1', 'Scope_control/1', 'autorouting', 'on');
    add_line(sub, 'sat/1', 'Scope_control/2', 'autorouting', 'on');

    add_block('simulink/Sinks/Scope', [sub '/Scope_perturbaciones'], ...
        'NumInputPorts', '3', 'Position', p(850, 230, 40, 40));
    add_line(sub, 'dist_input/1', 'Scope_perturbaciones/1', 'autorouting', 'on');
    add_line(sub, 'dist_output/1', 'Scope_perturbaciones/2', 'autorouting', 'on');
    add_line(sub, 'noise/1', 'Scope_perturbaciones/3', 'autorouting', 'on');

    add_block('simulink/Sinks/Scope', [sub '/Scope_sensibilidad'], ...
        'NumInputPorts', '2', 'Position', p(850, 290, 40, 40));
    add_line(sub, 'sum_dist_out/1', 'Scope_sensibilidad/1', 'autorouting', 'on');
    add_line(sub, 'G_plant/1', 'Scope_sensibilidad/2', 'autorouting', 'on');
end

function add_yaw_damper(sub, ctrl_type)
    p = @(x,y,w,h) [x y x+w y+h];

    add_block('simulink/Continuous/State-Space', [sub '/yaw_damper'], ...
        'A', 'Kyaw_A', 'B', 'Kyaw_B', 'C', 'Kyaw_C', 'D', 'Kyaw_D', ...
        'InitialCondition', '0', ...
        'Position', p(500, 320, 80, 40));
    add_block('simulink/Discontinuities/Saturation', [sub '/sat_rudder'], ...
        'UpperLimit', 'umax', 'LowerLimit', '-umax', ...
        'Position', p(620, 322, 50, 30));
    add_block('simulink/Sources/Constant', [sub '/r_input'], ...
        'Value', '0', 'Position', p(420, 330, 40, 20));
    add_block('simulink/Sinks/Scope', [sub '/Scope_yaw'], ...
        'NumInputPorts', '2', 'Position', p(780, 350, 40, 40));

    add_line(sub, 'r_input/1', 'yaw_damper/1', 'autorouting', 'on');
    add_line(sub, 'yaw_damper/1', 'sat_rudder/1', 'autorouting', 'on');
    add_line(sub, 'yaw_damper/1', 'Scope_yaw/1', 'autorouting', 'on');
    add_line(sub, 'sat_rudder/1', 'Scope_yaw/2', 'autorouting', 'on');
end
