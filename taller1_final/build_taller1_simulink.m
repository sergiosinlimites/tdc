%% Construye taller1_simulink.slx desde cero
% Modelo Simulink lineal acoplado con selector SAS-CAS/H_inf, ruido de
% medicion y saturacion de actuadores.

clear; clc;

project_dir = fileparts(mfilename('fullpath'));
addpath(project_dir);
run(fullfile(project_dir, 'init_simulink.m'));

mdl = 'taller1_simulink';
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
    'init_simulink.m') ''')']);
set_param(mdl, 'StopTime', 't_final', 'Solver', 'ode45', ...
    'SaveFormat', 'Dataset');

% --- Referencias, ruido y selector ---
add_block('simulink/Sources/Step', [mdl '/theta_ref'], ...
    'Time', 't_step', 'Before', '0', 'After', 'theta_ref_step', ...
    'Position', [40 65 80 95]);
add_block('simulink/Sources/Step', [mdl '/phi_ref'], ...
    'Time', 't_step', 'Before', '0', 'After', 'phi_ref_step', ...
    'Position', [40 245 80 275]);
add_block('simulink/Sources/Random Number', [mdl '/noise_theta'], ...
    'Mean', '0', 'Variance', 'noise_var_theta', ...
    'SampleTime', 'noise_sample_time', 'Seed', '11', ...
    'Position', [220 125 280 155]);
add_block('simulink/Sources/Random Number', [mdl '/noise_phi'], ...
    'Mean', '0', 'Variance', 'noise_var_phi', ...
    'SampleTime', 'noise_sample_time', 'Seed', '17', ...
    'Position', [220 305 280 335]);
add_block('simulink/Sources/Constant', [mdl '/zero'], ...
    'Value', '0', 'Position', [700 510 735 535]);
add_block('simulink/Sources/Constant', [mdl '/control_mode'], ...
    'Value', 'control_mode', 'Position', [350 445 410 475]);

% --- Planta y demux ---
add_block('simulink/Continuous/State-Space', [mdl '/UAV_linmodel'], ...
    'A', 'A_full', 'B', 'B_full', 'C', 'C_sim', 'D', 'D_sim', ...
    'InitialCondition', 'zeros(size(A_full,1),1)', ...
    'Position', [850 210 955 270]);
add_block('simulink/Signal Routing/Demux', [mdl '/y_demux'], ...
    'Outputs', '5', 'Position', [1010 185 1020 295]);

% --- Medicion y error theta ---
add_block('simulink/Math Operations/Sum', [mdl '/theta_meas_sum'], ...
    'Inputs', '++', 'Position', [325 108 350 132]);
add_block('simulink/Math Operations/Sum', [mdl '/theta_error'], ...
    'Inputs', '+-', 'Position', [125 70 150 95]);

% --- Medicion y error phi ---
add_block('simulink/Math Operations/Sum', [mdl '/phi_meas_sum'], ...
    'Inputs', '++', 'Position', [325 288 350 312]);
add_block('simulink/Math Operations/Sum', [mdl '/phi_error'], ...
    'Inputs', '+-', 'Position', [125 250 150 275]);

% --- Controladores theta ---
add_block('simulink/Continuous/Transfer Fcn', [mdl '/CAS_PI_theta'], ...
    'Numerator', 'cas_pi_theta_num', 'Denominator', 'cas_pi_theta_den', ...
    'Position', [215 45 305 85]);
add_block('simulink/Math Operations/Gain', [mdl '/SAS_D_q'], ...
    'Gain', 'sas_D_q', 'Position', [215 135 305 165]);
add_block('simulink/Math Operations/Sum', [mdl '/elevator_raw_sum'], ...
    'Inputs', '+-', 'Position', [365 75 390 115]);
add_block('simulink/Continuous/State-Space', [mdl '/Hinf_theta'], ...
    'A', 'Ktheta_A', 'B', 'Ktheta_B', 'C', 'Ktheta_C', 'D', 'Ktheta_D', ...
    'InitialCondition', 'zeros(size(Ktheta_A,1),1)', ...
    'Position', [215 90 305 130]);
add_block('simulink/Signal Routing/Switch', [mdl '/switch_theta'], ...
    'Threshold', '0.5', 'Position', [460 65 500 125]);
add_block('simulink/Discontinuities/Saturation', [mdl '/sat_elevator'], ...
    'UpperLimit', 'umax', 'LowerLimit', '-umax', ...
    'Position', [560 82 620 118]);
add_block('simulink/Sources/Sine Wave', [mdl '/dist_elevator'], ...
    'Amplitude', 'dist_amp', 'Frequency', 'dist_freq', ...
    'Position', [565 130 625 160]);
add_block('simulink/Math Operations/Sum', [mdl '/elevator_input'], ...
    'Inputs', '++', 'Position', [655 92 680 118]);

% --- Controladores phi ---
add_block('simulink/Continuous/Transfer Fcn', [mdl '/CAS_PI_phi'], ...
    'Numerator', 'cas_pi_phi_num', 'Denominator', 'cas_pi_phi_den', ...
    'Position', [215 225 305 265]);
add_block('simulink/Math Operations/Gain', [mdl '/SAS_D_p'], ...
    'Gain', 'sas_D_p', 'Position', [215 315 305 345]);
add_block('simulink/Math Operations/Sum', [mdl '/aileron_raw_sum'], ...
    'Inputs', '+-', 'Position', [365 255 390 295]);
add_block('simulink/Continuous/State-Space', [mdl '/Hinf_phi'], ...
    'A', 'Kphi_A', 'B', 'Kphi_B', 'C', 'Kphi_C', 'D', 'Kphi_D', ...
    'InitialCondition', 'zeros(size(Kphi_A,1),1)', ...
    'Position', [215 270 305 310]);
add_block('simulink/Signal Routing/Switch', [mdl '/switch_phi'], ...
    'Threshold', '0.5', 'Position', [460 245 500 305]);
add_block('simulink/Discontinuities/Saturation', [mdl '/sat_aileron'], ...
    'UpperLimit', 'umax', 'LowerLimit', '-umax', ...
    'Position', [560 262 620 298]);
add_block('simulink/Sources/Sine Wave', [mdl '/dist_aileron'], ...
    'Amplitude', 'dist_amp', 'Frequency', 'dist_freq', 'Phase', 'pi/4', ...
    'Position', [565 310 625 340]);
add_block('simulink/Math Operations/Sum', [mdl '/aileron_input'], ...
    'Inputs', '++', 'Position', [655 272 680 298]);

% --- Yaw damper ---
add_block('simulink/Continuous/State-Space', [mdl '/yaw_damper'], ...
    'A', 'Kyaw_A', 'B', 'Kyaw_B', 'C', 'Kyaw_C', 'D', 'Kyaw_D', ...
    'InitialCondition', '0', 'Position', [560 365 640 405]);
add_block('simulink/Discontinuities/Saturation', [mdl '/sat_rudder'], ...
    'UpperLimit', 'umax', 'LowerLimit', '-umax', ...
    'Position', [690 367 750 403]);
add_block('simulink/Sources/Sine Wave', [mdl '/dist_rudder'], ...
    'Amplitude', 'dist_amp', 'Frequency', 'dist_freq', 'Phase', 'pi/2', ...
    'Position', [690 420 750 450]);
add_block('simulink/Math Operations/Sum', [mdl '/rudder_input'], ...
    'Inputs', '++', 'Position', [780 377 805 403]);

% --- Mux 8 entradas ---
add_block('simulink/Signal Routing/Mux', [mdl '/u_mux'], ...
    'Inputs', '8', 'Position', [760 185 775 315]);

% --- Logging ---
add_block('simulink/Signal Routing/Mux', [mdl '/y_log_mux'], ...
    'Inputs', '4', 'Position', [1120 170 1135 245]);
add_block('simulink/Sinks/To Workspace', [mdl '/y_log'], ...
    'VariableName', 'simout_y', 'SaveFormat', 'Structure With Time', ...
    'Position', [1185 185 1285 215]);
add_block('simulink/Signal Routing/Mux', [mdl '/u_log_mux'], ...
    'Inputs', '3', 'Position', [655 70 670 150]);
add_block('simulink/Sinks/To Workspace', [mdl '/u_log'], ...
    'VariableName', 'simout_u', 'SaveFormat', 'Structure With Time', ...
    'Position', [705 90 805 120]);
add_block('simulink/Signal Routing/Mux', [mdl '/sas_cas_log_mux'], ...
    'Inputs', '17', 'Position', [1120 275 1135 475]);
add_block('simulink/Sinks/To Workspace', [mdl '/sas_cas_log'], ...
    'VariableName', 'simout_sas_cas', 'SaveFormat', 'Structure With Time', ...
    'Position', [1185 345 1310 375]);

% --- Conexiones: planta ---
add_line(mdl, 'UAV_linmodel/1', 'y_demux/1', 'autorouting', 'on');
add_line(mdl, 'u_mux/1', 'UAV_linmodel/1', 'autorouting', 'on');

% --- Conexiones: lazo theta ---
add_line(mdl, 'y_demux/1', 'theta_meas_sum/1', 'autorouting', 'on');
add_line(mdl, 'noise_theta/1', 'theta_meas_sum/2', 'autorouting', 'on');
add_line(mdl, 'theta_ref/1', 'theta_error/1', 'autorouting', 'on');
add_line(mdl, 'theta_meas_sum/1', 'theta_error/2', 'autorouting', 'on');
add_line(mdl, 'theta_error/1', 'CAS_PI_theta/1', 'autorouting', 'on');
add_line(mdl, 'theta_error/1', 'Hinf_theta/1', 'autorouting', 'on');
add_line(mdl, 'y_demux/4', 'SAS_D_q/1', 'autorouting', 'on');
add_line(mdl, 'CAS_PI_theta/1', 'elevator_raw_sum/1', 'autorouting', 'on');
add_line(mdl, 'SAS_D_q/1', 'elevator_raw_sum/2', 'autorouting', 'on');
add_line(mdl, 'Hinf_theta/1', 'switch_theta/1', 'autorouting', 'on');
add_line(mdl, 'control_mode/1', 'switch_theta/2', 'autorouting', 'on');
add_line(mdl, 'elevator_raw_sum/1', 'switch_theta/3', 'autorouting', 'on');
add_line(mdl, 'switch_theta/1', 'sat_elevator/1', 'autorouting', 'on');
add_line(mdl, 'sat_elevator/1', 'elevator_input/1', 'autorouting', 'on');
add_line(mdl, 'dist_elevator/1', 'elevator_input/2', 'autorouting', 'on');

% --- Conexiones: lazo phi ---
add_line(mdl, 'y_demux/2', 'phi_meas_sum/1', 'autorouting', 'on');
add_line(mdl, 'noise_phi/1', 'phi_meas_sum/2', 'autorouting', 'on');
add_line(mdl, 'phi_ref/1', 'phi_error/1', 'autorouting', 'on');
add_line(mdl, 'phi_meas_sum/1', 'phi_error/2', 'autorouting', 'on');
add_line(mdl, 'phi_error/1', 'CAS_PI_phi/1', 'autorouting', 'on');
add_line(mdl, 'phi_error/1', 'Hinf_phi/1', 'autorouting', 'on');
add_line(mdl, 'y_demux/3', 'SAS_D_p/1', 'autorouting', 'on');
add_line(mdl, 'CAS_PI_phi/1', 'aileron_raw_sum/1', 'autorouting', 'on');
add_line(mdl, 'SAS_D_p/1', 'aileron_raw_sum/2', 'autorouting', 'on');
add_line(mdl, 'Hinf_phi/1', 'switch_phi/1', 'autorouting', 'on');
add_line(mdl, 'control_mode/1', 'switch_phi/2', 'autorouting', 'on');
add_line(mdl, 'aileron_raw_sum/1', 'switch_phi/3', 'autorouting', 'on');
add_line(mdl, 'switch_phi/1', 'sat_aileron/1', 'autorouting', 'on');
add_line(mdl, 'sat_aileron/1', 'aileron_input/1', 'autorouting', 'on');
add_line(mdl, 'dist_aileron/1', 'aileron_input/2', 'autorouting', 'on');

% --- Conexiones: yaw damper ---
add_line(mdl, 'y_demux/5', 'yaw_damper/1', 'autorouting', 'on');
add_line(mdl, 'yaw_damper/1', 'sat_rudder/1', 'autorouting', 'on');
add_line(mdl, 'sat_rudder/1', 'rudder_input/1', 'autorouting', 'on');
add_line(mdl, 'dist_rudder/1', 'rudder_input/2', 'autorouting', 'on');

% --- Mapear a las 8 entradas de planta ---
add_line(mdl, 'zero/1', 'u_mux/1', 'autorouting', 'on');
add_line(mdl, 'elevator_input/1', 'u_mux/2', 'autorouting', 'on');
add_line(mdl, 'rudder_input/1', 'u_mux/3', 'autorouting', 'on');
add_line(mdl, 'zero/1', 'u_mux/4', 'autorouting', 'on');
add_line(mdl, 'zero/1', 'u_mux/5', 'autorouting', 'on');
add_line(mdl, 'zero/1', 'u_mux/6', 'autorouting', 'on');
add_line(mdl, 'zero/1', 'u_mux/7', 'autorouting', 'on');
add_line(mdl, 'aileron_input/1', 'u_mux/8', 'autorouting', 'on');

% --- Logging de salidas ---
add_line(mdl, 'y_demux/1', 'y_log_mux/1', 'autorouting', 'on');
add_line(mdl, 'theta_ref/1', 'y_log_mux/2', 'autorouting', 'on');
add_line(mdl, 'y_demux/2', 'y_log_mux/3', 'autorouting', 'on');
add_line(mdl, 'phi_ref/1', 'y_log_mux/4', 'autorouting', 'on');
add_line(mdl, 'y_log_mux/1', 'y_log/1', 'autorouting', 'on');

add_line(mdl, 'elevator_input/1', 'u_log_mux/1', 'autorouting', 'on');
add_line(mdl, 'aileron_input/1', 'u_log_mux/2', 'autorouting', 'on');
add_line(mdl, 'rudder_input/1', 'u_log_mux/3', 'autorouting', 'on');
add_line(mdl, 'u_log_mux/1', 'u_log/1', 'autorouting', 'on');

% --- Logging SAS/CAS detallado ---
add_line(mdl, 'theta_ref/1', 'sas_cas_log_mux/1', 'autorouting', 'on');
add_line(mdl, 'y_demux/1', 'sas_cas_log_mux/2', 'autorouting', 'on');
add_line(mdl, 'y_demux/4', 'sas_cas_log_mux/3', 'autorouting', 'on');
add_line(mdl, 'CAS_PI_theta/1', 'sas_cas_log_mux/4', 'autorouting', 'on');
add_line(mdl, 'SAS_D_q/1', 'sas_cas_log_mux/5', 'autorouting', 'on');
add_line(mdl, 'switch_theta/1', 'sas_cas_log_mux/6', 'autorouting', 'on');
add_line(mdl, 'sat_elevator/1', 'sas_cas_log_mux/7', 'autorouting', 'on');
add_line(mdl, 'phi_ref/1', 'sas_cas_log_mux/8', 'autorouting', 'on');
add_line(mdl, 'y_demux/2', 'sas_cas_log_mux/9', 'autorouting', 'on');
add_line(mdl, 'y_demux/3', 'sas_cas_log_mux/10', 'autorouting', 'on');
add_line(mdl, 'CAS_PI_phi/1', 'sas_cas_log_mux/11', 'autorouting', 'on');
add_line(mdl, 'SAS_D_p/1', 'sas_cas_log_mux/12', 'autorouting', 'on');
add_line(mdl, 'switch_phi/1', 'sas_cas_log_mux/13', 'autorouting', 'on');
add_line(mdl, 'sat_aileron/1', 'sas_cas_log_mux/14', 'autorouting', 'on');
add_line(mdl, 'y_demux/5', 'sas_cas_log_mux/15', 'autorouting', 'on');
add_line(mdl, 'yaw_damper/1', 'sas_cas_log_mux/16', 'autorouting', 'on');
add_line(mdl, 'sat_rudder/1', 'sas_cas_log_mux/17', 'autorouting', 'on');
add_line(mdl, 'sas_cas_log_mux/1', 'sas_cas_log/1', 'autorouting', 'on');

save_system(mdl, slx_path);
fprintf('Modelo guardado en: %s\n', slx_path);
