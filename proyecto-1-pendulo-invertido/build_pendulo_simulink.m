%% Construye pendulo_invertido.slx desde cero
% El modelo Simulink implementa el lazo lineal:
%   x_dot = A_lin*x + B_lin*u
%   u = sat(-K_lqr*x)

clear; clc;

project_dir = fileparts(mfilename('fullpath'));
addpath(project_dir);
run(fullfile(project_dir, 'init_pendulo_simulink.m'));

mdl = 'pendulo_invertido';
slx_path = fullfile(project_dir, [mdl '.slx']);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

if exist(slx_path, 'file')
    delete(slx_path);
end

new_system(mdl);
open_system(mdl);
set_param(mdl, 'InitFcn', ['run(''' fullfile(project_dir, 'init_pendulo_simulink.m') ''')']);
set_param(mdl, 'StopTime', 't_final', 'Solver', 'ode45', 'SaveFormat', 'StructureWithTime');

add_block('simulink/Continuous/State-Space', [mdl '/Planta_Lineal'], ...
    'A', 'A_lin', 'B', 'B_lin', 'C', 'C_lin', 'D', 'D_lin', ...
    'InitialCondition', 'x0_sim', 'Position', [250 115 340 165]);
add_block('simulink/Math Operations/Gain', [mdl '/Controlador_LQR'], ...
    'Gain', '-K_lqr', 'Multiplication', 'Matrix(K*u)', ...
    'Position', [410 115 505 165]);
add_block('simulink/Discontinuities/Saturation', [mdl '/Saturacion_Actuador'], ...
    'UpperLimit', 'umax', 'LowerLimit', '-umax', ...
    'Position', [560 120 640 160]);
add_block('simulink/Sinks/To Workspace', [mdl '/Estados_To_Workspace'], ...
    'VariableName', 'simout_x', 'SaveFormat', 'Structure With Time', ...
    'Position', [410 35 545 70]);
add_block('simulink/Sinks/To Workspace', [mdl '/Control_To_Workspace'], ...
    'VariableName', 'simout_u', 'SaveFormat', 'Structure With Time', ...
    'Position', [690 40 825 75]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_Estados'], ...
    'Position', [410 210 500 250]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_Control'], ...
    'Position', [690 210 780 250]);

add_line(mdl, 'Planta_Lineal/1', 'Controlador_LQR/1', 'autorouting', 'on');
add_line(mdl, 'Controlador_LQR/1', 'Saturacion_Actuador/1', 'autorouting', 'on');
add_line(mdl, 'Saturacion_Actuador/1', 'Planta_Lineal/1', 'autorouting', 'on');
add_line(mdl, 'Planta_Lineal/1', 'Estados_To_Workspace/1', 'autorouting', 'on');
add_line(mdl, 'Saturacion_Actuador/1', 'Control_To_Workspace/1', 'autorouting', 'on');
add_line(mdl, 'Planta_Lineal/1', 'Scope_Estados/1', 'autorouting', 'on');
add_line(mdl, 'Saturacion_Actuador/1', 'Scope_Control/1', 'autorouting', 'on');

save_system(mdl, slx_path);
fprintf('Modelo guardado en: %s\n', slx_path);
