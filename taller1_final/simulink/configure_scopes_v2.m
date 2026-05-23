%% Configura scopes de taller1_simulink_v2.slx
% Agrega leyendas, titulos, etiquetas de eje y nombres de senales
% sin mover ningun bloque. Tambien habilita logging en scopes clave.
% Correr despues de build_taller1_simulink_v2.m (o cuando el .slx este abierto).

mdl = 'taller1_simulink_v2';
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

%% ========================================================================
%  NOMBRES DE SENALES (aparecen en la leyenda del scope)
%  ========================================================================
% Funcion auxiliar: nombra la linea que llega al puerto n de un bloque
name_input = @(blk, port, nm) set_input_name(blk, port, nm);

% --- Lazo_PID_theta ---
sub = [mdl '/Lazo_PID_theta'];
name_input([sub '/Scope_ref_vs_y'],    1, 'ref \theta [rad]');
name_input([sub '/Scope_ref_vs_y'],    2, '\theta [rad]');
name_input([sub '/Scope_error'],       1, 'e_\theta [rad]');
name_input([sub '/Scope_control'],     1, 'u_{raw} [rad]');
name_input([sub '/Scope_control'],     2, 'u_{sat} [rad]');
name_input([sub '/Scope_PI_vs_D'],     1, 'PI out [rad]');
name_input([sub '/Scope_PI_vs_D'],     2, 'D_{SAS} out [rad]');
name_input([sub '/Scope_perturbaciones'], 1, 'dist entrada');
name_input([sub '/Scope_perturbaciones'], 2, 'dist salida');
name_input([sub '/Scope_perturbaciones'], 3, 'ruido \theta');
name_input([sub '/Scope_rate'],        1, 'q [rad/s]');

% --- Lazo_PID_phi ---
sub = [mdl '/Lazo_PID_phi'];
name_input([sub '/Scope_ref_vs_y'],    1, 'ref \phi [rad]');
name_input([sub '/Scope_ref_vs_y'],    2, '\phi [rad]');
name_input([sub '/Scope_error'],       1, 'e_\phi [rad]');
name_input([sub '/Scope_control'],     1, 'u_{raw} [rad]');
name_input([sub '/Scope_control'],     2, 'u_{sat} [rad]');
name_input([sub '/Scope_PI_vs_D'],     1, 'PI out [rad]');
name_input([sub '/Scope_PI_vs_D'],     2, 'D_{SAS} out [rad]');
name_input([sub '/Scope_perturbaciones'], 1, 'dist entrada');
name_input([sub '/Scope_perturbaciones'], 2, 'dist salida');
name_input([sub '/Scope_perturbaciones'], 3, 'ruido \phi');
name_input([sub '/Scope_rate'],        1, 'p [rad/s]');
name_input([sub '/Scope_yaw'],         1, 'yaw damper out');
name_input([sub '/Scope_yaw'],         2, 'rudder sat [rad]');

% --- Lazo_Hinf_theta ---
sub = [mdl '/Lazo_Hinf_theta'];
name_input([sub '/Scope_ref_vs_y'],    1, 'ref \theta [rad]');
name_input([sub '/Scope_ref_vs_y'],    2, '\theta [rad]');
name_input([sub '/Scope_error'],       1, 'e_\theta [rad]');
name_input([sub '/Scope_control'],     1, 'K_{Hinf} out [rad]');
name_input([sub '/Scope_control'],     2, 'u_{sat} [rad]');
name_input([sub '/Scope_perturbaciones'], 1, 'dist entrada');
name_input([sub '/Scope_perturbaciones'], 2, 'dist salida');
name_input([sub '/Scope_perturbaciones'], 3, 'ruido \theta');
name_input([sub '/Scope_sensibilidad'], 1, 'y + dist_out');
name_input([sub '/Scope_sensibilidad'], 2, '\theta [rad]');

% --- Lazo_Hinf_phi ---
sub = [mdl '/Lazo_Hinf_phi'];
name_input([sub '/Scope_ref_vs_y'],    1, 'ref \phi [rad]');
name_input([sub '/Scope_ref_vs_y'],    2, '\phi [rad]');
name_input([sub '/Scope_error'],       1, 'e_\phi [rad]');
name_input([sub '/Scope_control'],     1, 'K_{Hinf} out [rad]');
name_input([sub '/Scope_control'],     2, 'u_{sat} [rad]');
name_input([sub '/Scope_perturbaciones'], 1, 'dist entrada');
name_input([sub '/Scope_perturbaciones'], 2, 'dist salida');
name_input([sub '/Scope_perturbaciones'], 3, 'ruido \phi');
name_input([sub '/Scope_sensibilidad'], 1, 'y + dist_out');
name_input([sub '/Scope_sensibilidad'], 2, '\phi [rad]');
name_input([sub '/Scope_yaw'],         1, 'yaw damper out');
name_input([sub '/Scope_yaw'],         2, 'rudder sat [rad]');

% --- Scopes comparativos (top-level) ---
name_input([mdl '/Comparar_theta'],   1, 'referencia');
name_input([mdl '/Comparar_theta'],   2, 'PID \theta');
name_input([mdl '/Comparar_theta'],   3, 'H\infty \theta');
name_input([mdl '/Comparar_phi'],     1, 'referencia');
name_input([mdl '/Comparar_phi'],     2, 'PID \phi');
name_input([mdl '/Comparar_phi'],     3, 'H\infty \phi');
name_input([mdl '/Comparar_control'], 1, 'PID elevator');
name_input([mdl '/Comparar_control'], 2, 'H\infty elevator');
name_input([mdl '/Comparar_control'], 3, 'PID aileron');
name_input([mdl '/Comparar_control'], 4, 'H\infty aileron');

%% ========================================================================
%  CONFIGURACION DE SCOPES
%  ========================================================================
% Cada scope recibe: titulo, etiqueta Y, leyenda activada, grid,
% y logging hacia workspace para calcular metricas post-simulacion.

% Formato: {path, titulo, ylabel, nombre_variable_log}
scope_cfg = {
    % --- PID theta ---
    [mdl '/Lazo_PID_theta/Scope_ref_vs_y'], ...
        'PID \theta — Seguimiento de referencia', 'angulo [rad]', 'log_pid_theta_y';
    [mdl '/Lazo_PID_theta/Scope_error'], ...
        'PID \theta — Error de seguimiento', 'error [rad]', 'log_pid_theta_e';
    [mdl '/Lazo_PID_theta/Scope_control'], ...
        'PID \theta — Senal de control (cruda vs saturada)', 'elevator [rad]', 'log_pid_theta_u';
    [mdl '/Lazo_PID_theta/Scope_PI_vs_D'], ...
        'PID \theta — Contribucion PI vs D_{SAS}', 'magnitud [rad]', '';
    [mdl '/Lazo_PID_theta/Scope_perturbaciones'], ...
        'PID \theta — Perturbaciones y ruido', 'magnitud', '';
    [mdl '/Lazo_PID_theta/Scope_rate'], ...
        'PID \theta — Tasa de cabeceo q', 'q [rad/s]', '';
    % --- PID phi ---
    [mdl '/Lazo_PID_phi/Scope_ref_vs_y'], ...
        'PID \phi — Seguimiento de referencia', 'angulo [rad]', 'log_pid_phi_y';
    [mdl '/Lazo_PID_phi/Scope_error'], ...
        'PID \phi — Error de seguimiento', 'error [rad]', 'log_pid_phi_e';
    [mdl '/Lazo_PID_phi/Scope_control'], ...
        'PID \phi — Senal de control (cruda vs saturada)', 'aileron [rad]', 'log_pid_phi_u';
    [mdl '/Lazo_PID_phi/Scope_PI_vs_D'], ...
        'PID \phi — Contribucion PI vs D_{SAS}', 'magnitud [rad]', '';
    [mdl '/Lazo_PID_phi/Scope_perturbaciones'], ...
        'PID \phi — Perturbaciones y ruido', 'magnitud', '';
    [mdl '/Lazo_PID_phi/Scope_rate'], ...
        'PID \phi — Tasa de alabeo p', 'p [rad/s]', '';
    [mdl '/Lazo_PID_phi/Scope_yaw'], ...
        'PID \phi — Yaw damper (salida y saturacion)', 'rudder [rad]', '';
    % --- Hinf theta ---
    [mdl '/Lazo_Hinf_theta/Scope_ref_vs_y'], ...
        'H\infty \theta — Seguimiento de referencia', 'angulo [rad]', 'log_hinf_theta_y';
    [mdl '/Lazo_Hinf_theta/Scope_error'], ...
        'H\infty \theta — Error de seguimiento', 'error [rad]', 'log_hinf_theta_e';
    [mdl '/Lazo_Hinf_theta/Scope_control'], ...
        'H\infty \theta — Senal de control (cruda vs saturada)', 'elevator [rad]', 'log_hinf_theta_u';
    [mdl '/Lazo_Hinf_theta/Scope_perturbaciones'], ...
        'H\infty \theta — Perturbaciones y ruido', 'magnitud', '';
    [mdl '/Lazo_Hinf_theta/Scope_sensibilidad'], ...
        'H\infty \theta — Rechazo de perturbacion de salida (S_0)', 'angulo [rad]', '';
    % --- Hinf phi ---
    [mdl '/Lazo_Hinf_phi/Scope_ref_vs_y'], ...
        'H\infty \phi — Seguimiento de referencia', 'angulo [rad]', 'log_hinf_phi_y';
    [mdl '/Lazo_Hinf_phi/Scope_error'], ...
        'H\infty \phi — Error de seguimiento', 'error [rad]', 'log_hinf_phi_e';
    [mdl '/Lazo_Hinf_phi/Scope_control'], ...
        'H\infty \phi — Senal de control (cruda vs saturada)', 'aileron [rad]', 'log_hinf_phi_u';
    [mdl '/Lazo_Hinf_phi/Scope_perturbaciones'], ...
        'H\infty \phi — Perturbaciones y ruido', 'magnitud', '';
    [mdl '/Lazo_Hinf_phi/Scope_sensibilidad'], ...
        'H\infty \phi — Rechazo de perturbacion de salida (S_0)', 'angulo [rad]', '';
    [mdl '/Lazo_Hinf_phi/Scope_yaw'], ...
        'H\infty \phi — Yaw damper (salida y saturacion)', 'rudder [rad]', '';
    % --- Comparativos ---
    [mdl '/Comparar_theta'], ...
        'Comparacion \theta — PID vs H\infty', 'angulo [rad]', 'log_cmp_theta';
    [mdl '/Comparar_phi'], ...
        'Comparacion \phi — PID vs H\infty', 'angulo [rad]', 'log_cmp_phi';
    [mdl '/Comparar_control'], ...
        'Comparacion control — PID vs H\infty (elevator y aileron)', 'senal [rad]', 'log_cmp_ctrl';
};

for k = 1:size(scope_cfg, 1)
    blk   = scope_cfg{k, 1};
    ttl   = scope_cfg{k, 2};
    ylbl  = scope_cfg{k, 3};
    logv  = scope_cfg{k, 4};

    try
        set_param(blk, 'ShowLegend',  'on');
        set_param(blk, 'ShowGrid',    'on');
        set_param(blk, 'Title',       ttl);
        set_param(blk, 'YLabel',      ylbl);
        set_param(blk, 'TimeRange',   'auto');

        if ~isempty(logv)
            set_param(blk, 'SaveToWorkspace', 'on');
            set_param(blk, 'SaveName',        logv);
            set_param(blk, 'DataFormat',      'Structure With Time');
            set_param(blk, 'LimitDataPoints', 'off');
        end
    catch ME
        fprintf('[AVISO] %s: %s\n', blk, ME.message);
    end
end

%% ========================================================================
%  GUARDAR
%  ========================================================================
save_system(mdl);
fprintf('\nconfigure_scopes_v2.m: scopes configurados (%d en total).\n', ...
    size(scope_cfg, 1));
fprintf('  Leyendas activas, grid activado, logging en scopes clave.\n');
fprintf('  Despues de simular, usa compute_metrics_v2() para Ts, sobrepaso, RMS.\n');

%% ========================================================================
%  FUNCION LOCAL
%  ========================================================================
function set_input_name(blk, port_idx, nm)
    try
        ph = get_param(blk, 'PortHandles');
        lh = get_param(ph.Inport(port_idx), 'Line');
        if lh ~= -1
            set_param(lh, 'Name', nm);
        end
    catch
        % silencioso si la linea no existe todavia
    end
end
