%% Calcula metricas de desempeno despues de simular taller1_simulink_v2.slx
% Requiere que configure_scopes_v2.m haya habilitado logging en los scopes.
% Las variables log_* deben existir en el workspace despues de sim().
%
% USO:
%   sim('taller1_simulink_v2');
%   compute_metrics_v2;

tol_band = 0.05;  % banda de establecimiento: ±5% del valor final

logs = {
    'log_pid_theta_y',   'PID \theta',     1, deg2rad(theta_ref_amp);
    'log_hinf_theta_y',  'H-inf \theta',   1, deg2rad(theta_ref_amp);
    'log_pid_phi_y',     'PID \phi',       1, deg2rad(phi_ref_amp);
    'log_hinf_phi_y',    'H-inf \phi',     1, deg2rad(phi_ref_amp);
};

ctrl_logs = {
    'log_pid_theta_u',   'PID elevator',   2;
    'log_hinf_theta_u',  'H-inf elevator', 2;
    'log_pid_phi_u',     'PID aileron',    2;
    'log_hinf_phi_u',    'H-inf aileron',  2;
};

fprintf('\n%s\n', repmat('=', 1, 72));
fprintf('  METRICAS DE DESEMPENO — taller1_simulink_v2\n');
fprintf('  ref_theta=%.1f deg  ref_phi=%.1f deg  umax=%.1f deg  ', ...
    rad2deg(theta_ref_amp), rad2deg(phi_ref_amp), rad2deg(umax));
fprintf('ruido=%d  dist=%d\n', noise_enabled, dist_enabled);
fprintf('%s\n\n', repmat('=', 1, 72));

fprintf('%-16s %10s %10s %10s %10s %10s\n', ...
    'Lazo', 'Ts [s]', 'OS [%%]', 'e_ss [deg]', 'RMS_e [deg]', 'Sat [%%]');
fprintf('%s\n', repmat('-', 1, 68));

for k = 1:size(logs, 1)
    vname  = logs{k, 1};
    label  = logs{k, 2};
    ch     = logs{k, 3};  % canal de salida (1=y, 2=ref en algunos)
    ref_v  = logs{k, 4};

    if ~evalin('base', ['exist(''' vname ''', ''var'')'])
        fprintf('%-16s  [sin datos — simula primero]\n', label);
        continue;
    end

    logdata = evalin('base', vname);
    t = logdata.time;
    y = logdata.signals(ch).values;

    ref_deg = rad2deg(ref_v);
    y_deg   = rad2deg(y);

    % Tiempo de establecimiento (±5% del valor final tras t_step)
    post = t >= t_step;
    y_post = y_deg(post);
    t_post = t(post);
    yf = y_post(end);
    if abs(ref_deg) > 0.1
        in_band = abs(y_post - ref_deg) <= tol_band * abs(ref_deg);
    else
        in_band = true(size(y_post));
    end
    last_out = find(~in_band, 1, 'last');
    if isempty(last_out)
        Ts = 0;
    elseif last_out >= numel(t_post)
        Ts = NaN;
    else
        Ts = t_post(last_out + 1) - t_step;
    end

    % Sobrepaso
    if ref_deg > 0
        OS = max(0, (max(y_deg) - ref_deg) / ref_deg * 100);
    elseif ref_deg < 0
        OS = max(0, (ref_deg - min(y_deg)) / abs(ref_deg) * 100);
    else
        OS = max(abs(y_deg)) * 100;
    end

    % Error en estado estable (ultimos 2s)
    tail = t >= (t(end) - 2);
    e_ss = mean(ref_deg - y_deg(tail));

    % RMS del error (desde t_step)
    e_rms = rms(ref_deg - y_deg(post));

    % Saturacion (del control correspondente)
    u_vname = ctrl_logs{k, 1};
    sat_pct = NaN;
    if evalin('base', ['exist(''' u_vname ''', ''var'')'])
        udata = evalin('base', u_vname);
        u_sat = udata.signals(2).values;  % canal 2 = u_sat
        sat_pct = mean(abs(u_sat) >= umax * 0.999) * 100;
    end

    fprintf('%-16s %10.3f %10.1f %10.3f %10.3f %10.1f\n', ...
        label, Ts, OS, e_ss, e_rms, sat_pct);
end

fprintf('%s\n', repmat('-', 1, 68));
fprintf('Ts = tiempo hasta entrar en banda ±5%%  |  OS = sobrepaso  |  Sat = %% tiempo saturado\n\n');

%% Comparacion rapida PID vs Hinf
if evalin('base', "exist('log_cmp_theta','var')") && ...
   evalin('base', "exist('log_cmp_phi','var')")
    fprintf('Senal de comparacion disponible en log_cmp_theta y log_cmp_phi.\n');
end
