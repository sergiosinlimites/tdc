function sens = analisis_sensibilidades(channels, pid_data, hinf_data, cfg)
%ANALISIS_SENSIBILIDADES Calcula S, T y K*S para PID y H_inf.

% Paso 1: calcular metricas de sensibilidad para los lazos PID.
sens.theta.pid = loop_metrics(channels.theta, pid_data.K_theta, cfg);
sens.phi.pid = loop_metrics(channels.phi, pid_data.K_phi, cfg);

% Paso 2: calcular las mismas metricas para los lazos H_inf.
sens.theta.hinf = loop_metrics(channels.theta, hinf_data.K_theta, cfg);
sens.phi.hinf = loop_metrics(channels.phi, hinf_data.K_phi, cfg);

% Paso 3: guardar pesos para comparar S, T y KS contra 1/W1, 1/W3 y 1/W2.
sens.weights = hinf_data.weights;
end

function m = loop_metrics(G, K, cfg)
%LOOP_METRICS Cierra un lazo SISO y calcula L, S, T y KS.

% L es el lazo abierto. Desde aqui salen todas las funciones de sensibilidad.
L = minreal(G*K, cfg.minreal_tol);

% S mide error/perturbaciones; T mide seguimiento/ruido; KS mide control.
S = minreal(feedback(1, L), cfg.minreal_tol);
T = minreal(feedback(L, 1), cfg.minreal_tol);
KS = minreal(K*S, cfg.minreal_tol);

% Se guardan sistemas y normas para graficar y resumir en el reporte.
m.L = L;
m.S = S;
m.T = T;
m.KS = KS;
m.closed_loop = T;
m.stable = isstable(T);
m.norm_S = safe_norminf(S);
m.norm_T = safe_norminf(T);
m.norm_KS = safe_norminf(KS);
m.bandwidth = bandwidth(T);
end

function n = safe_norminf(sys)
%SAFE_NORMINF Calcula norma H_inf si es posible y evita detener el flujo.
try
    n = norm(sys, inf);
catch
    n = NaN;
end
end
