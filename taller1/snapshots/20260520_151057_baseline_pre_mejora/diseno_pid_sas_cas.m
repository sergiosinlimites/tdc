function pid_data = diseno_pid_sas_cas(channels, cfg)
%DISENO_PID_SAS_CAS Diseno clasico SAS/CAS para theta y phi.
%
% La arquitectura SAS/CAS separa dos tareas que en un PID normal aparecen
% mezcladas. El CAS hace seguimiento con PI sobre el error de angulo. El SAS
% agrega amortiguamiento realimentando velocidad angular medida (q para
% pitch, p para roll). Si la referencia es casi constante, derivar el error
% de angulo equivale aproximadamente a usar -q o -p; por eso esta estructura
% se puede ver como un PID implementado como PI de tracking mas damper.

s = tf('s');
tau = cfg.pid.derivative_tau;

% Paso 1: construir los controladores de angulo. Se usan dos ejes porque el
% taller controla pitch con elevator (theta/elevator) y roll con aileron
% (phi/aileron). Yaw no se trata como tracking principal; se deja con un
% damper de r para amortiguar el modo lateral.
pid_data.K_theta = minreal(cfg.pid.kp_theta + cfg.pid.ki_theta/s + ...
    cfg.pid.kd_theta*s/(tau*s + 1), cfg.minreal_tol);
pid_data.K_phi = minreal(cfg.pid.kp_phi + cfg.pid.ki_phi/s + ...
    cfg.pid.kd_phi*s/(tau*s + 1), cfg.minreal_tol);

% Paso 2: nombrar entradas y salidas para que las interconexiones y el
% reporte sean legibles.
pid_data.K_theta.InputName = {'e_theta'};
pid_data.K_theta.OutputName = {'elevator'};
pid_data.K_phi.InputName = {'e_phi'};
pid_data.K_phi.OutputName = {'aileron'};

% Paso 3: guardar ganancias y cerrar cada lazo SISO nominal para diagnostico.
pid_data.gains = cfg.pid;
pid_data.theta_loop = feedback(channels.theta*pid_data.K_theta, 1);
pid_data.phi_loop = feedback(channels.phi*pid_data.K_phi, 1);

% Paso 4: verificar estabilidad nominal de los lazos desacoplados.
pid_data.theta_stable = isstable(pid_data.theta_loop);
pid_data.phi_stable = isstable(pid_data.phi_loop);

pid_data.notes = ['Control clasico tipo SAS/CAS: PI de angulo mas ', ...
    'amortiguamiento por velocidad angular filtrada.'];
end
