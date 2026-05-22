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

% Paso 1: disenar explicitamente SAS y CAS. Las funciones de root locus
% tambien exportan las figuras que justifican signo y magnitud.
export_figures = true;
if isfield(cfg, 'export_design_figures')
    export_figures = cfg.export_design_figures;
end
sas_data = diseno_sas_root_locus(channels, cfg, export_figures);
cas_data = diseno_cas_pi_root_locus(channels, sas_data, cfg, export_figures);

gains = cfg.pid;
gains.kd_theta = sas_data.D_q;
gains.kd_phi = sas_data.D_p;
gains.kp_theta = cas_data.theta.Kp;
gains.ki_theta = cas_data.theta.Ki;
gains.kp_phi = cas_data.phi.Kp;
gains.ki_phi = cas_data.phi.Ki;

% Paso 2: construir los controladores equivalentes de angulo. Se usan dos ejes porque el
% taller controla pitch con elevator (theta/elevator) y roll con aileron
% (phi/aileron). Yaw no se trata como tracking principal; se deja con un
% damper de r para amortiguar el modo lateral.
pid_data.K_theta = minreal(gains.kp_theta + gains.ki_theta/s + ...
    gains.kd_theta*s/(tau*s + 1), cfg.minreal_tol);
pid_data.K_phi = minreal(gains.kp_phi + gains.ki_phi/s + ...
    gains.kd_phi*s/(tau*s + 1), cfg.minreal_tol);

% Paso 3: nombrar entradas y salidas para que las interconexiones y el
% reporte sean legibles.
pid_data.K_theta.InputName = {'e_theta'};
pid_data.K_theta.OutputName = {'elevator'};
pid_data.K_phi.InputName = {'e_phi'};
pid_data.K_phi.OutputName = {'aileron'};

% Paso 4: guardar ganancias y cerrar cada lazo SISO nominal para diagnostico.
pid_data.sas = sas_data;
pid_data.cas = cas_data;
pid_data.gains = gains;
pid_data.theta_loop = feedback(channels.theta*pid_data.K_theta, 1);
pid_data.phi_loop = feedback(channels.phi*pid_data.K_phi, 1);

% Paso 5: verificar estabilidad nominal de los lazos desacoplados.
pid_data.theta_stable = isstable(pid_data.theta_loop);
pid_data.phi_stable = isstable(pid_data.phi_loop);

pid_data.notes = ['Control SAS/CAS propio: CAS PI de angulo mas SAS ', ...
    'por velocidad angular medido y justificado con root locus.'];
end
