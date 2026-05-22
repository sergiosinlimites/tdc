function metrics = evaluar_saturacion_controlador(sim, cfg)
%EVALUAR_SATURACION_CONTROLADOR Calcula metricas temporales y de actuador.

limit_rad = cfg.spec.control_limit_rad;
limit_deg = cfg.spec.control_limit_deg;
post = sim.t >= cfg.sim.t_step;
tail = sim.t >= max(cfg.sim.t_final - 2, cfg.sim.t_step);

u_aw = sim.u_sat - sim.u_raw;
sat_mask = any(abs(sim.u_sat) >= limit_rad*0.999, 2);

metrics.max_abs_theta_deg = max(abs(sim.theta_deg));
metrics.max_abs_phi_deg = max(abs(sim.phi_deg));
metrics.max_abs_elevator_deg = max(abs(sim.elevator_deg));
metrics.max_abs_aileron_deg = max(abs(sim.aileron_deg));
metrics.max_abs_rudder_deg = max(abs(sim.rudder_deg));
metrics.max_abs_elevator_raw_deg = rad2deg(max(abs(sim.u_raw(:, 1))));
metrics.max_abs_aileron_raw_deg = rad2deg(max(abs(sim.u_raw(:, 2))));
metrics.max_abs_rudder_raw_deg = rad2deg(max(abs(sim.u_raw(:, 3))));
metrics.max_abs_aw_deg = rad2deg(max(abs(u_aw), [], 'all'));

theta_error = sim.theta_ref_deg - sim.theta_deg;
phi_error = sim.phi_ref_deg - sim.phi_deg;
metrics.theta_final_error_deg = mean(theta_error(tail));
metrics.phi_final_error_deg = mean(phi_error(tail));
metrics.theta_abs_final_error_deg = abs(metrics.theta_final_error_deg);
metrics.phi_abs_final_error_deg = abs(metrics.phi_final_error_deg);
metrics.theta_rms_error_deg = rms(theta_error(post));
metrics.phi_rms_error_deg = rms(phi_error(post));
metrics.sat_fraction = mean(sat_mask);
metrics.sat_time_s = trapz(sim.t, double(sat_mask));
metrics.recovery_time_s = recovery_time_after_saturation(sim.t, sat_mask);

metrics.exceeds_limit = any(abs(sim.u_raw) > limit_rad, 'all');
metrics.limit_deg = limit_deg;
end

function tr = recovery_time_after_saturation(t, sat_mask)
idx = find(sat_mask, 1, 'last');
if isempty(idx)
    tr = 0;
elseif idx == numel(t)
    tr = NaN;
else
    tr = t(idx + 1) - t(1);
end
end
