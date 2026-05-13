function sim = simulacion_no_lineal_pendulo(p, K, x0, opts)
%SIMULACION_NO_LINEAL_PENDULO Simula la planta no lineal con u = sat(-K*x).
%
% opts.saturar     : true/false, aplica saturacion de actuador.
% opts.disturbio   : true/false, agrega pulso de fuerza externa.
% opts.d_amp       : amplitud del pulso [N].
% opts.d_t0/d_t1   : intervalo del pulso [s].
% opts.t_final     : tiempo final [s].

arguments
    p struct
    K double
    x0 double = p.x0_default
    opts.saturar logical = true
    opts.disturbio logical = false
    opts.d_amp double = 0
    opts.d_t0 double = 1.0
    opts.d_t1 double = 1.05
    opts.t_final double = p.t_final
end

ode_opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
dynamics = @(t, x) pendulo_dinamica_no_lineal(t, x, p, K, opts);
[t, x] = ode45(dynamics, [0 opts.t_final], x0(:), ode_opts);

u_unsat = -(K * x.').';
u = u_unsat;
if opts.saturar
    u = min(max(u_unsat, -p.umax), p.umax);
end

d = arrayfun(@(tk) fuerza_disturbio(tk, opts), t);

sim.t = t;
sim.x = x;
sim.u = u;
sim.u_unsat = u_unsat;
sim.d = d;
sim.saturar = opts.saturar;
sim.x0 = x0(:);
sim.max_abs_theta_deg = max(abs(rad2deg(x(:, 3))));
sim.max_abs_x_m = max(abs(x(:, 1)));
sim.max_abs_u_N = max(abs(u));
end

function dx = pendulo_dinamica_no_lineal(t, xs, p, K, opts)
u = -K * xs(:);

if opts.saturar
    u = min(max(u, -p.umax), p.umax);
end

u = u + fuerza_disturbio(t, opts);

x_dot = xs(2);
theta = xs(3);
theta_dot = xs(4);

sin_t = sin(theta);
cos_t = cos(theta);
den = p.M + p.m * sin_t^2;

x_ddot = (u - p.b*x_dot - p.m*p.g*sin_t*cos_t ...
          + p.m*p.l*theta_dot^2*sin_t) / den;
theta_ddot = (p.g*sin_t - x_ddot*cos_t) / p.l;

dx = [x_dot; x_ddot; theta_dot; theta_ddot];
end

function d = fuerza_disturbio(t, opts)
if opts.disturbio && t >= opts.d_t0 && t <= opts.d_t1
    d = opts.d_amp;
else
    d = 0;
end
end
