function sim = simulacion_lineal_pendulo(A, B, K, x0, t_final)
%SIMULACION_LINEAL_PENDULO Simula el sistema lineal cerrado sin saturacion.

if nargin < 5
    t_final = 6.0;
end

Acl = A - B*K;
t = linspace(0, t_final, 1201).';
sys_cl = ss(Acl, zeros(4, 1), eye(4), zeros(4, 1));
[x, ~] = initial(sys_cl, x0(:), t);
u = -(K * x.').';

sim.t = t;
sim.x = x;
sim.u = u;
sim.x0 = x0(:);
sim.max_abs_theta_deg = max(abs(rad2deg(x(:, 3))));
sim.max_abs_x_m = max(abs(x(:, 1)));
sim.max_abs_u_N = max(abs(u));
end
