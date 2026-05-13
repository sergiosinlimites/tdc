function sim = simulacion_observador_lineal_pendulo(A, B, C_meas, K, L, x0, xhat0, t_final)
%SIMULACION_OBSERVADOR_LINEAL_PENDULO Lazo lineal con estados estimados.

if nargin < 7 || isempty(xhat0)
    xhat0 = zeros(size(x0(:)));
end

if nargin < 8
    t_final = 6.0;
end

n = size(A, 1);
Acl_aug = [A,              -B*K;
           L*C_meas, A - B*K - L*C_meas];

t = linspace(0, t_final, 1201).';
sys_aug = ss(Acl_aug, zeros(2*n, 1), eye(2*n), zeros(2*n, 1));
[z, ~] = initial(sys_aug, [x0(:); xhat0(:)], t);

x = z(:, 1:n);
xhat = z(:, n+1:end);
u = -(K * xhat.').';
e = x - xhat;

sim.t = t;
sim.x = x;
sim.xhat = xhat;
sim.e = e;
sim.u = u;
sim.max_abs_estimation_error = max(vecnorm(e, 2, 2));
sim.max_abs_theta_deg = max(abs(rad2deg(x(:, 3))));
sim.max_abs_x_m = max(abs(x(:, 1)));
sim.max_abs_u_N = max(abs(u));
end
