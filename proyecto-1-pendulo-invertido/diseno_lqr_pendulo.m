function lqr_data = diseno_lqr_pendulo(A, B, p, Q, R)
%DISENO_LQR_PENDULO Disena realimentacion de estados u = -K*x.

if nargin < 4 || isempty(Q)
    Q = p.Q;
end

if nargin < 5 || isempty(R)
    R = p.R;
end

[K, S, polos_cerrados] = lqr(A, B, Q, R);

lqr_data.K = K;
lqr_data.S = S;
lqr_data.Q = Q;
lqr_data.R = R;
lqr_data.Acl = A - B*K;
lqr_data.polos_cerrados = polos_cerrados;
end
