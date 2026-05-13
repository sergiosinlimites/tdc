function obs = diseno_observador_pendulo(A, B, C_meas, K)
%DISENO_OBSERVADOR_PENDULO Observador de Luenberger para medir x y theta.
%
% El observador estima las velocidades cuando solo estan disponibles
% posicion del carro y angulo del pendulo:
%   y = C_meas*x
%   xhat_dot = A*xhat + B*u + L*(y - C_meas*xhat)

polos_obs = [-12, -13, -14, -15];
L = place(A.', C_meas.', polos_obs).';

obs.C_meas = C_meas;
obs.L = L;
obs.polos = eig(A - L*C_meas);
obs.polos_deseados = polos_obs(:);
obs.Acl_estimado = A - B*K - L*C_meas;
end
