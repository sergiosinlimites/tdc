%% Inicializacion del modelo Simulink pendulo_invertido.slx
% Este script deja en el workspace las variables que usa el modelo:
% A_lin, B_lin, C_lin, D_lin, K_lqr, x0_sim, umax.

project_dir = fileparts(mfilename('fullpath'));
addpath(project_dir);

p = parametros_pendulo();
[A_lin, B_lin, C_lin, D_lin] = modelo_lineal_pendulo(p);
lqr_data = diseno_lqr_pendulo(A_lin, B_lin, p);

K_lqr = lqr_data.K;
x0_sim = p.x0_default;
umax = p.umax;
t_final = p.t_final;
