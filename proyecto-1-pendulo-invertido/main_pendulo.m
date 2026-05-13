%% Proyecto 1: Pendulo invertido sobre carro
% Ejecuta el flujo completo:
% 1) parametros y modelo lineal,
% 2) controlabilidad/observabilidad,
% 3) LQR,
% 4) simulaciones lineales/no lineales con saturacion,
% 5) graficas y resumen numerico.

clear; clc; close all;

project_dir = fileparts(mfilename('fullpath'));
addpath(project_dir);

p = parametros_pendulo();
[A, B, C, D, sys] = modelo_lineal_pendulo(p);
lqr_data = diseno_lqr_pendulo(A, B, p);
K = lqr_data.K;

results = analisis_resultados(p, A, B, C, K);

fprintf('\n=== Parametros nominales ===\n');
fprintf('M = %.3f kg, m = %.3f kg, l = %.3f m, b = %.3f N*s/m, g = %.2f m/s^2\n', ...
    p.M, p.m, p.l, p.b, p.g);
fprintf('Saturacion: |u| <= %.2f N\n', p.umax);

fprintf('\n=== Modelo lineal ===\n');
disp('A ='); disp(A);
disp('B ='); disp(B);
disp('C ='); disp(C);
disp('D ='); disp(D);
disp('Polos abiertos ='); disp(results.polos_abiertos.');
fprintf('rank(ctrb(A,B)) = %d de 4\n', results.controlabilidad_rank);
fprintf('rank(obsv(A,C)) = %d de 4\n', results.observabilidad_full_rank);
fprintf('rank(obsv(A,C_meas)) = %d de 4\n', results.observabilidad_medida_rank);

fprintf('\n=== LQR ===\n');
disp('Q ='); disp(lqr_data.Q);
fprintf('R = %.4f\n', lqr_data.R);
disp('K ='); disp(K);
disp('Polos cerrados ='); disp(results.polos_cerrados.');

fprintf('\n=== Observador de Luenberger ===\n');
disp('C_meas ='); disp(results.observador.C_meas);
disp('L ='); disp(results.observador.L);
disp('Polos del observador ='); disp(results.observador.polos.');
fprintf('Error maximo ||x - xhat||_2 con xhat(0)=0: %.4f\n', ...
    results.observador.sim.max_abs_estimation_error);

fprintf('\n=== Casos no lineales con saturacion ===\n');
for i = 1:numel(results.nolineal_sat)
    sim = results.nolineal_sat(i);
    fprintf('theta0 = %5.1f deg | max |x| = %.3f m | max |theta| = %.2f deg | max |u| = %.2f N\n', ...
        rad2deg(sim.x0(3)), sim.max_abs_x_m, sim.max_abs_theta_deg, sim.max_abs_u_N);
end

fprintf('\nGraficas guardadas en: %s\n', fullfile(project_dir, 'figures'));
fprintf('Modelo SS MATLAB disponible en variable sys. Ejecuta build_pendulo_simulink para regenerar el .slx.\n');
