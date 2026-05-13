function results = analisis_resultados(p, A, B, C, K)
%ANALISIS_RESULTADOS Ejecuta los casos minimos de validacion del proyecto.

results.controlabilidad_rank = rank(ctrb(A, B));
results.observabilidad_full_rank = rank(obsv(A, C));
results.observabilidad_medida_rank = rank(obsv(A, p.C_meas));
results.polos_abiertos = eig(A);
results.polos_cerrados = eig(A - B*K);
results.observador = diseno_observador_pendulo(A, B, p.C_meas, K);
results.observador.sim = simulacion_observador_lineal_pendulo(A, B, ...
    p.C_meas, K, results.observador.L, p.x0_default, zeros(4, 1), p.t_final);

theta_cases_deg = [12, 15, 20, 30];
for i = 1:numel(theta_cases_deg)
    x0 = [0; 0; deg2rad(theta_cases_deg(i)); 0];
    results.lineal(i) = simulacion_lineal_pendulo(A, B, K, x0, p.t_final);
    results.nolineal_sat(i) = simulacion_no_lineal_pendulo(p, K, x0, ...
        saturar=true, t_final=p.t_final);
end

results.nolineal_disturbio = simulacion_no_lineal_pendulo(p, K, p.x0_default, ...
    saturar=true, disturbio=true, d_amp=2.0, d_t0=1.0, d_t1=1.06, ...
    t_final=p.t_final);

variants(1).name = "Base";
variants(1).Q = p.Q;
variants(1).R = p.R;
variants(2).name = "Mas peso en theta";
variants(2).Q = diag([10, 1, 900, 40]);
variants(2).R = p.R;
variants(3).name = "Menos esfuerzo";
variants(3).Q = p.Q;
variants(3).R = 0.25;

for i = 1:numel(variants)
    lqr_variant = diseno_lqr_pendulo(A, B, p, variants(i).Q, variants(i).R);
    variants(i).K = lqr_variant.K;
    variants(i).polos = lqr_variant.polos_cerrados;
    variants(i).sim = simulacion_lineal_pendulo(A, B, lqr_variant.K, ...
        p.x0_default, p.t_final);
end
results.variantes_lqr = variants;

crear_graficas(results, p);
end

function crear_graficas(results, p)
out_dir = fullfile(fileparts(mfilename('fullpath')), 'figures');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

base_lin = results.lineal(2);
base_nl = results.nolineal_sat(2);

fig = figure('Name', 'Comparacion lineal vs no lineal', 'Visible', 'off');
tiledlayout(3, 1);
nexttile;
plot(base_lin.t, base_lin.x(:, 1), 'LineWidth', 1.3); hold on;
plot(base_nl.t, base_nl.x(:, 1), '--', 'LineWidth', 1.3);
ylabel('x [m]');
grid on;
legend('Lineal', 'No lineal sat.', 'Location', 'best');
nexttile;
plot(base_lin.t, rad2deg(base_lin.x(:, 3)), 'LineWidth', 1.3); hold on;
plot(base_nl.t, rad2deg(base_nl.x(:, 3)), '--', 'LineWidth', 1.3);
ylabel('\theta [deg]');
grid on;
nexttile;
plot(base_lin.t, base_lin.u, 'LineWidth', 1.3); hold on;
plot(base_nl.t, base_nl.u, '--', 'LineWidth', 1.3);
yline(p.umax, ':'); yline(-p.umax, ':');
ylabel('u [N]');
xlabel('t [s]');
grid on;
exportgraphics(fig, fullfile(out_dir, 'comparacion_lineal_no_lineal.png'));
close(fig);

fig = figure('Name', 'Barrido condiciones iniciales', 'Visible', 'off');
hold on;
for i = 1:numel(results.nolineal_sat)
    sim = results.nolineal_sat(i);
    plot(sim.t, rad2deg(sim.x(:, 3)), 'LineWidth', 1.2, ...
        'DisplayName', sprintf('%g deg', rad2deg(sim.x0(3))));
end
yline(2, ':'); yline(-2, ':');
grid on;
xlabel('t [s]');
ylabel('\theta [deg]');
legend('Location', 'best');
exportgraphics(fig, fullfile(out_dir, 'barrido_condiciones_iniciales.png'));
close(fig);

fig = figure('Name', 'Variantes Q R', 'Visible', 'off');
hold on;
for i = 1:numel(results.variantes_lqr)
    variant = results.variantes_lqr(i);
    plot(variant.sim.t, rad2deg(variant.sim.x(:, 3)), 'LineWidth', 1.2, ...
        'DisplayName', variant.name);
end
grid on;
xlabel('t [s]');
ylabel('\theta [deg]');
legend('Location', 'best');
exportgraphics(fig, fullfile(out_dir, 'variantes_lqr.png'));
close(fig);

fig = figure('Name', 'Observador lineal', 'Visible', 'off');
tiledlayout(2, 1);
nexttile;
plot(results.observador.sim.t, rad2deg(results.observador.sim.x(:, 3)), ...
    'LineWidth', 1.3); hold on;
plot(results.observador.sim.t, rad2deg(results.observador.sim.xhat(:, 3)), ...
    '--', 'LineWidth', 1.3);
grid on;
ylabel('\theta [deg]');
legend('Real', 'Estimado', 'Location', 'best');
nexttile;
plot(results.observador.sim.t, results.observador.sim.e, 'LineWidth', 1.1);
grid on;
xlabel('t [s]');
ylabel('e = x - xhat');
exportgraphics(fig, fullfile(out_dir, 'observador_lineal.png'));
close(fig);
end
