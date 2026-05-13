function animacion_pendulo_interactiva()
%ANIMACION_PENDULO_INTERACTIVA Simulacion visual interactiva del pendulo.
%
% Abre una ventana con el carro y el pendulo moviendose en tiempo real.
% El deslizador aplica una fuerza horizontal al carro. Si el LQR esta activo,
% esa fuerza se suma como perturbacion externa; si se desactiva, la fuerza del
% deslizador pasa a ser la entrada manual completa.

project_dir = fileparts(mfilename('fullpath'));
addpath(project_dir);

p = parametros_pendulo();
[A, B] = modelo_lineal_pendulo(p);
lqr_data = diseno_lqr_pendulo(A, B, p);
K = lqr_data.K;

dt = 0.02;
t = 0;
xs = p.x0_default(:);
manual_force = 0;
lqr_enabled = true;
paused = false;

cart_w = 0.34;
cart_h = 0.16;
wheel_r = 0.035;
rod_len = 2*p.l;
axis_half_width = 1.25;

fig = figure( ...
    'Name', 'Pendulo invertido interactivo', ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'CloseRequestFcn', @cerrar_figura);

ax = axes( ...
    'Parent', fig, ...
    'Units', 'normalized', ...
    'Position', [0.07 0.29 0.86 0.66]);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlim(ax, [-axis_half_width axis_half_width]);
ylim(ax, [-0.18 0.86]);
xlabel(ax, 'x del carro [m]');
ylabel(ax, 'altura [m]');
title(ax, 'Pendulo invertido sobre carro');

plot(ax, [-axis_half_width axis_half_width], [0 0], 'Color', [0.25 0.25 0.25], 'LineWidth', 1.5);
cart_patch = rectangle(ax, 'Position', [0 0 cart_w cart_h], ...
    'Curvature', 0.08, 'FaceColor', [0.15 0.42 0.75], ...
    'EdgeColor', [0.08 0.18 0.30], 'LineWidth', 1.5);
wheel_l = rectangle(ax, 'Position', [0 0 2*wheel_r 2*wheel_r], ...
    'Curvature', 1, 'FaceColor', [0.08 0.08 0.08], 'EdgeColor', 'none');
wheel_rh = rectangle(ax, 'Position', [0 0 2*wheel_r 2*wheel_r], ...
    'Curvature', 1, 'FaceColor', [0.08 0.08 0.08], 'EdgeColor', 'none');
rod_line = plot(ax, [0 0], [0 0], 'LineWidth', 4, 'Color', [0.86 0.24 0.18]);
bob_marker = plot(ax, 0, 0, 'o', 'MarkerSize', 18, ...
    'MarkerFaceColor', [0.96 0.68 0.18], 'MarkerEdgeColor', [0.45 0.25 0.04], ...
    'LineWidth', 1.5);
pivot_marker = plot(ax, 0, 0, 'o', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.05 0.05 0.05], 'MarkerEdgeColor', 'none');
force_arrow = quiver(ax, 0, 0, 0, 0, 0, ...
    'LineWidth', 2.0, 'MaxHeadSize', 0.8, 'Color', [0.08 0.55 0.25]);

status_text = uicontrol(fig, 'Style', 'text', ...
    'Units', 'normalized', ...
    'Position', [0.07 0.20 0.86 0.045], ...
    'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', ...
    'FontName', 'Consolas', ...
    'String', '');

uicontrol(fig, 'Style', 'text', ...
    'Units', 'normalized', ...
    'Position', [0.07 0.135 0.25 0.035], ...
    'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', ...
    'String', 'Fuerza manual / disturbio [N]');

force_slider = uicontrol(fig, 'Style', 'slider', ...
    'Units', 'normalized', ...
    'Position', [0.32 0.142 0.36 0.03], ...
    'Min', -p.umax, ...
    'Max', p.umax, ...
    'Value', 0, ...
    'Callback', @cambiar_fuerza);

force_label = uicontrol(fig, 'Style', 'text', ...
    'Units', 'normalized', ...
    'Position', [0.70 0.135 0.11 0.035], ...
    'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', ...
    'FontName', 'Consolas', ...
    'String', '0.00 N');

uicontrol(fig, 'Style', 'checkbox', ...
    'Units', 'normalized', ...
    'Position', [0.07 0.075 0.18 0.04], ...
    'BackgroundColor', 'w', ...
    'Value', 1, ...
    'String', 'LQR activo', ...
    'Callback', @cambiar_lqr);

pause_button = uicontrol(fig, 'Style', 'pushbutton', ...
    'Units', 'normalized', ...
    'Position', [0.29 0.075 0.13 0.045], ...
    'String', 'Pausar', ...
    'Callback', @alternar_pausa);

uicontrol(fig, 'Style', 'pushbutton', ...
    'Units', 'normalized', ...
    'Position', [0.45 0.075 0.13 0.045], ...
    'String', 'Reiniciar', ...
    'Callback', @reiniciar);

uicontrol(fig, 'Style', 'pushbutton', ...
    'Units', 'normalized', ...
    'Position', [0.61 0.075 0.13 0.045], ...
    'String', 'Empujon +', ...
    'Callback', @(~, ~) aplicar_empujon(0.8));

uicontrol(fig, 'Style', 'pushbutton', ...
    'Units', 'normalized', ...
    'Position', [0.77 0.075 0.13 0.045], ...
    'String', 'Empujon -', ...
    'Callback', @(~, ~) aplicar_empujon(-0.8));

timer_obj = timer( ...
    'ExecutionMode', 'fixedRate', ...
    'Period', dt, ...
    'BusyMode', 'drop', ...
    'TimerFcn', @avanzar_simulacion);

actualizar_dibujo(0);
start(timer_obj);

    function avanzar_simulacion(~, ~)
        if ~ishandle(fig)
            detener_timer();
            return;
        end

        if paused
            return;
        end

        xs = rk4_step(xs, dt);
        t = t + dt;

        if abs(xs(1)) > axis_half_width
            xs(1) = sign(xs(1))*axis_half_width;
            xs(2) = 0;
        end

        actualizar_dibujo(calcular_control(xs));
    end

    function u = calcular_control(x_actual)
        if lqr_enabled
            u = -K*x_actual + manual_force;
        else
            u = manual_force;
        end

        u = min(max(u, -p.umax), p.umax);
    end

    function x_next = rk4_step(x_actual, h)
        k1 = dinamica_con_control(x_actual);
        k2 = dinamica_con_control(x_actual + 0.5*h*k1);
        k3 = dinamica_con_control(x_actual + 0.5*h*k2);
        k4 = dinamica_con_control(x_actual + h*k3);
        x_next = x_actual + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
    end

    function dx = dinamica_con_control(x_actual)
        dx = dinamica_no_lineal(x_actual, calcular_control(x_actual));
    end

    function dx = dinamica_no_lineal(x_actual, u)
        x_dot = x_actual(2);
        theta = x_actual(3);
        theta_dot = x_actual(4);

        sin_t = sin(theta);
        cos_t = cos(theta);
        den = p.M + p.m*sin_t^2;

        x_ddot = (u - p.b*x_dot - p.m*p.g*sin_t*cos_t ...
            + p.m*p.l*theta_dot^2*sin_t) / den;
        theta_ddot = (p.g*sin_t - x_ddot*cos_t) / p.l;

        dx = [x_dot; x_ddot; theta_dot; theta_ddot];
    end

    function actualizar_dibujo(total_force)
        cart_x = xs(1);
        theta = xs(3);

        cart_left = cart_x - cart_w/2;
        cart_bottom = wheel_r;
        pivot_x = cart_x;
        pivot_y = cart_bottom + cart_h;
        bob_x = pivot_x + rod_len*sin(theta);
        bob_y = pivot_y + rod_len*cos(theta);

        set(cart_patch, 'Position', [cart_left cart_bottom cart_w cart_h]);
        set(wheel_l, 'Position', [cart_x - 0.11 - wheel_r, 0, 2*wheel_r, 2*wheel_r]);
        set(wheel_rh, 'Position', [cart_x + 0.11 - wheel_r, 0, 2*wheel_r, 2*wheel_r]);
        set(rod_line, 'XData', [pivot_x bob_x], 'YData', [pivot_y bob_y]);
        set(bob_marker, 'XData', bob_x, 'YData', bob_y);
        set(pivot_marker, 'XData', pivot_x, 'YData', pivot_y);
        set(force_arrow, ...
            'XData', cart_x, ...
            'YData', cart_bottom + 0.5*cart_h, ...
            'UData', 0.30*total_force/p.umax, ...
            'VData', 0);

        mode_text = 'manual';
        if lqr_enabled
            mode_text = 'LQR + disturbio';
        end

        set(status_text, 'String', sprintf( ...
            't = %5.2f s | modo = %-14s | u = %+6.2f N | x = %+6.3f m | theta = %+7.2f deg', ...
            t, mode_text, total_force, xs(1), rad2deg(xs(3))));

        drawnow limitrate;
    end

    function cambiar_fuerza(src, ~)
        manual_force = get(src, 'Value');
        set(force_label, 'String', sprintf('%+.2f N', manual_force));
    end

    function cambiar_lqr(src, ~)
        lqr_enabled = logical(get(src, 'Value'));
    end

    function alternar_pausa(~, ~)
        paused = ~paused;
        if paused
            set(pause_button, 'String', 'Reanudar');
        else
            set(pause_button, 'String', 'Pausar');
        end
    end

    function reiniciar(~, ~)
        t = 0;
        xs = p.x0_default(:);
        manual_force = 0;
        set(force_slider, 'Value', 0);
        set(force_label, 'String', '0.00 N');
        actualizar_dibujo(calcular_control(xs));
    end

    function aplicar_empujon(delta_theta_dot)
        xs(4) = xs(4) + delta_theta_dot;
    end

    function cerrar_figura(~, ~)
        detener_timer();
        delete(fig);
    end

    function detener_timer()
        if exist('timer_obj', 'var') && isa(timer_obj, 'timer') && isvalid(timer_obj)
            stop(timer_obj);
            delete(timer_obj);
        end
    end
end
