function p = parametros_pendulo()
%PARAMETROS_PENDULO Parametros nominales del pendulo invertido sobre carro.
%
% Convencion:
%   theta = 0 rad representa el pendulo vertical hacia arriba.
%   x_s = [x; x_dot; theta; theta_dot].
%   u es la fuerza horizontal aplicada al carro en N.

p.M = 0.50;        % masa del carro [kg]
p.m = 0.20;        % masa del pendulo [kg]
p.l = 0.30;        % distancia al centro de masa [m]
p.b = 0.10;        % friccion viscosa del carro [N*s/m]
p.g = 9.81;        % gravedad [m/s^2]
p.umax = 10.0;     % limite simetrico del actuador [N]

p.x0_default = [0; 0; deg2rad(5); 0];
p.t_final = 6.0;

% Seleccion base LQR. Se penaliza fuerte theta porque es el estado critico
% para mantener el pendulo en la region local de validez.
p.Q = diag([10, 1, 300, 20]);
p.R = 0.05;

% Salidas medidas para el caso con observador: posicion y angulo.
p.C_meas = [1 0 0 0;
            0 0 1 0];
end
