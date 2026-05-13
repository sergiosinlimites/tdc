function [A, B, C, D, sys] = modelo_lineal_pendulo(p)
%MODELO_LINEAL_PENDULO Modelo lineal alrededor del equilibrio vertical.
%
% El modelo parte de la planta no lineal de pendulo puntual:
%   (M+m)xdd + b*xd + m*l*(thetadd*cos(theta)-thetad^2*sin(theta)) = u
%   l*thetadd = g*sin(theta) - xdd*cos(theta)
%
% Linealizado en theta = 0:
%   xdd      = (u - b*x_dot - m*g*theta)/M
%   thetadd = ((M+m)*g*theta + b*x_dot - u)/(M*l)

M = p.M;
m = p.m;
l = p.l;
b = p.b;
g = p.g;

A = [0,      1,             0, 0;
     0,   -b/M,        -m*g/M, 0;
     0,      0,             0, 1;
     0, b/(M*l), (M+m)*g/(M*l), 0];

B = [0;
     1/M;
     0;
    -1/(M*l)];

C = eye(4);
D = zeros(4, 1);

sys = ss(A, B, C, D);
sys.StateName = {'x', 'x_dot', 'theta', 'theta_dot'};
sys.InputName = {'u'};
sys.OutputName = sys.StateName;
end
