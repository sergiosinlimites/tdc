function channels = seleccionar_canales_uav(plant, cfg)
%SELECCIONAR_CANALES_UAV Define las plantas usadas para diseno y validacion.

tol = cfg.minreal_tol;

% Paso 1: extraer canales longitudinales SISO para diseno de theta y q.
channels.theta = minreal(plant.long('theta', 'elevator'), tol);
channels.theta.InputName = {'elevator'};
channels.theta.OutputName = {'theta'};

channels.q = minreal(plant.long('q', 'elevator'), tol);
channels.q.InputName = {'elevator'};
channels.q.OutputName = {'q'};

% Paso 2: extraer canales laterales SISO para diseno de phi, p y r.
channels.phi = minreal(plant.lat('phi', 'aileron'), tol);
channels.phi.InputName = {'aileron'};
channels.phi.OutputName = {'phi'};

channels.p = minreal(plant.lat('p', 'aileron'), tol);
channels.p.InputName = {'aileron'};
channels.p.OutputName = {'p'};

channels.r = minreal(plant.lat('r', 'rudder'), tol);
channels.r.InputName = {'rudder'};
channels.r.OutputName = {'r'};

% Paso 3: preparar modelos MIMO para diagnosticar acoplamiento y una
% posible segunda iteracion de control robusto multivariable.
channels.angle_mimo = minreal(plant.full({'theta', 'phi'}, ...
    {'elevator', 'aileron'}), tol);
channels.angle_mimo.InputName = {'elevator', 'aileron'};
channels.angle_mimo.OutputName = {'theta', 'phi'};

channels.lat_mimo = minreal(plant.lat({'phi', 'r'}, ...
    {'aileron', 'rudder'}), tol);
channels.lat_mimo.InputName = {'aileron', 'rudder'};
channels.lat_mimo.OutputName = {'phi', 'r'};
end
