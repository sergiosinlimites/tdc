function hinf_data = diseno_hinf_taller1(channels, cfg)
%DISENO_HINF_TALLER1 Sintesis H_inf por sensibilidad mixta.

% Paso 1: confirmar que Robust Control Toolbox esta disponible. Sin estas
% funciones no se puede ejecutar la sintesis H_inf en MATLAB.
required = {'mixsyn', 'makeweight', 'hinfsyn', 'augw'};
for k = 1:numel(required)
    if isempty(which(required{k}))
        error(['La funcion %s no esta disponible. Habilite Robust Control ', ...
            'Toolbox antes de ejecutar el taller.'], required{k});
    end
end

% Paso 2: crear pesos W1, W2 y W3. Estos pesos convierten especificaciones
% fisicas (seguimiento, control, ruido) en un problema matematico H_inf.
[W1, W2, W3] = construir_pesos(cfg);
hinf_data.weights.W1 = W1;
hinf_data.weights.W2 = W2;
hinf_data.weights.W3 = W3;

% Paso 3: sintetizar un controlador por eje. Se parte SISO porque el taller
% no exige MIMO desde el inicio y asi el resultado es mas trazable.
hinf_data.theta = synthesize_axis(channels.theta, W1, W2, W3, ...
    cfg.minreal_tol, 'theta/elevator');
hinf_data.phi = synthesize_axis(channels.phi, W1, W2, W3, ...
    cfg.minreal_tol, 'phi/aileron');

hinf_data.K_theta = hinf_data.theta.K;
hinf_data.K_phi = hinf_data.phi.K;
hinf_data.notes = ['Sintesis SISO por eje con mixsyn. La validacion ', ...
    'temporal se hace sobre linmodel acoplado.'];
end

function [W1, W2, W3] = construir_pesos(cfg)
%CONSTRUIR_PESOS Define las fronteras deseadas para S, KS y T.
% W1 se hace grande a baja frecuencia para obligar S pequeno: con ganancia
% baja-frecuencia 5, la cota aproximada es S < 1/5 = 0.2 en seguimiento.
% La transicion se ubica en wb = 2*pi*8 rad/s porque esa es la banda de
% actuador/referencia tomada del enunciado y de los modelos UAV.
W1 = makeweight(cfg.hinf.W1_low_gain, cfg.spec.wb, ...
    cfg.hinf.W1_high_gain);

% W2 es constante porque aqui no se impuso una banda especial para el
% esfuerzo: se quiere limitar K*S de forma pareja en la zona analizada. Con
% W2 = 0.15 la cota inicial es K*S < 1/0.15 ~= 6.67; se escogio como valor
% intermedio tras probar pesos mas y menos agresivos.
W2 = cfg.hinf.W2_gain;

% W3 se deja pequeno a baja frecuencia para no impedir seguimiento, y grande
% a alta frecuencia para que T caiga donde dominan ruido, perturbaciones
% rapidas y dinamicas no modeladas. La transicion usa wp = 2*pi*6 rad/s por
% la frecuencia de perturbacion especificada en el taller.
W3 = makeweight(cfg.hinf.W3_low_gain, cfg.spec.wp, ...
    cfg.hinf.W3_high_gain);

% Los nombres ayudan a leer interconexiones y salidas ponderadas.
W1.InputName = {'e'};
W1.OutputName = {'z_s'};
W3.InputName = {'y'};
W3.OutputName = {'z_t'};
end

function axis_data = synthesize_axis(G, W1, W2, W3, tol, label)
%SYNTHESIZE_AXIS Ejecuta mixsyn y resume el controlador obtenido.
[K, CL, gamma, info] = mixsyn(G, W1, W2, W3);

% Se simplifica la realizacion para retirar cancelaciones numericas obvias.
K = minreal(ss(K), tol);
CL = minreal(ss(CL), tol);

axis_data.label = label;
axis_data.K = K;
axis_data.CL = CL;
axis_data.gamma = gamma;
axis_data.info = info;
axis_data.order = order(K);
axis_data.controller_poles = pole(K);
axis_data.controller_stable = isstable(K);
axis_data.closed_loop_stable = isstable(CL);
end
