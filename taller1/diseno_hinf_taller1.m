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
% Se permiten pesos distintos por eje para poder apretar phi sin mover theta.
theta_weights = construir_pesos(cfg_for_axis(cfg, 'theta'));
phi_weights = construir_pesos(cfg_for_axis(cfg, 'phi'));
hinf_data.weights.theta = theta_weights;
hinf_data.weights.phi = phi_weights;
hinf_data.weights.W1 = theta_weights.W1;
hinf_data.weights.W2 = theta_weights.W2;
hinf_data.weights.W3 = theta_weights.W3;

% Paso 3: sintetizar un controlador por eje. Se parte SISO porque el taller
% no exige MIMO desde el inicio y asi el resultado es mas trazable.
hinf_data.theta = synthesize_axis(channels.theta, theta_weights.W1, ...
    theta_weights.W2, theta_weights.W3, cfg.minreal_tol, ...
    'theta/elevator');
hinf_data.phi = synthesize_axis(channels.phi, phi_weights.W1, ...
    phi_weights.W2, phi_weights.W3, cfg.minreal_tol, 'phi/aileron');

hinf_data.K_theta = hinf_data.theta.K;
hinf_data.K_phi = hinf_data.phi.K;
hinf_data.notes = ['Sintesis SISO por eje con mixsyn. La validacion ', ...
    'temporal se hace sobre linmodel acoplado.'];
end

function axis_cfg = cfg_for_axis(cfg, axis_name)
%CFG_FOR_AXIS Mezcla pesos globales con sobreescrituras por eje.
axis_cfg = cfg;
if ~isfield(cfg.hinf, axis_name)
    return;
end

axis_hinf = cfg.hinf.(axis_name);
names = fieldnames(axis_hinf);
for k = 1:numel(names)
    axis_cfg.hinf.(names{k}) = axis_hinf.(names{k});
end
end

function weights = construir_pesos(cfg)
%CONSTRUIR_PESOS Define las fronteras deseadas para S, KS y T.
% W1 se hace grande a baja frecuencia para obligar S pequeno: la cota
% aproximada de seguimiento queda S < 1/W1_low_gain.
% La transicion se ubica en wb = 2*pi*8 rad/s porque esa es la banda de
% actuador/referencia tomada del enunciado y de los modelos UAV.
W1 = makeweight(cfg.hinf.W1_low_gain, cfg.hinf.W1_cross_frequency, ...
    cfg.hinf.W1_high_gain);

% W2 penaliza esfuerzo de control. Si W2_low_gain y W2_high_gain son
% diferentes, se crea un peso propio que aumenta con la frecuencia para
% castigar control ruidoso; si son iguales, queda el peso constante
% seleccionado por el barrido temporal.
W2 = construir_peso_control(cfg);

% W3 se deja pequeno a baja frecuencia para no impedir seguimiento, y grande
% a alta frecuencia para que T caiga donde dominan ruido, perturbaciones
% rapidas y dinamicas no modeladas. La transicion usa wp = 2*pi*6 rad/s por
% la frecuencia de perturbacion especificada en el taller.
W3 = makeweight(cfg.hinf.W3_low_gain, cfg.hinf.W3_cross_frequency, ...
    cfg.hinf.W3_high_gain);

% Los nombres ayudan a leer interconexiones y salidas ponderadas.
W1.InputName = {'e'};
W1.OutputName = {'z_s'};
if isa(W2, 'DynamicSystem')
    W2.InputName = {'u'};
    W2.OutputName = {'z_u'};
end
W3.InputName = {'y'};
W3.OutputName = {'z_t'};

weights.W1 = W1;
weights.W2 = W2;
weights.W3 = W3;
end

function W2 = construir_peso_control(cfg)
%CONSTRUIR_PESO_CONTROL Crea W2 constante o con rolloff reproducible.
low_gain = cfg.hinf.W2_low_gain;
high_gain = cfg.hinf.W2_high_gain;

if abs(high_gain - low_gain) < 1e-12
    W2 = cfg.hinf.W2_gain;
    return;
end

s = tf('s');
wz = cfg.hinf.W2_cross_frequency;
wp = wz*high_gain/low_gain;
W2 = low_gain*(s/wz + 1)/(s/wp + 1);
W2 = minreal(W2, cfg.minreal_tol);
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
