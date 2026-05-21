function analysis = analisis_planta_uav(plant, channels, cfg)
%ANALISIS_PLANTA_UAV Calcula datos base de la planta y canales.

% Paso 1: polos de cada modelo. Sirven para saber si la planta nominal ya
% trae modos lentos, rapidos o inestables antes de cerrar el lazo.
analysis.full.poles = pole(plant.full);
analysis.lat.poles = pole(plant.lat);
analysis.long.poles = pole(plant.long);

% Paso 2: ceros de transmision. En H_inf importan porque limitan que tan
% rapido puede ser el seguimiento sin pagar con control o sobreimpulso.
% El numero de ceros puede cambiar entre full, lat y long porque tzero
% calcula ceros invariantes de cada mapa entrada-salida completo. Al separar
% la planta en modelos lateral/longitudinal se cambian entradas, salidas y
% estados retenidos, asi que la estructura de ceros no tiene que coincidir
% ni ser la suma de las partes.
analysis.full.tzeros = tzero(plant.full);
analysis.lat.tzeros = tzero(plant.lat);
analysis.long.tzeros = tzero(plant.long);

% Paso 3: ganancias DC para revisar signos y acoplamientos de baja
% frecuencia entre entradas y salidas.
analysis.full.dcgain = safe_dcgain(plant.full);
analysis.lat.dcgain = safe_dcgain(plant.lat);
analysis.long.dcgain = safe_dcgain(plant.long);

% Paso 4: repetir el diagnostico para cada canal usado en diseño.
analysis.channels.theta = channel_info(channels.theta);
analysis.channels.phi = channel_info(channels.phi);
analysis.channels.q = channel_info(channels.q);
analysis.channels.p = channel_info(channels.p);
analysis.channels.r = channel_info(channels.r);
analysis.channels.angle_mimo = channel_info(channels.angle_mimo);
analysis.channels.lat_mimo = channel_info(channels.lat_mimo);

% Paso 5: controlabilidad. Si el rango es menor que el numero de estados,
% no todos los modos pueden moverse con las entradas disponibles. En el
% modelo completo esto puede pasar por estados cinematicos, acoplamientos o
% actuadores que no excitan todos los modos. En los modelos desacoplados se
% analiza un subconjunto mas pequeno, por eso el rango puede coincidir con n.
analysis.controllability.full_rank = rank(ctrb(plant.full.A, plant.full.B));
analysis.controllability.full_n = size(plant.full.A, 1);
analysis.controllability.lat_rank = rank(ctrb(plant.lat.A, plant.lat.B));
analysis.controllability.lat_n = size(plant.lat.A, 1);
analysis.controllability.long_rank = rank(ctrb(plant.long.A, plant.long.B));
analysis.controllability.long_n = size(plant.long.A, 1);

% Paso 6: observabilidad. Se verifica que las salidas del modelo contengan
% informacion suficiente sobre los estados del sistema lineal.
analysis.observability.full_rank = rank(obsv(plant.full.A, plant.full.C));
analysis.observability.full_n = size(plant.full.A, 1);
analysis.observability.lat_rank = rank(obsv(plant.lat.A, plant.lat.C));
analysis.observability.lat_n = size(plant.lat.A, 1);
analysis.observability.long_rank = rank(obsv(plant.long.A, plant.long.C));
analysis.observability.long_n = size(plant.long.A, 1);

analysis.notes = sprintf(['Ancho de banda objetivo: %.2f rad/s. ', ...
    'Perturbaciones de entrada hasta %.2f rad/s.'], ...
    cfg.spec.wb, cfg.spec.wp);
end

function info = channel_info(sys)
%CHANNEL_INFO Resume orden, polos, ceros, ganancia y estabilidad.
info.order = order(sys);
info.size = size(sys);
info.poles = pole(sys);
info.zeros = tzero(sys);
info.tzeros = info.zeros;
info.dcgain = safe_dcgain(sys);
info.stable = isstable(sys);
end

function g = safe_dcgain(sys)
%SAFE_DCGAIN Evita que una ganancia DC no computable detenga todo el flujo.
try
    g = dcgain(sys);
catch
    g = NaN(size(sys));
end
end
