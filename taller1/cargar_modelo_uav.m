 function plant = cargar_modelo_uav(cfg)
%CARGAR_MODELO_UAV Carga los modelos lineales entregados para el taller.

% Paso 1: verificar que el archivo de datos exista.
if ~isfile(cfg.data.model_path)
    error('No se encontro modelo_lin.mat en: %s', cfg.data.model_path);
end

% Paso 2: cargar el .mat y confirmar que contiene los tres modelos usados.
data = load(cfg.data.model_path);
required = {'linmodel', 'latmod', 'longmod'};
for k = 1:numel(required)
    if ~isfield(data, required{k})
        error('modelo_lin.mat no contiene la variable requerida %s.', required{k});
    end
end

% Paso 3: convertir a modelos de espacio de estados para analisis/control.
plant.full = ss(data.linmodel);
plant.lat = ss(data.latmod);
plant.long = ss(data.longmod);

% Paso 4: normalizar nombres de entradas/salidas del modelo completo.
plant.full.InputName = cfg.signals.full_inputs;
plant.full.OutputName = cfg.signals.full_outputs;

% Paso 5: guardar metadatos utiles para imprimir y reportar.
plant.info.full_order = order(plant.full);
plant.info.lat_order = order(plant.lat);
plant.info.long_order = order(plant.long);
plant.info.full_size = size(plant.full);
plant.info.lat_size = size(plant.lat);
plant.info.long_size = size(plant.long);
end
