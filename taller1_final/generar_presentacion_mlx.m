%% Genera taller1_presentacion.mlx a partir de taller1_presentacion_src.m
% Ejecutar este script en MATLAB para convertir el archivo fuente
% .m en un Live Script .mlx.

project_dir = fileparts(mfilename('fullpath'));
src = fullfile(project_dir, 'taller1_presentacion_src.m');
dst = fullfile(project_dir, 'taller1_presentacion.mlx');

if ~isfile(src)
    error('No se encontro el archivo fuente: %s', src);
end

matlab.internal.liveeditor.openAndSave(src, dst);
matlab.internal.liveeditor.executeAndSave(dst);
fprintf('Live Script generado y ejecutado: %s\n', dst);
