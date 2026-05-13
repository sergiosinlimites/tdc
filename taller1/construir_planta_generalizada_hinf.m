function P = construir_planta_generalizada_hinf(G, W1, W2, W3)
%CONSTRUIR_PLANTA_GENERALIZADA_HINF Construye P para documentar H_inf.
%
% Para la implementacion se usa mixsyn, pero este archivo deja explicita la
% planta aumentada equivalente usada en sensibilidad mixta.

% Paso unico: augw construye la planta generalizada que penaliza error,
% control y salida ponderada con W1, W2 y W3.
P = augw(G, W1, W2, W3);
end
