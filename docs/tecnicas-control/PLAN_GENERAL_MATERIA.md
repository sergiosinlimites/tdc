# Plan general de Tecnicas de Control

Esta guia organiza lo que toca hacer en la materia, los tres talleres y el
proyecto final. La idea es trabajar en capas: primero entender el enunciado,
luego construir una version reproducible en MATLAB, despues pasar a Simulink y
finalmente escribir el informe con resultados verificables.

## 1. Mapa de la materia

Fuente principal:

- `drive/TDC/Programa-2.pdf`

La asignatura se enfoca en tecnicas modernas de control:

- control multivariable;
- limitaciones de desempeno;
- estabilidad de Lyapunov;
- linealizacion de sistemas no lineales;
- control robusto `Hinf` / `H2`;
- planta generalizada de regulacion;
- control no lineal;
- control por modo deslizante;
- control adaptativo y `MRAC`;
- anti-windup y estabilidad absoluta.

## 2. Evaluacion

Segun el programa:

| Actividad | Peso |
| --- | ---: |
| Taller 1 | 20% |
| Taller 2 | 20% |
| Taller 3 | 20% |
| Proyecto final | 40% |

El proyecto final se reparte asi:

| Entrega | Peso |
| --- | ---: |
| Informe parcial 1 | 15% |
| Informe parcial 2 | 15% |
| Presentacion final | 10% |

El archivo `drive/TDC/Fechas Importantes.docx` registra estas fechas:

| Actividad | Fecha |
| --- | --- |
| Tarea 1 | mayo 15 |
| Tarea 2 | junio 5 |
| P1 | junio 12 |
| Tarea 3 | junio 26 |
| P2 | junio 29 - julio 1 |

## 3. Materiales encontrados en Drive

Talleres:

- `drive/TDC/02. TAREAS/T1/taller1_2022.pdf`
- `drive/TDC/02. TAREAS/T2/Taller2_2021_2.pdf`
- `drive/TDC/02. TAREAS/T3/Adpat_tarea-1.pdf`
- `drive/TDC/02. TAREAS/T3/main_V4.pdf`

Proyecto:

- `drive/TDC/PROYECTO/modelo_planta_aeropendulo.pdf`

Recursos utiles:

- `drive/TDC/01. NOTAS DE CLASE/H_inf/`
- `drive/TDC/01. NOTAS DE CLASE/SMC/`
- `drive/TDC/01. NOTAS DE CLASE/Adaptive/MRAC/`
- `drive/TDC/04. Otros Recursos/UAV_SIM_AEM/`
- `drive/TDC/04. Otros Recursos/H_inf/`
- `drive/TDC/04. Otros Recursos/SMC/`
- `drive/TDC/04. Otros Recursos/Adaptative Control/`
- `drive/TDC/03. BIBLIO/`

## 4. Documentos de trabajo creados

Usa estos documentos como tablero base:

- [Taller 1: Robustez e Hinf en UAV](./talleres/TALLER_1_ROBUSTO_HINF_UAV.md)
- [Taller 2: Modo deslizante en motor DC](./talleres/TALLER_2_SMC_MOTOR_DC.md)
- [Taller 3: MRAC en MuPAL longitudinal](./talleres/TALLER_3_MRAC_MUPAL.md)
- [Proyecto final: Aeropendulo](./proyecto-final/PROYECTO_FINAL_AEROPENDULO.md)

## 5. Estrategia comun para todos los talleres

Cada taller debe tener la misma estructura de trabajo:

1. Leer el enunciado y extraer variables, parametros y entregables.
2. Crear una carpeta limpia del taller.
3. Definir un archivo de parametros como centro de verdad.
4. Implementar el modelo nominal.
5. Verificar dimensiones, polos, estabilidad, controlabilidad u observabilidad segun aplique.
6. Disenar el controlador solicitado.
7. Simular casos nominales y casos con perturbacion/incertidumbre.
8. Exportar figuras desde codigo.
9. Escribir el informe desde resultados reproducibles.
10. Cerrar con una tabla de conclusiones tecnicas.

La regla importante: ningun resultado del informe debe depender de una grafica
generada manualmente. Todo debe salir de scripts o de simulaciones reproducibles.

## 6. Estructura recomendada del repositorio

```text
tdc/
|-- docs/
|   `-- tecnicas-control/
|       |-- PLAN_GENERAL_MATERIA.md
|       |-- talleres/
|       |   |-- TALLER_1_ROBUSTO_HINF_UAV.md
|       |   |-- TALLER_2_SMC_MOTOR_DC.md
|       |   `-- TALLER_3_MRAC_MUPAL.md
|       `-- proyecto-final/
|           `-- PROYECTO_FINAL_AEROPENDULO.md
|-- taller1/
|   |-- src/
|   |-- models/
|   |-- figures/
|   |-- data/
|   `-- report/
|-- taller2/
|-- taller3/
`-- proyecto-final-aeropendulo/
```

Cada carpeta de taller deberia seguir esta forma:

```text
tallerN/
|-- README.md
|-- src/
|   |-- main_tallerN.m
|   |-- params_tallerN.m
|   |-- build_model_tallerN.m
|   |-- design_controller_tallerN.m
|   |-- run_simulations_tallerN.m
|   `-- export_figures_tallerN.m
|-- models/
|   `-- tallerN_model.slx
|-- figures/
|-- data/
`-- report/
    `-- informe_tallerN.md
```

## 7. Guidelines de codigo MATLAB

Prioridades:

- codigo reproducible;
- nombres claros;
- funciones pequenas;
- parametros centralizados;
- graficas exportadas automaticamente;
- unidades explicitas;
- nada de numeros magicos regados en scripts.

Patron recomendado:

```matlab
function results = main_taller1()
    clc;
    close all;

    cfg = params_taller1();
    plant = build_uav_plant(cfg);
    controllers = design_controllers(plant, cfg);
    results = run_simulations(plant, controllers, cfg);
    export_figures(results, cfg);
end
```

Un archivo de parametros debe devolver un `struct`:

```matlab
function cfg = params_taller1()
    cfg.paths.root = fileparts(mfilename('fullpath'));
    cfg.paths.figures = fullfile(cfg.paths.root, '..', 'figures');

    cfg.sim.tFinal = 20;
    cfg.sim.dt = 0.01;

    cfg.units.deg = pi / 180;
    cfg.limits.uMaxDeg = 30;
end
```

Para pruebas rapidas, usar `assert`:

```matlab
assert(size(A, 1) == size(A, 2), 'A debe ser cuadrada.');
assert(size(A, 1) == size(B, 1), 'A y B no son compatibles.');
```

## 8. Guidelines de Simulink

Usar Simulink cuando aporte valor:

- interconexion de plantas generalizadas;
- ruido de medicion;
- saturaciones;
- comparacion de controladores;
- diagramas claros para informe;
- simulacion de software de control.

Para simulaciones desde MATLAB, preferir `Simulink.SimulationInput`:

```matlab
in = Simulink.SimulationInput('taller1_model');
in = in.setModelParameter('StopTime', '20');
in = in.setVariable('cfg', cfg);
out = sim(in);
```

Para barridos:

```matlab
in = repmat(Simulink.SimulationInput('taller1_model'), numel(cases), 1);

for k = 1:numel(cases)
    in(k) = Simulink.SimulationInput('taller1_model');
    in(k) = in(k).setVariable('caseCfg', cases(k));
end

out = sim(in, 'UseFastRestart', 'on');
```

## 9. Uso posible del MCP de Simulink y MATLAB

Este workspace ya tiene herramientas de Simulink/MATLAB disponibles desde MCP.
El flujo sano es:

1. Abrir MATLAB localmente.
2. Cargar o instalar el toolbox del MCP si hace falta.
3. Ejecutar en MATLAB:

```matlab
satk_initialize
```

4. Desde Codex, usar herramientas como:

- `detect_matlab_toolboxes` para confirmar version y toolboxes;
- `evaluate_matlab_code` para ejecutar expresiones pequenas;
- `run_matlab_file` para correr scripts `.m`;
- `check_matlab_code` para analizar calidad de codigo;
- `model_overview` para entender un modelo Simulink;
- `model_read` para inspeccionar bloques y conexiones;
- `model_edit` para construir o modificar modelos;
- `model_test` para pruebas Gherkin cuando haya Simulink Test.

No conviene usar MCP para todo. Conviene usarlo cuando:

- queremos inspeccionar un `.slx` sin abrirlo manualmente;
- necesitamos editar bloques de forma controlada;
- queremos ejecutar pruebas sobre modelos;
- hay que validar rapidamente que MATLAB tiene los toolboxes necesarios.

## 10. Ruta por semanas

### Semana 1: Taller 1

- entender la planta UAV y los ejes;
- revisar recursos de UAV y Hinf;
- montar modelo nominal;
- disenar PID/SAS/CAS inicial;
- empezar planta generalizada para Hinf.

### Semana 2: Taller 1

- cerrar controlador Hinf;
- comparar `S`, `T` y `KS`;
- simular ruido e incertidumbre;
- escribir informe.

### Semana 3: Taller 2

- derivar motor DC con carga puntual;
- implementar control SM puro;
- definir cotas de incertidumbre;
- simular chattering.

### Semana 4: Taller 2

- implementar control equivalente;
- comparar SM puro vs SM con equivalente;
- exportar graficas y conclusiones.

### Semana 5: Proyecto P1

- definir planta aeropendulo;
- propuesta economica y tecnologica;
- seleccion preliminar de sensores, actuador y arquitectura;
- primer modelo fisico.

### Semana 6: Taller 3

- implementar MuPAL longitudinal;
- definir modelo de referencia;
- analizar condiciones de acoplamiento;
- simular MRAC con paso y tren de pulsos.

### Semana 7: Proyecto P2

- cerrar modelo;
- disenar controlador;
- simular no linealidad, saturacion y ruido;
- preparar informe parcial 2.

### Semana 8: Presentacion final

- limpiar resultados;
- seleccionar figuras principales;
- preparar defensa: problema, modelo, controlador, resultados, limites.

## 11. Criterio de calidad para informes

Cada informe debe responder:

- cual es el modelo;
- que supuestos se hicieron;
- que controlador se diseno;
- por que el diseno cumple o no cumple;
- que casos se simularon;
- que limitaciones se encontraron;
- que se haria distinto si hubiera hardware real.

Estructura minima:

1. Resumen.
2. Enunciado y objetivos.
3. Modelo matematico.
4. Diseno del controlador.
5. Implementacion MATLAB/Simulink.
6. Resultados.
7. Discusion.
8. Conclusiones.
9. Anexos de codigo.

## 12. Proxima accion recomendada

Como vamos empezando el Taller 1, el siguiente paso es crear la carpeta
`taller1/` con `src/`, `models/`, `figures/` y `report/`, y construir primero
un script que cargue o reconstruya el modelo UAV nominal. El Hinf viene despues;
primero necesitamos una planta confiable.
