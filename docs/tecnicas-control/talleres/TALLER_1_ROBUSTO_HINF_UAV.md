# Taller 1: Control robusto y analisis de desempeno en UAV

Fuente:

- `drive/TDC/02. TAREAS/T1/taller1_2022.pdf`

## 1. Objetivo

Disenar dos controladores para un UAV:

1. controlador clasico tipo `PID`, organizado como `SAS/CAS`;
2. controlador robusto `Hinf`.

Luego comparar su desempeno nominal, su estabilidad robusta y su desempeno
robusto mediante simulacion y analisis frecuencial.

## 2. Lo que pide el enunciado

El problema se enfoca en seguimiento de referencias tipo set-point:

- `theta` en el eje longitudinal;
- `phi` en el eje lateral/direccional;
- rango de referencia: `[-40, 40] deg`.

Tambien exige:

- rechazo de ruido de medicion;
- robustez ante incertidumbre aditiva inversa;
- simulacion en MATLAB/Simulink;
- comparacion entre controlador clasico y controlador `Hinf`.

## 3. Especificaciones tecnicas

| Requisito | Valor |
| --- | --- |
| Ancho de banda minimo/maximo indicado | `8 Hz` |
| Perturbaciones de entrada hasta | `6 Hz` |
| Ruido lateral/direccional | potencia promedio `0.001` |
| Ruido longitudinal | potencia promedio `0.0001` |
| Saturacion de control | `+-30 deg` |
| Referencias de posicion | `theta`, `phi` |
| Referencias de velocidad angular | se asumen nulas |

El sistema debe ser:

- criticamente amortiguado en sus ejes;
- lo mas rapido posible;
- con esfuerzo de control pequeno;
- con ejes longitudinal y lateral/direccional lo mas desacoplados posible.

## 4. Interpretacion de la planta

Eje lateral/direccional:

- entradas: rudder y ailerons;
- salidas: yaw `psi`, roll `phi`, velocidades `r` y `p`.

Eje longitudinal:

- entrada: elevator;
- salida: pitch `theta` y velocidad de pitch `q`.

Para el diseno clasico:

- `SAS`: realimentacion de velocidades para agregar amortiguamiento;
- `CAS`: seguimiento de referencia mediante PI.

Para `Hinf`:

- construir planta generalizada;
- escoger filtros de desempeno;
- tener en cuenta ceros y restricciones que imponen.

## 5. Recursos del Drive

Prioridad alta:

- `drive/TDC/04. Otros Recursos/UAV_SIM_AEM/Simulation_Lin/`
- `drive/TDC/04. Otros Recursos/UAV_SIM_AEM/Simulation_Lin/Lin_Sim/get_UAV_matrices.m`
- `drive/TDC/04. Otros Recursos/UAV_SIM_AEM/Simulation_Lin/Controllers/`
- `drive/TDC/04. Otros Recursos/H_inf/`
- `drive/TDC/01. NOTAS DE CLASE/H_inf/`

Bibliografia util:

- `drive/TDC/03. BIBLIO/Multivariable Feedback Control Analysis and Design - Skogestad.pdf`
- `drive/TDC/03. BIBLIO/Linear_Robust_control_Limebeer.pdf`

## 6. Estructura recomendada

```text
taller1/
|-- README.md
|-- src/
|   |-- main_taller1.m
|   |-- params_taller1.m
|   |-- load_uav_model.m
|   |-- split_axes.m
|   |-- design_pid_sas_cas.m
|   |-- design_hinf_controller.m
|   |-- analyze_frequency_response.m
|   |-- run_simulations_taller1.m
|   `-- export_figures_taller1.m
|-- models/
|   `-- taller1_uav.slx
|-- figures/
|-- data/
`-- report/
    `-- informe_taller1.md
```

## 7. Paso a paso

### Paso 0. Levantar el entorno

Verificar toolboxes:

```matlab
ver
```

Necesarios o muy utiles:

- Control System Toolbox;
- Robust Control Toolbox;
- Simulink;
- Simulink Control Design, si esta disponible.

Con MCP, se puede pedir:

- `detect_matlab_toolboxes`;
- `evaluate_matlab_code` con `ver`;
- `model_overview` si ya existe un `.slx`;
- `model_read` para inspeccionar bloques.

### Paso 1. Cargar la planta nominal

Primero buscar si los recursos UAV ya entregan matrices:

```matlab
cfg = params_taller1();
plant = load_uav_model(cfg);
```

La funcion debe devolver:

```matlab
plant.A
plant.B
plant.C
plant.D
plant.sys
plant.states
plant.inputs
plant.outputs
```

Validaciones minimas:

```matlab
assert(size(plant.A,1) == size(plant.A,2), 'A no es cuadrada.');
assert(size(plant.A,1) == size(plant.B,1), 'A y B no son compatibles.');
assert(size(plant.C,2) == size(plant.A,2), 'C y A no son compatibles.');
```

### Paso 2. Separar ejes

Crear modelos desacoplados para diseno:

```matlab
axes = split_axes(plant, cfg);

sysLong = axes.longitudinal.sys;
sysLat = axes.lateral.sys;
```

Pero simular tambien con modelo acoplado:

```matlab
sysFull = plant.sys;
```

### Paso 3. Analisis inicial

Calcular:

```matlab
polesOpen = eig(plant.A);
dcGain = dcgain(plant.sys);
```

Generar:

- polos de lazo abierto;
- respuesta escalon de la planta nominal;
- `sigma(plant.sys)`;
- posibles ceros con `tzero`.

Preguntas que deben quedar respondidas:

- hay modos inestables?
- hay ceros no minimos?
- que tan acoplados estan los ejes?
- que canales son mas faciles de controlar?

### Paso 4. Disenar SAS/CAS

Para SAS:

- realimentar velocidades angulares;
- buscar amortiguamiento;
- empezar con ganancias proporcionales.

Para CAS:

- cerrar PI sobre referencia de angulo;
- limitar esfuerzo a `+-30 deg`.

Patron de codigo:

```matlab
controllers.pid = design_pid_sas_cas(axes, cfg);
```

La salida debe incluir:

```matlab
controllers.pid.longitudinal
controllers.pid.lateral
controllers.pid.notes
```

### Paso 5. Analizar `S`, `T` y `KS`

Para un lazo `L = G*K`:

```matlab
S = feedback(eye(size(L)), L);
T = feedback(L, eye(size(L)));
KS = K * S;

sigma(S, T, KS);
```

Para MIMO, usar valores singulares:

```matlab
sigma(S);
sigma(T);
sigma(KS);
```

El informe debe comentar:

- desempeno nominal;
- rechazo a perturbaciones;
- sensibilidad a ruido;
- esfuerzo de control;
- estabilidad robusta;
- desempeno robusto.

### Paso 6. Planta generalizada para `Hinf`

Construir una interconexion que incluya:

- referencias;
- error de seguimiento;
- ruido de medicion;
- incertidumbre o perturbacion;
- ponderaciones de sensibilidad;
- ponderaciones de accion de control;
- salidas medidas hacia el controlador.

Funciones posibles:

```matlab
sumblk
connect
augw
mixsyn
hinfsyn
```

Ruta simple:

```matlab
P = build_generalized_plant(plant, weights, cfg);
[Khinf, CL, gamma] = hinfsyn(P, nmeas, ncon);
```

Ruta mas directa si aplica:

```matlab
[Khinf, CL, gamma] = mixsyn(G, W1, W2, W3);
```

### Paso 7. Escoger filtros

Filtro de sensibilidad `W1`:

- alto a baja frecuencia para exigir seguimiento/rechazo;
- bajo a alta frecuencia para permitir roll-off.

Filtro de control `W2`:

- penaliza esfuerzo de control;
- ayuda a respetar `+-30 deg`.

Filtro de sensibilidad complementaria `W3`:

- limita ruido y alta frecuencia;
- ayuda a robustez.

Ejemplo inicial, no definitivo:

```matlab
s = tf('s');

wb = 2*pi*8;
wp = 2*pi*6;

W1 = makeweight(10, wb, 0.01);
W2 = makeweight(0.01, wb, 10);
W3 = makeweight(0.01, wp, 10);
```

Despues se ajusta segun las graficas de `sigma`.

### Paso 8. Simulaciones

Casos minimos:

1. referencia `theta = 10 deg`;
2. referencia `theta = -10 deg`;
3. referencia `phi = 10 deg`;
4. referencia `phi = -10 deg`;
5. referencia grande cerca de `40 deg`;
6. ruido de medicion;
7. perturbacion de entrada;
8. comparacion PID vs `Hinf`.

Cada caso debe guardar:

- referencia;
- salida;
- error;
- accion de control;
- saturacion;
- senales con ruido.

### Paso 9. Modelo Simulink

El modelo debe tener:

- bloque de referencia;
- controlador PID/SAS/CAS;
- controlador `Hinf`;
- selector de controlador;
- planta UAV acoplada;
- ruido de medicion;
- perturbacion de entrada;
- saturacion de actuador;
- scopes o logging.

Con MCP se puede:

- inspeccionar el modelo con `model_overview`;
- leer bloques con `model_read`;
- editar conexiones con `model_edit`;
- correr pruebas con `model_test` si hay Simulink Test.

Para simular desde MATLAB:

```matlab
in = Simulink.SimulationInput('taller1_uav');
in = in.setModelParameter('StopTime', '20');
in = in.setVariable('cfg', cfg);
in = in.setVariable('controllers', controllers);
out = sim(in);
```

## 8. Figuras obligatorias

Minimo:

- valores singulares de la planta;
- `S`, `T`, `KS` para PID;
- `S`, `T`, `KS` para `Hinf`;
- seguimiento de referencia PID vs `Hinf`;
- accion de control PID vs `Hinf`;
- efecto del ruido de medicion;
- efecto de perturbacion/incertidumbre;
- respuesta del modelo acoplado.

## 9. Informe

Estructura:

1. Objetivo.
2. Planta UAV y supuestos.
3. Diseno PID/SAS/CAS.
4. Diseno `Hinf`.
5. Planta generalizada y filtros.
6. Analisis frecuencial.
7. Simulacion MATLAB/Simulink.
8. Comparacion de resultados.
9. Conclusiones.

## 10. Checklist de cierre

- [ ] Modelo nominal cargado.
- [ ] Ejes separados para diseno.
- [ ] Modelo acoplado usado para simulacion.
- [ ] PID/SAS/CAS funcionando.
- [ ] Planta generalizada documentada.
- [ ] `Hinf` sintetizado.
- [ ] `S`, `T`, `KS` graficados.
- [ ] Ruido de medicion incluido.
- [ ] Perturbacion/incertidumbre incluida.
- [ ] Saturacion `+-30 deg` incluida.
- [ ] Informe escrito con conclusiones comparativas.

## 11. Primer siguiente paso

Crear `taller1/src/params_taller1.m` y `taller1/src/load_uav_model.m`. Antes de
disenar controladores, necesitamos confirmar exactamente que matrices del UAV
estan disponibles en los recursos del Drive.
