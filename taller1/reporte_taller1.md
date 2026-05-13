# Taller 1: Control robusto H_inf y analisis de desempeno en UAV

## 1. Objetivo

El objetivo del taller es disenar y comparar dos estrategias de control para un UAV linealizado:

1. Control clasico tipo `PID`, organizado como `SAS/CAS`.
2. Control robusto `H_inf` por sensibilidad mixta.

El desempeno se analiza en seguimiento de referencias tipo set-point para los angulos:

```math
theta,\ phi \in [-40, 40]\ deg
```

Tambien se verifica rechazo a ruido de medicion, perturbaciones de entrada, esfuerzo de control y desempeno sobre la planta acoplada.

## 2. Archivos implementados

El taller quedo organizado con la misma metodologia modular del proyecto 1:

```text
taller1/
|-- main_taller1.m
|-- parametros_taller1.m
|-- cargar_modelo_uav.m
|-- seleccionar_canales_uav.m
|-- analisis_planta_uav.m
|-- diseno_pid_sas_cas.m
|-- diseno_hinf_taller1.m
|-- construir_planta_generalizada_hinf.m
|-- analisis_sensibilidades.m
|-- simulacion_taller1.m
|-- crear_graficas_taller1.m
|-- init_taller1_simulink.m
|-- build_taller1_simulink.m
|-- taller1_uav.slx
|-- figures/
|-- results/
`-- reporte_taller1.md
```

El flujo principal es:

```matlab
main_taller1
```

El modelo Simulink se regenera con:

```matlab
build_taller1_simulink
```

Para una explicacion conceptual mas pausada de incertidumbre multiplicativa, incertidumbre inversa, `S`, `T`, `KS`, pesos `W1`, `W2`, `W3` y la matriz de planta generalizada de las diapositivas, revisar:

```text
README.md
```

## 3. Planta UAV

El archivo fuente del taller es:

```text
drive/TDC/02. TAREAS/T1/modelo_lin.mat
```

Este archivo contiene:

```text
linmodel : modelo completo acoplado
latmod   : modelo lateral/direccional
longmod  : modelo longitudinal
```

Datos confirmados:

```text
linmodel:
  13 estados
  8 entradas
  14 salidas

latmod:
  entradas: aileron, rudder
  salidas: beta, p, r, phi, psi

longmod:
  entradas: elevator, throttle
  salidas: V, alpha, q, theta, h, ax, az
```

Para diseno se usan canales desacoplados:

```matlab
G_theta = longmod('theta','elevator');
G_phi   = latmod('phi','aileron');
```

Para validacion temporal se usa:

```matlab
linmodel
```

Esto sigue el enunciado: disenar con modelos desacoplados y simular en el modelo acoplado.

## 4. Especificaciones del taller

Las especificaciones se codificaron en `parametros_taller1.m`:

```matlab
cfg.spec.ref_max_deg = 40;
cfg.spec.control_limit_deg = 30;
cfg.spec.bandwidth_hz = 8;
cfg.spec.perturbation_hz = 6;
cfg.spec.noise_power_long = 1e-4;
cfg.spec.noise_power_lat = 1e-3;
```

En radianes:

```math
u_{max} = 30 deg = pi/6 rad
```

```math
omega_b = 2 pi 8
```

```math
omega_p = 2 pi 6
```

## 5. Teoria base de H_inf

### 5.1 Norma H_inf

Para un sistema estable `G(s)`, la norma H_inf se define como:

```math
||G||_\infty = sup_\omega \bar{\sigma}(G(j\omega))
```

Donde `sigma_bar` es el maximo valor singular. En sistemas SISO coincide con el pico de magnitud de Bode; en sistemas MIMO mide la maxima amplificacion posible para cualquier direccion de entrada.

Por eso el taller pide usar:

```matlab
sigma(sys)
```

### 5.2 Sensibilidad

Para una planta `G` y un controlador `K`:

```math
L = GK
```

La sensibilidad es:

```math
S = (I + GK)^{-1}
```

La sensibilidad complementaria es:

```math
T = GK(I + GK)^{-1}
```

Y se cumple:

```math
S + T = I
```

Interpretacion:

- `S` pequeno a baja frecuencia: buen seguimiento y rechazo de perturbaciones lentas.
- `T` pequeno a alta frecuencia: menor amplificacion de ruido de medicion.
- `K*S` pequeno: menor esfuerzo de control y mejor margen ante incertidumbre aditiva.

El compromiso central es que `S` y `T` no pueden ser pequenos en todas las frecuencias al mismo tiempo.

### 5.3 Sensibilidad mixta

El diseno H_inf usado en el taller busca minimizar:

```math
||
[ W_1 S
  W_2 K S
  W_3 T ]
||_\infty
= gamma
```

Donde:

- `W1` penaliza error de seguimiento.
- `W2` penaliza accion de control.
- `W3` penaliza sensibilidad complementaria y ruido.

La idea ideal seria lograr:

```math
gamma < 1
```

En esta primera sintesis reproducible se obtuvo `gamma > 1`, lo que significa que los pesos son factibles para diseno estable, pero todavia no cumplen estrictamente todas las cotas impuestas. Aun asi, el controlador es estable y sirve para la comparacion pedida por el taller. Los pesos quedan parametrizados para iteracion posterior.

## 6. Planta generalizada

La planta aumentada se construye conceptualmente como:

```math
P = augw(G, W_1, W_2, W_3)
```

Y el controlador se sintetiza con:

```matlab
[K, CL, gamma] = mixsyn(G, W1, W2, W3);
```

El archivo:

```text
construir_planta_generalizada_hinf.m
```

deja explicita la llamada equivalente:

```matlab
P = augw(G, W1, W2, W3);
```

## 7. Pesos H_inf usados

Los pesos se definen en `diseno_hinf_taller1.m` a partir de parametros:

```matlab
W1 = makeweight(5, wb, 0.05);
W2 = 0.15;
W3 = makeweight(0.02, wp, 5);
```

Interpretacion:

- `W1`: exige que `S` sea pequeno a baja frecuencia.
- `W2`: limita el tamano de `K*S`.
- `W3`: exige que `T` caiga en alta frecuencia.

Se probaron valores de `W2 = 0.08`, `0.15` y `0.30`. El valor `0.15` se dejo como compromiso inicial:

- `0.08` hacia el controlador mas agresivo y aumenta `K*S`.
- `0.30` reduce control, pero produjo un pico grande en `S/T` para `theta`.
- `0.15` mantiene estabilidad y reduce parcialmente el esfuerzo sin degradar tanto la sensibilidad.

## 8. Control clasico PID/SAS-CAS

El controlador clasico se implementa en:

```text
diseno_pid_sas_cas.m
```

La arquitectura usada es:

```text
CAS:
  PI sobre theta_ref - theta
  PI sobre phi_ref - phi

SAS:
  accion derivativa filtrada asociada a q y p
  yaw damper sobre r
```

Control longitudinal:

```math
u_e = K_{p,theta} e_theta + K_{i,theta} int e_theta dt - K_{d,theta} q
```

Control lateral:

```math
u_a = K_{p,phi} e_phi + K_{i,phi} int e_phi dt - K_{d,phi} p
```

Yaw damper:

```math
K_{YD}(s) = 0.065 s/(s + 2)
```

Ganancias base:

```matlab
kp_theta = -0.84;
ki_theta = -0.23;
kd_theta = -0.08;

kp_phi = -0.52;
ki_phi = -0.20;
kd_phi = -0.07;
```

Estas ganancias no fueron inventadas ni salen de la sintesis H_inf. Vienen del controlador baseline del paquete UAV_SIM_AEM:

```text
drive/TDC/04. Otros Recursos/UAV_SIM_AEM/Simulation_Lin/Controllers/baseline_gains.m
```

La correspondencia usada es:

| En `baseline_gains.m` | En este taller | Valor |
|---|---:|---:|
| `kp_PT` | `kp_theta` | `-0.84` |
| `ki_PT` | `ki_theta` | `-0.23` |
| `kp_PD` | `kd_theta` | `-0.08` |
| `kp_RT` | `kp_phi` | `-0.52` |
| `ki_RT` | `ki_phi` | `-0.20` |
| `kp_RD` | `kd_phi` | `-0.07` |

En el archivo original, `PT` significa `Pitch Tracker`, `PD` significa `Pitch Damper`, `RT` significa `Roll Tracker` y `RD` significa `Roll Damper`. Por eso aqui se interpretan como CAS de angulo mas SAS de velocidad angular.

## 9. Resultados frecuenciales

La corrida final imprimio:

```text
Hinf theta gamma: 1.6202 | orden K: 7
Hinf phi gamma:   1.5743 | orden K: 6
```

Normas aproximadas:

```text
theta PID : ||S||=1.136 ||T||=1.000 ||KS||=3.507
theta Hinf: ||S||=1.141 ||T||=0.762 ||KS||=10.527

phi PID   : ||S||=1.117 ||T||=1.056 ||KS||=2.853
phi Hinf  : ||S||=1.137 ||T||=0.685 ||KS||=10.257
```

Lectura:

- El controlador H_inf reduce `T` frente al PID en ambos ejes, lo cual es bueno para ruido de medicion.
- El controlador H_inf actual tiene mayor `K*S`, por lo que pide mas accion de control.
- El PID base resulta menos agresivo y en la simulacion temporal conserva errores RMS menores para los casos probados.
- Esta comparacion muestra el compromiso real del diseno: no basta sintetizar H_inf; hay que iterar pesos para balancear seguimiento, ruido y control.

Figuras generadas:

```text
figures/planta_sigma.png
figures/sensibilidades_theta.png
figures/sensibilidades_phi.png
figures/comparacion_sensibilidades.png
```

## 10. Simulacion final acoplada

La simulacion temporal se implementa en:

```text
simulacion_taller1.m
```

Incluye:

- planta acoplada `linmodel`;
- saturacion `+-30 deg`;
- ruido de medicion;
- perturbacion sinusoidal de entrada hasta `6 Hz`;
- comparacion PID vs H_inf;
- referencias de `theta` y `phi`;
- casos grandes de `40 deg`.

Casos ejecutados:

```text
theta_10
theta_minus_10
phi_10
phi_minus_10
theta_phi_10
theta_40
phi_40
noise_disturbance
```

Resumen final:

```text
pid  theta_10           RMS theta=  1.425 deg RMS phi=  0.238 deg sat=  0.0%
pid  theta_minus_10     RMS theta=  1.425 deg RMS phi=  0.238 deg sat=  0.0%
pid  phi_10             RMS theta=  0.043 deg RMS phi=  1.346 deg sat=  0.0%
pid  phi_minus_10       RMS theta=  0.043 deg RMS phi=  1.346 deg sat=  0.0%
pid  theta_phi_10       RMS theta=  1.427 deg RMS phi=  1.435 deg sat=  0.0%
pid  theta_40           RMS theta=  5.698 deg RMS phi=  0.952 deg sat=  0.1%
pid  phi_40             RMS theta=  0.172 deg RMS phi=  5.383 deg sat=  0.0%
pid  noise_disturbance  RMS theta=  1.418 deg RMS phi=  1.461 deg sat=  0.0%

hinf theta_10           RMS theta=  3.381 deg RMS phi=  3.158 deg sat=  0.2%
hinf theta_minus_10     RMS theta=  3.381 deg RMS phi=  3.158 deg sat=  0.2%
hinf phi_10             RMS theta=  0.054 deg RMS phi=  4.724 deg sat=  0.1%
hinf phi_minus_10       RMS theta=  0.054 deg RMS phi=  4.724 deg sat=  0.1%
hinf theta_phi_10       RMS theta=  3.355 deg RMS phi=  3.140 deg sat=  0.2%
hinf theta_40           RMS theta= 14.279 deg RMS phi= 12.287 deg sat=  0.5%
hinf phi_40             RMS theta=  0.146 deg RMS phi= 25.261 deg sat=  0.3%
hinf noise_disturbance  RMS theta=  3.354 deg RMS phi=  4.190 deg sat=  6.0%
```

Figuras temporales:

```text
figures/sim_theta_10.png
figures/sim_phi_10.png
figures/sim_theta_phi_10.png
figures/sim_theta_40.png
figures/sim_phi_40.png
figures/sim_noise_disturbance.png
```

## 11. Modelo Simulink

El modelo:

```text
taller1_uav.slx
```

se genera automaticamente con:

```matlab
build_taller1_simulink
```

Incluye:

- planta lineal acoplada;
- selector `PID/H_inf` mediante `control_mode`;
- referencias `theta_ref` y `phi_ref`;
- ruido de medicion en `theta` y `phi`;
- perturbaciones sinusoidales de entrada en `elevator`, `aileron` y `rudder`;
- saturacion de actuadores;
- logging a `simout_y` y `simout_u`.

Para evitar un lazo algebraico artificial, el bloque de planta en Simulink expone solo:

```text
theta, phi, p, q, r
```

con matriz `D_sim = 0`, ya que esas son las mediciones usadas por el controlador.

Validacion realizada:

```matlab
run('init_taller1_simulink.m')
in = Simulink.SimulationInput('taller1_uav');
in = in.setModelParameter('StopTime','3');
out = sim(in);
disp(out.who)
```

Salida confirmada:

```text
simout_u
simout_y
tout
```

## 12. Como reproducir todo

Desde MATLAB:

```matlab
cd('/home/sergio/Escritorio/tdc/taller1')
main_taller1
```

Esto debe:

1. cargar `modelo_lin.mat`;
2. analizar planta y canales;
3. disenar PID/SAS-CAS;
4. sintetizar H_inf;
5. calcular `S`, `T`, `K*S`;
6. simular casos finales;
7. exportar figuras;
8. guardar `results/taller1_results.mat`.

Para regenerar Simulink:

```matlab
cd('/home/sergio/Escritorio/tdc/taller1')
build_taller1_simulink
```

Para simular el modelo:

```matlab
run('init_taller1_simulink.m')
in = Simulink.SimulationInput('taller1_uav');
in = in.setModelParameter('StopTime','12');
out = sim(in);
```

## 13. Como verificar

### 13.1 Toolboxes

```matlab
which hinfsyn
which mixsyn
which augw
which makeweight
```

Todas deben apuntar a Robust Control Toolbox.

### 13.2 Estabilidad

Despues de `main_taller1`:

```matlab
isstable(pid_data.theta_loop)
isstable(pid_data.phi_loop)
isstable(hinf_data.theta.CL)
isstable(hinf_data.phi.CL)
```

### 13.3 Sensibilidades

```matlab
sens.theta.pid.norm_S
sens.theta.pid.norm_T
sens.theta.pid.norm_KS

sens.theta.hinf.norm_S
sens.theta.hinf.norm_T
sens.theta.hinf.norm_KS
```

Repetir para `phi`.

### 13.4 Figuras

Revisar:

```text
taller1/figures/
```

Debe contener las figuras de planta, sensibilidades y simulaciones.

### 13.5 Simulink

```matlab
build_taller1_simulink
run('init_taller1_simulink.m')
in = Simulink.SimulationInput('taller1_uav');
in = in.setModelParameter('StopTime','3');
out = sim(in);
disp(out.who)
```

Debe mostrar `simout_u`, `simout_y` y `tout`.

## 14. Conclusiones tecnicas

1. El PID/SAS-CAS base es estable, simple y tiene buen desempeno temporal para las referencias probadas.
2. El H_inf sintetizado es estable y reduce la sensibilidad complementaria `T`, lo cual va en la direccion correcta para ruido de medicion.
3. El H_inf actual aumenta `K*S`, por lo que amplifica mas la accion de control que el PID.
4. En simulacion acoplada, el PID tiene menor error RMS que el H_inf con los pesos actuales.
5. El resultado no invalida H_inf: muestra que el exito del metodo depende fuertemente de la seleccion de pesos y de la compatibilidad entre el modelo usado para diseno y la planta acoplada usada para simulacion.
6. Para una segunda iteracion, conviene probar pesos MIMO o una planta generalizada que incluya explicitamente los canales `theta`, `phi`, `q`, `p`, `r` y la penalizacion de actuadores de forma conjunta.

## 15. Siguiente iteracion recomendada

Si se quiere mejorar el controlador H_inf, el siguiente paso tecnico no es cambiar de lenguaje, sino ajustar el planteamiento en MATLAB:

1. Probar un diseno MIMO para:

   ```matlab
   G_mimo = linmodel({'theta','phi'}, {'elevator','aileron'});
   ```

2. Incluir penalizacion separada para `elevator` y `aileron`.
3. Agregar un peso de ruido mas fuerte en `W3`.
4. Comparar `gamma`, `S`, `T`, `K*S` y simulacion acoplada.
5. Solo despues de aprobar esta version, portar el flujo a Python.
