# README de `simulacion_taller1.m`

Este documento explica solamente el archivo `simulacion_taller1.m`. La idea es entender como se simulan el controlador PID/SAS-CAS y el controlador H_inf sobre la planta acoplada `linmodel`.

## 1. Objetivo del archivo

`simulacion_taller1.m` no disena controladores. Su trabajo es tomar controladores ya calculados y probarlos en tiempo continuo.

Entrada principal:

```matlab
sim_results = simulacion_taller1(plant, pid_data, hinf_data, cfg)
```

Entradas:

- `plant`: estructura con modelos lineales. Aqui se usa `plant.full`, que corresponde al modelo acoplado completo.
- `pid_data`: controlador clasico PID/SAS-CAS calculado en `diseno_pid_sas_cas.m`.
- `hinf_data`: controladores H_inf calculados en `diseno_hinf_taller1.m`.
- `cfg`: configuracion general, escenarios, limites, ruido y tiempos.

Salida:

- `sim_results.pid`: simulaciones con PID/SAS-CAS.
- `sim_results.hinf`: simulaciones con H_inf.
- `sim_results.summary`: resumen compacto de errores RMS, maximos y saturacion.

## 2. Flujo general

El archivo sigue esta secuencia:

```text
simulacion_taller1
  -> run_family('pid')
       -> simulate_case(...) para cada escenario
  -> run_family('hinf')
       -> simulate_case(...) para cada escenario
  -> summarize_results(...)
```

Primero se simulan todos los escenarios con PID. Luego se reinicia la semilla aleatoria y se simulan los mismos escenarios con H_inf. Eso hace que ambos controladores vean el mismo ruido y la comparacion sea justa.

## 3. Que se simula exactamente

La planta completa usa:

```math
\dot{x} = A x + B u
```

```math
y = C x
```

Donde:

- `x`: estados de la planta acoplada.
- `u`: vector de 8 entradas del modelo UAV.
- `y`: vector de salidas del modelo UAV.

El estado total que integra `ode45` es:

```math
z =
\begin{bmatrix}
x_{planta} \\
x_{controlador}
\end{bmatrix}
```

Para PID, `x_controlador` tiene 3 estados:

```text
xi_theta  -> integrador del error de theta
xi_phi    -> integrador del error de phi
x_yaw     -> estado del yaw damper
```

Para H_inf, `x_controlador` contiene:

```text
estados de K_theta
estados de K_phi
estado del yaw damper
```

Por eso el controlador H_inf tiene mas estados: sus controladores son dinamicos, no solo ganancias PI+D.

## 4. Escenarios

Los escenarios vienen de `cfg.scenarios`.

Cada escenario define:

- nombre;
- referencia de `theta`;
- referencia de `phi`;
- si hay ruido;
- si hay perturbacion de entrada.

Ejemplos:

```matlab
theta_10
phi_10
theta_phi_10
theta_40
noise_disturbance
```

La referencia no aparece desde `t = 0`. Se activa en:

```matlab
cfg.sim.t_step
```

Esto permite ver una respuesta al escalon.

## 5. Ruido de medicion

El ruido se genera en `make_noise`.

Si el escenario no tiene ruido:

```matlab
noise.theta = 0
noise.phi = 0
noise.q = 0
noise.p = 0
noise.r = 0
```

Si el escenario tiene ruido, se usa ruido blanco discreto:

```matlab
sqrt(potencia)*randn(...)
```

Potencias usadas:

- longitudinal: `cfg.spec.noise_power_long = 1e-4`;
- lateral: `cfg.spec.noise_power_lat = 1e-3`.

El ruido se suma a mediciones, no a la salida real de la planta:

```text
theta_meas = theta + ruido_theta
phi_meas   = phi   + ruido_phi
q_meas     = q     + ruido_q
p_meas     = p     + ruido_p
r_meas     = r     + ruido_r
```

Eso representa sensores imperfectos.

## 6. Perturbacion de entrada

La perturbacion se calcula en `input_disturbance`.

Si el escenario la activa y el tiempo ya paso de `cfg.sim.disturbance_start`, se agrega una senal sinusoidal:

```math
d(t) = a
\begin{bmatrix}
\sin(\omega t) \\
\sin(\omega t + \pi/4) \\
\sin(\omega t + \pi/2)
\end{bmatrix}
```

Con:

```matlab
w = 2*pi*cfg.spec.perturbation_hz
a = cfg.sim.disturbance_amp_rad
```

En el taller:

```matlab
cfg.spec.perturbation_hz = 6
```

Por eso la perturbacion vive a `6 Hz`, escrita en MATLAB como frecuencia angular `2*pi*6 rad/s`.

## 7. Vector completo de entrada

La planta `linmodel` espera 8 entradas:

```text
throttle
elevator
rudder
l_aileron
r_aileron
l_flap
r_flap
aileron
```

Pero en la simulacion se controlan principalmente:

```text
elevator
rudder
aileron
```

La funcion `full_input_vector` arma el vector completo:

```matlab
u = zeros(8,1)
u(2) = elevator + perturbacion_elevator
u(3) = rudder   + perturbacion_rudder
u(8) = aileron  + perturbacion_aileron
```

Las demas entradas quedan en cero.

## 8. Controlador PID/SAS-CAS

En `make_controller('pid',...)`, el PID se prepara con:

- dos integradores para `theta` y `phi`;
- un yaw damper dinamico para `r`.

La ley usada en `controller_output` es:

```math
u_e =
K_{p,\theta} e_\theta
+ K_{i,\theta} \xi_\theta
- K_{d,\theta} q_{meas}
```

```math
u_a =
K_{p,\phi} e_\phi
+ K_{i,\phi} \xi_\phi
- K_{d,\phi} p_{meas}
```

Esto se interpreta asi:

- `PI` sobre error angular: parte CAS, seguimiento.
- realimentacion de `q` o `p`: parte SAS, amortiguamiento.

No se deriva directamente el error para evitar ruido y golpe derivativo. Se usa la velocidad angular medida, que para referencia constante cumple aproximadamente:

```math
\dot{e}_\theta = -\dot{\theta} = -q
```

## 9. Anti-windup

La simulacion incluye saturacion de actuador:

```matlab
u_sat = saturate(u_raw, limit)
```

El limite viene de:

```matlab
cfg.spec.control_limit_rad
```

El anti-windup corrige el integrador cuando el comando calculado supera el limite:

```matlab
xi_dot = error + antiwindup*(u_sat - u_raw)
```

Si no hay saturacion:

```matlab
u_sat - u_raw = 0
```

Entonces el integrador sigue el error normal. Si hay saturacion, ese termino empuja el integrador para que no siga creciendo sin control.

## 10. Controlador H_inf

En `make_controller('hinf',...)`, cada controlador H_inf se convierte a espacio de estados:

```matlab
[A, B, C, D] = ssdata(ss(K))
```

Luego se simula como cualquier sistema dinamico:

```math
\dot{x}_K = A_K x_K + B_K e
```

```math
u = C_K x_K + D_K e
```

Para `theta`:

```text
entrada: e_theta
salida: elevator
```

Para `phi`:

```text
entrada: e_phi
salida: aileron
```

El yaw damper se conserva igual que en PID para no mezclar el analisis de H_inf de pitch/roll con un diseno nuevo de yaw.

## 11. Funcion `closed_loop_ode`

Esta es la funcion que ve `ode45`.

En cada instante:

1. separa estados de planta y controlador;
2. calcula salidas reales `y = C*x`;
3. agrega ruido a las mediciones;
4. calcula referencias y errores;
5. calcula control crudo;
6. satura el control;
7. arma el vector de 8 entradas;
8. calcula derivadas de planta y controlador;
9. devuelve `dz`.

La ecuacion completa queda:

```math
\dot{z} =
\begin{bmatrix}
A x + B u \\
\dot{x}_{controlador}
\end{bmatrix}
```

## 12. Reconstruccion de resultados

`ode45` solo entrega `t` y `z`. Despues, `reconstruct_simulation` vuelve a recorrer la solucion para guardar senales utiles:

- salidas reales;
- salidas medidas;
- referencias;
- errores;
- control crudo;
- control saturado;
- vector completo de entrada;
- ruido usado;
- metricas.

Esto se hace porque durante la integracion no conviene guardar variables globales o ir acumulando arreglos dentro del ODE.

## 13. Metricas

`metrics_for_case` calcula:

- maximo absoluto de `theta`;
- maximo absoluto de `phi`;
- maximo absoluto de `elevator`;
- maximo absoluto de `aileron`;
- maximo absoluto de `rudder`;
- error final promedio en los ultimos 2 segundos;
- error RMS despues del escalon;
- fraccion de tiempo con algun actuador saturado.

El error RMS se calcula despues de `cfg.sim.t_step`, para no incluir el tramo antes de aplicar referencia.

## 14. Como leer un resultado `sim`

Ejemplo:

```matlab
load results/taller1_results.mat
sim_results.pid(1)
```

Campos importantes:

- `t`: vector de tiempo.
- `theta_deg`, `phi_deg`: salidas en grados.
- `theta_ref_deg`, `phi_ref_deg`: referencias en grados.
- `elevator_deg`, `aileron_deg`, `rudder_deg`: comandos saturados.
- `u_raw`: comando antes de saturacion.
- `u_sat`: comando despues de saturacion.
- `metrics`: resumen numerico del caso.

Para comparar un escenario:

```matlab
sim_results.pid(1).metrics
sim_results.hinf(1).metrics
```

## 15. Que revisar si algo sale raro

Si el sistema explota:

- revisar que `isstable` del lazo nominal sea verdadero;
- revisar signos de `kp`, `ki`, `kd`;
- revisar que `idx` encuentre bien `theta`, `phi`, `p`, `q`, `r`;
- revisar saturacion en `u_raw` y `u_sat`.

Si H_inf parece peor que PID:

- no significa que el codigo este mal;
- puede significar que los pesos `W1`, `W2`, `W3` todavia no representan bien las prioridades;
- revisar `K*S`, porque si crece mucho el controlador esta pagando con actuador;
- revisar `T`, porque si baja en alta frecuencia puede estar ganando robustez a cambio de seguimiento temporal.

Si el ruido cambia entre PID y H_inf:

- revisar que `rng(cfg.sim.rng_seed)` se ejecute antes de cada familia.

## 16. Resumen corto

`simulacion_taller1.m` es el puente entre teoria y comportamiento temporal. Toma los controladores, los mete en la planta acoplada, agrega ruido, perturbaciones y saturacion, y entrega metricas comparables para PID y H_inf.
