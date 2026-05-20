# Plan de mejora del Taller 1: SAS/CAS propio, validacion de figuras y Hinf

## 1. Motivo del rediseño

El taller actual debe corregirse por cinco razones principales:

1. El controlador SAS/CAS no fue diseñado completamente desde la planta, sino que usa ganancias baseline encontradas en el paquete UAV.
2. La arquitectura que pide el profesor es `PI + D`, donde el `PI` es el CAS y el `D` es el SAS. El SAS debe recibir la salida de la planta interna, por ejemplo `q` o `p`, y restarse contra la velocidad/comando generado por el PI antes de entrar al actuador.
3. El diseño debe justificarse con root locus, especialmente para elegir signo y magnitud del SAS.
4. La saturacion de actuadores debe ser parte del criterio de diseño, no solo una grafica al final.
5. El `gamma` reportado por Hinf no puede aceptarse aislado si las figuras temporales muestran mal seguimiento, saturacion o acoplamiento.

## 2. Revision del estado actual

### 2.1 Archivos relevantes

- `Tarea_Definitiva_SAS_CAS.mlx`: guia actual del taller.
- `main_taller1.m`: flujo principal reproducible.
- `parametros_taller1.m`: configuracion, especificaciones y ganancias actuales.
- `diseno_pid_sas_cas.m`: controlador PID/SAS-CAS actual.
- `diseno_hinf_taller1.m`: sintesis Hinf con `mixsyn`.
- `simulacion_taller1.m`: simulacion temporal sobre `linmodel` acoplado.
- `build_taller1_simulink.m`: construccion del modelo `taller1_uav.slx`.
- `figures/`: figuras actuales que ya fueron revisadas por el profesor.
- `results/taller1_results.mat`: resultados numericos actuales.

### 2.2 Bloqueo reproducible detectado

Antes de rediseñar, hay que corregir `parametros_taller1.m`. Actualmente contiene texto pegado en esta linea:

```matlab
cfg.spec.noise_power_lat = 1e-3;Tarea_Definitiva_SAS_CAS.mlx
```

Eso hace que `parametros_taller1(project_dir)` falle. La primera tarea debe ser dejar el proyecto ejecutable desde cero.

## 3. Revision de las figuras actuales

Las figuras actuales respaldan la observacion del profesor: los `gamma` reportados no son suficientes para defender el desempeño real del Hinf.

### 3.1 Figuras temporales

En `figures/sim_theta_10.png`:

- La referencia de `theta` es 10 deg.
- El PID llega cerca de la referencia.
- Hinf se queda aproximadamente en 6-7 deg.
- Hinf ademas introduce deriva en `phi`, aunque la referencia de `phi` es 0.

En `figures/sim_phi_10.png`:

- La referencia de `phi` es 10 deg.
- El PID llega cerca de la referencia.
- Hinf se queda aproximadamente en 5-6 deg.
- El aileron de Hinf pega un pico grande cercano al limite de saturacion.

En `figures/sim_theta_phi_10.png`:

- Hinf no sigue bien `theta`.
- Hinf en `phi` deriva lentamente y cruza la referencia tarde.
- El comportamiento combinado muestra que el diseño SISO no esta garantizando buen desempeño en la planta acoplada.

En `figures/sim_theta_40.png`:

- Hinf queda muy por debajo de la referencia de 40 deg.
- Aparece acoplamiento lateral significativo en `phi`.
- El elevator llega al limite de saturacion.

En `figures/sim_phi_40.png`:

- Hinf llega solo a una fraccion de la referencia de 40 deg.
- El aileron alcanza saturacion.
- El PID tiene mejor seguimiento en esta comparacion.

En `figures/sim_noise_disturbance.png`:

- Hinf mantiene error de seguimiento apreciable.
- El aileron de Hinf se vuelve muy ruidoso.
- La señal de control de Hinf toca repetidamente limites cercanos a +/-30 deg.

### 3.2 Figuras de sensibilidad

En `figures/sensibilidades_theta.png`, `figures/sensibilidades_phi.png` y `figures/comparacion_sensibilidades.png` se observa:

- Hinf tiene menor `T` en baja frecuencia que el PID.
- Menor `T` en baja frecuencia significa peor transmision de referencia, lo cual coincide con que Hinf no llegue a `theta_ref` o `phi_ref`.
- `KS` de Hinf crece bastante en media/alta frecuencia.
- Ese aumento de `KS` coincide con actuadores mas ruidosos y saturacion en las simulaciones temporales.

### 3.3 Metricas actuales guardadas

Del archivo `results/taller1_results.mat`:

```text
gamma theta Hinf = 1.620155
gamma phi   Hinf = 1.574275
```

Pero esas cifras no deben presentarse como prueba de buen desempeño, porque las metricas temporales muestran lo contrario:

```text
Caso              PID theta RMS   Hinf theta RMS   PID phi RMS   Hinf phi RMS
theta_10              1.425           3.381           0.238          3.158
phi_10                0.043           0.054           1.346          4.724
theta_phi_10          1.427           3.355           1.435          3.140
theta_40              5.698          14.279           0.952         12.287
phi_40                0.172           0.146           5.383         25.261
noise_disturbance     1.418           3.354           1.461          4.190
```

Tambien hay saturacion en Hinf:

```text
Hinf theta_40:          50.00 % del tiempo con algun actuador saturado
Hinf phi_40:            33.00 % del tiempo con algun actuador saturado
Hinf noise_disturbance:  5.96 % del tiempo con algun actuador saturado
```

Y las normas actuales son:

```text
theta PID : ||S|| = 1.136, ||T|| = 1.000, ||KS|| =  3.507
theta Hinf: ||S|| = 1.141, ||T|| = 0.762, ||KS|| = 10.527

phi PID   : ||S|| = 1.117, ||T|| = 1.056, ||KS|| =  2.853
phi Hinf  : ||S|| = 1.137, ||T|| = 0.685, ||KS|| = 10.257
```

Interpretacion:

- El Hinf actual no esta ganando en sensibilidad.
- El Hinf actual transmite menos referencia.
- El Hinf actual paga mas esfuerzo de control.
- Por eso el `gamma` actual no representa el comportamiento que se esta viendo en las figuras.

## 4. Hipotesis tecnica sobre el problema Hinf actual

El `gamma` de `mixsyn` se esta calculando sobre el problema SISO nominal definido por los pesos `W1`, `W2`, `W3`. Ese `gamma` no incluye directamente:

- planta completa acoplada `linmodel`;
- saturacion de actuadores;
- ruido discreto usado en simulacion;
- perturbaciones simultaneas en elevator, aileron y rudder;
- tracking simultaneo de `theta` y `phi`;
- arquitectura PI+D fisica que pide el profesor.

Ademas, los pesos actuales permiten demasiado error de baja frecuencia. Si `W1_low_gain = 5` y `gamma` queda cerca de 1.6, la cota aproximada permite:

```text
S(0) <= gamma / W1_low_gain ~= 0.32
```

Eso equivale a permitir errores estacionarios grandes. Para tracking cercano a la referencia, se necesita forzar `S(0)` mucho mas bajo, por ejemplo menor a 0.05 o 0.1 segun el criterio final.

## 5. Nueva estrategia general

El nuevo taller debe separar dos caminos:

1. Diseño clasico SAS/CAS propio con root locus.
2. Rediseño Hinf validado contra las mismas figuras y metricas temporales.

El Hinf no debe evaluarse solo por `gamma`. Debe aceptarse solo si tambien cumple:

- llega a referencia;
- no satura de forma persistente;
- tiene `KS` razonable;
- tolera ruido y perturbaciones;
- funciona en la planta acoplada;
- mejora o al menos compite claramente contra el SAS/CAS propio.

## 6. Fase 0: limpieza y trazabilidad

### Objetivo

Dejar un punto de partida reproducible y auditable.

### Tareas

1. Corregir `parametros_taller1.m`.
2. Ejecutar `main_taller1.m` limpio.
3. Guardar una tabla baseline con:
   - ganancias PID actuales;
   - gammas Hinf actuales;
   - normas `S`, `T`, `KS`;
   - RMS de seguimiento;
   - maximo de actuadores;
   - porcentaje de saturacion;
   - comentario visual por figura.
4. Crear un snapshot antes de tocar el diseño.

### Entregables

- `results/baseline_actual_resumen.md`
- `results/taller1_results_baseline_actual.mat`

## 7. Fase 1: especificar la arquitectura PI+D del profesor

### Arquitectura pitch

```text
theta_ref ---> (+) ---> PI_theta ---> q_cmd ---> (+) ---> saturador ---> elevator ---> planta
                ^ -                         ^ -
                |                           |
              theta                         D_q*q
```

Ley de control:

```matlab
e_theta = theta_ref - theta_meas;
q_cmd   = Kp_theta*e_theta + Ki_theta*xi_theta;
u_e_raw = q_cmd - D_q*q_meas;
u_e     = sat(u_e_raw);
```

### Arquitectura roll

```matlab
e_phi   = phi_ref - phi_meas;
p_cmd   = Kp_phi*e_phi + Ki_phi*xi_phi;
u_a_raw = p_cmd - D_p*p_meas;
u_a     = sat(u_a_raw);
```

### Yaw

Yaw no debe diseñarse como tracking fuerte de `psi`. Se propone mantenerlo como yaw damper:

```matlab
u_r = -D_r * W_washout(s) * r
```

con:

```matlab
W_washout(s) = s/(s + a)
```

## 8. Fase 2: diseño SAS con root locus

### Objetivo

Diseñar `D_q`, `D_p` y opcionalmente `D_r` desde la planta, sin copiar ganancias.

### Tareas

1. Extraer canales:

```matlab
G_q = channels.q;   % elevator -> q
G_p = channels.p;   % aileron  -> p
G_r = channels.r;   % rudder   -> r
```

2. Analizar signo:

```matlab
dcgain(G_q)
dcgain(G_p)
dcgain(G_r)
step(G_q)
step(G_p)
step(G_r)
```

3. Hacer root locus con signo fisico correcto:

```matlab
rlocus(G_q)
rlocus(-G_q)
rlocus(G_p)
rlocus(-G_p)
```

4. Elegir ganancias SAS usando criterios:
   - polos dominantes con mayor amortiguamiento;
   - margen de fase razonable;
   - respuesta sin oscilacion excesiva;
   - esfuerzo de actuador compatible con +/-30 deg.

5. Guardar figuras:

```text
figures/root_locus_sas_q.png
figures/root_locus_sas_p.png
figures/root_locus_sas_r.png
```

### Entregable

- `diseno_sas_root_locus.m`

## 9. Fase 3: diseño CAS PI con root locus

### Objetivo

Diseñar el `PI` externo sobre la planta ya amortiguada por SAS.

### Planta equivalente pitch

Si:

```matlab
u_e = q_cmd - D_q*q
q   = G_q*u_e
theta = G_theta*u_e
```

entonces la planta externa de `q_cmd` a `theta` es:

```matlab
G_theta_sas = minreal(G_theta/(1 + D_q*G_q));
```

### Planta equivalente roll

```matlab
G_phi_sas = minreal(G_phi/(1 + D_p*G_p));
```

### Diseño PI

Diseñar:

```matlab
PI_theta = Kp_theta + Ki_theta/s;
PI_phi   = Kp_phi   + Ki_phi/s;
```

con:

```matlab
rlocus(PI_theta*G_theta_sas)
margin(PI_theta*G_theta_sas)
step(feedback(PI_theta*G_theta_sas,1))
```

### Criterios

- Seguir 10 deg y 30 deg sin error estacionario apreciable.
- Probar 40 deg como caso de estres.
- Evitar saturacion persistente.
- Mantener acoplamiento `theta <-> phi` bajo.
- Mantener `KS` menor que el Hinf actual y razonable frente al limite de actuador.

### Entregable

- `diseno_cas_pi_root_locus.m`

## 10. Fase 4: saturacion y anti-windup

### Objetivo

Convertir saturacion en criterio formal de aceptacion.

### Tareas

1. Separar siempre:

```matlab
u_raw
u_sat
u_aw = u_sat - u_raw
```

2. Mantener anti-windup por back-calculation:

```matlab
xi_dot = error + Kaw*(u_sat - u_raw)
```

3. Evaluar por escenario:
   - `max_abs_elevator_deg`;
   - `max_abs_aileron_deg`;
   - `max_abs_rudder_deg`;
   - `sat_fraction`;
   - tiempo de recuperacion tras saturacion;
   - error final.

4. Rechazar cualquier ganancia que:
   - sature de forma persistente;
   - logre tracking solo por saturacion;
   - produzca control ruidoso cerca de +/-30 deg.

### Entregable

- `evaluar_saturacion_controlador.m`

## 11. Fase 5: rediseño Hinf considerando las figuras

### Objetivo

Reducir `gamma`, pero solo si el desempeño temporal mejora.

### Problema actual

El Hinf actual tiene `gamma` moderado, pero:

- no llega a referencia;
- tiene `T` bajo en baja frecuencia;
- tiene `KS` alto;
- satura mas que el PID;
- se comporta mal ante ruido y perturbacion.

Por tanto, el nuevo objetivo no es solamente bajar `gamma`; es bajar `gamma` sujeto a restricciones temporales.

### Cambios propuestos en pesos

1. Hacer `W1` mas exigente a baja frecuencia para forzar tracking:

```text
Objetivo inicial: S(0) < 0.05 a 0.10
```

2. Hacer `W2` dependiente de frecuencia para penalizar control ruidoso en media/alta frecuencia.

3. Revisar `W3` para no dejar pasar ruido por `T`, pero sin matar el tracking de baja frecuencia.

4. Comparar pesos en una tabla, no solo con una figura.

### Barrido reproducible

Crear:

```text
optimizar_pesos_hinf.m
```

que pruebe combinaciones de:

```matlab
W1_low_gain
W1_cross_frequency
W2_gain
W2_rolloff
W3_low_gain
W3_high_gain
W3_cross_frequency
```

Cada candidato debe evaluarse con:

- `gamma`;
- estabilidad;
- orden del controlador;
- `||S||`, `||T||`, `||KS||`;
- RMS temporal;
- error final;
- saturacion;
- desempeño con ruido;
- desempeño en `theta_phi_10`.

### Regla nueva

Un Hinf con menor `gamma` se rechaza si:

- no llega a la referencia;
- aumenta saturacion;
- empeora `KS`;
- solo funciona en SISO pero falla en `linmodel`.

## 12. Fase 6: actualizacion de Simulink

### Problema actual

`taller1_uav.slx` tiene `PID_theta` y `PID_phi` como bloques `TransferFcn` aplicados al error de angulo. Eso no muestra la estructura fisica pedida por el profesor.

### Cambio requerido

Actualizar `build_taller1_simulink.m` para crear explicitamente:

- bloque `CAS_PI_theta`;
- bloque `SAS_D_q`;
- suma `q_cmd - D_q*q`;
- saturador elevator;
- bloque `CAS_PI_phi`;
- bloque `SAS_D_p`;
- suma `p_cmd - D_p*p`;
- saturador aileron;
- yaw damper con washout.

Tambien se debe loggear:

```text
theta_ref, theta, q, q_cmd, D_q*q, elevator_raw, elevator_sat
phi_ref, phi, p, p_cmd, D_p*p, aileron_raw, aileron_sat
r, rudder_raw, rudder_sat
```

## 13. Fase 7: actualizacion del Live Script

`Tarea_Definitiva_SAS_CAS.mlx` debe ser una guia paso a paso que llame al codigo modular, no un script largo desconectado.

### Nueva estructura sugerida

1. Objetivo del taller.
2. Planta UAV y señales.
3. Por que SAS/CAS equivale a PI+D.
4. Revision critica del estado anterior.
5. Diseño SAS con root locus.
6. Diseño CAS PI con root locus.
7. Saturacion y anti-windup.
8. Rediseño Hinf y significado real de `gamma`.
9. Comparacion final:
   - SAS/CAS propio;
   - Hinf rediseñado;
   - baseline anterior solo como referencia historica.
10. Conclusiones.

### Reglas para el `.mlx`

- Cada seccion debe explicar que se esta haciendo antes del codigo.
- Cada figura debe tener interpretacion.
- No se deben poner ganancias sin justificar.
- No se debe reportar `gamma` sin mostrar tambien seguimiento, saturacion y `KS`.

## 14. Fase 8: criterios de aceptacion finales

El taller corregido se acepta si:

1. El SAS/CAS fue diseñado desde root locus, no copiado.
2. El diagrama PI+D coincide con lo pedido por el profesor.
3. `theta` y `phi` siguen referencias de 10 deg y 30 deg.
4. El caso de 40 deg se presenta como estres y se explica si satura.
5. La saturacion se cuantifica por escenario.
6. Hinf no se defiende solo con `gamma`.
7. El Hinf rediseñado mejora respecto al Hinf actual en:
   - error RMS;
   - error final;
   - saturacion;
   - `KS`;
   - ruido/perturbacion.
8. Las figuras finales no contradicen la conclusion escrita.

## 15. Orden de implementacion recomendado

1. Corregir `parametros_taller1.m`.
2. Crear resumen numerico del estado actual.
3. Crear `diseno_sas_root_locus.m`.
4. Crear `diseno_cas_pi_root_locus.m`.
5. Integrar esos resultados en `diseno_pid_sas_cas.m`.
6. Actualizar `simulacion_taller1.m` para reportar mas metricas.
7. Crear `optimizar_pesos_hinf.m`.
8. Rediseñar pesos Hinf.
9. Actualizar `build_taller1_simulink.m`.
10. Regenerar figuras.
11. Reescribir `Tarea_Definitiva_SAS_CAS.mlx`.
12. Actualizar `reporte_taller1.md`.

## 16. Entregables esperados

```text
taller1/PLAN_MEJORA_TALLER1_SAS_CAS_HINF.md
taller1/results/baseline_actual_resumen.md
taller1/diseno_sas_root_locus.m
taller1/diseno_cas_pi_root_locus.m
taller1/evaluar_saturacion_controlador.m
taller1/optimizar_pesos_hinf.m
taller1/figures/root_locus_sas_q.png
taller1/figures/root_locus_sas_p.png
taller1/figures/root_locus_cas_theta.png
taller1/figures/root_locus_cas_phi.png
taller1/figures/comparacion_temporal_final.png
taller1/figures/comparacion_saturacion_final.png
taller1/figures/comparacion_gamma_hinf.png
```

## 17. Conclusion para orientar el rediseño

El estado actual no debe defenderse diciendo que `gamma` es bajo. Las figuras muestran que el Hinf actual no cumple el objetivo fisico de seguimiento y esfuerzo de control. El nuevo taller debe rediseñar primero el SAS/CAS propio con root locus y luego rehacer Hinf con una validacion mas completa: `gamma` solo cuenta si las respuestas temporales, saturacion y sensibilidad tambien son coherentes.
