# Bitacora de validacion SAS/CAS y Hinf

Fecha de inicio de bitacora: 2026-05-21.

Este documento es una bitacora de seguimiento para no perder el hilo entre iteraciones del Taller 1. El objetivo no es reemplazar el reporte final, sino registrar que se probo, que fallo, que decision se tomo y cual es el siguiente experimento razonable.

## 1. Estado actual del taller

El taller ya tiene tres lineas de trabajo separadas:

1. Baseline anterior congelado.
2. SAS/CAS propio con arquitectura `PI + D`.
3. Hinf redisenado y validado contra simulaciones temporales, saturacion y sensibilidades.

Archivos principales:

```text
main_taller1.m
parametros_taller1.m
diseno_sas_root_locus.m
diseno_cas_pi_root_locus.m
diseno_pid_sas_cas.m
diseno_hinf_taller1.m
simulacion_taller1.m
evaluar_saturacion_controlador.m
optimizar_pesos_hinf.m
validacion_extrema_taller1.m
reporte_taller1.md
```

Artefactos de resultados:

```text
results/baseline_actual_resumen.md
results/taller1_results_baseline_actual.mat
results/taller1_results.mat
results/hinf_weight_sweep_resumen.md
results/revision_figuras_hinf_y_validacion_extrema.md
results/validacion_extrema_taller1.mat
```

## 2. Baseline anterior

El baseline anterior se congelo antes de redisenar:

```text
results/taller1_results_baseline_actual.mat
results/baseline_actual_resumen.md
snapshots/20260520_151057_baseline_pre_mejora/
```

Hallazgo principal:

- El Hinf baseline tenia `gamma` moderado, pero fallaba en seguimiento temporal.
- En `theta_10`, Hinf tenia RMS theta aproximadamente `3.381 deg`.
- En `phi_10`, Hinf tenia RMS phi aproximadamente `4.724 deg`.
- En `noise_disturbance`, Hinf saturaba cerca de `6 %` del tiempo.
- `KS` era alto:

```text
theta Hinf baseline: ||KS|| = 10.527
phi Hinf baseline:   ||KS|| = 10.257
```

Conclusion del baseline:

> No se debe defender Hinf solo por `gamma`. Las figuras temporales, saturacion y `KS` contradicen una conclusion positiva.

## 3. SAS/CAS propio

Se implemento una arquitectura `PI + D` fisica:

```text
theta_ref -> PI_theta -> q_cmd -> q_cmd - D_q*q -> saturador -> elevator -> planta
phi_ref   -> PI_phi   -> p_cmd -> p_cmd - D_p*p -> saturador -> aileron  -> planta
```

Ganancias actuales:

| Lazo | Ganancia | Valor |
|---|---|---:|
| SAS pitch | `D_q` | -0.20 |
| CAS pitch | `Kp_theta` | -1.00 |
| CAS pitch | `Ki_theta` | -0.30 |
| SAS roll | `D_p` | 0.05 |
| CAS roll | `Kp_phi` | -0.35 |
| CAS roll | `Ki_phi` | -0.18 |

Figuras asociadas:

```text
figures/root_locus_sas_q.png
figures/root_locus_sas_p.png
figures/root_locus_sas_r.png
figures/root_locus_cas_theta.png
figures/root_locus_cas_phi.png
```

Lectura:

- SAS/CAS llega bien a `phi` en estado estacionario.
- SAS/CAS satura poco o nada en los escenarios principales.
- SAS/CAS es mas conservador en RMS que Hinf para algunos casos, pero su comportamiento final de `phi` es mas defendible.

## 4. Hinf redisenado actual

Pesos actuales:

```matlab
W1 = makeweight(80, wb, 0.05);
W2 = 1.0;
W3 = makeweight(0.005, wp, 15);
```

Resultado actual:

```text
gamma theta = 3.6603
gamma phi   = 3.5018
```

Sensibilidades principales:

| Lazo | `||S||` | `||T||` | `||KS||` |
|---|---:|---:|---:|
| theta SAS/CAS | 1.234 | 1.000 | 8.138 |
| theta Hinf | 1.222 | 0.954 | 4.303 |
| phi SAS/CAS | 2.050 | 1.225 | 1.317 |
| phi Hinf | 1.202 | 0.956 | 3.471 |

Lectura:

- Hinf mejoro mucho frente al Hinf baseline.
- Hinf redujo `KS` en theta frente al baseline.
- En `phi`, Hinf tiene mejor `S` que SAS/CAS, pero `KS` queda `2.64` veces mayor que SAS/CAS.
- Esa combinacion explica por que puede verse mejor en RMS, pero quedar mas delicado frente a actuador, ruido y pruebas fuertes.

## 5. Revision critica de figuras

Figuras revisadas con atencion:

```text
figures/sensibilidades_phi.png
figures/sim_phi_10.png
figures/sim_phi_30.png
figures/sim_phi_40.png
figures/sim_noise_disturbance.png
figures/comparacion_temporal_final.png
figures/comparacion_saturacion_final.png
```

Hallazgos:

- En `sim_phi_10.png`, Hinf queda por debajo de la referencia de `phi` por un error final visible, aunque el RMS sea menor que el baseline.
- En `sim_phi_30.png`, Hinf tambien queda por debajo de la referencia.
- En `sim_phi_40.png`, el error final de Hinf en `phi` es mas claro.
- En `sensibilidades_phi.png`, `KS` de Hinf es mas alto que SAS/CAS, anticipando aileron mas agresivo.
- En pruebas de ruido/perturbacion, Hinf mejora frente al baseline, pero todavia no queda libre de sobrepasos o error final.

Conclusion de revision:

> Hinf actual es una mejora sobre el Hinf baseline, pero no cumple como solucion final robusta para `phi`.

## 6. Barrido de saturacion

Se probo subir el limite de actuador para revisar si el problema era solo saturacion:

```text
30 deg
45 deg
60 deg
```

Resultado para `phi_40` con Hinf:

| Limite actuador | Error final phi [deg] | RMS phi [deg] | Saturacion [%] |
|---:|---:|---:|---:|
| 30 | 3.041 | 4.907 | 0.75 |
| 45 | 2.616 | 4.339 | 0.42 |
| 60 | 2.373 | 4.009 | 0.25 |

Interpretacion:

- Aumentar el limite del actuador mejora RMS y reduce saturacion.
- Aun con `60 deg`, Hinf sigue sin llegar exactamente a `phi`.
- Por tanto, el problema no es solo saturacion; tambien es arquitectura/pesos/modelo de sintesis.

Figura asociada:

```text
figures/validacion_saturacion_phi_hinf.png
```

## 7. Pruebas mas fuertes

Se agregaron pruebas con:

- limite de actuador de `60 deg`;
- perturbacion de entrada de `3 deg`;
- ruido con desviacion estandar tres veces mayor;
- referencias mas grandes y combinadas.

Escenarios:

```text
phi_60_hard
theta_phi_45_hard
noise_dist_x3_hard
```

Resultados Hinf:

| Escenario | RMS theta [deg] | RMS phi [deg] | Error final phi [deg] | Flags |
|---|---:|---:|---:|---|
| `phi_60_hard` | 0.150 | 6.735 | 4.092 | `phi_no_llega` |
| `theta_phi_45_hard` | 4.311 | 4.308 | 3.336 | `phi_no_llega` |
| `noise_dist_x3_hard` | 1.794 | 2.288 | 1.632 | `phi_no_llega`, `phi_sobrepasa` |

Figura asociada:

```text
figures/validacion_extrema_hinf.png
```

Conclusion:

> Hinf no queda aprobado en pruebas fuertes. En particular, `phi` mantiene error final y en el caso con ruido/perturbacion fuerte tambien puede sobrepasar.

## 8. Recomendaciones para la siguiente iteracion

### 8.1 No defender Hinf actual como final

El Hinf actual debe describirse como:

```text
Mejora frente al Hinf baseline, pero no solucion final validada.
```

No debe describirse como:

```text
Control robusto final que cumple todos los requisitos.
```

### 8.2 Separar conclusiones por eje

Theta:

- Hinf mejora varios RMS.
- Hinf reduce `KS` frente al Hinf baseline.
- El comportamiento es mas defendible que en `phi`.

Phi:

- Hinf no llega exactamente a referencia.
- Hinf tiene mayor `KS` que SAS/CAS.
- Hinf falla en `phi_40`, `phi_60_hard` y pruebas combinadas.

### 8.3 Probar Hinf MIMO

Siguiente experimento recomendado:

```matlab
G_mimo = plant.full({'theta','phi'}, {'elevator','aileron'});
```

Objetivo:

- disenar un controlador Hinf que vea el acoplamiento `theta-phi`;
- evitar que el diseno SISO de cada eje ignore interacciones;
- penalizar `elevator` y `aileron` de forma conjunta;
- comparar contra SAS/CAS y Hinf SISO.

### 8.4 Agregar integracion explicita para tracking

El error final de `phi` sugiere que se debe probar una planta generalizada con integradores de error o una estructura que garantice seguimiento de escalones.

Hipotesis:

```text
El Hinf SISO actual reduce S, pero no impone tracking exacto en la planta acoplada saturada.
```

Prueba sugerida:

- agregar estados integradores para `theta_ref - theta` y `phi_ref - phi`;
- sintetizar sobre planta aumentada;
- revisar si mejora error final sin disparar `KS`.

### 8.5 Penalizar aileron y KS lateral

La sensibilidad de `phi` muestra que `KS` Hinf es alto frente a SAS/CAS.

Se debe probar:

- mayor penalizacion de control lateral;
- pesos distintos para elevator y aileron;
- `W2_phi` mas fuerte que `W2_theta`;
- penalizacion de alta frecuencia para reducir aileron ruidoso.

### 8.6 Mantener pruebas extremas como criterio de aceptacion

Cada nueva version debe correr:

```matlab
main_taller1
validacion_extrema_taller1
```

Criterios minimos para aceptar Hinf:

| Criterio | Objetivo |
|---|---:|
| Error final `phi_30` | menor a 1.5 deg |
| Error final `phi_40` | menor a 2.0 deg |
| Saturacion persistente | menor a 5 % |
| `KS_phi` | no mucho mayor que SAS/CAS |
| `noise_dist_x3_hard` | sin `phi_no_llega` ni `phi_sobrepasa` |

## 9. Iteracion 1 - 2026-05-20 - pesos Hinf especificos para phi

Objetivo:

- Ejecutar la recomendacion de no tocar saturacion.
- Redisenar primero el Hinf SISO de `phi`.
- Probar `W1_phi` mas fuerte en baja frecuencia y `W2_phi` mas fuerte en alta frecuencia.

Cambios realizados:

- `diseno_hinf_taller1.m` ahora acepta pesos por eje (`theta` y `phi`).
- `theta` conserva los pesos redisenados previos.
- `phi` usa pesos propios.
- `optimizar_pesos_hinf.m` ahora barre candidatos enfocados en `phi`.
- `crear_graficas_taller1.m` grafica la cota de peso correspondiente a cada eje.

Parametros finales seleccionados:

```matlab
% theta se mantiene:
W1_theta = makeweight(80, wb, 0.05);
W2_theta = 1.0;
W3_theta = makeweight(0.005, wp, 15);

% phi se rediseno:
W1_phi = makeweight(220, wb, 0.05);
W2_phi = 0.80*(s/wp + 1)/(s/(wp*3.20/0.80) + 1);
W3_phi = makeweight(0.005, wp, 15);
```

Barrido:

El nuevo barrido guardado en:

```text
results/hinf_weight_sweep_resumen.md
```

selecciono el candidato 10:

| W1 phi low | W2 phi low | W2 phi high | W2 cross | gamma phi | KS phi | err phi30 | err phi40 | sat max |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 220 | 0.8 | 3.2 | 37.7 | 3.778 | 3.080 | 1.284 | 1.943 | 1.71 % |

Resultados nominales con `main_taller1`:

| Caso Hinf | RMS phi [deg] | Error final phi [deg] | Saturacion [%] |
|---|---:|---:|---:|
| `phi_30` | 2.818 | 1.284 | 0.625 |
| `phi_40` | 4.108 | 1.943 | 0.833 |
| `noise_disturbance` | 0.900 | 0.500 | 0.042 |

Sensibilidades:

| Lazo | `||S||` | `||T||` | `||KS||` |
|---|---:|---:|---:|
| phi SAS/CAS | 2.050 | 1.225 | 1.317 |
| phi Hinf iteracion 1 | 1.333 | 1.010 | 3.080 |

Resultados de `validacion_extrema_taller1`:

| Escenario Hinf | RMS phi [deg] | Error final phi [deg] | Sobrepaso phi [deg] | Saturacion [%] | Flags |
|---|---:|---:|---:|---:|---|
| `phi_60_hard` | 5.708 | 2.569 | 0.000 | 0.583 | `ok` |
| `theta_phi_45_hard` | 3.877 | 2.152 | 2.515 | 0.583 | `ok` |
| `noise_dist_x3_hard` | 2.211 | 0.972 | 4.511 | 0.083 | `phi_sobrepasa` |

Lectura:

- La iteracion cumple los criterios nominales de error final en `phi_30` y `phi_40`.
- El problema `phi_no_llega` desaparece incluso en pruebas fuertes.
- `KS_phi` baja frente al Hinf anterior (`3.471 -> 3.080`), pero sigue siendo `2.34` veces el valor SAS/CAS.
- Subir mas `W2_phi` alto en una ronda focal redujo muy poco el sobrepaso hard (`4.51 -> 4.40 deg`) y empeoro algo el error final combinado, por lo que no se adopto.

Decision:

- [x] aceptar como mejora Hinf SISO de `phi`
- [ ] aceptar como solucion final sin matices
- [x] mantener riesgo residual por `phi_sobrepasa` en `noise_dist_x3_hard`

Notas:

- No se cambio el limite de saturacion.
- No se paso todavia a Hinf MIMO porque la falla principal de seguimiento (`phi_no_llega`) ya no aparece.
- Si el criterio hard exige eliminar tambien `phi_sobrepasa`, la siguiente iteracion debe atacar rechazo de ruido/perturbacion lateral; MIMO o integracion explicita quedan como extensiones, pero el precompensador de referencia ya no parece la primera necesidad.

## 10. Plantilla para nuevas entradas

Copiar esta plantilla por cada intento nuevo:

```text
## Iteracion N - fecha

Objetivo:
- 

Cambios realizados:
- 

Parametros principales:
- 

Comandos ejecutados:
- main_taller1
- validacion_extrema_taller1

Resultados clave:
- gamma theta:
- gamma phi:
- ||KS|| theta:
- ||KS|| phi:
- error final phi_40:
- RMS phi_40:
- saturacion phi_40:
- flags en hard tests:

Decision:
- [ ] aceptar
- [ ] rechazar
- [ ] seguir ajustando

Notas:
- 
```

## 11. Proxima accion propuesta

La siguiente accion ya no debe empezar por saturacion ni por precompensador de tracking. Despues de esta iteracion, el orden recomendado es:

1. Mantener esta version SISO por eje como baseline mejorado.
2. Si se exige eliminar `phi_sobrepasa` en `noise_dist_x3_hard`, probar rechazo lateral de ruido/perturbacion:
   - aumentar `W3_phi` en alta frecuencia;
   - probar un peso de control lateral con cruce mas bajo;
   - comparar contra no empeorar `phi_30` y `phi_40`.
3. Si el acoplamiento sigue limitando el resultado, crear un diseno Hinf MIMO sin borrar el Hinf SISO actual:

```text
diseno_hinf_mimo_taller1.m
```

4. Agregar comparacion en simulacion:

```text
SAS/CAS
Hinf SISO
Hinf MIMO
```

5. Repetir `validacion_extrema_taller1`.

La meta no es bajar `gamma` a toda costa. La meta es que las figuras temporales y la tabla de validacion extrema dejen de contradecir la conclusion.
