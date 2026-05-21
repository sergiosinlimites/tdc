# Taller 1: SAS/CAS PI+D y Hinf para UAV

## 1. Objetivo

El taller compara dos controladores para el UAV linealizado:

1. SAS/CAS propio con arquitectura `PI + D`.
2. Hinf por sensibilidad mixta, validado con las mismas simulaciones temporales.

El criterio final no es solo `gamma`: tambien se revisan seguimiento, saturacion, esfuerzo de control `KS`, ruido, perturbacion y comportamiento sobre `linmodel` acoplado.

## 2. Flujo reproducible

Ejecutar desde MATLAB:

```matlab
cd('/home/sergio/Escritorio/tdc/taller1')
main_taller1
```

El flujo:

1. carga `modelo_lin.mat`;
2. extrae canales `theta/elevator`, `q/elevator`, `phi/aileron`, `p/aileron`, `r/rudder`;
3. disena SAS con root locus;
4. disena CAS PI sobre la planta amortiguada;
5. sintetiza Hinf con pesos redisenados;
6. simula todos los escenarios en `linmodel`;
7. exporta resultados y figuras.

El baseline anterior se congelo en:

```text
results/taller1_results_baseline_actual.mat
results/baseline_actual_resumen.md
```

## 3. Planta y senales

El archivo fuente es:

```text
drive/TDC/02. TAREAS/T1/modelo_lin.mat
```

Contiene:

| Modelo | Uso |
|---|---|
| `longmod` | diseno longitudinal `theta`, `q` |
| `latmod` | diseno lateral `phi`, `p`, `r` |
| `linmodel` | validacion acoplada final |

Entradas principales:

```text
elevator, aileron, rudder
```

Salidas principales:

```text
theta, phi, p, q, r
```

## 4. Arquitectura SAS/CAS implementada

Pitch:

```matlab
e_theta = theta_ref - theta_meas;
q_cmd   = Kp_theta*e_theta + Ki_theta*xi_theta;
u_e_raw = q_cmd - D_q*q_meas;
u_e     = sat(u_e_raw);
```

Roll:

```matlab
e_phi   = phi_ref - phi_meas;
p_cmd   = Kp_phi*e_phi + Ki_phi*xi_phi;
u_a_raw = p_cmd - D_p*p_meas;
u_a     = sat(u_a_raw);
```

Anti-windup:

```matlab
xi_dot = error + Kaw*(u_sat - u_raw)
```

El yaw se mantiene como damper con washout:

```matlab
K_yaw(s) = 0.065*s/(s + 2)
```

## 5. Diseno SAS con root locus

Archivo:

```text
diseno_sas_root_locus.m
```

Ganancias elegidas:

| Lazo SAS | Ganancia |
|---|---:|
| `D_q` | -0.20 |
| `D_p` | 0.05 |
| `D_r` | -0.065 equivalente en la lectura root-locus |

Figuras:

```text
figures/root_locus_sas_q.png
figures/root_locus_sas_p.png
figures/root_locus_sas_r.png
```

La decision clave es el signo: `D_q` negativo amortigua el canal `q/elevator`, mientras que en roll el lazo interno estable y util aparece con `D_p` positivo.

## 6. Diseno CAS PI

Archivo:

```text
diseno_cas_pi_root_locus.m
```

Plantas externas:

```matlab
G_theta_sas = minreal(G_theta/(1 + D_q*G_q));
G_phi_sas   = minreal(G_phi/(1 + D_p*G_p));
```

Ganancias finales:

| CAS | `Kp` | `Ki` |
|---|---:|---:|
| theta | -1.00 | -0.30 |
| phi | -0.35 | -0.18 |

Figuras:

```text
figures/root_locus_cas_theta.png
figures/root_locus_cas_phi.png
```

## 7. Redisenio Hinf

Archivo:

```text
diseno_hinf_taller1.m
optimizar_pesos_hinf.m
```

Pesos finales:

```matlab
W1 = makeweight(80, wb, 0.05);
W2 = 1.0;
W3 = makeweight(0.005, wp, 15);
```

El barrido quedo documentado en:

```text
results/hinf_weight_sweep.mat
results/hinf_weight_sweep_resumen.md
```

Mejor candidato del barrido:

| W1 low | W2 | W3 low | W3 high | gamma theta | gamma phi |
|---:|---:|---:|---:|---:|---:|
| 80 | 1.0 | 0.005 | 15 | 3.660 | 3.502 |

El `gamma` sube frente al baseline porque los pesos ahora son mas exigentes. La mejora se defiende por las simulaciones y por `KS`, no por presentar un `gamma` aislado.

## 8. Sensibilidades finales

| Lazo | `||S||` | `||T||` | `||KS||` |
|---|---:|---:|---:|
| theta SAS/CAS | 1.234 | 1.000 | 8.138 |
| theta Hinf | 1.222 | 0.954 | 4.303 |
| phi SAS/CAS | 2.050 | 1.225 | 1.317 |
| phi Hinf | 1.202 | 0.956 | 3.471 |

Lectura:

- Hinf redisenado baja fuertemente `KS` en theta frente al Hinf baseline.
- En phi, SAS/CAS conserva menor esfuerzo, pero Hinf mejora tracking y sensibilidad frente al Hinf anterior.
- El Hinf nuevo transmite mejor la referencia que el Hinf baseline; por eso las figuras temporales ya no contradicen la conclusion.

## 9. Simulacion acoplada final

Escenarios ejecutados:

```text
theta_10, theta_minus_10, phi_10, phi_minus_10,
theta_phi_10, theta_30, phi_30, theta_phi_30,
theta_40, phi_40, noise_disturbance
```

Resumen principal:

| Control | Escenario | RMS theta [deg] | RMS phi [deg] | Sat [%] |
|---|---|---:|---:|---:|
| SAS/CAS | theta_10 | 1.423 | 0.294 | 0.0 |
| SAS/CAS | phi_10 | 0.039 | 1.135 | 0.0 |
| SAS/CAS | theta_phi_10 | 1.433 | 1.218 | 0.0 |
| SAS/CAS | theta_30 | 4.268 | 0.882 | 0.0 |
| SAS/CAS | phi_30 | 0.116 | 3.404 | 0.0 |
| SAS/CAS | theta_phi_30 | 4.298 | 3.653 | 0.0 |
| SAS/CAS | theta_40 | 5.686 | 1.179 | 0.1 |
| SAS/CAS | phi_40 | 0.154 | 4.539 | 0.0 |
| SAS/CAS | noise_disturbance | 1.427 | 1.261 | 0.0 |
| Hinf | theta_10 | 0.844 | 0.824 | 0.1 |
| Hinf | phi_10 | 0.027 | 0.859 | 0.0 |
| Hinf | theta_phi_10 | 0.842 | 0.865 | 0.1 |
| Hinf | theta_30 | 3.032 | 2.467 | 1.0 |
| Hinf | phi_30 | 0.075 | 3.338 | 0.5 |
| Hinf | theta_phi_30 | 3.038 | 2.993 | 1.0 |
| Hinf | theta_40 | 4.396 | 3.284 | 1.7 |
| Hinf | phi_40 | 0.098 | 4.871 | 0.7 |
| Hinf | noise_disturbance | 0.837 | 0.957 | 0.1 |

Comparado con el Hinf baseline:

- `theta_10`: RMS theta baja de 3.381 a 0.844 deg.
- `phi_10`: RMS phi baja de 4.724 a 0.859 deg.
- `theta_phi_10`: ambos ejes bajan a menos de 0.9 deg RMS.
- `noise_disturbance`: RMS theta baja de 3.354 a 0.837 deg y RMS phi baja de 4.190 a 0.957 deg.
- `KS theta` baja de 10.527 a 4.303.

El caso de 40 deg queda como prueba de estres: Hinf mejora el error, pero acepta saturacion breve; SAS/CAS satura menos y es mas conservador.

## 10. Figuras finales

Figuras de diseno:

```text
figures/root_locus_sas_q.png
figures/root_locus_sas_p.png
figures/root_locus_sas_r.png
figures/root_locus_cas_theta.png
figures/root_locus_cas_phi.png
```

Figuras de validacion:

```text
figures/sim_theta_10.png
figures/sim_phi_10.png
figures/sim_theta_phi_10.png
figures/sim_theta_30.png
figures/sim_phi_30.png
figures/sim_theta_phi_30.png
figures/sim_theta_40.png
figures/sim_phi_40.png
figures/sim_noise_disturbance.png
figures/comparacion_temporal_final.png
figures/comparacion_saturacion_final.png
figures/comparacion_gamma_hinf.png
```

## 11. Revision critica de Hinf

Despues de revisar nuevamente las figuras, el Hinf final no debe presentarse como un controlador que cumple todos los requisitos de seguimiento. La figura mas clara es `sim_phi_40.png`: Hinf queda por debajo de la referencia de `phi`, aunque la saturacion sea breve. En `sim_phi_30.png` se ve el mismo patron con menor magnitud.

La figura `sensibilidades_phi.png` tambien anticipa el problema:

```text
||KS|| phi SAS/CAS = 1.317
||KS|| phi Hinf    = 3.471
KS_Hinf/KS_SAS     = 2.64
```

Hinf mejora `S` de baja frecuencia frente a SAS/CAS, pero mantiene mayor esfuerzo `KS` en `phi`. Eso significa que se acerca mejor a la referencia nominal, pero con aileron mas agresivo y mayor sensibilidad a ruido, cambios rapidos y saturacion.

Se agrego una validacion adicional:

```text
validacion_extrema_taller1.m
results/revision_figuras_hinf_y_validacion_extrema.md
results/validacion_extrema_taller1.mat
figures/validacion_saturacion_phi_hinf.png
figures/validacion_extrema_hinf.png
```

Barrido de limite de actuador para `phi_40` con Hinf:

| Limite actuador | Error final phi [deg] | RMS phi [deg] | Saturacion [%] |
|---:|---:|---:|---:|
| 30 deg | 3.041 | 4.907 | 0.75 |
| 45 deg | 2.616 | 4.339 | 0.42 |
| 60 deg | 2.373 | 4.009 | 0.25 |

Subir la saturacion ayuda al transitorio, pero Hinf sigue sin llegar exactamente a `phi`. Eso apunta a una limitacion del diseno SISO/pesos, no solo al limite de actuador.

Pruebas mas fuertes con limite de 60 deg, perturbacion de 3 deg y ruido tres veces mayor en desviacion estandar:

| Escenario | RMS theta Hinf [deg] | RMS phi Hinf [deg] | Error final phi [deg] | Flags |
|---|---:|---:|---:|---|
| `phi_60_hard` | 0.150 | 6.735 | 4.092 | `phi_no_llega` |
| `theta_phi_45_hard` | 4.311 | 4.308 | 3.336 | `phi_no_llega` |
| `noise_dist_x3_hard` | 1.794 | 2.288 | 1.632 | `phi_no_llega`, `phi_sobrepasa` |

Conclusion de esta revision: Hinf no queda aprobado como solucion final robusta para `phi`; queda como iteracion mejor que el Hinf baseline, pero todavia requiere redisenio. La ruta tecnica mas razonable es pasar a sintesis MIMO o imponer integracion/seguimiento exacto en la planta generalizada, y luego repetir esta validacion extrema.

## 12. Modelo Simulink

Se regenera con:

```matlab
build_taller1_simulink
```

El modelo ahora contiene bloques explicitos:

```text
CAS_PI_theta, SAS_D_q, elevator_raw_sum, sat_elevator
CAS_PI_phi,   SAS_D_p, aileron_raw_sum,  sat_aileron
yaw_damper, sat_rudder
```

Tambien registra:

```text
simout_sas_cas
simout_u
simout_y
tout
```

`simout_sas_cas` incluye referencias, angulos, velocidades, comandos PI, aportes SAS, senales raw y senales saturadas.

## 13. Conclusiones

1. El SAS/CAS final ya no depende de ganancias prestadas: `D_q`, `D_p` y los PI externos estan separados y justificados.
2. El diagrama Simulink coincide con la arquitectura `PI + D` pedida por el profesor.
3. Hinf baseline no era defendible solo por `gamma`; las figuras mostraban mal seguimiento y mucho `KS`.
4. Hinf redisenado mejora claramente frente al Hinf baseline, pero no cumple tracking estricto de `phi` en referencias medianas/grandes.
5. La comparacion final debe presentarse como compromiso: SAS/CAS es simple, llega mejor a `phi` y satura poco; Hinf nuevo reduce errores RMS en varios casos, pero todavia queda condicionado por error final en `phi`, mayor `KS` lateral y sensibilidad a pruebas fuertes.
