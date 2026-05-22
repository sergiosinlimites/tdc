# Reporte de revision Hinf, sensibilidades y pruebas fuertes

Fecha: 2026-05-20.

## 1. Estado de la revision

Se revisaron nuevamente las figuras temporales y de sensibilidad, con foco en `phi`, porque las respuestas `sim_phi_10.png`, `sim_phi_30.png`, `sim_phi_40.png` y `sensibilidades_phi.png` muestran una contradiccion importante:

- Hinf reduce RMS frente al baseline anterior.
- Hinf no llega exactamente a la referencia de `phi`.
- Hinf requiere mas esfuerzo de aileron que SAS/CAS.
- Subir la saturacion ayuda, pero no corrige completamente el error final.

Por eso Hinf no debe presentarse como controlador final aprobado sin matices.

## 2. Como se llego a estos resultados

Primero se ejecuto el flujo principal:

```matlab
main_taller1
```

Ese flujo disena:

- SAS/CAS con `D_q = -0.20`, `D_p = 0.05`;
- CAS PI con `Kp_theta = -1.00`, `Ki_theta = -0.30`, `Kp_phi = -0.35`, `Ki_phi = -0.18`;
- Hinf con `W1 = makeweight(80, wb, 0.05)`, `W2 = 1.0`, `W3 = makeweight(0.005, wp, 15)`.

Despues se ejecuto:

```matlab
validacion_extrema_taller1
```

Ese script hizo dos cosas:

1. Barrido de saturacion con limites de actuador de `30`, `45` y `60 deg`.
2. Pruebas fuertes con limite `60 deg`, referencias grandes, perturbacion de `3 deg` y ruido multiplicado por `3` en desviacion estandar.

Finalmente se extrajeron frecuencias pico de `S`, `T` y `KS` con:

```matlab
[valor, frecuencia] = norm(sys, inf)
```

## 3. Lectura de S, T y KS para Hinf

| eje | gamma | `||S||` | w pico S [rad/s] | f pico S [Hz] | `||T||` | w pico T [rad/s] | f pico T [Hz] | `||KS||` | w pico KS [rad/s] | f pico KS [Hz] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| theta | 3.6603 | 1.2219 | 29.6509 | 4.7191 | 0.9543 | 0.0240 | 0.0038 | 4.3028 | 0.0000 | 0.0000 |
| phi | 3.5018 | 1.2018 | 31.2435 | 4.9726 | 0.9562 | 0.0000 | 0.0000 | 3.4714 | 171.8082 | 27.3441 |

Interpretacion:

- `S` mide error y rechazo de perturbaciones lentas. El pico de `S` aparece cerca de `4.7-5.0 Hz`, muy cerca de la frecuencia de perturbacion del taller (`6 Hz`) y por debajo de la banda objetivo de actuador (`8 Hz`). Esto indica que hay una zona de sensibilidad delicada justo donde se esta probando el sistema.
- `T` mide transmision de referencia y ruido de medicion. En ambos ejes `||T|| < 1`, y el pico esta cerca de DC. Eso confirma que Hinf no amplifica la referencia, pero tambien explica por que el seguimiento puede quedar por debajo de la referencia: `T(0)` no queda exactamente en `1`.
- `KS` mide esfuerzo de control. En `theta`, el pico queda en DC; el problema principal es esfuerzo de baja frecuencia. En `phi`, el pico de `KS` esta en `171.8 rad/s` (`27.3 Hz`), bastante por encima de la banda objetivo de `8 Hz`. Eso es una senal clara de control de aileron agresivo en alta frecuencia.

La figura `sensibilidades_phi.png` confirma esta lectura: Hinf mejora `S` de baja frecuencia, pero paga con `KS` mayor. Numeros clave:

| metrica phi | SAS/CAS | Hinf |
|---|---:|---:|
| `||KS||` | 1.317 | 3.471 |
| relacion Hinf/SAS | - | 2.64 |
| `S_phi(1e-2 rad/s)` aproximado | - | 4.42 % |

Esto anticipa lo que se ve en las figuras temporales: Hinf es rapido, pero queda con sesgo de seguimiento y usa mas aileron.

## 4. Barrido de saturacion

### Resultado para `phi_40`

| control | limite [deg] | RMS phi [deg] | error final phi [deg] | saturacion [%] | diagnostico |
|---|---:|---:|---:|---:|---|
| SAS/CAS | 30 | 4.578 | 0.021 | 0.00 | ok |
| SAS/CAS | 45 | 4.578 | 0.021 | 0.00 | ok |
| SAS/CAS | 60 | 4.578 | 0.021 | 0.00 | ok |
| Hinf | 30 | 4.907 | 3.041 | 0.75 | `phi_no_llega` |
| Hinf | 45 | 4.339 | 2.616 | 0.42 | `phi_no_llega` |
| Hinf | 60 | 4.009 | 2.373 | 0.25 | `phi_no_llega` |

Subir la saturacion de `30` a `60 deg` reduce RMS y error final en Hinf, pero no elimina el problema. Esto indica que la saturacion no es la unica causa; tambien hay sesgo de baja frecuencia por el diseno de sensibilidad mixta SISO.

### Resultado para `theta_phi_30`

| control | limite [deg] | RMS theta [deg] | RMS phi [deg] | error final theta [deg] | error final phi [deg] | diagnostico |
|---|---:|---:|---:|---:|---:|---|
| SAS/CAS | 30 | 4.321 | 3.680 | 1.028 | 0.267 | ok |
| SAS/CAS | 45 | 4.321 | 3.680 | 1.028 | 0.267 | ok |
| SAS/CAS | 60 | 4.321 | 3.680 | 1.028 | 0.267 | ok |
| Hinf | 30 | 3.071 | 3.026 | 1.490 | 2.025 | `phi_no_llega` |
| Hinf | 45 | 2.810 | 2.820 | 1.425 | 2.296 | `phi_no_llega` |
| Hinf | 60 | 2.682 | 2.717 | 1.398 | 2.449 | `phi_no_llega` |

Aqui Hinf reduce RMS frente a SAS/CAS, pero queda peor en error final de `phi`. La figura puede parecer buena si solo se mira el transitorio, pero el criterio de seguimiento estacionario no se cumple.

## 5. Pruebas fuertes

Se uso limite de actuador `60 deg`, perturbacion de entrada de `3 deg` y ruido tres veces mas grande en desviacion estandar.

| control | escenario | RMS theta [deg] | RMS phi [deg] | error final theta [deg] | error final phi [deg] | sobrepaso phi [deg] | saturacion [%] | diagnostico |
|---|---|---:|---:|---:|---:|---:|---:|---|
| SAS/CAS | `phi_60_hard` | 0.232 | 6.867 | 0.019 | 0.031 | 5.307 | 0.00 | ok |
| SAS/CAS | `theta_phi_45_hard` | 6.482 | 5.520 | 1.543 | 0.400 | 4.397 | 0.00 | ok |
| SAS/CAS | `noise_dist_x3_hard` | 2.887 | 2.567 | 0.558 | 0.101 | 3.361 | 0.00 | `phi_sobrepasa` |
| Hinf | `phi_60_hard` | 0.150 | 6.735 | 0.031 | 4.092 | 0.000 | 0.50 | `phi_no_llega` |
| Hinf | `theta_phi_45_hard` | 4.311 | 4.308 | 2.160 | 3.336 | 4.040 | 0.58 | `phi_no_llega` |
| Hinf | `noise_dist_x3_hard` | 1.794 | 2.288 | 0.857 | 1.632 | 5.134 | 0.08 | `phi_no_llega`, `phi_sobrepasa` |

Hinf no se descontrola en el sentido de divergir, pero si falla tracking final en `phi` bajo pruebas grandes. En ruido fuerte, ademas aparece sobrepaso lateral.

## 6. Figuras utiles y limpieza

Figuras utiles para defender esta revision:

- `figures/sensibilidades_phi.png`
- `figures/sensibilidades_theta.png`
- `figures/comparacion_sensibilidades.png`
- `figures/sim_phi_10.png`
- `figures/sim_phi_30.png`
- `figures/sim_phi_40.png`
- `figures/comparacion_temporal_final.png`
- `figures/comparacion_saturacion_final.png`
- `figures/validacion_saturacion_phi_hinf.png`
- `figures/validacion_extrema_hinf.png`

Se elimino el archivo temporal no usado:

```text
results/validacion_extrema_taller1_partial.mat
```

No se borraron imagenes de `figures/` porque las actuales quedan referenciadas por el reporte general o por esta revision. Si se quiere dejar solo un conjunto minimo para entrega, se pueden conservar las diez figuras anteriores y mover las demas a un snapshot.

## 7. Conclusion

Hinf mejora el RMS frente al Hinf baseline y en algunos escenarios frente a SAS/CAS, pero no cumple completamente el requisito de seguimiento de `phi`. La evidencia principal es:

1. `T_phi(0)` queda por debajo de `1`, asi que el seguimiento estacionario queda sesgado.
2. `S_phi` tiene un pico cerca de `5 Hz`, zona cercana a las perturbaciones del taller.
3. `KS_phi` tiene un pico en `27.3 Hz`, lo que anticipa accion de aileron agresiva ante ruido y cambios rapidos.
4. Aumentar saturacion de `30` a `60 deg` mejora el transitorio, pero Hinf sigue marcado como `phi_no_llega`.

Recomendacion: presentar el Hinf actual como iteracion diagnostica, no como resultado final. Para aprobar Hinf haria falta redisenar el planteamiento, idealmente con sintesis MIMO `theta-phi`, integracion explicita o una estructura de precompensacion que fuerce ganancia DC unitaria sin disparar `KS`.
