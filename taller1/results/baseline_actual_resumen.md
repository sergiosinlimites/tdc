# Baseline actual antes del redisenio

Fecha de captura: 2026-05-20.

Archivo congelado:

```text
results/taller1_results_baseline_actual.mat
```

## Ganancias SAS/CAS heredadas

| Ganancia | Valor |
|---|---:|
| `kp_theta` | -0.84 |
| `ki_theta` | -0.23 |
| `kd_theta` | -0.08 |
| `kp_phi` | -0.52 |
| `ki_phi` | -0.20 |
| `kd_phi` | -0.07 |

Estas ganancias corresponden al baseline del paquete UAV, no a un redisenio desde root locus.

## Hinf baseline

| Eje | gamma | Orden K |
|---|---:|---:|
| theta/elevator | 1.6202 | 7 |
| phi/aileron | 1.5743 | 6 |

## Sensibilidades baseline

| Lazo | `||S||` | `||T||` | `||KS||` |
|---|---:|---:|---:|
| theta SAS/CAS heredado | 1.136 | 1.000 | 3.507 |
| theta Hinf | 1.141 | 0.762 | 10.527 |
| phi SAS/CAS heredado | 1.117 | 1.056 | 2.853 |
| phi Hinf | 1.137 | 0.685 | 10.257 |

## Simulacion acoplada baseline

| Control | Escenario | RMS theta [deg] | RMS phi [deg] | Saturacion [%] |
|---|---|---:|---:|---:|
| SAS/CAS heredado | theta_10 | 1.425 | 0.238 | 0.0 |
| SAS/CAS heredado | theta_minus_10 | 1.425 | 0.238 | 0.0 |
| SAS/CAS heredado | phi_10 | 0.043 | 1.346 | 0.0 |
| SAS/CAS heredado | phi_minus_10 | 0.043 | 1.346 | 0.0 |
| SAS/CAS heredado | theta_phi_10 | 1.427 | 1.435 | 0.0 |
| SAS/CAS heredado | theta_40 | 5.698 | 0.952 | 0.1 |
| SAS/CAS heredado | phi_40 | 0.172 | 5.383 | 0.0 |
| SAS/CAS heredado | noise_disturbance | 1.418 | 1.461 | 0.0 |
| Hinf baseline | theta_10 | 3.381 | 3.158 | 0.2 |
| Hinf baseline | theta_minus_10 | 3.381 | 3.158 | 0.2 |
| Hinf baseline | phi_10 | 0.054 | 4.724 | 0.1 |
| Hinf baseline | phi_minus_10 | 0.054 | 4.724 | 0.1 |
| Hinf baseline | theta_phi_10 | 3.355 | 3.140 | 0.2 |
| Hinf baseline | theta_40 | 14.279 | 12.287 | 0.5 |
| Hinf baseline | phi_40 | 0.146 | 25.261 | 0.3 |
| Hinf baseline | noise_disturbance | 3.354 | 4.190 | 6.0 |

## Lectura

El Hinf baseline no se debe defender solo por `gamma`: reduce `T`, pero sube mucho `KS`, no llega bien a referencia y ante ruido/perturbacion satura con frecuencia.
