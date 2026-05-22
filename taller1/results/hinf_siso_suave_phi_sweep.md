# Barrido suave Hinf SISO phi

Regla: Aceptar solo si: phi_30 < 1.5 deg, phi_40 < 2.0 deg, theta_phi_45_hard sin phi_no_llega, KS_phi <= baseline/3.080 y noise_dist_x3_hard con menor sobrepaso que el baseline.

## Baseline

- tag: `baseline`
- pesos: W1 220, W2 0.80 -> 3.20, W2 cross 37.7, W3 high 15
- gamma phi: 3.778; KS phi: 3.080
- phi_30 error final: 1.284 deg; phi_40 error final: 1.943 deg
- theta_phi_45_hard error final phi: 2.152 deg; flags: `ok`
- noise_dist_x3_hard sobrepaso phi: 4.511 deg; flags: `phi_sobrepasa`
- core ok: 1; ruido mejora: 0; aceptado: 0

## Decision

Candidatos aceptados: 1. Mejor candidato: `soft_015`.

- tag: `soft_015`
- pesos: W1 220, W2 0.90 -> 3.20, W2 cross 45, W3 high 20
- gamma phi: 3.825; KS phi: 3.003
- phi_30 error final: 1.314 deg; phi_40 error final: 1.985 deg
- theta_phi_45_hard error final phi: 2.181 deg; flags: `ok`
- noise_dist_x3_hard sobrepaso phi: 4.484 deg; flags: `phi_sobrepasa`
- core ok: 1; ruido mejora: 1; aceptado: 1

Lectura: el candidato aceptado es una micro-mejora formal, pero no elimina
`phi_sobrepasa`. Si el objetivo es quitar ese flag, esta ronda confirma que
conviene cerrar SISO y pasar a Hinf MIMO theta-phi o rechazo lateral adicional.

## Top por score

| cand | tag | W1 | W2 low | W2 high | W2 cross | W3 high | gamma phi | KS phi | err phi30 | err phi40 | err combo | over noise | combo flags | noise flags | core | mejora ruido | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|:---:|:---:|:---:|
| 16 | `soft_015` | 220 | 0.90 | 3.20 | 45 | 20 | 3.825 | 3.003 | 1.314 | 1.985 | 2.181 | 4.484 | `ok` | `phi_sobrepasa` | 1 | 1 | 1 |
| 1 | `baseline` | 220 | 0.80 | 3.20 | 37.7 | 15 | 3.778 | 3.080 | 1.284 | 1.943 | 2.152 | 4.511 | `ok` | `phi_sobrepasa` | 1 | 0 | 0 |
| 10 | `soft_009` | 220 | 0.80 | 3.20 | 37.7 | 18 | 3.778 | 3.080 | 1.291 | 1.954 | 2.151 | 4.511 | `ok` | `phi_sobrepasa` | 1 | 0 | 0 |
| 11 | `soft_010` | 220 | 0.80 | 3.20 | 37.7 | 20 | 3.778 | 3.080 | 1.288 | 1.948 | 2.152 | 4.513 | `ok` | `phi_sobrepasa` | 1 | 0 | 0 |
| 9 | `soft_008` | 220 | 0.80 | 3.20 | 37.7 | 12 | 3.778 | 3.082 | 1.260 | 1.906 | 2.150 | 4.515 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 3 | `soft_002` | 200 | 0.80 | 3.20 | 37.7 | 15 | 3.775 | 3.077 | 1.391 | 2.104 | 2.231 | 4.546 | `ok` | `phi_no_llega, phi_sobrepasa` | 0 | 0 | 0 |
| 8 | `soft_007` | 220 | 0.80 | 3.20 | 45 | 15 | 3.721 | 3.224 | 1.248 | 1.881 | 2.094 | 4.550 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 4 | `soft_003` | 220 | 0.70 | 3.20 | 37.7 | 15 | 3.634 | 3.305 | 1.267 | 1.918 | 2.026 | 4.587 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 12 | `soft_011` | 180 | 0.70 | 2.40 | 37.7 | 12 | 3.591 | 3.307 | 0.917 | 1.381 | 2.215 | 4.654 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 7 | `soft_006` | 220 | 0.80 | 2.80 | 37.7 | 15 | 3.760 | 3.087 | 1.471 | 2.226 | 2.097 | 4.518 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 13 | `soft_012` | 200 | 0.80 | 2.80 | 37.7 | 15 | 3.757 | 3.084 | 1.582 | 2.394 | 2.171 | 4.548 | `ok` | `phi_no_llega, phi_sobrepasa` | 0 | 0 | 0 |
| 17 | `soft_016` | 220 | 0.80 | 2.80 | 45 | 20 | 3.676 | 3.206 | 1.539 | 2.327 | 1.988 | 4.552 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 14 | `soft_013` | 200 | 0.80 | 2.80 | 45 | 15 | 3.673 | 3.202 | 1.624 | 2.454 | 2.065 | 4.582 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 5 | `soft_004` | 220 | 0.90 | 3.20 | 37.7 | 15 | 3.911 | 2.895 | 1.244 | 1.880 | 2.277 | 4.456 | `phi_no_llega` | `phi_no_llega, phi_sobrepasa` | 0 | 1 | 0 |
| 15 | `soft_014` | 220 | 0.90 | 3.20 | 37.7 | 18 | 3.911 | 2.894 | 1.060 | 1.600 | 2.294 | 4.453 | `phi_no_llega` | `phi_no_llega, phi_sobrepasa` | 0 | 1 | 0 |

## Todos los candidatos

| cand | tag | W1 | W2 low | W2 high | W2 cross | W3 high | gamma phi | KS phi | err phi30 | err phi40 | err combo | over noise | combo flags | noise flags | core | mejora ruido | ok |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|:---:|:---:|:---:|
| 1 | `baseline` | 220 | 0.80 | 3.20 | 37.7 | 15 | 3.778 | 3.080 | 1.284 | 1.943 | 2.152 | 4.511 | `ok` | `phi_sobrepasa` | 1 | 0 | 0 |
| 2 | `soft_001` | 180 | 0.80 | 3.20 | 37.7 | 15 | 3.771 | 3.073 | 1.520 | 2.299 | 2.329 | 4.586 | `phi_no_llega` | `phi_no_llega, phi_sobrepasa` | 0 | 0 | 0 |
| 3 | `soft_002` | 200 | 0.80 | 3.20 | 37.7 | 15 | 3.775 | 3.077 | 1.391 | 2.104 | 2.231 | 4.546 | `ok` | `phi_no_llega, phi_sobrepasa` | 0 | 0 | 0 |
| 4 | `soft_003` | 220 | 0.70 | 3.20 | 37.7 | 15 | 3.634 | 3.305 | 1.267 | 1.918 | 2.026 | 4.587 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 5 | `soft_004` | 220 | 0.90 | 3.20 | 37.7 | 15 | 3.911 | 2.895 | 1.244 | 1.880 | 2.277 | 4.456 | `phi_no_llega` | `phi_no_llega, phi_sobrepasa` | 0 | 1 | 0 |
| 6 | `soft_005` | 220 | 0.80 | 2.40 | 37.7 | 15 | 3.707 | 3.080 | 1.955 | 2.964 | 1.888 | 4.500 | `ok` | `phi_sobrepasa` | 0 | 1 | 0 |
| 7 | `soft_006` | 220 | 0.80 | 2.80 | 37.7 | 15 | 3.760 | 3.087 | 1.471 | 2.226 | 2.097 | 4.518 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 8 | `soft_007` | 220 | 0.80 | 3.20 | 45 | 15 | 3.721 | 3.224 | 1.248 | 1.881 | 2.094 | 4.550 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 9 | `soft_008` | 220 | 0.80 | 3.20 | 37.7 | 12 | 3.778 | 3.082 | 1.260 | 1.906 | 2.150 | 4.515 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 10 | `soft_009` | 220 | 0.80 | 3.20 | 37.7 | 18 | 3.778 | 3.080 | 1.291 | 1.954 | 2.151 | 4.511 | `ok` | `phi_sobrepasa` | 1 | 0 | 0 |
| 11 | `soft_010` | 220 | 0.80 | 3.20 | 37.7 | 20 | 3.778 | 3.080 | 1.288 | 1.948 | 2.152 | 4.513 | `ok` | `phi_sobrepasa` | 1 | 0 | 0 |
| 12 | `soft_011` | 180 | 0.70 | 2.40 | 37.7 | 12 | 3.591 | 3.307 | 0.917 | 1.381 | 2.215 | 4.654 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 13 | `soft_012` | 200 | 0.80 | 2.80 | 37.7 | 15 | 3.757 | 3.084 | 1.582 | 2.394 | 2.171 | 4.548 | `ok` | `phi_no_llega, phi_sobrepasa` | 0 | 0 | 0 |
| 14 | `soft_013` | 200 | 0.80 | 2.80 | 45 | 15 | 3.673 | 3.202 | 1.624 | 2.454 | 2.065 | 4.582 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 15 | `soft_014` | 220 | 0.90 | 3.20 | 37.7 | 18 | 3.911 | 2.894 | 1.060 | 1.600 | 2.294 | 4.453 | `phi_no_llega` | `phi_no_llega, phi_sobrepasa` | 0 | 1 | 0 |
| 16 | `soft_015` | 220 | 0.90 | 3.20 | 45 | 20 | 3.825 | 3.003 | 1.314 | 1.985 | 2.181 | 4.484 | `ok` | `phi_sobrepasa` | 1 | 1 | 1 |
| 17 | `soft_016` | 220 | 0.80 | 2.80 | 45 | 20 | 3.676 | 3.206 | 1.539 | 2.327 | 1.988 | 4.552 | `ok` | `phi_sobrepasa` | 0 | 0 | 0 |
| 18 | `soft_017` | 180 | 0.90 | 3.20 | 45 | 20 | 3.818 | 2.997 | 1.554 | 2.346 | 2.359 | 4.558 | `phi_no_llega` | `phi_no_llega, phi_sobrepasa` | 0 | 0 | 0 |
| 19 | `soft_018` | 200 | 0.90 | 3.20 | 45 | 20 | 3.822 | 3.000 | 1.423 | 2.149 | 2.261 | 4.516 | `phi_no_llega` | `phi_no_llega, phi_sobrepasa` | 0 | 0 | 0 |
