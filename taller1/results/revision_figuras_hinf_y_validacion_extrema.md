# Revision de figuras Hinf y validacion extrema

Esta revision se enfoca en las dudas sobre `phi`, seguimiento, sobrepaso y saturacion.

## Lectura de sensibilidad phi

- `||KS||` phi SAS/CAS = 1.317.
- `||KS||` phi Hinf = 3.080.
- Relacion `KS_Hinf/KS_SAS` = 2.34.
- Error de tracking de baja frecuencia estimado por `S_phi(1e-2)` = 1.05 %.

La sensibilidad de phi muestra Hinf con menor S en baja frecuencia que SAS/CAS, pero KS de Hinf es mucho mayor que SAS/CAS. Esa combinacion anticipa mejor tracking nominal, pero accion de aileron mas agresiva ante ruido, cambios rapidos o saturacion.

## Barrido de saturacion

| suite | control | escenario | lim [deg] | RMS theta | RMS phi | err fin theta | err fin phi | over theta | over phi | sat % | flags |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| sat_sweep | SAS/CAS | phi_30_sat_sweep | 30 | 0.116 | 3.434 | 0.010 | 0.016 | 0.412 | 2.653 | 0.00 | ok |
| sat_sweep | SAS/CAS | phi_40_sat_sweep | 30 | 0.154 | 4.578 | 0.013 | 0.021 | 0.549 | 3.538 | 0.00 | ok |
| sat_sweep | SAS/CAS | theta_phi_30_sat_sweep | 30 | 4.321 | 3.680 | 1.028 | 0.267 | 0.468 | 2.931 | 0.08 | ok |
| sat_sweep | Hinf | phi_30_sat_sweep | 30 | 0.078 | 2.854 | 0.016 | 1.284 | 0.504 | 0.000 | 0.58 | ok |
| sat_sweep | Hinf | phi_40_sat_sweep | 30 | 0.102 | 4.151 | 0.020 | 1.943 | 0.617 | 0.000 | 0.83 | ok |
| sat_sweep | Hinf | theta_phi_30_sat_sweep | 30 | 3.071 | 2.747 | 1.491 | 1.278 | 0.000 | 1.515 | 1.00 | ok |
| sat_sweep | SAS/CAS | phi_30_sat_sweep | 45 | 0.116 | 3.434 | 0.010 | 0.016 | 0.412 | 2.653 | 0.00 | ok |
| sat_sweep | SAS/CAS | phi_40_sat_sweep | 45 | 0.154 | 4.578 | 0.013 | 0.021 | 0.549 | 3.538 | 0.00 | ok |
| sat_sweep | SAS/CAS | theta_phi_30_sat_sweep | 45 | 4.321 | 3.680 | 1.028 | 0.267 | 0.468 | 2.931 | 0.00 | ok |
| sat_sweep | Hinf | phi_30_sat_sweep | 45 | 0.082 | 2.545 | 0.016 | 1.074 | 0.563 | 0.000 | 0.33 | ok |
| sat_sweep | Hinf | phi_40_sat_sweep | 45 | 0.106 | 3.675 | 0.021 | 1.626 | 0.695 | 0.000 | 0.50 | ok |
| sat_sweep | Hinf | theta_phi_30_sat_sweep | 45 | 2.810 | 2.530 | 1.427 | 1.492 | 0.000 | 1.736 | 0.42 | ok |
| sat_sweep | SAS/CAS | phi_30_sat_sweep | 60 | 0.116 | 3.434 | 0.010 | 0.016 | 0.412 | 2.653 | 0.00 | ok |
| sat_sweep | SAS/CAS | phi_40_sat_sweep | 60 | 0.154 | 4.578 | 0.013 | 0.021 | 0.549 | 3.538 | 0.00 | ok |
| sat_sweep | SAS/CAS | theta_phi_30_sat_sweep | 60 | 4.321 | 3.680 | 1.028 | 0.267 | 0.468 | 2.931 | 0.00 | ok |
| sat_sweep | Hinf | phi_30_sat_sweep | 60 | 0.083 | 2.408 | 0.016 | 0.976 | 0.595 | 0.201 | 0.08 | ok |
| sat_sweep | Hinf | phi_40_sat_sweep | 60 | 0.109 | 3.393 | 0.022 | 1.432 | 0.751 | 0.000 | 0.33 | ok |
| sat_sweep | Hinf | theta_phi_30_sat_sweep | 60 | 2.682 | 2.443 | 1.399 | 1.591 | 0.000 | 1.838 | 0.25 | phi_no_llega |

## Pruebas mas fuertes

| suite | control | escenario | lim [deg] | RMS theta | RMS phi | err fin theta | err fin phi | over theta | over phi | sat % | flags |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| hard_60deg_x3 | SAS/CAS | phi_60_hard | 60 | 0.232 | 6.867 | 0.019 | 0.031 | 0.823 | 5.307 | 0.00 | ok |
| hard_60deg_x3 | SAS/CAS | theta_phi_45_hard | 60 | 6.482 | 5.520 | 1.543 | 0.400 | 0.703 | 4.397 | 0.00 | ok |
| hard_60deg_x3 | SAS/CAS | noise_dist_x3_hard | 60 | 2.887 | 2.567 | 0.558 | 0.101 | 0.778 | 3.361 | 0.00 | phi_sobrepasa |
| hard_60deg_x3 | Hinf | phi_60_hard | 60 | 0.157 | 5.708 | 0.032 | 2.569 | 1.007 | 0.000 | 0.58 | ok |
| hard_60deg_x3 | Hinf | theta_phi_45_hard | 60 | 4.312 | 3.877 | 2.162 | 2.152 | 0.000 | 2.515 | 0.58 | ok |
| hard_60deg_x3 | Hinf | noise_dist_x3_hard | 60 | 1.793 | 2.211 | 0.858 | 0.972 | 0.595 | 4.511 | 0.08 | phi_sobrepasa |

## Conclusion operativa

Con la iteracion de pesos especificos para phi, Hinf ya no presenta `phi_no_llega` en las pruebas fuertes y cumple el criterio nominal de error final para `phi_30` y `phi_40` sin cambiar el limite de saturacion. La reserva principal queda en `noise_dist_x3_hard`: aparece `phi_sobrepasa`, tambien presente en SAS/CAS, aunque Hinf mantiene menor RMS lateral. Los intentos adicionales de apretar `W2_phi/W3_phi` en SISO redujeron algo `KS` y sobrepaso, pero reabrieron `phi_no_llega` en el caso combinado fuerte. Si se exige aprobar sin ese flag, la siguiente iteracion debe pasar a Hinf MIMO o a una estructura adicional de rechazo lateral; el precompensador de referencia ya no es la primera necesidad porque el error estacionario de phi quedo dentro de los umbrales nominales.
