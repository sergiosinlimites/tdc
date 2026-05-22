# Intentos Hinf SISO para ruido lateral

Fecha: 2026-05-21.

## Checkpoint previo

Antes de modificar pesos se creo:

```text
taller1/snapshots/20260521_232727_checkpoint_pre_mejora_hinf_ruido/
```

Ese checkpoint conserva la version estable anterior:

- `W1_phi_low = 220`
- `W2_phi_low = 0.80`
- `W2_phi_high = 3.20`
- `W2_phi_cross = wp`
- `W3_phi_high = 15`
- `W3_phi_cross = wp`

## Objetivo

Reducir `phi_sobrepasa` en `noise_dist_x3_hard` sin tocar saturacion y sin
romper los criterios ya cumplidos:

- `phi_30` con error final menor a `1.5 deg`;
- `phi_40` con error final menor a `2.0 deg`;
- sin reabrir `phi_no_llega` en validacion extrema.

## Resultado de la exploracion

El barrido SISO por pesos encontro candidatos que reducian `KS_phi` y el
sobrepaso de ruido, pero todos los que mejoraban de forma material el esfuerzo
lateral reabrieron `phi_no_llega` en el escenario combinado fuerte
`theta_phi_45_hard`.

Comparacion de puntos representativos:

| Caso | W1 phi | W2 low | W2 high | W2 cross | W3 high | W3 cross | gamma phi | KS phi | err phi40 | err combo hard | over noise hard | Decision |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| checkpoint | 220 | 0.80 | 3.20 | 37.7 | 15 | 37.7 | 3.778 | 3.080 | 1.943 | 2.152 | 4.512 | conservar |
| agresivo 1 | 320 | 1.00 | 5.50 | 18.0 | 45 | 22.0 | 4.561 | 2.191 | 0.946 | 2.393 | 4.026 | rechazar |
| suave 1 | 220 | 0.85 | 3.50 | 30.0 | 20 | 37.7 | 3.947 | 2.794 | 1.836 | 2.330 | 4.427 | rechazar |
| suave 2 | 260 | 0.90 | 4.00 | 25.0 | 30 | 30.0 | 4.130 | 2.507 | 1.793 | 2.338 | 4.292 | rechazar |

## Decision

Se rechaza aplicar estos cambios al controlador entregable. La configuracion
se restaura al checkpoint estable porque:

- el sobrepaso en ruido no bajo por debajo del umbral de `3 deg`;
- los candidatos con menor `KS_phi` reabrieron `phi_no_llega` en
  `theta_phi_45_hard`;
- la mejora SISO por pesos parece estar en el limite de la arquitectura actual.

## Siguiente accion recomendada

No seguir apretando solo `W2_phi/W3_phi` en SISO. La siguiente mejora real debe
probar una de estas dos rutas:

1. Hinf MIMO `theta-phi` para que el controlador vea el acoplamiento.
2. Estructura de filtrado/rechazo lateral especifica para ruido, validando que
   no afecte tracking combinado.
