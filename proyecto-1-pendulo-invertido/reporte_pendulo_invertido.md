# Proyecto 1: Péndulo Invertido Sobre Carro

## Resumen

Se implementó una versión reproducible del proyecto en MATLAB/Simulink con:

- modelo no lineal del péndulo invertido sobre carro;
- linealización alrededor de `theta = 0`, vertical hacia arriba;
- verificación de controlabilidad y observabilidad;
- diseño `LQR`;
- simulaciones lineales y no lineales con saturación de actuador;
- observador de Luenberger usando mediciones de `x` y `theta`;
- modelo Simulink lineal en lazo cerrado con saturación.

## Convención y estados

Los estados son:

```math
x_s = [x,\ \dot{x},\ \theta,\ \dot{\theta}]^T
```

La convención usada es `theta = 0` para el péndulo vertical hacia arriba. La entrada `u` es la fuerza horizontal sobre el carro.

## Parámetros nominales

| Parámetro | Valor | Unidad |
| --- | ---: | --- |
| `M` | `0.50` | kg |
| `m` | `0.20` | kg |
| `l` | `0.30` | m |
| `b` | `0.10` | N*s/m |
| `g` | `9.81` | m/s^2 |
| `umax` | `10.00` | N |

## Modelo lineal

Las matrices obtenidas son:

```matlab
A =
         0    1.0000         0         0
         0   -0.2000   -3.9240         0
         0         0         0    1.0000
         0    0.6667   45.7800         0

B =
         0
    2.0000
         0
   -6.6667
```

El sistema abierto tiene polos:

```matlab
[0, -6.7953, -0.1428, 6.7382]
```

El polo positivo confirma la inestabilidad del equilibrio vertical sin control.

## LQR

Selección base:

```matlab
Q = diag([10, 1, 300, 20]);
R = 0.05;
```

Ganancia:

```matlab
K = [-14.1421, -19.4555, -129.8634, -26.7706]
```

Polos de lazo cerrado:

```matlab
[-133.92, -3.88, -0.98 + 0.90i, -0.98 - 0.90i]
```

## Verificaciones

```matlab
rank(ctrb(A,B)) = 4 de 4
rank(obsv(A,C)) = 4 de 4
rank(obsv(A,C_meas)) = 4 de 4
```

Con `C_meas = [1 0 0 0; 0 0 1 0]`, medir posición y ángulo permite estimar los cuatro estados.

## Casos no lineales con saturación

| `theta0` | max `|x|` | max `|theta|` | max `|u|` |
| ---: | ---: | ---: | ---: |
| `3 deg` | `0.056 m` | `3.00 deg` | `6.80 N` |
| `5 deg` | `0.094 m` | `5.00 deg` | `10.00 N` |
| `8 deg` | `0.152 m` | `8.00 deg` | `10.00 N` |
| `10 deg` | `0.193 m` | `10.00 deg` | `10.00 N` |

Los casos de `5` a `10` grados alcanzan saturación, pero permanecen estables en la simulación no lineal local.

## Observador

Se diseñó un observador de Luenberger con polos:

```matlab
[-12, -13, -14, -15]
```

La matriz `L` calculada es:

```matlab
L =
   26.7344    1.0081
  175.2893    9.4982
    1.6540   27.0656
   31.1406  228.8154
```

Con `xhat(0)=0`, el error máximo de estimación en la simulación lineal fue:

```matlab
max ||x - xhat||_2 = 0.4332
```

## Archivos principales

- `main_pendulo.m`: ejecuta todo el flujo y genera resultados.
- `parametros_pendulo.m`: define parámetros físicos, saturación y pesos LQR.
- `modelo_lineal_pendulo.m`: construye `A`, `B`, `C`, `D`.
- `diseno_lqr_pendulo.m`: calcula `K`.
- `simulacion_lineal_pendulo.m`: simula el lazo lineal cerrado.
- `simulacion_no_lineal_pendulo.m`: simula la planta no lineal con `ode45`.
- `diseno_observador_pendulo.m`: calcula el observador.
- `simulacion_observador_lineal_pendulo.m`: valida el lazo con estados estimados.
- `analisis_resultados.m`: ejecuta barridos y exporta gráficas.
- `init_pendulo_simulink.m`: inicializa variables usadas por Simulink.
- `build_pendulo_simulink.m`: regenera `pendulo_invertido.slx`.
- `pendulo_invertido.slx`: modelo Simulink lineal LQR con saturación.

## Limitaciones

El controlador `LQR` se diseñó con el modelo lineal, así que la estabilidad verificada es local. La saturación introduce una no linealidad adicional: aunque los casos probados hasta `10 deg` funcionaron, no debe interpretarse como estabilidad global ni como garantía para inclinaciones grandes.
