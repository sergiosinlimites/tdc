# Proyecto 1: Péndulo invertido sobre carro

## 1. Estrategia de implementación

La idea es construir el proyecto en capas, de menor a mayor complejidad, para que siempre tengamos una versión funcionando:

1. modelo lineal base;
2. diseño `LQR`;
3. simulación del lazo cerrado;
4. planta no lineal;
5. saturación;
6. observador o Kalman;
7. análisis y presentación final.

## 2. Estructura mínima de archivos recomendada

Puedes organizarlo así:

```text
proyecto-1-pendulo-invertido/
├── main_pendulo.m
├── parametros_pendulo.m
├── modelo_lineal_pendulo.m
├── simulacion_no_lineal_pendulo.m
├── diseno_lqr_pendulo.m
├── analisis_resultados.m
└── pendulo_invertido.slx
```

Si prefieres trabajar primero sin Simulink, puedes arrancar solo con scripts `.m` y pasar luego al modelo gráfico.

## 3. Fases de trabajo

### Fase 0. Definición física

Entregables:

- parámetros nominales elegidos;
- definición de estados;
- convención de signos;
- objetivos de desempeño.

Checklist:

- escoger unidades SI;
- fijar si `theta = 0` es la vertical;
- decidir si incluir fricción del carro.

### Fase 1. Modelo lineal

Entregables:

- matrices `A`, `B`, `C`, `D`;
- verificación de dimensiones;
- autovalores del sistema abierto.

Funciones MATLAB típicas:

- `ss`
- `eig`
- `rank`
- `ctrb`
- `obsv`

Resultado esperado:

- ver claramente que el sistema abierto es inestable.

### Fase 2. Diseño `LQR`

Entregables:

- primera selección de `Q` y `R`;
- ganancia `K`;
- autovalores de `A-BK`.

Punto de partida razonable:

```math
Q = \mathrm{diag}([10,\ 1,\ 100,\ 10]), \quad R = 0.1
```

Esto no es definitivo; solo es una base para comenzar.

Funciones MATLAB típicas:

- `lqr`
- `care`
- `eig`

Preguntas que debes contestar:

- qué pasa si penalizas mucho `theta`;
- qué pasa si penalizas mucho `u` mediante `R`;
- cómo cambia el compromiso entre rapidez y esfuerzo.

### Fase 3. Simulación del sistema lineal en lazo cerrado

Entregables:

- respuesta temporal;
- historia de la señal de control;
- comparación de varios `Q` y `R`.

Funciones MATLAB típicas:

- `initial`
- `lsim`
- `step`

Casos mínimos:

- pequeña inclinación inicial;
- perturbación breve;
- distintas condiciones iniciales en `theta`.

### Fase 4. Planta no lineal

Entregables:

- simulación con `ode45` o modelo Simulink no lineal;
- comparación contra el diseño lineal.

Funciones MATLAB típicas:

- `ode45`
- `sim`

Resultado esperado:

- comprobar hasta qué punto el controlador lineal sigue funcionando bien.

### Fase 5. Saturación del actuador

Entregables:

- simulación con `sat(u)`;
- análisis de cuándo el actuador limita;
- discusión de degradación de desempeño.

Objetivo:

- mostrar que el `LQR` ideal puede pedir más fuerza de la físicamente disponible.

### Fase 6. Observador o Kalman

Entregables:

- versión con estados estimados;
- comparación entre control con estados completos y con estimación.

Opciones:

- observador de Luenberger usando `place`;
- filtro de Kalman con `lqe` o `kalman`.

### Fase 7. Presentación final

Entregables:

- gráficas principales;
- tabla de parámetros;
- tabla de casos probados;
- conclusiones técnicas;
- limitaciones y trabajo futuro.

## 4. Diseño de pruebas

### Prueba 1. Estabilización nominal

- condición inicial pequeña en ángulo;
- sin perturbación externa;
- sin saturación al inicio.

Criterio:

- convergencia estable al equilibrio.

### Prueba 2. Sensibilidad a condiciones iniciales

- repetir con `3`, `5`, `8` y `10` grados.

Criterio:

- identificar rango de validez local del diseño.

### Prueba 3. Barrido de `Q` y `R`

- aumentar penalización del ángulo;
- aumentar penalización del esfuerzo de control.

Criterio:

- explicar el compromiso rapidez vs. agresividad.

### Prueba 4. Saturación

- imponer límite de fuerza.

Criterio:

- observar si crece el tiempo de establecimiento o si se pierde estabilidad.

### Prueba 5. Planta no lineal

- usar el mismo `K` sobre el modelo no lineal.

Criterio:

- comparar trayectoria y esfuerzo respecto al modelo lineal.

### Prueba 6. Observador

- cerrar lazo con estados estimados.

Criterio:

- verificar que la estimación converge y que el desempeño sigue siendo aceptable.

## 5. Qué debes mostrar en la entrega

Como mínimo:

- diagrama del sistema;
- ecuaciones del modelo;
- matrices `A`, `B`, `C`, `D`;
- verificación de controlabilidad;
- diseño `LQR`;
- interpretación física de `Q` y `R`;
- autovalores de lazo abierto y lazo cerrado;
- gráficas de `x`, `theta` y `u`;
- comparación lineal vs. no lineal;
- efecto de saturación.

Si quieres que quede más fuerte:

- incluir observador o Kalman;
- incluir interpretación por Lyapunov del sistema cerrado;
- estimar región práctica de operación segura.

## 6. Extensiones opcionales

- seguimiento de referencia de posición con prefiltro;
- controlador con integrador para error estacionario;
- comparación `LQR` vs. asignación de polos;
- comparación `LQR` vs. `MPC` básico;
- discretización y control digital con `dlqr`;
- ruido de medición y filtro de Kalman.

## 7. Siguiente paso recomendado

El siguiente paso más útil no es abrir Simulink todavía, sino fijar una versión base del modelo y escoger parámetros nominales. Con eso ya podemos escribir el primer script MATLAB y calcular `A`, `B`, `C`, `D`, controlabilidad y el primer `LQR`.
