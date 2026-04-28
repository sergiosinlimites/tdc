# Proyecto 1: Péndulo invertido sobre carro

## 1. Propósito del proyecto

Diseñar un controlador en MATLAB/Simulink que estabilice un péndulo invertido montado sobre un carro, manteniendo el ángulo cerca de la vertical y regulando el desplazamiento del carro alrededor de una referencia pequeña o de cero.

Este proyecto está pensado como el primer proyecto fuerte de la ruta porque concentra los conceptos centrales de control moderno sin exigir todavía una arquitectura demasiado grande.

## 2. Objetivos de aprendizaje

Al terminar este proyecto deberías poder:

- formular el modelo no lineal del sistema;
- linealizar alrededor del equilibrio inestable;
- construir el modelo en espacio de estados;
- verificar controlabilidad y observabilidad;
- diseñar un controlador `LQR`;
- justificar la selección de `Q` y `R`;
- evaluar estabilidad del sistema cerrado con `A-BK`;
- introducir estimación de estados cuando no todos los estados se midan;
- comparar simulación lineal contra simulación no lineal;
- documentar limitaciones por saturación y región local de validez.

## 3. Alcance

Este proyecto sí incluye:

- planta no lineal del péndulo sobre carro;
- linealización cerca de `theta = 0` si defines `theta` como desviación respecto a la vertical;
- regulación al equilibrio;
- perturbaciones pequeñas e impulso inicial;
- saturación del actuador;
- versión con todos los estados medidos;
- versión extendida con observador o filtro de Kalman.

Este proyecto no incluye todavía:

- seguimiento agresivo de trayectorias;
- control no lineal exacto;
- hardware en tiempo real;
- identificación experimental.

## 4. Variables e interfaz externa

### Entradas

- fuerza horizontal aplicada al carro `u` en `N`;
- perturbación opcional `d` como fuerza externa o impulso breve.

### Salidas de interés

- posición del carro `x` en `m`;
- velocidad del carro `x_dot` en `m/s`;
- ángulo del péndulo `theta` en `rad`;
- velocidad angular `theta_dot` en `rad/s`;
- señal de control `u`.

### Parámetros físicos mínimos

- masa del carro `M`;
- masa del péndulo `m`;
- longitud al centro de masa `l`;
- gravedad `g`;
- fricción viscosa del carro `b`, si decides incluirla.

## 5. Modelo base

### Estados

Se recomienda usar:

```math
x_s =
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
```

### Punto de equilibrio

Trabajar alrededor de:

```math
x = 0,\quad \dot{x} = 0,\quad \theta = 0,\quad \dot{\theta} = 0,\quad u = 0
```

asumiendo que `theta = 0` representa la posición vertical.

### Modelo lineal esperado

La forma objetivo es:

```math
\dot{x}_s = Ax_s + Bu
```

```math
y = Cx_s + Du
```

No hace falta fijar una sola convención de signos desde ahora, pero sí debes mantener una convención consistente en todo el proyecto.

## 6. Arquitectura funcional recomendada

### Bloques del modelo

- subsistema de parámetros;
- subsistema de planta no lineal;
- subsistema de linealización o modelo lineal equivalente;
- controlador `LQR`;
- saturación de actuador;
- observador de estados opcional;
- bloque de perturbaciones y condiciones iniciales;
- bloque de visualización y métricas.

### Arquitectura de validación

Usa dos lazos de simulación:

- uno con planta lineal para diseñar y ajustar rápidamente;
- otro con planta no lineal para validar robustez local.

## 7. Objetivos de control cuantitativos

Para que el proyecto tenga criterios claros, propongo estas metas iniciales:

- estabilizar el péndulo desde una desviación inicial de `5` a `10` grados;
- llevar `theta` hacia cero sin oscilación sostenida;
- mantener `|theta| < 2` grados después del transitorio principal;
- limitar el desplazamiento del carro a un rango razonable, por ejemplo `|x| < 0.5 m`;
- usar una fuerza de control compatible con una saturación definida, por ejemplo `|u| <= 10 N`.

Estas metas pueden endurecerse después.

## 8. Relación con el curso

Este proyecto conecta directamente con:

- espacio de estados;
- realimentación de estados;
- controlabilidad;
- observabilidad;
- `LQR`;
- ecuación algebraica de Riccati;
- estabilidad del sistema cerrado;
- Lyapunov para interpretar la estabilidad;
- `LQG` si agregas estimador de Kalman.

## 9. Ruta técnica sugerida

### Etapa 1

- modelar el sistema y fijar parámetros nominales;
- obtener `A`, `B`, `C`, `D`;
- revisar autovalores del sistema abierto.

### Etapa 2

- verificar controlabilidad con la matriz `ctrb(A,B)`;
- verificar observabilidad si no mides todos los estados.

### Etapa 3

- diseñar `LQR` con una selección inicial de `Q` y `R`;
- calcular `K` y autovalores de `A-BK`.

### Etapa 4

- simular respuesta ante condiciones iniciales;
- estudiar efecto de cambiar `Q` y `R`.

### Etapa 5

- agregar saturación;
- repetir simulación sobre la planta no lineal;
- identificar pérdida de desempeño o de estabilidad.

### Etapa 6

- agregar observador o filtro de Kalman;
- cerrar el lazo con estados estimados.

## 10. Riesgos técnicos que debes vigilar

- elegir mal la convención del ángulo y diseñar el controlador con signos inconsistentes;
- diseñar con modelo lineal fuera de la región de validez;
- pedir una fuerza imposible al actuador;
- asumir que medir posición y ángulo implica conocer también las velocidades sin filtrado ni estimación;
- concluir estabilidad global cuando solo validaste estabilidad local.

## 11. Cierre esperado del proyecto

Al finalizar, deberías tener:

- un script o `live script` que derive o cargue el modelo;
- un modelo en Simulink para la planta y el controlador;
- una comparación entre planta lineal y no lineal;
- un conjunto de pruebas con diferentes condiciones iniciales;
- una discusión de cómo `Q` y `R` cambian el comportamiento;
- una extensión opcional con observador o Kalman.
