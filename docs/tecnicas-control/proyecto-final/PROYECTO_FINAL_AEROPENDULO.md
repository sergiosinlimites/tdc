# Proyecto final: Aeropendulo

Fuentes:

- `drive/TDC/PROYECTO/modelo_planta_aeropendulo.pdf`
- `drive/TDC/Programa-2.pdf`

El PDF del proyecto contiene el plano de piezas del aeropendulo. No trae texto
extraible; visualmente muestra piezas mecanicas tipo base, soporte, brazo,
separadores/arandelas y placas de montaje.

## 1. Objetivo del proyecto

Disenar e implementar el control de un sistema mecanico tipo aeropendulo. La
entrega final debe incluir:

- propuesta tecnologica;
- propuesta economica;
- proceso de seleccion de equipos;
- diseno de planta;
- diseno de controlador;
- simulacion MATLAB/Simulink;
- resultados y presentacion.

## 2. Interpretacion del sistema

Un aeropendulo normalmente consiste en:

- un brazo que gira alrededor de un pivote;
- un motor/propulsor que genera empuje;
- gravedad como torque restaurador o desestabilizante segun la geometria;
- friccion en el eje;
- medicion angular;
- actuador con saturacion.

Variable principal:

```math
\theta = \text{angulo del brazo}
```

Entrada:

```math
u = \text{comando al motor}
```

Salida:

```math
y = \theta
```

Estados recomendados:

```math
x =
\begin{bmatrix}
\theta \\
\dot{\theta}
\end{bmatrix}
```

## 3. Alcance recomendado

Version minima fuerte:

- modelo no lineal de segundo orden;
- linealizacion alrededor de un punto de operacion;
- identificacion o estimacion de parametros;
- controlador PID/LQR;
- saturacion de actuador;
- ruido de medicion;
- simulacion en Simulink;
- propuesta de hardware real.

Version avanzada:

- control robusto ante incertidumbre de masa/empuje;
- modo deslizante;
- anti-windup;
- observador de velocidad;
- validacion con barrido de parametros;
- pruebas automatizadas con MCP/Simulink Test.

## 4. Modelo fisico base

Ecuacion rotacional:

```math
J\ddot{\theta} + b\dot{\theta} + mgl\sin(\theta) = \tau_u
```

El torque del actuador puede modelarse como:

```math
\tau_u = l_u F(u)
```

Si el empuje depende aproximadamente del cuadrado de la velocidad del motor:

```math
F(u) = k_f u^2
```

Para una primera version linealizada, se puede usar:

```math
\tau_u \approx K_u u
```

Forma de estado:

```math
\dot{x}_1 = x_2
```

```math
\dot{x}_2 = \frac{1}{J}\left(K_u u - b x_2 - mgl\sin(x_1)\right)
```

## 5. Linealizacion

Alrededor de un punto de operacion:

```math
x_0 =
\begin{bmatrix}
\theta_0 \\
0
\end{bmatrix}
```

```math
u_0 = \frac{mgl\sin(\theta_0)}{K_u}
```

Modelo incremental:

```math
\delta\dot{x} = A\delta x + B\delta u
```

con:

```math
A =
\begin{bmatrix}
0 & 1 \\
-\frac{mgl\cos(\theta_0)}{J} & -\frac{b}{J}
\end{bmatrix}
```

```math
B =
\begin{bmatrix}
0 \\
\frac{K_u}{J}
\end{bmatrix}
```

## 6. Estructura recomendada

```text
proyecto-final-aeropendulo/
|-- README.md
|-- src/
|   |-- main_aeropendulo.m
|   |-- params_aeropendulo.m
|   |-- nonlinear_model_aeropendulo.m
|   |-- linearize_aeropendulo.m
|   |-- design_pid_aeropendulo.m
|   |-- design_lqr_aeropendulo.m
|   |-- run_simulations_aeropendulo.m
|   `-- export_figures_aeropendulo.m
|-- models/
|   `-- aeropendulo.slx
|-- cad/
|   `-- modelo_planta_aeropendulo.pdf
|-- figures/
|-- data/
`-- report/
    |-- informe_parcial_1.md
    |-- informe_parcial_2.md
    `-- presentacion_final.md
```

## 7. Fase P1: propuesta y modelo

Objetivo:

- demostrar que entendemos la planta y que el proyecto es viable.

Entregables:

- descripcion del sistema;
- diagrama mecanico;
- lista de componentes;
- modelo fisico preliminar;
- parametros estimados;
- punto de operacion;
- riesgos tecnicos;
- plan de validacion.

Contenido minimo:

1. Planteamiento del problema.
2. Arquitectura fisica.
3. Sensores y actuadores.
4. Modelo matematico.
5. Supuestos.
6. Primeras simulaciones abiertas.
7. Cronograma.
8. Presupuesto preliminar.

## 8. Fase P2: controlador y simulacion

Objetivo:

- cerrar el lazo y demostrar desempeno.

Entregables:

- controlador disenado;
- modelo Simulink;
- simulaciones nominales;
- simulaciones con incertidumbre;
- analisis de saturacion;
- seleccion final de hardware;
- presupuesto actualizado.

Casos de simulacion:

- referencia pequena;
- referencia media;
- perturbacion angular inicial;
- ruido de medicion;
- variacion de masa o inercia;
- saturacion de motor;
- comparacion PID vs LQR o tecnica elegida.

## 9. Presentacion final

Estructura sugerida:

1. Problema.
2. Planta mecanica.
3. Modelo.
4. Controlador.
5. Simulink.
6. Resultados.
7. Seleccion tecnologica.
8. Costos.
9. Limitaciones.
10. Conclusiones.

Maximo impacto:

- una grafica clara de seguimiento;
- una grafica clara de esfuerzo de control;
- un diagrama Simulink limpio;
- una tabla de requisitos vs resultados;
- una tabla de costos.

## 10. Codigo base

Parametros:

```matlab
function cfg = params_aeropendulo()
    cfg.g = 9.81;

    cfg.plant.J = 1.0e-3;
    cfg.plant.b = 1.0e-3;
    cfg.plant.m = 0.15;
    cfg.plant.l = 0.12;
    cfg.plant.Ku = 0.03;

    cfg.op.theta0 = 20 * pi / 180;
    cfg.op.omega0 = 0;
    cfg.op.u0 = cfg.plant.m * cfg.g * cfg.plant.l * sin(cfg.op.theta0) / cfg.plant.Ku;

    cfg.sim.tFinal = 10;
    cfg.sim.dt = 0.001;

    cfg.control.uMin = 0;
    cfg.control.uMax = 1;
end
```

Modelo no lineal:

```matlab
function dx = nonlinear_model_aeropendulo(~, x, u, cfg)
    theta = x(1);
    omega = x(2);

    J = cfg.plant.J;
    b = cfg.plant.b;
    m = cfg.plant.m;
    l = cfg.plant.l;
    g = cfg.g;
    Ku = cfg.plant.Ku;

    uSat = max(min(u, cfg.control.uMax), cfg.control.uMin);

    thetaDot = omega;
    omegaDot = (Ku * uSat - b * omega - m * g * l * sin(theta)) / J;

    dx = [thetaDot; omegaDot];
end
```

Linealizacion:

```matlab
function lin = linearize_aeropendulo(cfg)
    J = cfg.plant.J;
    b = cfg.plant.b;
    m = cfg.plant.m;
    l = cfg.plant.l;
    g = cfg.g;
    Ku = cfg.plant.Ku;
    theta0 = cfg.op.theta0;

    A = [0, 1;
        -(m*g*l*cos(theta0))/J, -b/J];

    B = [0; Ku/J];
    C = [1, 0];
    D = 0;

    lin.A = A;
    lin.B = B;
    lin.C = C;
    lin.D = D;
    lin.sys = ss(A, B, C, D);
end
```

## 11. Simulink y MCP

Modelo Simulink minimo:

- referencia angular;
- controlador;
- saturacion;
- planta no lineal;
- sensor angular;
- ruido;
- logging de `theta`, `omega`, `u`.

Flujo con MCP:

1. Abrir MATLAB.
2. Ejecutar:

```matlab
satk_initialize
```

3. Usar herramientas:

- `detect_matlab_toolboxes` para validar toolboxes;
- `evaluate_matlab_code` para pruebas rapidas;
- `model_overview` para leer la arquitectura;
- `model_read` para inspeccionar bloques;
- `model_edit` para crear o ajustar el modelo;
- `run_matlab_file` para ejecutar `main_aeropendulo.m`;
- `check_matlab_code` antes de entregar.

Simulacion programatica:

```matlab
in = Simulink.SimulationInput('aeropendulo');
in = in.setModelParameter('StopTime', '10');
in = in.setVariable('cfg', cfg);
out = sim(in);
```

## 12. Seleccion tecnologica

Componentes a justificar:

- motor o propulsor;
- ESC o driver;
- sensor angular;
- microcontrolador;
- fuente de alimentacion;
- estructura mecanica;
- rodamientos/eje;
- elementos de seguridad.

Tabla minima:

| Componente | Opcion | Criterio | Costo estimado |
| --- | --- | --- | ---: |
| Sensor angular | Encoder/IMU/potenciometro | resolucion, ruido, facilidad | por definir |
| Actuador | Motor DC/BLDC | empuje, respuesta, driver | por definir |
| Controlador | Arduino/STM32/ESP32 | frecuencia, PWM, ADC | por definir |
| Fuente | Bateria/fuente DC | corriente maxima | por definir |

## 13. Criterios de desempeno

Definir objetivos cuantitativos:

- error estacionario menor a `2 deg`;
- sobrepaso menor a `15%`;
- tiempo de establecimiento menor a `3 s`;
- control dentro de saturacion;
- estabilidad ante perturbacion inicial;
- desempeno aceptable con `+-20%` de incertidumbre.

Estos valores se pueden ajustar cuando se conozca mejor la planta.

## 14. Checklist de cierre

- [ ] Plano mecanico copiado a `cad/`.
- [ ] Parametros nominales definidos.
- [ ] Modelo no lineal implementado.
- [ ] Linealizacion validada.
- [ ] Controlador PID o LQR disenado.
- [ ] Simulink construido.
- [ ] Saturacion incluida.
- [ ] Ruido de medicion incluido.
- [ ] Barrido de incertidumbre realizado.
- [ ] Presupuesto incluido.
- [ ] Seleccion tecnologica justificada.
- [ ] Presentacion final preparada.
