# Taller 2: Control robusto de modo deslizante para motor DC

Fuente:

- `drive/TDC/02. TAREAS/T2/Taller2_2021_2.pdf`

## 1. Objetivo

Disenar e implementar controladores por modo deslizante para un motor DC de iman
permanente con una carga puntual variable. El objetivo es seguir referencias de
posicion en el rango:

```math
\phi \in [-30, 30]\ deg
```

El taller compara dos estrategias:

1. modo deslizante puro;
2. modo deslizante con control equivalente.

## 2. Datos del problema

La inductancia de armadura se asume muy pequena:

```math
L \ll 1
```

Por eso se obtiene un modelo de segundo orden con integrador.

Parametros nominales:

| Parametro | Simbolo | Valor |
| --- | --- | ---: |
| Resistencia | `R` | `8 ohm` |
| Momento de inercia | `J` | `9.85e-3 kg m^2` |
| Friccion viscosa | `B` | `2.52e-3 N m s/rad` |
| Constante de motor | `Km` | `1.57e-2 N m/A` |

Carga:

- masa puntual `M <= 300 g`;
- distancia al eje `l = 5 cm`;
- masa y distancia pueden variar `20%`;
- parametros del sistema pueden variar `20%`.

Mediciones disponibles:

- posicion angular;
- velocidad angular.

## 3. Estados y error

El enunciado define:

```math
x_1 = \theta
```

```math
x_2 = \dot{\theta}
```

Error de posicion:

```math
x_e = x_1 - x_r
```

Como `x_r` es constante:

```math
\dot{x}_e = x_2
```

Superficie de deslizamiento:

```math
s = a x_e + x_2
```

Cotas:

```math
|x_1| \leq \pi
```

```math
|x_2| \leq \pi/2
```

## 4. Controlador 1: modo deslizante puro

La ley de control es:

```math
u = -\beta sign(s)
```

El diseno consiste en escoger:

- `a > 0`;
- `beta > 0`.

Condicion indicada:

```math
\beta > \frac{|a x_2 - f|}{|g|}
```

Esto busca garantizar estabilidad y llegada a la superficie.

## 5. Controlador 2: control equivalente + modo deslizante

Ley:

```math
u = u_{eq} + u_{sm}
```

con:

```math
u_{sm} = -\beta sign(s)
```

Para un sistema:

```math
\dot{x} = f(x) + g(x)u
```

el control equivalente nominal es:

```math
u_{eq} = -\frac{a x_2 + \hat{h}(x)}{\hat{g}(x)}
```

Luego, con incertidumbre:

```math
\dot{s} = \delta(x) + g(x)u_{sm}
```

Condicion de robustez:

```math
\frac{|\delta(x)|}{|g(x)|} < \beta
```

## 6. Estructura recomendada

```text
taller2/
|-- README.md
|-- src/
|   |-- main_taller2.m
|   |-- params_taller2.m
|   |-- motor_dynamics.m
|   |-- uncertainty_cases.m
|   |-- design_smc_pure.m
|   |-- design_smc_equivalent.m
|   |-- run_simulations_taller2.m
|   `-- export_figures_taller2.m
|-- models/
|   `-- taller2_motor_dc.slx
|-- figures/
|-- data/
`-- report/
    `-- informe_taller2.md
```

## 7. Paso a paso

### Paso 0. Preparar parametros

```matlab
function cfg = params_taller2()
    cfg.motor.R = 8;
    cfg.motor.J = 9.85e-3;
    cfg.motor.B = 2.52e-3;
    cfg.motor.Km = 1.57e-2;

    cfg.load.M = 0.300;
    cfg.load.l = 0.05;
    cfg.load.variation = 0.20;

    cfg.g = 9.81;
    cfg.units.deg = pi / 180;

    cfg.bounds.x1 = pi;
    cfg.bounds.x2 = pi / 2;

    cfg.ref.valuesDeg = [-30, -15, 15, 30];
    cfg.sim.tFinal = 10;
end
```

### Paso 1. Derivar el modelo

Con `L` despreciable:

```math
i = \frac{u - K_m \dot{\theta}}{R}
```

Torque del motor:

```math
\tau_m = K_m i
```

Dinamica rotacional:

```math
J \ddot{\theta} + B \dot{\theta} + \tau_g(\theta) = \tau_m
```

La carga puntual genera un torque gravitacional del tipo:

```math
\tau_g(\theta) = M g l \cos(\theta)
```

La forma exacta puede cambiar segun la convencion del angulo. Lo importante es
documentar si `theta = 0` corresponde a barra horizontal, como dice el enunciado.

Forma de estado:

```math
\dot{x}_1 = x_2
```

```math
\dot{x}_2 = h(x) + g(x)u
```

### Paso 2. Implementar dinamica

```matlab
function dx = motor_dynamics(~, x, controller, cfg, p)
    theta = x(1);
    omega = x(2);

    ref = controller.ref;
    u = controller.law(theta, omega, ref, cfg, p);
    u = max(min(u, cfg.control.uMax), -cfg.control.uMax);

    h = compute_h(theta, omega, cfg, p);
    g = compute_g(cfg, p);

    dx = [omega; h + g * u];
end
```

Evitar esconder parametros dentro de la funcion. Todo debe venir por `cfg` o por
un caso `p`.

### Paso 3. Escoger `a`

Sobre la superficie:

```math
s = 0
```

entonces:

```math
x_2 = -a x_e
```

El error se comporta como:

```math
\dot{x}_e = -a x_e
```

Por eso `a` controla la rapidez nominal sobre la superficie.

Valores iniciales:

```matlab
aCandidates = [2, 4, 6, 8];
```

### Paso 4. Calcular cota para `beta`

Para cada combinacion de incertidumbre:

1. evaluar el peor caso de `|a*x2 - f|/|g|`;
2. agregar margen;
3. simular.

Patron:

```matlab
beta = margin * betaMin;
```

con:

```matlab
margin = 1.2;
```

### Paso 5. Simular modo deslizante puro

Casos minimos:

- referencia `-30 deg`;
- referencia `-15 deg`;
- referencia `15 deg`;
- referencia `30 deg`;
- parametro nominal;
- parametro con masa alta y longitud alta;
- parametro con masa baja y longitud baja;
- variacion de `R`, `J`, `B`, `Km`.

Guardar:

- `theta`;
- `omega`;
- `s`;
- `u`;
- error;
- energia o funcion candidata de Lyapunov si se usa.

### Paso 6. Control equivalente nominal

Implementar:

```matlab
ueq = -(a * omega + hHat) / gHat;
usm = -beta * sign_smooth(s, eps);
u = ueq + usm;
```

Usar una version suavizada para simular mejor:

```matlab
function y = sign_smooth(s, eps)
    y = s / (abs(s) + eps);
end
```

El informe debe aclarar que la teoria usa `sign(s)`, pero que la simulacion
puede usar una capa limite para reducir conmutacion numerica.

### Paso 7. Comparar controladores

Comparar:

- tiempo de llegada;
- error en estado estacionario;
- amplitud de la accion de control;
- chattering;
- sensibilidad a incertidumbre;
- desempeno con parametros nominales;
- desempeno con parametros extremos.

## 8. Simulink

El modelo debe tener:

- referencia constante;
- calculo de error;
- superficie `s`;
- controlador SM puro;
- controlador equivalente + SM;
- selector de controlador;
- dinamica del motor;
- saturacion;
- logging de `theta`, `omega`, `u`, `s`.

Simulacion desde MATLAB:

```matlab
in = Simulink.SimulationInput('taller2_motor_dc');
in = in.setModelParameter('StopTime', '10');
in = in.setVariable('cfg', cfg);
out = sim(in);
```

Con MCP:

- `model_overview` para revisar arquitectura;
- `model_read` para verificar conexiones;
- `model_edit` para crear o ajustar bloques;
- `model_test` si queremos pruebas persistentes tipo pass/fail.

## 9. Figuras obligatorias

- respuesta de posicion para varias referencias;
- velocidad angular;
- superficie de deslizamiento `s`;
- senal de control `u`;
- comparacion SM puro vs SM equivalente;
- caso nominal vs caso incierto;
- zoom de control para observar chattering.

## 10. Informe

Estructura:

1. Objetivo.
2. Modelo del motor y supuestos.
3. Cotas de incertidumbre.
4. Diseno SM puro.
5. Diseno SM con control equivalente.
6. Simulaciones.
7. Comparacion.
8. Conclusiones.

## 11. Checklist de cierre

- [ ] Modelo no lineal derivado.
- [ ] Convencion de angulo documentada.
- [ ] Parametros nominales en `params_taller2.m`.
- [ ] Casos de incertidumbre definidos.
- [ ] `a` justificado.
- [ ] `beta` justificado con cota.
- [ ] SM puro simulado.
- [ ] SM equivalente simulado.
- [ ] Chattering comentado.
- [ ] Informe con graficas reproducibles.
