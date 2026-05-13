# Taller 3: Control adaptativo MRAC en MuPAL longitudinal

Fuentes:

- `drive/TDC/02. TAREAS/T3/Adpat_tarea-1.pdf`
- `drive/TDC/02. TAREAS/T3/main_V4.pdf`

## 1. Objetivo

Disenar un controlador adaptativo con modelo de referencia (`MRAC`) para el eje
longitudinal del avion JAXA MuPAL. Se desea controlar el pitch `theta`
manteniendo los estados estables.

## 2. Planta dada

Estados:

```math
x = [u_x,\ u_z,\ \theta,\ q]^T
```

Donde:

- `u_x`: velocidad en el eje x;
- `u_z`: velocidad en el eje z;
- `theta`: pitch;
- `q`: velocidad de pitch.

Matriz de planta:

```matlab
A = [-0.0175  0.173   -9.77   -5.63;
     -0.192  -1.09    -0.846  64.6;
      0       0        0       1;
      0.0081 -0.0738   0.0062 -1.9];

B = [-0.428;
      4.91;
      0;
      4.22];
```

## 3. Modelo de referencia dado

```matlab
Am = [125.6   -0.03    0.166    12.56;
      -0.052  -1.02  -1554.7  -427.82;
       0       0       0        1;
       0.128  -0.0142 -1335.49 -425.12];

Bm = [-138.1;
       1584.2;
       0;
       1361.6];
```

Nota: en el PDF el primer valor de `Bm` aparece con formato ambiguo
`-0138.1`; se interpreta como `-138.1`.

## 4. Lo que pide el taller

### Paso 1

- definir el modelo de referencia;
- simular sistema y modelo de referencia ante entrada tipo paso;
- generar graficas de estados;
- verificar si existen ganancias que cumplen condiciones de acoplamiento;
- si no se encuentran, comentar dificultades.

### Paso 2

- definir leyes de control para que el error:

```math
e = x_m - x
```

tienda a cero;

- demostrar estabilidad;
- seleccionar parametros de diseno;
- simular entrada paso;
- graficar variables relevantes;
- reportar valor final de las ganancias;
- calcular `A + B K`;
- comparar `A + B K` con `Am`;
- repetir con entrada tren de pulsos;
- comentar resultados.

## 5. Estructura recomendada

```text
taller3/
|-- README.md
|-- src/
|   |-- main_taller3.m
|   |-- params_taller3.m
|   |-- mupAL_model.m
|   |-- reference_model.m
|   |-- check_matching_conditions.m
|   |-- mrac_dynamics.m
|   |-- run_step_case.m
|   |-- run_pulse_train_case.m
|   `-- export_figures_taller3.m
|-- models/
|   `-- taller3_mrac_mupal.slx
|-- figures/
|-- data/
`-- report/
    `-- informe_taller3.md
```

## 6. Paso a paso

### Paso 0. Parametros

```matlab
function cfg = params_taller3()
    cfg.A = [-0.0175  0.173   -9.77   -5.63;
             -0.192  -1.09    -0.846  64.6;
              0       0        0       1;
              0.0081 -0.0738   0.0062 -1.9];

    cfg.B = [-0.428; 4.91; 0; 4.22];

    cfg.Am = [125.6   -0.03    0.166    12.56;
              -0.052  -1.02  -1554.7  -427.82;
               0       0       0        1;
               0.128  -0.0142 -1335.49 -425.12];

    cfg.Bm = [-138.1; 1584.2; 0; 1361.6];

    cfg.C = [0 0 1 0];
    cfg.D = 0;

    cfg.sim.tFinal = 10;
    cfg.sim.dt = 0.001;
    cfg.ref.stepAmplitude = 5 * pi / 180;
end
```

### Paso 1. Verificar dimensiones y estabilidad

```matlab
cfg = params_taller3();

assert(size(cfg.A,1) == 4 && size(cfg.A,2) == 4);
assert(size(cfg.B,1) == 4 && size(cfg.B,2) == 1);

eigA = eig(cfg.A);
eigAm = eig(cfg.Am);
```

Preguntas:

- el modelo de referencia es estable?
- la planta es estable?
- los polos de `Am` son razonables?
- que dinamica impone el modelo de referencia?

### Paso 2. Simular planta y referencia sin adaptacion

```matlab
sys = ss(cfg.A, cfg.B, eye(4), zeros(4,1));
sysm = ss(cfg.Am, cfg.Bm, eye(4), zeros(4,1));

t = 0:cfg.sim.dt:cfg.sim.tFinal;
r = cfg.ref.stepAmplitude * ones(size(t));

y = lsim(sys, r, t);
ym = lsim(sysm, r, t);
```

Graficar los cuatro estados para comparar.

### Paso 3. Condiciones de acoplamiento

En MRAC ideal se busca que existan ganancias constantes tales que:

```math
A + B K_x = A_m
```

```math
B K_r = B_m
```

Pero como `B` es columna, `B K_x` tiene rango maximo 1. Por eso no siempre se
puede igualar una matriz completa `A_m - A`.

Chequeo numerico:

```matlab
DeltaA = cfg.Am - cfg.A;
rankB = rank(cfg.B);
rankAug = rank([cfg.B DeltaA]);
```

Para `B K_r = Bm`:

```matlab
Kr_ls = cfg.B \ cfg.Bm;
Bm_error = norm(cfg.B * Kr_ls - cfg.Bm);
```

Para `A + B Kx = Am`:

```matlab
Kx_ls = cfg.B \ (cfg.Am - cfg.A);
Acl_error = norm(cfg.A + cfg.B * Kx_ls - cfg.Am, 'fro');
```

Si el error no es pequeno, el informe debe explicarlo.

### Paso 4. Ley adaptativa

Una forma base:

```math
u = K_x(t)x + K_r(t)r
```

con error:

```math
e = x - x_m
```

La adaptacion suele construirse usando una matriz `P` que resuelve:

```math
A_m^T P + P A_m = -Q
```

con `Q > 0`.

En MATLAB:

```matlab
Q = eye(4);
P = lyap(cfg.Am', Q);
```

Una ley adaptativa tipica:

```math
\dot{K}_x = -\Gamma_x x e^T P B
```

```math
\dot{K}_r = -\Gamma_r r e^T P B
```

El signo puede cambiar segun se use `e = x - xm` o `e = xm - x`. Documentar la
convencion y mantenerla consistente.

### Paso 5. Implementar dinamica aumentada

Estados de simulacion:

```text
z = [x; xm; vec(Kx); Kr]
```

Para SISO:

- `x`: 4 estados;
- `xm`: 4 estados;
- `Kx`: 1x4;
- `Kr`: escalar.

Total: 13 estados.

Patron:

```matlab
function dz = mrac_dynamics(t, z, cfg, inputFcn)
    x = z(1:4);
    xm = z(5:8);
    Kx = reshape(z(9:12), 1, 4);
    Kr = z(13);

    r = inputFcn(t);
    u = Kx * x + Kr * r;

    dx = cfg.A * x + cfg.B * u;
    dxm = cfg.Am * xm + cfg.Bm * r;

    e = x - xm;
    PB = cfg.P * cfg.B;
    scalar = e' * PB;

    dKx = -cfg.GammaX * scalar * x';
    dKr = -cfg.GammaR * scalar * r;

    dz = [dx; dxm; dKx(:); dKr];
end
```

### Paso 6. Seleccionar parametros

Parametros iniciales:

```matlab
cfg.Q = eye(4);
cfg.P = lyap(cfg.Am', cfg.Q);
cfg.GammaX = diag([1, 1, 1, 1]);
cfg.GammaR = 1;
```

Luego ajustar:

- si las ganancias divergen, bajar `Gamma`;
- si la respuesta es lenta, subir gradualmente `Gamma`;
- si hay oscilacion numerica, revisar paso de simulacion y condicionamiento.

### Paso 7. Entrada paso

```matlab
inputFcn = @(t) cfg.ref.stepAmplitude;
```

Guardar:

- estados `x`;
- estados `xm`;
- error `e`;
- control `u`;
- ganancias `Kx(t)` y `Kr(t)`;
- valor final de ganancias.

### Paso 8. Tren de pulsos

```matlab
function r = pulse_train(t, amp, period, duty)
    phase = mod(t, period) / period;
    r = amp * (phase < duty);
end
```

Repetir:

- estados;
- error;
- control;
- ganancias;
- comparacion con el caso paso.

## 7. Simulink

El modelo debe tener:

- planta `A,B`;
- modelo de referencia `Am,Bm`;
- calculo de error;
- leyes adaptativas;
- integradores de ganancias;
- entrada paso y tren de pulsos;
- logging.

Desde MATLAB:

```matlab
in = Simulink.SimulationInput('taller3_mrac_mupal');
in = in.setModelParameter('StopTime', '10');
in = in.setVariable('cfg', cfg);
out = sim(in);
```

Con MCP:

- `model_overview` para revisar arquitectura;
- `model_read` para confirmar senales;
- `model_edit` para modificar bloques;
- `model_test` para pruebas si hay Simulink Test.

## 8. Figuras obligatorias

- estados de planta vs modelo de referencia ante paso;
- error de seguimiento ante paso;
- senal de control ante paso;
- evolucion de `Kx`;
- evolucion de `Kr`;
- estados ante tren de pulsos;
- error ante tren de pulsos;
- comparacion `A + B*Kx_final` vs `Am`.

## 9. Informe

Estructura:

1. Objetivo.
2. Planta MuPAL longitudinal.
3. Modelo de referencia.
4. Condiciones de acoplamiento.
5. Ley adaptativa.
6. Simulacion paso.
7. Simulacion tren de pulsos.
8. Valor final de ganancias.
9. Comparacion `A + B*K` contra `Am`.
10. Conclusiones.

## 10. Checklist de cierre

- [ ] Matrices `A`, `B`, `Am`, `Bm` implementadas.
- [ ] Modelo de referencia simulado.
- [ ] Planta nominal simulada.
- [ ] Condiciones de acoplamiento revisadas.
- [ ] Ley adaptativa documentada.
- [ ] Entrada paso simulada.
- [ ] Tren de pulsos simulado.
- [ ] Ganancias finales reportadas.
- [ ] `A + B*K` comparada con `Am`.
- [ ] Dificultades tecnicas explicadas.
