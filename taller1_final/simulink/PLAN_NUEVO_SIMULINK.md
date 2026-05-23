# Plan: Nuevo Modelo Simulink — `taller1_simulink_v2.slx`

## 1. Problema con el modelo actual (`taller1_simulink.slx`)

| Problema | Descripción |
|---|---|
| Lazos combinados | PI+D y H∞ comparten la misma planta, demux y conexiones, seleccionados por un `Switch` con `control_mode`. Se pierde la trazabilidad. |
| Sin scopes | Solo hay `To Workspace` para logging post-simulación. No hay visualización interactiva. |
| Bloques artificiales | El Mux de 8 entradas, el Switch dual, y el logging de 17 señales (sas_cas_log_mux) existen solo para manejar la combinación de ambos casos. |

## 2. Filosofía del nuevo modelo

- **Lazos completamente separados**: cada controlador tiene su propia instancia de planta, sus propias señales y sus propios scopes.
- **Scopes estratégicos** en cada punto relevante del lazo: referencia, error, salida del controlador (cruda), saturación, salida de planta, medición ruidosa.
- **Escenarios configurables** por variante de subsistema: saturación, distintos tipos de entrada, ruido, perturbaciones.
- **Autocontenido**: un solo `init_simulink_v2.m` carga todo; el modelo se puede abrir y simular sin pasos previos.

---

## 3. Arquitectura general — 4 subsistemas principales

```
taller1_simulink_v2.slx
├── [Subsistema] Lazo_PID_theta       ← PI+D longitudinal (elevator)
├── [Subsistema] Lazo_PID_phi         ← PI+D lateral (aileron + yaw damper)
├── [Subsistema] Lazo_Hinf_theta      ← H∞ longitudinal (elevator)
├── [Subsistema] Lazo_Hinf_phi        ← H∞ lateral (aileron + yaw damper)
└── [Bloque común] Generador_Señales  ← Referencias, ruido, perturbaciones
```

Cada subsistema `Lazo_*` es un lazo cerrado completo e independiente con:
- Su propia copia de la planta (State-Space del canal SISO correspondiente)
- Su controlador
- Saturación de actuadores
- Puntos de inyección de ruido y perturbación
- Scopes locales

---

## 4. Detalle de cada subsistema

### 4.1 `Lazo_PID_theta` — Control PI+D de pitch

```
                        ┌─────────────┐
  theta_ref ──►(+)─e──►│ PI (CAS)    │──►(+)──►[ Saturación ]──►(+)──►[ G_theta ]──┐
               (-)      └─────────────┘   (-)                    (+)                 │
                │                          │                      │                   │
                │         ┌─────────┐      │     ┌──────────┐    │                   │
                │         │ D (SAS) │◄─────┼─────│ G_q      │    │  dist_input       │
                │         │ = kd*q  │      │     └──────────┘    │  (perturbación    │
                │         └─────────┘      │                     │   de entrada)     │
                │                          │                     │                   │
                └──────────────────────────┼─────────────────────┼───────────────────┘
                                           │                     │
                                      ruido_medición        dist_salida
                                      (noise_theta)         (perturbación
                                                             de salida)
```

**Bloques internos:**
- `Step` / `Signal Generator` → referencia theta
- `Sum` → error = ref - medición
- `Transfer Fcn` → PI: `(kp*s + ki) / s`
- `Gain` → D: `kd` sobre señal `q` (lazo interno SAS)
- `Sum` → PI_out - D_out = elevator_raw
- `Saturation` → ±umax (configurable: 30, 45, 60 deg)
- `Sum` → u_sat + perturbación de entrada
- `State-Space` → `G_theta` (theta/elevator)
- `Sum` → y + perturbación de salida
- `Sum` → y_perturbed + ruido → medición

**Scopes (7 puntos):**

| # | Scope | Señales | Propósito |
|---|---|---|---|
| 1 | `Scope_ref_vs_y_theta_PID` | theta_ref, theta | Seguimiento de referencia |
| 2 | `Scope_error_theta_PID` | e_theta | Convergencia del error |
| 3 | `Scope_ctrl_raw_sat_theta_PID` | u_raw, u_sat, ±umax | Ver saturación del actuador |
| 4 | `Scope_componentes_PID_theta` | PI_out, D_out, sum | Contribución PI vs D |
| 5 | `Scope_perturbaciones_theta_PID` | dist_in, dist_out, noise | Señales externas |
| 6 | `Scope_planta_interna_theta` | q (rate), theta (angle) | Comportamiento interno de la planta |
| 7 | `Scope_sensibilidad_theta_PID` | e_theta ante perturbación | Verificar rechazo de perturbaciones |

---

### 4.2 `Lazo_PID_phi` — Control PI+D de roll + yaw damper

Estructura idéntica a `Lazo_PID_theta` pero para el canal lateral:
- PI sobre `e_phi`
- D (SAS) sobre señal `p` (roll rate)
- Planta: `G_phi` (phi/aileron)
- Yaw damper: washout `0.065*s/(s+2)` sobre `r` → rudder (lazo auxiliar)
- Saturación en aileron y rudder

**Scopes adicionales:**
| # | Scope | Señales |
|---|---|---|
| 1 | `Scope_ref_vs_y_phi_PID` | phi_ref, phi |
| 2 | `Scope_error_phi_PID` | e_phi |
| 3 | `Scope_ctrl_raw_sat_phi_PID` | aileron_raw, aileron_sat |
| 4 | `Scope_yaw_damper` | r, rudder_cmd, rudder_sat |
| 5 | `Scope_acoplamiento_phi` | phi vs theta (crosstalk desde planta acoplada si se usa planta MIMO) |

---

### 4.3 `Lazo_Hinf_theta` — Control H∞ de pitch

```
                        ┌──────────────┐
  theta_ref ──►(+)─e──►│ K_theta_hinf │──►[ Saturación ]──►(+)──►[ G_theta ]──┐
               (-)      │ (State-Space)│                    (+)                 │
                │       └──────────────┘                     │                  │
                │                                            │                  │
                └────────────────────────────────────────────┼──────────────────┘
                                                             │
                                                        ruido + dist
```

**Scopes (6 puntos):**

| # | Scope | Señales | Propósito |
|---|---|---|---|
| 1 | `Scope_ref_vs_y_theta_Hinf` | theta_ref, theta | Seguimiento |
| 2 | `Scope_error_theta_Hinf` | e_theta | Convergencia |
| 3 | `Scope_ctrl_raw_sat_theta_Hinf` | u_raw, u_sat, ±umax | Esfuerzo de control y saturación |
| 4 | `Scope_estados_K_theta` | estados internos del controlador H∞ | Dinámica del controlador |
| 5 | `Scope_perturbaciones_theta_Hinf` | dist_in, dist_out, noise | Señales externas |
| 6 | `Scope_sensibilidad_theta_Hinf` | respuesta a perturbación sola | Verificar S y T |

---

### 4.4 `Lazo_Hinf_phi` — Control H∞ de roll + yaw damper

Análogo a `Lazo_Hinf_theta` pero con `K_phi_hinf` y yaw damper compartido.

---

### 4.5 `Generador_Señales` — Bloque de entradas configurables

Subsistema que genera todas las señales exógenas y las distribuye a los 4 lazos:

| Señal | Tipo | Parámetro configurable |
|---|---|---|
| `theta_ref` | Step / Ramp / Sine / Square | `ref_type_theta`, `ref_amp_theta` |
| `phi_ref` | Step / Ramp / Sine / Square | `ref_type_phi`, `ref_amp_phi` |
| `noise_theta` | Random Number (banda limitada) | `noise_power_long`, `noise_sample_time` |
| `noise_phi` | Random Number (banda limitada) | `noise_power_lat`, `noise_sample_time` |
| `dist_input_theta` | Sine a frecuencia perturbación | `dist_amp`, `dist_freq` |
| `dist_input_phi` | Sine con fase | `dist_amp`, `dist_freq`, fase π/4 |
| `dist_output_theta` | Sine o Step (emula d_o) | Configurable |
| `dist_output_phi` | Sine o Step (emula d_o) | Configurable |

---

## 5. Casos/escenarios según las diapositivas (Sofrony_c.pdf)

Las diapositivas plantean los siguientes problemas de control robusto que deben verse en simulación:

### 5.1 Problema de saturación (slides 5-7)

> "Si consideramos que la fuerza de actuación está limitada tal que −ū ≤ u ≤ ū"

**Implementación en Simulink:**
- Bloque `Saturation` con límite variable `umax` en workspace
- Escenarios: `umax = 30°, 45°, 60°` (barrido de saturación)
- Referencia grande (40°) para forzar saturación
- **Lo que se ve**: PID pierde seguimiento y se bloquea; H∞ mantiene fase pero con error de estado estable

**Scopes clave:** `Scope_ctrl_raw_sat_*` muestra claramente cuándo el actuador se satura.

### 5.2 Perturbación de salida d_o (slides 15, 21, 28)

> "Atenuación a baja frecuencia de perturbaciones d_o en la salida y"

**Implementación:**
- Señal sinusoidal sumada DESPUÉS de la planta (antes del sensor)
- Frecuencia configurable: 6 Hz (del enunciado) o sweep
- **Lo que se ve**: σ̄(S₀) determina la atenuación. H∞ con W1 alto → mejor rechazo.

### 5.3 Perturbación de entrada d_i (slides 21)

> "Atenuación de perturbaciones de entrada (di) en la salida (u) → σ̄(Si) pequeño"

**Implementación:**
- Señal sinusoidal sumada DESPUÉS del actuador, ANTES de la planta
- Emula una perturbación en el actuador (viento, fricción)
- **Lo que se ve**: el lazo PI+D compensa parcialmente; H∞ atenúa mejor en banda.

### 5.4 Ruido de medición η (slides 21)

> "Atenuación de ruido de medición (η) en la salida (y) → σ̄(T₀) pequeño"

**Implementación:**
- `Random Number` sumado a la salida medida (entre planta y comparador)
- Potencias separadas: longitudinal `1e-4`, lateral `1e-3`
- **Lo que se ve**: H∞ con W3 (peso en T) filtra ruido a alta frecuencia; PID lo amplifica por el derivativo.

### 5.5 Esfuerzo de control limitado — KS (slides 21-22)

> "Para limitar la acción de control → σ̄(KS₀)"

**Implementación:**
- Scope que muestra `u_raw` antes de saturar vs `u_sat` después
- Comparar amplitud y espectro del control entre PID y H∞
- W2 en H∞ penaliza KS alto → menor esfuerzo pero posible peor tracking

### 5.6 Seguimiento de referencia — T₀ ≈ 1 (slides 21)

> "Para tener buen seguimiento de referencia → σ̄(T₀) y σ(T₀) ≈ 1"

**Implementación:**
- Entradas tipo Step, Ramp y Sinusoidal para evaluar tracking
- Scope de `ref vs y` directamente
- **Lo que se ve**: T₀ ≈ 1 en baja frecuencia = buen seguimiento del escalón. T₀ < 1 en alta = filtra ruido.

### 5.7 Incertidumbre multiplicativa de entrada (slides 19, 48-49)

> "G = G₀(I + Δᵢ)"

**Implementación (avanzada, opcional):**
- Bloque `Gain` variable entre controlador y planta que modifica la ganancia ±20%
- Emula incertidumbre multiplicativa de entrada
- **Lo que se ve**: estabilidad robusta si σ̄(Tᵢ) < 1/‖Δᵢ‖

### 5.8 Diferentes tipos de entrada (referencia)

| Tipo | Propósito |
|---|---|
| Step (escalón) | Tracking nominal, tiempo de establecimiento, sobrepaso |
| Ramp (rampa) | Error de seguimiento en régimen (tipo 1 vs tipo 0) |
| Sine (seno) | Respuesta en frecuencia, tracking en banda vs rechazo fuera de banda |
| Square (cuadrada) | Peor caso para saturación, ver anti-windup |
| Pulse | Respuesta impulsiva, recuperación |

---

## 6. Matriz de scopes completa

### Resumen por subsistema

| Subsistema | # Scopes | Señales principales |
|---|---|---|
| Lazo_PID_theta | 7 | ref, y, e, PI_out, D_out, u_raw, u_sat, dist, noise, q |
| Lazo_PID_phi | 6 | ref, y, e, PI_out, D_out, u_raw, u_sat, yaw_r, yaw_rud |
| Lazo_Hinf_theta | 6 | ref, y, e, u_raw, u_sat, K_states, dist, noise |
| Lazo_Hinf_phi | 6 | ref, y, e, u_raw, u_sat, K_states, yaw_r, yaw_rud |
| Comparación | 3 | PID_theta vs Hinf_theta, PID_phi vs Hinf_phi, control total |
| **Total** | **28** | |

### Scopes de comparación (top-level)

| Scope | Señales | Propósito |
|---|---|---|
| `Scope_Comparar_theta` | theta_PID, theta_Hinf, ref | Comparar directamente tracking longitudinal |
| `Scope_Comparar_phi` | phi_PID, phi_Hinf, ref | Comparar tracking lateral |
| `Scope_Comparar_control` | u_sat_PID, u_sat_Hinf, ±umax | Comparar esfuerzo de control |

---

## 7. Variables del workspace (init_simulink_v2.m)

### Parámetros de planta
```matlab
% Canales SISO en espacio de estados
[Gtheta_A, Gtheta_B, Gtheta_C, Gtheta_D]  % theta/elevator
[Gphi_A, Gphi_B, Gphi_C, Gphi_D]            % phi/aileron
[Gq_A, Gq_B, Gq_C, Gq_D]                   % q/elevator (para SAS)
[Gp_A, Gp_B, Gp_C, Gp_D]                   % p/aileron (para SAS)
```

### Parámetros de controlador
```matlab
% PID/SAS-CAS
kp_theta, ki_theta, kd_theta
kp_phi, ki_phi, kd_phi
sas_D_q, sas_D_p
cas_pi_theta_num, cas_pi_theta_den
cas_pi_phi_num, cas_pi_phi_den

% H-inf (State-Space)
[Ktheta_A, Ktheta_B, Ktheta_C, Ktheta_D]
[Kphi_A, Kphi_B, Kphi_C, Kphi_D]

% Yaw damper
[Kyaw_A, Kyaw_B, Kyaw_C, Kyaw_D]
```

### Parámetros de escenario (configurables)
```matlab
% Referencias
theta_ref_amp = deg2rad(10);    % Amplitud de referencia theta
phi_ref_amp   = deg2rad(10);    % Amplitud de referencia phi
t_step = 1.0;                   % Tiempo de activación del escalón

% Saturación
umax = deg2rad(30);             % Límite de actuadores (cambiar para barrido)

% Ruido
noise_power_long = 1e-4;
noise_power_lat  = 1e-3;
noise_sample_time = 0.005;
noise_enabled = 1;              % 0 = sin ruido, 1 = con ruido

% Perturbaciones
dist_amp  = deg2rad(1.0);
dist_freq = 2*pi*6;             % 6 Hz
dist_enabled = 1;               % 0 = sin perturbación

% Simulación
t_final = 12.0;
```

---

## 8. Relación con `taller1_completo.m`

El `.m` hace cosas que el Simulink debe reflejar visualmente:

| Sección del .m | Qué refleja el Simulink |
|---|---|
| §4 Diseño SAS (D_q, D_p) | Ganancia `D` en lazo interno PID visible como bloque separado |
| §5 Diseño CAS PI | Bloque `Transfer Fcn` con PI explícito |
| §6 PID equivalente | La suma PI + D con filtro derivativo |
| §7 Síntesis H∞ | Bloque `State-Space` del controlador Hinf |
| §9.3 Ruido y perturbación | Generadores de señales con switches on/off |
| §9.4 Saturación + anti-windup | Bloque `Saturation` (anti-windup no es trivial en Simulink; ver nota) |
| §9.4 ODE lazo cerrado | El propio Simulink resuelve la ODE al simular |
| §10 Figuras temporales | Los Scopes muestran lo mismo que las figuras del .m |
| Validación extrema | Cambiar `umax` y `ref_amp` para reproducir escenarios extremos |

### Nota sobre anti-windup

En el `.m` se implementa anti-windup con ganancia `aw = 8.0` sobre la diferencia `u_sat - u_raw`. En Simulink se puede implementar con:
- Opción 1: PID Controller block con anti-windup integrado (back-calculation)
- Opción 2: Diagrama manual: integrador + feedback de saturación
- **Recomendación**: Usar PID Controller block de Simulink con `AntiWindupMethod = 'back-calculation'` y `Kb = 8.0`

---

## 9. Plan de construcción (script `build_taller1_simulink_v2.m`)

### Fase 1: Infraestructura
1. Crear modelo vacío `taller1_simulink_v2`
2. Configurar solver (ode45), StopTime (`t_final`), InitFcn (correr `init_simulink_v2.m`)

### Fase 2: Subsistema `Generador_Señales`
3. Crear subsistema con Signal Generators configurables
4. Agregar switches para habilitar/deshabilitar ruido y perturbaciones
5. Crear outports para cada señal

### Fase 3: Subsistema `Lazo_PID_theta`
6. Crear subsistema encapsulado
7. Agregar planta `G_theta` como State-Space
8. Agregar planta auxiliar `G_q` para el lazo SAS (derivativo sobre q)
9. Agregar PI como Transfer Fcn + ganancia D
10. Agregar Saturation configurable
11. Agregar puntos de inyección: ruido (suma en medición), dist_input (suma post-saturación), dist_output (suma post-planta)
12. Agregar 7 scopes

### Fase 4: Subsistema `Lazo_PID_phi`
13. Clonar estructura de theta adaptada a phi
14. Agregar yaw damper (State-Space con washout)
15. Agregar scopes

### Fase 5: Subsistema `Lazo_Hinf_theta`
16. Crear subsistema con K_theta_hinf como State-Space
17. Misma estructura de planta, saturación, perturbaciones
18. Scopes incluyendo estados internos de K

### Fase 6: Subsistema `Lazo_Hinf_phi`
19. Clonar estructura de Hinf_theta para phi
20. Agregar yaw damper

### Fase 7: Comparación top-level
21. Conectar outports de cada subsistema a scopes de comparación
22. Agregar 3 scopes comparativos

### Fase 8: Logging
23. Agregar `To Workspace` en puntos clave para post-procesamiento
24. Usar `SignalLogging` con nombres descriptivos

---

## 10. Cómo usar el modelo para distintos escenarios

### Escenario 1: Tracking nominal sin saturación
```matlab
theta_ref_amp = deg2rad(10); phi_ref_amp = deg2rad(10);
umax = deg2rad(60);  % Límite alto → no satura
noise_enabled = 0; dist_enabled = 0;
```

### Escenario 2: Saturación fuerte con referencia grande
```matlab
theta_ref_amp = deg2rad(40); phi_ref_amp = deg2rad(40);
umax = deg2rad(30);  % Límite nominal → satura
noise_enabled = 0; dist_enabled = 0;
```

### Escenario 3: Ruido + perturbación (robustez)
```matlab
theta_ref_amp = deg2rad(10); phi_ref_amp = deg2rad(10);
umax = deg2rad(30);
noise_enabled = 1; dist_enabled = 1;
```

### Escenario 4: Barrido de saturación (como validación extrema del .m)
```matlab
% Correr 3 veces con:
umax = deg2rad(30); % luego 45, luego 60
theta_ref_amp = deg2rad(30); phi_ref_amp = deg2rad(30);
```

### Escenario 5: Referencia sinusoidal (respuesta en frecuencia)
```matlab
% Cambiar Step por Signal Generator tipo seno en la referencia
% Ver tracking vs frecuencia — muestra el ancho de banda del controlador
```

---

## 11. Archivos a crear

| Archivo | Descripción |
|---|---|
| `build_taller1_simulink_v2.m` | Script que genera el .slx programáticamente |
| `init_simulink_v2.m` | Carga variables en workspace para simular |
| `taller1_simulink_v2.slx` | El modelo generado (no se edita a mano) |
| `PLAN_NUEVO_SIMULINK.md` | Este documento |

**No se modifica:** `taller1_simulink.slx`, `build_taller1_simulink.m`, `init_simulink.m`

---

## 12. Ventajas del nuevo diseño vs el actual

| Aspecto | Modelo actual | Modelo nuevo |
|---|---|---|
| Separación de lazos | Switch para alternar | 4 lazos independientes, simultáneos |
| Visualización | Solo To Workspace (offline) | 28 scopes en tiempo real |
| Claridad | Mux 8 + demux 5 + switch + mux 17 | Cada subsistema autocontenido |
| Escenarios | Cambiar `control_mode` y re-simular | Todos corren simultáneamente |
| Comparación | Post-proceso en .m | Scope comparativo directo |
| Perturbaciones | Solo dist_input | dist_input + dist_output + noise (slides) |
| Saturación | Fija en 30° | Variable por parámetro `umax` |
| Anti-windup | No implementado en Simulink | PID block con back-calculation |
| Tipos de referencia | Solo Step | Step / Ramp / Sine / Square configurables |

---

## 13. Diagrama de bloques conceptual (top-level)

```
┌─────────────────────────────────────────────────────────────────────┐
│                    taller1_simulink_v2.slx                           │
│                                                                     │
│  ┌─────────────────┐                                                │
│  │Generador_Señales│──refs, noise, dist────────────────────────┐    │
│  └─────────────────┘                                           │    │
│         │                │               │               │     │    │
│         ▼                ▼               ▼               ▼     │    │
│  ┌─────────────┐  ┌─────────────┐ ┌─────────────┐ ┌──────────┐│    │
│  │Lazo_PID_    │  │Lazo_PID_    │ │Lazo_Hinf_   │ │Lazo_Hinf_││    │
│  │theta        │  │phi          │ │theta        │ │phi       ││    │
│  │  [7 scopes] │  │  [6 scopes] │ │  [6 scopes] │ │[6 scopes]││    │
│  └──────┬──────┘  └──────┬──────┘ └──────┬──────┘ └─────┬────┘│    │
│         │                │               │               │     │    │
│         └────────────────┴───────────────┴───────────────┘     │    │
│                              │                                  │    │
│                              ▼                                  │    │
│                    ┌──────────────────┐                         │    │
│                    │ Scopes           │                         │    │
│                    │ Comparación (3)  │                         │    │
│                    └──────────────────┘                         │    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 14. Consideraciones técnicas

1. **Planta SISO vs MIMO**: Los lazos usan plantas SISO (`G_theta`, `G_phi`) para claridad. Para ver acoplamiento, se puede agregar un 5to subsistema opcional con la planta completa MIMO.

2. **Orden del controlador H∞**: K_theta_hinf y K_phi_hinf tienen ~5-7 estados cada uno. El State-Space block los maneja sin problema.

3. **Paso de integración**: ode45 con paso variable. Para el ruido muestreado usar `Sample Time = 0.005` en el Random Number.

4. **InitFcn callback**: El modelo ejecuta `init_simulink_v2.m` al abrirse, garantizando que las variables existan.

5. **Tamaño del modelo**: 4 subsistemas con ~15 bloques cada uno + generador + comparación ≈ 80-100 bloques totales. Manejable visualmente con subsistemas colapsados.
