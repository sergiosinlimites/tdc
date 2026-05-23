# Simulink — taller1_simulink_v2

## Archivos

| Archivo | Qué hace |
|---|---|
| `taller1_simulink_v2.slx` | Modelo con 4 lazos independientes: PID θ, PID φ, H∞ θ, H∞ φ |
| `init_simulink_v2.m` | Carga planta y sintetiza controladores H∞ → **correr una vez por sesión** |
| `configure_scopes_v2.m` | Leyendas y títulos en scopes → ya aplicado, no volver a correr |
| `compute_metrics_v2.m` | Calcula Ts, sobrepaso, RMS, saturación → correr después de cada sim |
| `build_taller1_simulink_v2.m` | Regenera el .slx desde cero → solo si se daña el modelo |

## Uso

**1. Al abrir MATLAB (una vez por sesión):**
```matlab
cd taller1_final/simulink
run('init_simulink_v2.m')
```

**2. Para cada prueba — cambiar variables y simular:**
```matlab
theta_ref_amp = deg2rad(20);   % referencia theta
phi_ref_amp   = deg2rad(10);   % referencia phi
umax          = deg2rad(30);   % límite actuador (30 / 45 / 60)
noise_enabled = 0;             % 1 = con ruido
dist_enabled  = 0;             % 1 = con perturbación a 6 Hz

sim('taller1_simulink_v2')
compute_metrics_v2             % imprime Ts, OS, RMS, saturación
```

## Variables configurables

| Variable | Descripción | Valor típico |
|---|---|---|
| `theta_ref_amp` | Referencia pitch | `deg2rad(10)` |
| `phi_ref_amp` | Referencia roll | `deg2rad(10)` |
| `umax` | Límite saturación actuadores | `deg2rad(30)` |
| `noise_enabled` | Ruido de medición on/off | `0` / `1` |
| `dist_enabled` | Perturbación sinusoidal on/off | `0` / `1` |
| `dist_amp` | Amplitud perturbación | `deg2rad(1.0)` |

## Scopes

Dentro de cada subsistema (`Lazo_PID_theta`, etc.):

| Scope | Qué muestra |
|---|---|
| `Scope_ref_vs_y` | Referencia vs salida — seguimiento |
| `Scope_control` | Control crudo vs saturado — ver saturación |
| `Scope_PI_vs_D` | Contribución PI vs D (solo PID) |
| `Scope_error` | Error de seguimiento |
| `Scope_perturbaciones` | Ruido y perturbaciones activas |

Top-level: `Comparar_theta`, `Comparar_phi`, `Comparar_control` — PID vs H∞ en la misma gráfica.
