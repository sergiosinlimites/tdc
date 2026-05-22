# Plan de consolidación: taller1/ → taller1_final/

## 1. Contexto y diagnóstico del estado actual

### 1.1 ¿Qué existe en `taller1/`?

El directorio tiene **19 archivos `.m`**, **9 archivos `.md`**, **1 `.mlx`**, **1 `.slx`**,
**23 figuras PNG**, **4 archivos `.mat`** de resultados, **3 snapshots** y **6 reportes de resultados**.
Todo esto se acumuló a lo largo de varias iteraciones de diseño y debugging.

**Archivos `.m` actuales y su rol:**

| Archivo | Rol | ¿Se conserva? |
|---|---|:---:|
| `main_taller1.m` | Orquestador principal | Se integra |
| `parametros_taller1.m` | Configuración centralizada | Se integra |
| `cargar_modelo_uav.m` | Carga `modelo_lin.mat` | Se integra |
| `seleccionar_canales_uav.m` | Extrae canales SISO y MIMO | Se integra |
| `analisis_planta_uav.m` | Polos, ceros, DC gain, controlabilidad | Se integra |
| `diseno_sas_root_locus.m` | Diseño SAS con root locus | Se integra |
| `diseno_cas_pi_root_locus.m` | Diseño CAS PI sobre planta SAS | Se integra |
| `diseno_pid_sas_cas.m` | Construye K_theta, K_phi tipo PI+D | Se integra |
| `diseno_hinf_taller1.m` | Síntesis H∞ con mixsyn | Se integra |
| `construir_planta_generalizada_hinf.m` | Wrapper de augw (documentación) | Se integra inline |
| `analisis_sensibilidades.m` | Calcula S, T, KS para ambos | Se integra |
| `simulacion_taller1.m` | Simulación ODE45 sobre linmodel | Se integra |
| `evaluar_saturacion_controlador.m` | Métricas de saturación/error | Se integra |
| `crear_graficas_taller1.m` | Genera todas las figuras | Se integra |
| `build_taller1_simulink.m` | Genera el .slx programáticamente | Se adapta |
| `init_taller1_simulink.m` | Variables para Simulink | Se adapta |
| `optimizar_pesos_hinf.m` | Barrido de pesos (exploración) | NO se incluye |
| `barrido_suave_hinf_siso_phi.m` | Mini-ronda de exploración | NO se incluye |
| `validacion_extrema_taller1.m` | Pruebas de estrés | Parcial (resultados) |

### 1.2 Estado del diseño H∞

- **Theta (longitudinal):** Funciona bien. gamma=3.66, KS=4.30, tracking aceptable.
- **Phi (lateral):** Mejorado tras iteraciones de pesos específicos. gamma=3.78, KS=3.08.
  - Error final phi_30: 1.28 deg (OK, < 1.5)
  - Error final phi_40: 1.94 deg (OK, < 2.0)
  - `phi_sobrepasa` en `noise_dist_x3_hard`: 4.51 deg (flag residual en pruebas extremas)
  - **NO satura de forma persistente** en escenarios nominales (< 1%)
- **Conclusión:** H∞ SISO es presentable para el taller, con la nota honesta de que phi tiene un flag residual en pruebas extremas con ruido 3x.

### 1.3 Pesos finales a usar

```matlab
% Theta
W1_theta = makeweight(80, 2*pi*8, 0.05);
W2_theta = 1.0;  % constante
W3_theta = makeweight(0.005, 2*pi*6, 15);

% Phi (iteración 1 aceptada)
W1_phi = makeweight(220, 2*pi*8, 0.05);
W2_phi = 0.80*(s/(2*pi*6) + 1) / (s/(2*pi*6 * 3.20/0.80) + 1);
W3_phi = makeweight(0.005, 2*pi*6, 15);
```

---

## 2. Entregables finales en `taller1_final/`

```text
taller1_final/
├── taller1_completo.m          ← Código unificado, ejecutable de principio a fin
├── taller1_presentacion.mlx    ← Live Script explicativo tipo notebook
├── taller1_simulink.slx        ← Modelo Simulink de acompañamiento
├── init_simulink.m             ← Inicialización de variables para el .slx
├── figures/                    ← Todas las figuras generadas
│   ├── planta_sigma.png
│   ├── root_locus_sas_q.png
│   ├── root_locus_sas_p.png
│   ├── root_locus_sas_r.png
│   ├── root_locus_cas_theta.png
│   ├── root_locus_cas_phi.png
│   ├── sensibilidades_theta.png
│   ├── sensibilidades_phi.png
│   ├── sim_theta_10.png
│   ├── sim_phi_10.png
│   ├── sim_theta_phi_10.png
│   ├── sim_theta_30.png
│   ├── sim_phi_30.png
│   ├── sim_theta_phi_30.png
│   ├── sim_theta_40.png
│   ├── sim_phi_40.png
│   ├── sim_noise_disturbance.png
│   ├── comparacion_temporal_final.png
│   ├── comparacion_saturacion_final.png
│   └── comparacion_sensibilidades.png
└── PLAN_TALLER1_FINAL.md       ← Este documento
```

---

## 3. Estructura del archivo `taller1_completo.m`

El archivo unifica todo el flujo en **secciones secuenciales** con `%%`, sin funciones externas.
Cada sección corresponde a un paso lógico del taller.

### Secciones propuestas:

```text
%% 0. Limpieza y configuración
     - clear, clc, close all
     - Todos los parámetros inline (no cfg struct externo)
     - Especificaciones del enunciado como variables simples

%% 1. Carga del modelo UAV
     - load modelo_lin.mat
     - ss() de linmodel, latmod, longmod
     - Nombres de I/O

%% 2. Extracción de canales de diseño
     - theta/elevator, q/elevator del longmod
     - phi/aileron, p/aileron, r/rudder del latmod
     - MIMO theta-phi (diagnóstico)

%% 3. Análisis de la planta
     - Polos, ceros, ganancia DC
     - Controlabilidad y observabilidad
     - Gráfica de valores singulares (sigma)

%% 4. Diseño SAS por root locus
     - Root locus de q/elevator → selección de D_q = -0.20
     - Root locus de p/aileron → selección de D_p = 0.05
     - Yaw damper con washout
     - Gráficas de root locus con polos seleccionados

%% 5. Diseño CAS PI sobre planta amortiguada
     - Planta externa: G_theta/(1 + D_q*G_q), G_phi/(1 + D_p*G_p)
     - PI de theta: Kp=-1.00, Ki=-0.30
     - PI de phi: Kp=-0.35, Ki=-0.18
     - Root locus y step del CAS
     - Verificación de estabilidad y márgenes

%% 6. Controlador PID/SAS-CAS equivalente
     - K_theta = Kp + Ki/s + Kd*s/(tau*s+1) con derivativo filtrado
     - K_phi análogo
     - Cierre de lazo nominal SISO

%% 7. Diseño H∞ por sensibilidad mixta
     7.1 Construcción de pesos W1, W2, W3
         - Explicación de cada peso y su relación con las especificaciones
         - Pesos separados para theta y phi
     7.2 Planta generalizada (augw)
         - Mostrar la estructura P = [W1, W1*G; 0, W2; -I, -G]
     7.3 Síntesis con mixsyn
         - K_theta_hinf, gamma_theta
         - K_phi_hinf, gamma_phi
     7.4 Verificación: controlador estable, lazo cerrado estable

%% 8. Análisis de sensibilidades S, T, KS
     - Cálculo para SAS/CAS y H∞ en ambos ejes
     - Normas H∞ aproximadas
     - Tabla comparativa
     - Gráficas: S, T, KS por eje con cotas 1/W1, 1/W2, 1/W3
     - Gráfica comparativa 3x2

%% 9. Simulación temporal sobre linmodel acoplado
     9.1 Preparación: controladores en espacio de estados
     9.2 Definición de escenarios
     9.3 Generación de ruido y perturbación
     9.4 Simulación con ode45 (planta + controlador)
         - Lazo cerrado: saturación + anti-windup
         - Vector de 8 entradas
     9.5 Reconstrucción de señales y métricas
     9.6 Resumen en consola

%% 10. Generación de figuras
     - Sigma de plantas
     - Root locus SAS y CAS
     - Sensibilidades por eje
     - Comparación de sensibilidades
     - Simulaciones temporales seleccionadas
     - Barras comparativas RMS y saturación

%% 11. Resumen y conclusiones (impreso en consola)
```

### Funciones auxiliares al final del archivo

El archivo tendrá funciones locales (al final) únicamente para lógica repetitiva que
no tiene sentido escribir inline varias veces:

```text
function dz = closed_loop_ode(...)       % ODE del lazo cerrado
function sim = reconstruct_simulation(...)  % Reconstrucción post-ODE
function n = sample_noise(...)           % Interpolación de ruido
function signals = measured_signals(...) % Referencias + mediciones
function [u_sat, u_raw, xcdot] = controller_output(...)  % Ley de control
function u = full_input_vector(...)      % 8 entradas del UAV
function d = input_disturbance(...)      % Perturbación sinusoidal
function metrics = compute_metrics(...)  % Error RMS, saturación, etc.
function export_png(fig, filename)       % Exportación con fondo blanco
function plot_sigma_response(...)        % Sigma numérico en dB
function style_axes()                    % Estilo visual uniforme
```

---

## 4. Estructura del archivo `taller1_presentacion.mlx`

El Live Script sigue la misma secuencia pero con **texto enriquecido, ecuaciones LaTeX
y figuras inline**. No replica todo el código de simulación (que es largo), sino que
lo referencia y muestra resultados.

### Estructura de secciones:

```text
1. Portada y objetivo
   - Título, autores, fecha
   - Objetivo del taller (textual)

2. Especificaciones de diseño
   - Tabla de especificaciones del enunciado
   - Ecuaciones de S, T, KS con LaTeX
   - Relación especificación → función de transferencia → peso

3. Modelo del UAV
   - Descripción de linmodel, latmod, longmod
   - Tabla de entradas/salidas
   - Figura: valores singulares de la planta

4. Diseño SAS (Stability Augmentation System)
   - Explicación de la arquitectura SAS
   - Ecuaciones de lazo interno
   - Figuras: root locus q, p, r
   - Tabla de ganancias seleccionadas

5. Diseño CAS (Control Augmentation System)  
   - Explicación de PI sobre planta amortiguada
   - Ecuaciones del controlador
   - Figuras: root locus CAS theta, phi
   - Tabla de ganancias, márgenes

6. Controlador PI+D equivalente
   - Ecuación unificada
   - Anti-windup
   - Yaw damper

7. Diseño H∞ por sensibilidad mixta
   7.1 Marco teórico: planta generalizada
       - Diagrama de bloques
       - Ecuaciones de P
       - Significado de gamma
   7.2 Pesos W1, W2, W3
       - Justificación física de cada peso
       - Figuras: Bode de pesos e inversas
   7.3 Síntesis
       - Resultados: gamma, orden de K
       - Tabla comparativa theta vs phi

8. Análisis de sensibilidades
   - Figuras: S, T, KS por eje con cotas
   - Tabla de normas H∞
   - Comparación PID vs H∞
   - Figura: comparación 3x2

9. Simulación y resultados
   - Descripción del esquema de simulación
   - Figuras seleccionadas: theta_10, phi_10, theta_phi_10, phi_30, phi_40, noise
   - Tabla de métricas RMS y saturación
   - Figuras de barras comparativas

10. Discusión y conclusiones
    - Fortalezas de SAS/CAS: simple, poco esfuerzo, llega bien a phi
    - Fortalezas de H∞: mejor tracking RMS, robustez frecuencial
    - Limitación de H∞ SISO en phi: KS mayor, flag residual en pruebas extremas
    - Posibles extensiones: H∞ MIMO, integración explícita

11. Referencias
    - Enunciado del taller
    - Notas de clase Sofrony_c.pdf
    - Documentación MATLAB
```

---

## 5. Modelo Simulink `taller1_simulink.slx`

### 5.1 Arquitectura del modelo

El Simulink se genera programáticamente con `build_taller1_simulink.m` adaptado.
Contiene:

```text
Entradas:
  - Step theta_ref, Step phi_ref

Medición ruidosa:
  - theta + noise → theta_meas
  - phi + noise → phi_meas

Error:
  - e_theta = theta_ref - theta_meas
  - e_phi = phi_ref - phi_meas

Control SAS/CAS:
  - CAS_PI_theta(e_theta) - SAS_D_q * q → elevator_raw
  - CAS_PI_phi(e_phi) - SAS_D_p * p → aileron_raw

Control H∞:
  - Hinf_theta(e_theta) → elevator_hinf
  - Hinf_phi(e_phi) → aileron_hinf

Selector (Switch):
  - control_mode: 0 = SAS/CAS, 1 = H∞

Saturación:
  - sat_elevator, sat_aileron, sat_rudder a ±30 deg

Perturbación:
  - Senos a 6 Hz sumados después de saturación

Planta:
  - State-Space linmodel (14 estados, 8 entradas, 14 salidas)
  - Demux → theta, phi, p, q, r

Yaw damper:
  - Washout 0.065*s/(s+2) sobre r → rudder

Logging:
  - simout_y: [theta, theta_ref, phi, phi_ref]
  - simout_u: [elevator, aileron, rudder]
```

### 5.2 Inicialización

`init_simulink.m` carga las variables necesarias desde el flujo de
`taller1_completo.m` o las calcula directamente.

---

## 6. Plan de ejecución (orden de trabajo)

### Fase 1: Archivo `.m` unificado
1. Crear `taller1_completo.m` con todas las secciones `%%`
2. Integrar inline: parámetros, carga, canales, análisis
3. Integrar diseño SAS root locus con generación de figuras
4. Integrar diseño CAS PI con figuras
5. Integrar diseño PI+D equivalente
6. Integrar diseño H∞: pesos, mixsyn, verificación
7. Integrar análisis de sensibilidades con figuras
8. Integrar simulación: ODE45, reconstrucción, métricas
9. Integrar generación de figuras temporales y barras
10. Integrar resumen final en consola
11. **Verificar** que corre sin errores (depende de MATLAB externo)

### Fase 2: Modelo Simulink
12. Adaptar `build_taller1_simulink.m` → genera `taller1_simulink.slx`
13. Crear `init_simulink.m` simplificado
14. Verificar que el .slx se construye correctamente

### Fase 3: Live Script `.mlx`
15. Crear `taller1_presentacion.mlx` con estructura de secciones
16. Agregar texto explicativo con ecuaciones LaTeX
17. Agregar código que genera figuras inline
18. Agregar tablas de resultados
19. Agregar conclusiones

### Fase 4: Figuras
20. Las figuras se generan al correr `taller1_completo.m`
21. Se copian también al directorio `figures/` para referencia independiente

---

## 7. Decisiones de diseño

### 7.1 ¿Qué NO se incluye?
- `optimizar_pesos_hinf.m` → fue exploración, no diseño final
- `barrido_suave_hinf_siso_phi.m` → fue exploración
- `validacion_extrema_taller1.m` → los resultados se mencionan en conclusiones
- Todos los READMEs, bitácoras y reportes intermedios
- Todos los snapshots
- Archivos `.mat` de resultados intermedios

### 7.2 ¿Qué se simplifica?
- La estructura `cfg` se reemplaza por variables simples al inicio del script
- Las funciones modulares se convierten en código inline donde sea posible
- Las funciones que se repiten (ODE, reconstruct, etc.) quedan como funciones
  locales al final del archivo

### 7.3 Convenciones
- Todas las figuras con fondo blanco, rejilla, colores distinguibles
- SAS/CAS en azul sólido, H∞ en rojo discontinuo, referencias en negro punteado
- Unidades: radianes internamente, grados en gráficas y reportes
- Frecuencias: rad/s internamente, Hz cuando se menciona el enunciado

---

## 8. Resumen de números clave a presentar

### Ganancias SAS/CAS

| Lazo | Ganancia | Valor |
|---|---|---:|
| SAS pitch | D_q | -0.20 |
| SAS roll | D_p | 0.05 |
| CAS pitch | Kp_theta | -1.00 |
| CAS pitch | Ki_theta | -0.30 |
| CAS roll | Kp_phi | -0.35 |
| CAS roll | Ki_phi | -0.18 |

### Resultados H∞

| Eje | gamma | Orden K |
|---|---:|---:|
| theta | 3.660 | ~5-7 |
| phi | 3.778 | ~5-7 |

### Sensibilidades comparadas

| Lazo | ‖S‖ | ‖T‖ | ‖KS‖ |
|---|---:|---:|---:|
| theta SAS/CAS | 1.234 | 1.000 | 8.138 |
| theta H∞ | 1.222 | 0.954 | 4.303 |
| phi SAS/CAS | 2.050 | 1.225 | 1.317 |
| phi H∞ | 1.333 | 1.010 | 3.080 |

### Estado de phi H∞

- Error final phi_30: **1.28 deg** (cumple < 1.5)
- Error final phi_40: **1.94 deg** (cumple < 2.0)
- Saturación nominal: **< 1%**
- Flag residual: `phi_sobrepasa` solo en pruebas extremas con ruido 3x
- **No satura de forma persistente** en ningún escenario nominal

---

## 9. Notas para la presentación

1. El diseño SAS/CAS se presenta primero como referencia clásica conocida.
2. H∞ se presenta como alternativa robusta que busca cumplir las mismas
   especificaciones de forma sistemática.
3. La comparación NO es "uno es mejor que otro" sino compromiso:
   - SAS/CAS: simple, poco aileron, tracking directo
   - H∞: mejor RMS global, T baja en alta frecuencia (rechazo de ruido),
     pero KS lateral mayor
4. Las figuras clave son:
   - Sensibilidades por eje (con cotas de peso)
   - Simulaciones phi_10, phi_30, phi_40 (donde se ve la diferencia)
   - Barras de RMS y saturación
5. Las conclusiones deben ser honestas sobre las limitaciones del H∞ SISO.
