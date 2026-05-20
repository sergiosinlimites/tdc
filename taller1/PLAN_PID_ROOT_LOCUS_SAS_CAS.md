# Plan para redisenar el PID/SAS-CAS con Root Locus y Bode

Este plan reemplaza el uso de ganancias baseline como diseno principal. Las ganancias anteriores quedan guardadas en:

```text
taller1/snapshots/20260513_185409_estado_actual/
```

La nueva meta es disenar el PID nosotros mismos, siguiendo la idea de clase:

```text
SAS -> amortiguamiento con velocidades angulares
CAS -> tracking con PI sobre angulos
```

## 1. Notas nuevas del profesor que guian el diseno

Apuntes de clase:

```text
Sistema de pitch es inverso.
Ver washout filter en el libro.
Pitch mantener en 30 grados.
Roll mantener en 30 grados.
Yaw debe ser 0; no debemos seguir referencia.
Con ese filtro queremos controlar la velocidad.
En posicion no queremos mucho seguir la posicion.
Debe haber cierta senal de control a media frecuencia.
```

Interpretacion:

- El canal pitch debe revisarse con mucho cuidado de signo.
- Pitch y roll son tracking de angulo.
- Yaw es amortiguamiento de velocidad, no tracking de `psi`.
- La posicion no debe dominar el controlador.
- El controlador no debe ser tan lento que no actue en media frecuencia.

## 2. Variables y canales

Canales a usar:

```text
Pitch:
  elevator -> theta
  elevator -> q

Roll:
  aileron -> phi
  aileron -> p

Yaw:
  rudder -> r
  rudder -> psi solo para verificar deriva, no para tracking fuerte
```

En MATLAB:

```matlab
G_theta = channels.theta;
G_q     = channels.q;
G_phi   = channels.phi;
G_p     = channels.p;
G_r     = channels.r;
```

## 3. Paso cero: congelar lo anterior

Ya se creo:

```text
taller1/snapshots/20260513_185409_estado_actual/
```

Ese snapshot contiene:

- `results/taller1_results.mat`;
- figuras anteriores;
- scripts `.m`;
- documentos `.md`;
- modelo `taller1_uav.slx`.

Si un nuevo PID empeora todo, se puede comparar contra ese estado.

## 4. Paso 1: analizar signos

Antes de Root Locus:

```matlab
dcgain(G_theta)
dcgain(G_q)
dcgain(G_phi)
dcgain(G_p)
dcgain(G_r)
step(G_theta)
step(G_q)
step(G_phi)
step(G_p)
step(G_r)
```

Preguntas:

- Si elevator aumenta, `theta` aumenta o disminuye?
- Si elevator aumenta, `q` aumenta o disminuye?
- Si aileron aumenta, `phi` aumenta o disminuye?
- Si rudder aumenta, `r` aumenta o disminuye?

El profesor dijo que pitch es inverso. Eso debe reflejarse en los signos del controlador.

## 5. Paso 2: disenar SAS de pitch

El SAS de pitch usa velocidad angular:

```math
\delta_e = \alpha - D_q q
```

La planta para mirar Root Locus es:

```matlab
G_q = elevator -> q
```

Ruta:

```matlab
rlocus(G_q)
sisotool(G_q)
margin(G_q*Kq)
```

Meta:

- mover polos dominantes hacia mayor amortiguamiento;
- no hacer el lazo demasiado rapido;
- no pedir demasiado elevator.

Validar:

```matlab
T_q = feedback(G_q*Kq, 1)
damp(T_q)
step(T_q)
```

## 6. Paso 3: disenar CAS de pitch

Una vez amortiguado pitch con SAS, se mira el canal externo:

```math
\theta_{ref} -> \theta
```

CAS:

```math
P_\theta(s) = K_{p,\theta} + \frac{K_{i,\theta}}{s}
```

Ruta:

```matlab
rlocus(G_theta_amortiguada)
bode(G_theta_amortiguada)
margin(Ptheta*G_theta_amortiguada)
```

Meta:

- seguir `theta_ref = 30 deg`;
- error estacionario pequeno;
- sobreimpulso bajo;
- control menor que `+-30 deg`;
- margen de fase razonable.

## 7. Paso 4: disenar SAS de roll

El SAS de roll usa `p`:

```math
\delta_a = \beta - D_p p
```

Ruta:

```matlab
rlocus(G_p)
sisotool(G_p)
margin(G_p*Kp_damper)
```

Meta:

- amortiguar roll;
- reducir oscilacion lateral;
- no excitar yaw de forma fuerte.

## 8. Paso 5: disenar CAS de roll

CAS:

```math
P_\phi(s) = K_{p,\phi} + \frac{K_{i,\phi}}{s}
```

Meta:

- seguir `phi_ref = 30 deg`;
- error estacionario pequeno;
- saturacion baja;
- acoplamiento en `theta` pequeno.

## 9. Paso 6: disenar yaw damper con washout

El profesor indico que yaw no debe seguir referencia. El objetivo es amortiguar velocidad.

Washout:

```math
W_{wo}(s) = \frac{s}{s + a}
```

Control:

```math
u_r = K_r W_{wo}(s) r
```

Interpretacion:

- Si `r` tiene una componente transitoria, el filtro la deja pasar y el rudder actua.
- Si hay una componente constante/lenta, el filtro la atenual; asi no se persigue posicion yaw como si fuera referencia.

Ruta:

```matlab
s = tf('s');
Wwo = s/(s + a);
K_yaw = Kr*Wwo;
rlocus(G_r*K_yaw)
margin(G_r*K_yaw)
```

Parametros a probar:

```matlab
a = 0.5, 1, 2, 4
Kr = barrido con signo correcto
```

La version anterior tenia:

```matlab
K_YD(s) = 0.065 s/(s + 2)
```

Eso queda como referencia, no como obligacion.

## 10. Paso 7: validar el PID completo

Con SAS y CAS listos:

```matlab
theta_ref = 30 deg
phi_ref   = 30 deg
yaw_ref   = 0
```

Simular:

- solo pitch;
- solo roll;
- pitch y roll simultaneos;
- yaw perturbado;
- ruido;
- perturbacion de entrada;
- caso de estres a 40 deg opcional.

Metricas:

- error RMS;
- error final;
- sobreimpulso;
- tiempo de establecimiento;
- maximo control;
- porcentaje de saturacion;
- acoplamiento `theta <-> phi`;
- comportamiento de `r` y `psi`.

## 11. Paso 8: comparar contra H_inf

Cuando el PID propio este listo, comparar:

```text
PID propio vs H_inf actual
PID propio vs PID baseline guardado
```

Graficas:

- `S`;
- `T`;
- `K*S`;
- respuesta temporal;
- control;
- ruido y perturbacion.

Lo importante no es que el PID propio gane todo, sino que sea un diseno completo y defendible.

## 12. Entregables nuevos

Archivos esperados despues de implementar esta ruta:

```text
diseno_pid_root_locus_sas_cas.m
analisis_pid_root_locus.m
figures/pid_root_locus_pitch.png
figures/pid_root_locus_roll.png
figures/pid_bode_pitch.png
figures/pid_bode_roll.png
figures/yaw_washout_bode.png
```

Tambien se debe actualizar:

```text
reporte_taller1.md
README.md
simulacion_taller1.m si cambian escenarios o controladores
```

## 13. Criterio de aceptacion del nuevo PID

El nuevo PID se acepta si:

- fue disenado con Root Locus/Bode y no copiado del baseline;
- pitch sigue `30 deg` con error razonable;
- roll sigue `30 deg` con error razonable;
- yaw no persigue referencia, pero amortigua `r`;
- no satura de forma permanente;
- tiene signos justificados por la planta inversa;
- funciona sobre `linmodel` acoplado.

## 14. Preguntas abiertas para implementar

Estas preguntas se resolveran con MATLAB:

- Que signo exacto necesita el damper de pitch?
- Que ganancia SAS da mejor amortiguamiento sin saturar?
- Conviene disenar CAS despues de cerrar SAS o usar una aproximacion en cascada?
- Que frecuencia de washout es mejor para yaw?
- El H_inf actual debe redisenarse sobre la planta con SAS cerrado o mantenerse como comparacion independiente?
