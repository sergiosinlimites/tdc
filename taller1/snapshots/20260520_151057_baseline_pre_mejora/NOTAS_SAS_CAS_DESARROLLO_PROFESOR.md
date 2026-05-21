# Notas sobre el desarrollo SAS/CAS de las fotos

Este documento interpreta las fotos enviadas de la clase. La conclusion corta es:

```text
El desarrollo sirve para entender el PID/SAS-CAS.
No reemplaza directamente el diseno H_inf, pero ayuda a justificar el PID base
y puede inspirar una alternativa H_inf de bajo orden con estructura fija.
```

## 1. Idea fisica del avion

En las notas aparece el avion con fuerzas de lift y drag y tres pares entrada-salida:

```text
elevator -> pitch theta
rudder   -> yaw psi
ailerons -> roll phi
```

Esto coincide con el taller:

- `theta/elevator` se usa para el eje longitudinal.
- `phi/aileron` se usa para el eje lateral.
- `r/rudder` se deja como yaw damper, no como tracking principal.

Tambien aparece:

```text
SAS -> amortiguamiento
CAS -> tracking
PI + D
```

Esa frase es la clave. El profesor esta separando el PID en dos tareas:

- `PI`: seguimiento de referencia, parte CAS.
- `D`: amortiguamiento usando velocidad angular, parte SAS.

## 2. Planta usada en el desarrollo

En la foto se escribe algo equivalente a:

```math
\begin{bmatrix}
\theta \\
\dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
g_1(s) \\
g_2(s)
\end{bmatrix}
\delta
```

Donde:

- `delta` es la deflexion del elevator;
- `theta` es el angulo de pitch;
- `dot(theta)` es la velocidad angular de pitch, aproximadamente `q`;
- `g1(s)` es la transferencia de elevator a pitch;
- `g2(s)` es la transferencia de elevator a pitch rate.

Como:

```math
\theta = \frac{1}{s}\dot{\theta}
```

entonces tambien se puede escribir:

```math
\theta = \frac{1}{s} g_2(s)\delta
```

Eso explica por que aparece el factor `1/s` en la primera foto.

## 3. Que hace el SAS

El SAS realimenta velocidad angular:

```math
\delta = \alpha - D\dot{\theta}
```

Aqui:

- `alpha` es el comando que viene del CAS;
- `D` es el damper;
- `dot(theta)` es la velocidad angular medida.

Sustituyendo en la planta de velocidad:

```math
\dot{\theta} = g_2(s)\delta
```

queda:

```math
\dot{\theta}
=
g_2(s)\left(\alpha - D\dot{\theta}\right)
```

Pasando terminos:

```math
\dot{\theta} + g_2(s)D\dot{\theta}
=
g_2(s)\alpha
```

```math
\left(1 + g_2(s)D\right)\dot{\theta}
=
g_2(s)\alpha
```

Por tanto:

```math
\frac{\dot{\theta}}{\alpha}
=
\frac{g_2(s)}{1 + g_2(s)D}
```

Y como:

```math
\theta = \frac{1}{s}\dot{\theta}
```

entonces:

```math
\frac{\theta}{\alpha}
=
\frac{g_2(s)}{s\left(1 + g_2(s)D\right)}
```

Eso es lo que se ve en la primera foto.

## 4. Por que dice `D negativo`

El signo de `D` depende de la convencion del modelo.

Si el canal elevator -> pitch rate tiene ganancia negativa, entonces el damper que fisicamente agrega amortiguamiento tambien puede aparecer con ganancia negativa en MATLAB.

En nuestro modelo ya se vio algo parecido:

```text
dcgain theta/elevator < 0
dcgain phi/aileron    < 0
```

Por eso las ganancias baseline del paquete UAV son negativas.

Lo importante no es que `D` sea positivo o negativo por costumbre, sino que el producto de signos cierre la realimentacion en sentido amortiguante.

## 5. Donde entra el CAS

Despues de amortiguar la planta con SAS, se agrega el CAS externo.

El CAS toma error de referencia:

```math
e_\theta = \theta_{ref} - \theta
```

y produce el comando `alpha`:

```math
\alpha = P(s)e_\theta
```

Si el CAS es PI:

```math
P(s) = K_p + \frac{K_i}{s}
```

Entonces el lazo externo busca que `theta` siga `theta_ref`.

## 6. Relacion con un PID normal

La ley completa puede verse como:

```math
\delta =
\left(K_p + \frac{K_i}{s}\right)e_\theta
- D\dot{\theta}
```

Si la referencia es constante o cambia lento:

```math
\dot{e}_\theta =
\dot{\theta}_{ref} - \dot{\theta}
\approx
-\dot{\theta}
```

Entonces:

```math
-D\dot{\theta}
```

cumple el papel de una accion derivativa sobre el error, pero sin derivar la referencia. Por eso se puede decir:

```text
SAS/CAS = PI de tracking + D de amortiguamiento
```

Eso es casi un PID, pero implementado de forma mas fisica:

- el PI mira angulo;
- el D mira velocidad angular medida.

## 7. Como conecta con nuestro codigo

En `diseno_pid_sas_cas.m` se usa:

```matlab
K_theta = kp_theta + ki_theta/s + kd_theta*s/(tau*s + 1)
K_phi   = kp_phi   + ki_phi/s   + kd_phi*s/(tau*s + 1)
```

Eso es una forma compacta de representar PI + derivada filtrada.

En la simulacion temporal, se implementa mas parecido al diagrama del profesor:

```matlab
u_elev_raw = kp_theta*e_theta + ki_theta*xi_theta ...
             - kd_theta*q_meas;

u_ail_raw = kp_phi*e_phi + ki_phi*xi_phi ...
            - kd_phi*p_meas;
```

Aqui:

- `kp`, `ki` son CAS;
- `kd*q_meas` y `kd*p_meas` son SAS;
- `q` es velocidad de pitch;
- `p` es velocidad de roll.

## 8. Entonces, para que sirve en el taller

Sirve para tres cosas:

1. Justificar por que el PID base no es un PID cualquiera, sino una arquitectura SAS/CAS.
2. Explicar por que usamos `theta/elevator` y `phi/aileron`.
3. Proponer una alternativa de bajo orden: disenar una estructura PI+D fija y tunearla con criterios robustos.

No reemplaza directamente:

```math
\min_K
\left\|
\begin{bmatrix}
W_1S \\
W_2KS \\
W_3T
\end{bmatrix}
\right\|_\infty
```

Pero si puede convertirse en una estructura candidata para `hinfstruct` o `systune`.

## 9. Posible extension robusta basada en las fotos

Una alternativa seria imponer:

```math
K(s) =
K_p + \frac{K_i}{s}
+ \frac{K_d s}{\tau s + 1}
```

y ajustar `Kp`, `Ki`, `Kd`, `tau` para minimizar una funcion robusta.

Ventaja:

- orden bajo;
- muy interpretable;
- conecta con SAS/CAS del profesor.

Desventaja:

- ya no es el H_inf libre de `mixsyn`;
- requiere una formulacion adicional con bloques ajustables.

## 10. Frase para el reporte

Una forma correcta de escribirlo:

```text
El desarrollo SAS/CAS de clase muestra que la accion derivativa del PID puede
entenderse fisicamente como realimentacion de velocidad angular para aumentar
amortiguamiento. Por eso el controlador clasico del taller se implementa como
PI de angulo mas damper de tasa. Esta arquitectura se usa como referencia
frente al controlador H_inf de sensibilidad mixta.
```
