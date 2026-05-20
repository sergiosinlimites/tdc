# Plan para bajar el orden de los controladores H_inf

Este plan responde a la inquietud: un companero obtuvo controladores `K` de orden 4, mientras que en esta version salieron:

```text
K_theta: orden 7
K_phi:   orden 6
```

## 1. Por que nuestros controladores salieron de orden 7 y 6

En sensibilidad mixta con `mixsyn`, el orden del controlador suele crecer aproximadamente con:

```text
orden de la planta nominal + orden de los pesos dinamicos
```

En este taller:

- canal `theta/elevator`: planta SISO de orden 5;
- canal `phi/aileron`: planta SISO de orden 4;
- `W1`: peso dinamico de primer orden;
- `W3`: peso dinamico de primer orden;
- `W2`: peso constante, no agrega estados.

Por eso se obtiene de forma natural:

```text
theta: 5 + 1 + 1 = 7
phi:   4 + 1 + 1 = 6
```

Entonces, para llegar a orden 4 hay que reducir algo:

- la planta usada para sintesis;
- los pesos;
- el controlador despues de sintetizar;
- o imponer una estructura fija de bajo orden.

## 2. Criterios para aceptar una reduccion

No basta con que el orden sea menor. Cada alternativa debe pasar estas pruebas:

1. `K` estable o al menos lazo cerrado estable.
2. `S`, `T` y `K*S` no deben empeorar drasticamente.
3. `gamma` no debe subir demasiado.
4. La simulacion temporal debe conservar seguimiento razonable.
5. La saturacion no debe aumentar.
6. El controlador reducido debe funcionar sobre `linmodel`, no solo sobre el canal SISO.

Una regla practica inicial:

```text
Aceptar reduccion si:
- el orden baja,
- el lazo sigue estable,
- gamma sube menos de 10% a 20%,
- las graficas S/T/KS conservan la forma,
- las metricas temporales no empeoran de forma fuerte.
```

## 3. Alternativa A: reducir el controlador despues de `mixsyn`

Esta es la ruta mas directa.

Procedimiento:

1. Sintetizar `K` como ya se hace.
2. Aplicar reduccion de orden a `K`.
3. Probar orden 6, 5, 4, 3.
4. Comparar sensibilidad y simulacion.

Idea MATLAB:

```matlab
K = hinf_data.theta.K;
K4 = balred(K, 4);
```

Luego:

```matlab
S4  = feedback(1, channels.theta*K4);
T4  = feedback(channels.theta*K4, 1);
KS4 = K4*S4;
```

Ventaja:

- conserva el diseno H_inf original como punto de partida.

Riesgo:

- al reducir despues, MATLAB no garantiza que el controlador reducido conserve la misma norma H_inf.

Esta debe ser la primera prueba porque es rapida.

## 4. Alternativa B: reducir la planta antes de `mixsyn`

Si el controlador hereda orden de la planta, una forma limpia de bajar orden es sintetizar sobre una planta reducida.

Procedimiento:

1. Reducir `channels.theta` de orden 5 a orden 3 o 4.
2. Reducir `channels.phi` de orden 4 a orden 3 o 4.
3. Sintetizar H_inf con esas plantas reducidas.
4. Validar el controlador sobre la planta original.

Idea MATLAB:

```matlab
Gtheta_r = balred(channels.theta, 3);
Gphi_r   = balred(channels.phi, 3);

[Ktheta_r, CLtheta_r, gamma_theta_r] = mixsyn(Gtheta_r, W1, W2, W3);
[Kphi_r, CLphi_r, gamma_phi_r] = mixsyn(Gphi_r, W1, W2, W3);
```

Ventaja:

- el controlador nace de menor orden.

Riesgo:

- si la reduccion borra un modo importante, el controlador puede funcionar bien en la planta reducida pero mal en `linmodel`.

Esta alternativa es fuerte si el companero obtuvo orden 4 usando un modelo mas simple.

## 5. Alternativa C: eliminar o simplificar un peso dinamico

Nuestros pesos dinamicos `W1` y `W3` agregan dos estados. Si uno se vuelve constante o se omite, el orden baja.

Opciones:

```matlab
W3 = [];
```

o:

```matlab
W3 = cfg.hinf.W3_high_gain;
```

o usar solo problema `S/KS`:

```matlab
[K, CL, gamma] = mixsyn(G, W1, W2, []);
```

Ventaja:

- baja el orden inmediatamente.

Riesgo:

- se pierde parte de la penalizacion explicita sobre ruido y alta frecuencia.
- puede no cubrir bien lo que el profesor espera de sensibilidad mixta completa.

Esta alternativa sirve para comparar, pero no debe ser la entrega final sin justificarla.

## 6. Alternativa D: usar `hinfstruct` o `systune` con estructura fija

En vez de dejar que `mixsyn` cree un controlador de orden libre, se puede imponer una estructura:

```text
K(s) = kp + ki/s + kd*s/(tau*s+1)
```

o una estructura de orden 4.

Ventaja:

- el orden queda fijado desde el inicio.
- se parece mas al desarrollo SAS/CAS del profesor.

Riesgo:

- requiere formular el problema tunable con bloques ajustables.
- puede ser mas trabajo y no siempre alcanza el mismo desempeno que H_inf libre.

Esta alternativa es teoricamente elegante si el profesor quiere conectar PID/SAS-CAS con robustez.

## 7. Alternativa E: hacer diseno por etapas SAS/CAS + H_inf pequeno

Otra idea es no pedirle todo a un solo `K` H_inf.

Procedimiento:

1. Cerrar primero un SAS clasico con realimentacion de tasa.
2. Obtener una planta amortiguada.
3. Disenar H_inf solo para el CAS externo.

Esto puede bajar orden porque H_inf ve una planta ya mas simple y mas amortiguada.

Con los apuntes de clase del 13 de mayo de 2026, esta alternativa sube de prioridad. El profesor recalco:

```text
pitch es inverso
yaw no debe seguir referencia
washout filter para controlar velocidad
pitch y roll alrededor de 30 deg
```

Eso sugiere que una planta ya amortiguada por SAS puede ser una base mas natural para un H_inf externo de menor orden. En ese caso, el orden bajo no se logra solo por `balred`, sino por repartir tareas:

- SAS clasico: amortiguamiento fisico con `q`, `p`, `r`;
- CAS/H_inf externo: seguimiento y robustez sobre una planta mas calmada.

Ventaja:

- conecta muy bien con las fotos del profesor.
- separa amortiguamiento y tracking.

Riesgo:

- cambia la arquitectura del taller.
- hay que documentar claramente que el H_inf se disena sobre una planta ya estabilizada por SAS.

## 8. Plan de trabajo recomendado

Orden recomendado de pruebas:

1. **Reduccion posterior de K**: probar `balred(K,4)` y validar.
2. **SAS interno + H_inf externo**: ahora es muy relevante por la clase de washout/yaw.
3. **Reduccion de planta antes de sintesis**: probar plantas reducidas y comparar.
4. **Sensibilidad mixta S/KS sin W3 dinamico**: medir cuanto baja el orden y cuanto empeora ruido.
5. **Estructura fija tipo PI+D con `hinfstruct/systune`**: si se quiere una solucion mas cercana a SAS/CAS.

## 9. Tabla de decision

| Alternativa | Baja orden | Mantiene teoria H_inf | Riesgo | Prioridad |
|---|---:|---:|---:|---:|
| Reducir `K` despues | Alta | Media | Medio | 1 |
| SAS + H_inf externo | Media | Alta | Cambia arquitectura | 2 |
| Reducir planta antes | Alta | Alta | Medio | 3 |
| Simplificar pesos | Alta | Media | Alto | 4 |
| `hinfstruct/systune` fijo | Total | Alta | Alto trabajo | 5 |

## 10. Que reportar si se logra orden 4

Si se consigue `K` de orden 4, el reporte debe incluir:

- orden original y orden reducido;
- metodo usado;
- `gamma` original y reducido;
- normas `||S||`, `||T||`, `||K*S||`;
- figuras `S`, `T`, `K*S`;
- simulacion temporal sobre `linmodel`;
- comentario honesto de la perdida o mejora.

La frase clave seria:

```text
La reduccion de orden se acepta porque conserva estabilidad y desempeno
frecuencial/temporal dentro de una degradacion pequena, a cambio de una
implementacion mas simple.
```
