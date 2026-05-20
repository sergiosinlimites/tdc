# Taller 1 - Guia conceptual para H_inf, incertidumbre, S, T, KS y planta generalizada

Esta guia explica las partes que suelen perderse en las diapositivas de `Sofrony_c.pdf` y como entran al taller del UAV. No es teoria por teoria: todo lo de aqui sirve para entender por que el taller pide `sigma(S)`, `sigma(T)`, `sigma(KS)`, filtros `W1`, `W2`, `W3`, ruido, perturbaciones e incertidumbre.

Archivos relacionados:

- Enunciado: `drive/TDC/02. TAREAS/T1/taller1_2022.pdf`
- Notas: `drive/TDC/01. NOTAS DE CLASE/H_inf/Sofrony_c.pdf`
- Implementacion: `main_taller1.m`
- Reporte de resultados: `reporte_taller1.md`
- Guia de simulacion: `README_simulacion_taller1.md`
- Plan de reduccion de orden: `PLAN_REDUCCION_ORDEN_HINF.md`
- Notas SAS/CAS de clase: `NOTAS_SAS_CAS_DESARROLLO_PROFESOR.md`

## 0. Antes de H_inf: que son SAS y CAS

En el taller aparecen dos ideas de arquitectura que conviene separar antes de hablar de robustez.

`SAS` significa `Stability Augmentation System`. Es un sistema de aumento de estabilidad: su trabajo principal no es seguir una referencia bonita, sino hacer que el avion sea mas amortiguado, mas estable y mas facil de controlar. En un UAV esto suele verse como realimentacion de tasas, por ejemplo `q`, `p` o `r`, para agregar amortiguamiento artificial.

`CAS` significa `Control Augmentation System`. Es un sistema de aumento de control: toma comandos del piloto o del autopiloto, por ejemplo una referencia de `theta` o `phi`, y los convierte en senales de actuador para que la aeronave siga esos comandos.

En una lectura sencilla:

- `SAS` estabiliza y amortigua la dinamica.
- `CAS` hace seguimiento de comandos.
- `H_inf` puede cumplir ambos papeles si el problema se formula bien: seguimiento, rechazo de perturbaciones, ruido, esfuerzo de control e incertidumbre.

En el proyecto se dejo una referencia clasica `PID SAS/CAS` para comparar contra el diseno `H_inf`. Esa referencia toma las ganancias baseline del paquete UAV_SIM_AEM (`Controllers/baseline_gains.m`): `kp_PT`, `ki_PT`, `kp_PD` para pitch/theta y `kp_RT`, `ki_RT`, `kp_RD` para roll/phi. La gracia no es decir que PID sea malo, sino tener una base conocida del material del UAV y luego ver que cambia cuando el controlador se disena con pesos robustos.

## 1. Idea central del control robusto

En control clasico uno suele disenar un controlador para una planta nominal:

```math
G_0(s)
```

Pero la planta real casi nunca es exactamente esa. Puede cambiar por:

- errores de modelado;
- saturacion;
- actuadores no ideales;
- dinamicas no incluidas;
- ruido de sensores;
- variacion de parametros;
- acoplamiento entre ejes.

Control robusto pregunta:

```text
Si la planta real no es exactamente G0, el lazo cerrado sigue funcionando?
```

En el taller:

- `latmod` y `longmod` son modelos simplificados para diseno;
- `linmodel` es la planta acoplada para simulacion;
- el ruido de medicion y la perturbacion de entrada representan efectos que el controlador debe tolerar;
- las graficas de `S`, `T`, `KS` son una forma de medir que tan robusto o fragil queda el lazo.

## 2. La diapositiva de lazo cerrado

La primera foto tiene este esquema:

```text
           e        u        di
r ---> (+) ---> K ----> (+) ----> G ----> (+) ---> y
       ^ -                         do      |
       |                                  |
       +------------- y + n <-------------+
```

Senales:

- `r`: referencia, por ejemplo `theta_ref` o `phi_ref`.
- `e`: error que ve el controlador.
- `K`: controlador.
- `u`: accion de control.
- `G`: planta.
- `di`: perturbacion de entrada, entra antes de la planta.
- `do`: perturbacion de salida, entra despues de la planta.
- `n` o `eta`: ruido de medicion, contamina lo que se realimenta.
- `y`: salida real de la planta.

La ecuacion de medicion es:

```math
e = r - (y + n)
```

La entrada a la planta es:

```math
u_{planta} = u + d_i
```

La salida final es:

```math
y = G(u + d_i) + d_o
```

Si se resuelve algebraicamente el lazo, aparecen las funciones `S`, `T`, `KS`, `Si` y `Ti`.

## 3. Que son S, T y KS

Para una planta `G` y un controlador `K`, el lazo abierto es:

```math
L = G K
```

La sensibilidad de salida es:

```math
S_0 = (I + G K)^{-1}
```

La sensibilidad complementaria de salida es:

```math
T_0 = (I + G K)^{-1} G K
```

Tambien se cumple:

```math
S_0 + T_0 = I
```

Esto es muy importante: si haces `S` pequeno en una zona de frecuencia, normalmente `T` crece en otra. No hay magia gratis.

Una forma mas cercana de leerlo es esta:

- `S` es lo que el lazo no alcanza a corregir.
- `T` es lo que el lazo si transmite hacia la salida.
- `KS` es el precio en actuador que se paga por corregir.

Por eso las tres curvas se leen juntas. Un controlador puede parecer excelente mirando solo `S`, porque sigue muy bien, pero al mismo tiempo puede exigir demasiado actuador en `KS` o dejar pasar ruido por `T`.

### 3.1 Interpretacion intuitiva de S

`S` dice que tanto se cuela el error o una perturbacion lenta.

Si:

```math
S(jw) pequeno
```

entonces:

- el error de seguimiento baja;
- las perturbaciones de baja frecuencia se rechazan mejor;
- el sistema sigue mejor referencias lentas.

En el UAV:

- si `S_theta` es pequeno a baja frecuencia, `theta` sigue mejor `theta_ref`;
- si `S_phi` es pequeno a baja frecuencia, `phi` sigue mejor `phi_ref`.

### 3.2 Interpretacion intuitiva de T

`T` dice cuanto de la referencia o del ruido de medicion termina en la salida.

Para seguimiento se quiere:

```math
T(jw) aprox 1
```

en bajas frecuencias, porque eso significa:

```math
y aprox r
```

Pero para ruido de medicion se quiere:

```math
T(jw) pequeno
```

en altas frecuencias, porque el ruido de sensor suele vivir alla.

En el UAV:

- `T` cerca de 1 a baja frecuencia ayuda a seguir `theta_ref` y `phi_ref`;
- `T` pequeno a alta frecuencia evita que el ruido de medicion excite los actuadores y la salida.

### 3.3 Interpretacion intuitiva de KS

`KS` es:

```math
K S
```

Mide cuanta accion de control se necesita para corregir errores o perturbaciones.

Si:

```math
KS(jw) grande
```

entonces el controlador esta pidiendo mucho actuador. Eso puede causar:

- saturacion;
- ruido amplificado en la senal de control;
- menor margen ante incertidumbre aditiva;
- comandos poco realistas para `elevator`, `aileron` o `rudder`.

En el taller, esto importa porque el enunciado exige:

```math
|u| <= 30 deg
```

Por eso el profesor insiste en mirar `KS`.

## 4. Que son S0, T0, Si y Ti

Las notas distinguen dos sensibilidades:

```math
S_0 = (I + G K)^{-1}
```

```math
S_i = (I + K G)^{-1}
```

Y dos sensibilidades complementarias:

```math
T_0 = (I + G K)^{-1} G K
```

```math
T_i = K G (I + K G)^{-1}
```

La diferencia es donde miras el lazo:

- subindice `0` o salida: lo que ocurre visto desde la salida;
- subindice `i` o entrada: lo que ocurre visto desde la entrada.

En SISO muchas veces no se nota la diferencia porque los productos conmutan de forma efectiva. En MIMO si importa, porque:

```math
G K \neq K G
```

Para el taller:

- en los canales SISO `theta/elevator` y `phi/aileron`, `S0` y `Si` son muy parecidos conceptualmente;
- si luego se hace un diseno MIMO, la diferencia se vuelve importante.

## 5. La diapositiva de ecuaciones de lazo cerrado

La diapositiva dice:

```math
y = T_0 r + S_0 G d_i + S_0 d_o - T_0 eta
```

```math
u = K S_0 r - K S_0 d_o + S_i d_i - K S_0 eta
```

Leido en palabras:

### 5.1 Salida y

```math
y = T_0 r
```

La referencia pasa a la salida por `T0`. Para buen seguimiento quieres `T0 aprox I` en baja frecuencia.

```math
y = S_0 G d_i
```

Una perturbacion de entrada `di` pasa por la planta y luego por `S0`. Para rechazarla quieres `S0` pequeno donde exista esa perturbacion.

```math
y = S_0 d_o
```

Una perturbacion de salida `do` entra directamente en la salida. Para rechazarla quieres `S0` pequeno.

```math
y = -T_0 eta
```

El ruido de medicion entra por la realimentacion. Para que no contamine la salida quieres `T0` pequeno en alta frecuencia.

### 5.2 Control u

```math
u = K S_0 r
```

La referencia exige accion de control. Si `KS0` es grande, seguir referencias puede requerir mucho actuador.

```math
u = -K S_0 d_o
```

Una perturbacion de salida tambien exige accion de control. Otra razon para mirar `KS0`.

```math
u = S_i d_i
```

La perturbacion de entrada afecta la senal interna de control segun `Si`.

```math
u = -K S_0 eta
```

El ruido de medicion puede convertirse en movimiento del actuador. Para evitarlo, `KS0` no debe ser enorme a alta frecuencia.

## 6. La diapositiva de especificaciones de diseno

La segunda foto parece una lista larga, pero en realidad dice: cada problema fisico se ve en una funcion de transferencia distinta.

| Objetivo fisico | Funcion que se revisa | Que se quiere |
| --- | --- | --- |
| Rechazar perturbaciones de salida `do` en `y` | `S0` | pequeno |
| Que `do` no obligue demasiado control `u` | `KS0` | pequeno |
| Rechazar perturbaciones de entrada `di` | `S0` o `Si` | pequeno |
| Rechazar ruido de medicion `eta` en `y` | `T0` | pequeno en alta frecuencia |
| Que ruido no mueva mucho actuador | `KS0` | pequeno en alta frecuencia |
| Seguir referencia | `T0` | aprox 1 en baja frecuencia |
| Limitar accion de control | `KS0` | pequeno |
| Robustez ante incertidumbre multiplicativa de entrada | `Ti` | pequeno donde hay incertidumbre |
| Robustez ante incertidumbre multiplicativa de salida | `T0` | pequeno donde hay incertidumbre |
| Robustez ante incertidumbre aditiva | `KS0` | pequeno |

### 6.1 Para que necesito esas especificaciones

Las necesitas porque te dicen que debes penalizar en el diseno.

Ejemplo:

Si el enunciado dice:

```text
rechazo a ruido de medicion
```

entonces eso entra como:

```text
hacer T pequeno en alta frecuencia
```

Si el enunciado dice:

```text
esfuerzo de control no mayor a +-30 deg
```

entonces eso entra como:

```text
hacer KS pequeno y simular saturacion
```

Si el enunciado dice:

```text
seguimiento de theta y phi
```

entonces eso entra como:

```text
hacer S pequeno a baja frecuencia
hacer T aprox 1 a baja frecuencia
```

Si el enunciado dice:

```text
incertidumbre aditiva inversa o perturbaciones de entrada
```

entonces eso entra como:

```text
analizar robustez con KS, S, Si o el canal que ve la incertidumbre
```

En codigo, estas especificaciones aparecen en:

- `parametros_taller1.m`: frecuencias, saturacion, ruido;
- `diseno_hinf_taller1.m`: pesos `W1`, `W2`, `W3`;
- `analisis_sensibilidades.m`: calculo de `S`, `T`, `KS`;
- `simulacion_taller1.m`: ruido, perturbacion y saturacion.

## 7. Incertidumbre: multiplicativa, aditiva e inversa

La planta nominal es:

```math
G_0
```

La planta real incierta se escribe como:

```math
G
```

La incertidumbre se representa con un bloque desconocido:

```math
Delta
```

La idea no es saber exactamente `Delta`, sino acotar su tamano:

```math
||Delta||_\infty <= algo
```

y garantizar estabilidad por pequena ganancia.

## 8. Incertidumbre multiplicativa de entrada

Forma de las notas:

```math
G = G_0 (I + Delta_i)
```

Interpretacion:

La incertidumbre multiplica la entrada de la planta. Es como si el actuador no aplicara exactamente el comando, sino:

```math
u_real = (I + Delta_i) u
```

Ejemplo SISO:

```math
G_0(s) = 2/(s+1)
```

Si el actuador tiene 20% de error de ganancia:

```math
Delta_i = 0.2
```

entonces:

```math
G(s) = G_0(s)(1 + 0.2) = 1.2 G_0(s)
```

Ejemplo UAV:

El elevador puede tener una ganancia real diferente a la modelada:

```text
comando elevator = 10 deg
movimiento efectivo = 12 deg
```

Eso se parece a incertidumbre multiplicativa de entrada en el canal `elevator -> theta`.

Robustez asociada en la diapositiva:

```text
incertidumbre multiplicativa de entrada -> Ti pequeno
```

## 9. Incertidumbre multiplicativa de salida

Forma de las notas:

```math
G = (I + Delta_o) G_0
```

Interpretacion:

La incertidumbre aparece en la salida. Es como si la salida modelada se deformara por dinamicas no modeladas o sensores/mediciones de salida.

Ejemplo SISO:

```math
G(s) = (1 + 0.1)G_0(s)
```

La salida real es 10% mayor que la nominal.

Ejemplo UAV:

El modelo predice `phi`, pero la dinamica real de alabeo tiene una amplificacion extra por aeroelasticidad, acoplamiento o sensor:

```text
phi_real aprox 1.1 phi_modelado
```

Robustez asociada:

```text
incertidumbre multiplicativa de salida -> T0 pequeno
```

## 10. Incertidumbre aditiva

Forma de las notas:

```math
G = G_0 + Delta_a
```

Interpretacion:

A la planta nominal se le suma una dinamica no modelada.

Ejemplo SISO:

```math
G_0(s) = 1/(s+1)
```

```math
Delta_a(s) = 0.1/(0.01s + 1)
```

Entonces:

```math
G(s) = 1/(s+1) + 0.1/(0.01s+1)
```

Ejemplo UAV:

El canal `aileron -> phi` tiene una dinamica adicional no incluida por flexibilidad, retardo de actuador o acoplamiento lateral:

```math
G_real = G_nominal + dinamica_extra
```

Robustez asociada en la diapositiva:

```text
incertidumbre aditiva -> KS0 pequeno
```

Por eso en el taller se mira `K*S`.

## 11. Incertidumbre inversa multiplicativa de entrada

Forma de las notas:

```math
G = G_0 (I - Delta_i)^{-1}
```

Interpretacion:

La incertidumbre aparece "dentro" de una realimentacion equivalente en la entrada. Si `Delta_i` crece, el efecto no es simplemente sumar o multiplicar; aparece un denominador.

Ejemplo SISO:

```math
G = G_0/(1 - Delta_i)
```

Si:

```math
Delta_i = 0.2
```

entonces:

```math
G = G_0/0.8 = 1.25 G_0
```

Si `Delta_i` se acerca a 1, la ganancia puede crecer muchisimo. Por eso las incertidumbres inversas pueden ser peligrosas.

Ejemplo UAV:

Un actuador con saturacion o una dinamica interna que reduce efectividad puede representarse como una modificacion inversa: el controlador cree que manda `u`, pero la planta se comporta como si necesitara mas autoridad de control.

## 12. Incertidumbre inversa multiplicativa de salida

Forma de las notas:

```math
G = (I - Delta_o)^{-1} G_0
```

Ejemplo SISO:

```math
G = G_0/(1 - Delta_o)
```

Interpretacion:

El error o dinamica no modelada aparece despues de la planta nominal, pero en forma inversa. Puede amplificar mucho la salida si `Delta_o` se acerca a 1.

Ejemplo UAV:

Una dinamica de sensor/salida no considerada que retroalimenta o amplifica la medicion de `theta` o `phi`.

## 13. Incertidumbre inversa aditiva

Forma de las notas:

```math
G = (I - G_0 Delta_a)^{-1} G_0
```

Esta es la que se menciona en el enunciado del taller.

Interpretacion:

No es simplemente:

```math
G_0 + Delta
```

sino una incertidumbre que entra de forma interna y modifica el denominador efectivo del sistema. Si el producto:

```math
G_0 Delta_a
```

se acerca a 1 en alguna frecuencia, puede haber amplificacion fuerte o perdida de estabilidad.

Ejemplo SISO:

```math
G_0(s) = 1/(s+1)
```

```math
Delta_a(s) = 0.2
```

Entonces:

```math
G(s) = (1 - G_0(s)0.2)^{-1}G_0(s)
```

Es decir:

```math
G(s) = G_0(s)/(1 - 0.2G_0(s))
```

Ejemplo UAV:

Una dinamica no modelada entre actuacion y rotacion que se "cierra" internamente con la planta. En el canal `elevator -> theta`, esto puede representar que parte del movimiento aerodinamico no modelado regresa como una modificacion efectiva de la dinamica longitudinal.

Para el taller, esto justifica mirar robustez y no solo respuesta al escalon.

## 14. Pequena ganancia: por que mirar normas

La herramienta mental es:

```math
||M||_\infty ||Delta||_\infty < 1
```

Si `M` es el sistema que ve la incertidumbre y `Delta` es la incertidumbre, el producto de ganancias debe ser menor que 1.

Por eso el profesor insiste en:

```matlab
sigma(S)
sigma(T)
sigma(KS)
```

Porque `sigma` muestra la ganancia maxima por frecuencia. Si una de esas curvas tiene un pico grande, esa frecuencia es una zona fragil.

## 15. Para que son W1, W2 y W3

En H_inf no le dices directamente al controlador:

```text
haz buen seguimiento y usa poco control
```

Se lo dices con pesos.

Los pesos son filtros que convierten frases de diseno en restricciones de frecuencia. No son constantes decorativas: le dicen al algoritmo donde debe ser exigente y donde puede relajarse.

En sensibilidad mixta, MATLAB intenta hacer pequeno:

```math
\gamma =
\left\|
\begin{bmatrix}
W_1 S \\
W_2 K S \\
W_3 T
\end{bmatrix}
\right\|_\infty
```

Si `gamma < 1`, entonces, de forma aproximada:

```math
|S| < |1/W_1|,
\quad
|KS| < |1/W_2|,
\quad
|T| < |1/W_3|
```

Por eso las graficas comparan `S`, `KS` y `T` contra las inversas de los pesos. La multiplicacion aparece porque no se penaliza la senal "cruda", sino la senal vista a traves del filtro de importancia. Donde `W1` es grande, un mismo error `S` pesa mucho mas; donde `W1` es pequeno, pesa menos.

Piensalo como una lupa dependiente de la frecuencia:

- `W1` agranda el error de seguimiento donde el seguimiento importa.
- `W2` agranda la accion de control donde no queremos gastar actuador.
- `W3` agranda la sensibilidad complementaria donde el ruido o dinamicas no modeladas son peligrosas.

## 16. W1: peso sobre S

`W1` penaliza:

```math
W_1 S
```

Si el objetivo es:

```math
W_1 S < 1
```

entonces:

```math
S < 1/W_1
```

Por eso en las graficas se compara `S` contra `1/W1`.

Uso:

- hacer `S` pequeno a baja frecuencia;
- mejorar seguimiento;
- mejorar rechazo de perturbaciones lentas.

La razon de fondo es que, en realimentacion unitaria, el error de seguimiento y muchas perturbaciones de baja frecuencia llegan multiplicadas por `S`. Si se reduce `S` en la zona donde viven las referencias lentas del UAV, la salida queda mas cerca de la referencia.

En el taller:

```matlab
W1 = makeweight(5, wb, 0.05);
```

Eso significa que queremos exigir mas a baja frecuencia que a alta frecuencia.

## 17. W2: peso sobre KS

`W2` penaliza:

```math
W_2 K S
```

Uso:

- limitar esfuerzo de control;
- evitar saturacion;
- evitar que ruido se convierta en movimiento de actuador;
- mejorar robustez ante incertidumbre aditiva.

La razon de fondo es que `KS` transforma error medido en comando de control. Si `KS` es grande, el controlador esta corrigiendo con mucha energia. Eso puede verse bien en seguimiento, pero puede romper la restriccion fisica de `+-30 deg` o excitar dinamicas que no estan en el modelo nominal.

En el taller:

```matlab
W2 = 0.15;
```

Si `W2` sube, el diseno castiga mas el control. El controlador se vuelve menos agresivo, pero puede perder seguimiento.

## 18. W3: peso sobre T

`W3` penaliza:

```math
W_3 T
```

Uso:

- bajar `T` en alta frecuencia;
- rechazar ruido de medicion;
- limitar efectos de dinamicas no modeladas de alta frecuencia;
- mejorar robustez ante incertidumbre multiplicativa de salida.

La razon de fondo es que el ruido de medicion entra por la realimentacion. En muchas configuraciones, su efecto sobre la salida esta gobernado por `T`, y su efecto sobre el control por `KS`. Ademas, la incertidumbre multiplicativa suele crecer en alta frecuencia, precisamente donde el modelo nominal deja de ser confiable. Por eso `W3` normalmente exige que `T` caiga arriba de cierta frecuencia.

En el taller:

```matlab
W3 = makeweight(0.02, wp, 5);
```

La idea es dejar que `T` sea util a baja frecuencia para seguimiento, pero castigarlo a frecuencias altas.

## 19. Sensibilidad mixta: que significa

Se llama sensibilidad mixta porque mezcla varios objetivos en un solo problema:

```math
min_K ||
[ W_1 S
  W_2 K S
  W_3 T ]
||_\infty
```

O sea, el controlador intenta al mismo tiempo:

- reducir error de seguimiento (`S`);
- limitar control (`KS`);
- rechazar ruido y alta frecuencia (`T`).

El resultado es un compromiso. Si exiges demasiado todo al mismo tiempo, no hay solucion buena.

`gamma` es el valor de ese compromiso. MATLAB no lo escoge a mano: `mixsyn` construye internamente una planta generalizada y llama a la sintesis H_inf para buscar un controlador estabilizante que minimice la norma H_inf desde entradas externas hacia salidas ponderadas. En palabras mas simples:

```text
gamma pequeno = todas las senales ponderadas quedaron pequenas.
gamma grande  = alguna exigencia pesa demasiado o el controlador no puede cumplirla bien.
```

En el taller se reporta `gamma` por eje porque cada canal SISO (`theta/elevator` y `phi/aileron`) se sintetiza por separado. Si despues cambias `W1`, `W2` o `W3`, `gamma` tambien cambia porque cambiaste el examen que debe pasar el controlador.

La norma no suma errores como una integral temporal. Mira el peor caso en frecuencia. Por eso una curva con un pico estrecho puede dominar el resultado aunque en simulacion temporal parezca aceptable.

## 20. Planta generalizada S/KS: como sale la matriz P

La tercera foto muestra una planta generalizada para el problema `S/KS`, es decir, usando solo `W1 S` y `W2 K S`.

La idea es construir un bloque `P` que reciba:

```math
[w; u]
```

y entregue:

```math
[z_1; z_2; v]
```

Donde:

- `w`: senal externa, por ejemplo perturbacion o referencia equivalente;
- `u`: senal de control que viene de `K`;
- `z1`: salida de desempeno asociada a error;
- `z2`: salida de desempeno asociada a control;
- `v`: senal medida que entra al controlador.

La diapositiva define:

```math
z_1 = W_1(w + G u)
```

```math
z_2 = W_2 u
```

```math
v = -w - G u
```

Nota: el signo depende de la convencion del diagrama. En la diapositiva aparece:

```math
v = -w - G u
```

porque el controlador recibe una senal con signo de realimentacion negativa.

Ahora se escribe cada ecuacion como combinacion de `w` y `u`.

### 20.1 Primera fila

```math
z_1 = W_1 w + W_1 G u
```

Entonces la primera fila de `P` es:

```math
[ W_1    W_1 G ]
```

### 20.2 Segunda fila

```math
z_2 = 0 w + W_2 u
```

Entonces la segunda fila es:

```math
[ 0      W_2 ]
```

### 20.3 Tercera fila

```math
v = -w - G u
```

Entonces la tercera fila es:

```math
[ -1     -G ]
```

### 20.4 Matriz completa

Uniendo las tres filas:

```math
P =
[ W_1    W_1 G
  0      W_2
 -1     -G    ]
```

Eso es exactamente lo que muestra la diapositiva.

## 21. Por que esa P produce S y KS

El controlador se conecta como:

```math
u = K v
```

Con:

```math
v = -w - G u
```

sustituyes:

```math
u = K(-w - G u)
```

```math
u + K G u = -K w
```

```math
(I + K G)u = -K w
```

```math
u = -(I + K G)^{-1}K w
```

En SISO esto se interpreta como una version de:

```math
u = -K S w
```

Luego:

```math
z_2 = W_2 u
```

queda proporcional a:

```math
W_2 K S
```

Y:

```math
z_1 = W_1(w + G u)
```

queda proporcional a:

```math
W_1 S
```

Por eso la planta generalizada fue armada asi: para que al cerrar el lazo con `K`, las salidas `z1` y `z2` midan exactamente las cosas que quieres minimizar.

## 22. Como se conecta esto con MATLAB

Puedes armar esa `P` a mano, pero MATLAB ya tiene funciones para hacerlo:

```matlab
P = augw(G, W1, W2, W3);
[K, CL, gamma] = hinfsyn(P, nmeas, ncon);
```

O directamente:

```matlab
[K, CL, gamma] = mixsyn(G, W1, W2, W3);
```

En este taller se usa:

```matlab
[K, CL, gamma] = mixsyn(G, W1, W2, W3);
```

en `diseno_hinf_taller1.m`.

## 23. Como entran las especificaciones al proyecto

El enunciado no solo da numeros: da instrucciones de diseno.

### 23.1 Seguimiento de referencia

Enunciado:

```text
theta y phi en [-40, 40] deg
```

Entra como:

- simulaciones `theta_10`, `phi_10`, `theta_40`, `phi_40`;
- `W1` para reducir `S`;
- analisis de `T` cerca de 1 a baja frecuencia.

### 23.2 Ruido de medicion

Enunciado:

```text
ruido lateral 0.001
ruido longitudinal 0.0001
```

Entra como:

- ruido en `simulacion_taller1.m`;
- ruido en `taller1_uav.slx`;
- `W3` para reducir `T` a alta frecuencia;
- revision de `KS`, porque el ruido tambien puede mover actuadores.

### 23.3 Perturbaciones de entrada

Enunciado:

```text
perturbaciones hasta 6 Hz
```

Entra como:

- `cfg.spec.perturbation_hz = 6`;
- perturbacion sinusoidal en simulacion;
- frecuencia `wp = 2*pi*6` para pesos.

### 23.3.1 Por que aparece `2*pi*6` o `2*pi*8`

En MATLAB, las funciones de sistemas continuos (`tf`, `ss`, `sigma`, `mixsyn`, `makeweight`) trabajan naturalmente con frecuencia angular:

```math
\omega \quad [rad/s]
```

Pero muchas especificaciones fisicas se dan en Hertz:

```math
f \quad [Hz] = [ciclos/s]
```

La conversion es:

```math
\omega = 2\pi f
```

Por eso:

```matlab
2*pi*6
```

significa `6 Hz` expresados como `37.70 rad/s`, y:

```matlab
2*pi*8
```

significa `8 Hz` expresados como `50.27 rad/s`.

En este taller, `2*pi*6` aparece porque el enunciado habla de perturbaciones hasta `6 Hz`. En la planeacion tambien se considero `2*pi*8` como una opcion mas conservadora para poner el corte de robustez algo por encima de la frecuencia de perturbacion. La regla practica es:

- usa la frecuencia del enunciado si quieres representar exactamente la especificacion;
- usa una frecuencia un poco mayor si quieres dejar margen para no disenar justo al borde.

### 23.4 Esfuerzo de control

Enunciado:

```text
control no debe superar +-30 deg
```

Entra como:

- saturacion en simulacion;
- `W2` para penalizar `KS`;
- metricas de saturacion.

### 23.5 Robustez

Enunciado:

```text
incertidumbre aditiva inversa
```

Entra como:

- interpretacion con pequena ganancia;
- revision de `KS`;
- comparacion de valores singulares con `sigma`.

## 24. Que mirar cuando revises las figuras

### 24.1 `sensibilidades_theta.png` y `sensibilidades_phi.png`

Mira:

- `S`: debe ser bajo a baja frecuencia.
- `T`: debe caer a alta frecuencia.
- `KS`: no debe tener picos demasiado altos.

### 24.2 `comparacion_sensibilidades.png`

Compara PID contra H_inf:

- si H_inf baja `T`, mejora ruido;
- si H_inf sube mucho `KS`, pide mas control;
- si `S` tiene picos grandes, hay fragilidad de seguimiento o perturbaciones.

### 24.3 Simulaciones

En las figuras `sim_*.png` revisa:

- seguimiento de `theta`;
- seguimiento de `phi`;
- control `elevator`;
- control `aileron`;
- saturacion en `+-30 deg`.

## 25. Resumen muy corto

Si solo recuerdas una cosa:

```text
S  = que tanto queda error/perturbacion.
T  = que tanto pasa referencia o ruido a la salida.
KS = cuanto control se gasta para corregir.
```

Y:

```text
W1 castiga S.
W2 castiga KS.
W3 castiga T.
```

Las especificaciones del profesor importan porque te dicen donde y cuanto castigar cada una.

En el taller:

- seguimiento -> `W1`, `S`, `T`;
- ruido -> `W3`, `T`, `KS`;
- esfuerzo -> `W2`, `KS`, saturacion;
- incertidumbre -> `sigma`, pequena ganancia, `T` o `KS` segun el tipo.

## 26. Funciones MATLAB usadas: entradas, salidas y toolbox

Esta seccion resume las funciones mas importantes usadas en el taller. La descripcion sigue la documentacion oficial de MathWorks.

### 26.1 `ss` - Control System Toolbox

Documentacion oficial: <https://www.mathworks.com/help/control/ref/ss.html>

Uso tipico:

```matlab
sys = ss(A, B, C, D)
sys = ss(modelo_existente)
```

Entradas esperadas:

- `A`: matriz de estados de tamano `n x n`.
- `B`: matriz de entrada de tamano `n x m`.
- `C`: matriz de salida de tamano `p x n`.
- `D`: matriz de paso directo de tamano `p x m`.

Salida:

- `sys`: modelo dinamico en espacio de estados con `n` estados, `m` entradas y `p` salidas.

En el taller se usa para asegurar que `linmodel`, `latmod`, `longmod` y los controladores queden como modelos compatibles con analisis y simulacion.

### 26.2 `tf` - Control System Toolbox

Documentacion oficial: <https://www.mathworks.com/help/control/ref/tf.html>

Uso tipico:

```matlab
s = tf('s')
G = tf(num, den)
```

Entradas esperadas:

- `num`: coeficientes del numerador.
- `den`: coeficientes del denominador.
- `tf('s')`: crea la variable simbolica de Laplace para escribir transferencias como expresiones.

Salida:

- `G`: modelo de funcion de transferencia.

En el taller se usa para escribir controladores PID filtrados y el lazo de yaw de forma compacta.

### 26.3 `minreal` - Control System Toolbox

Documentacion oficial: <https://www.mathworks.com/help/control/ref/dynamicsystem.minreal.html>

Uso tipico:

```matlab
sysr = minreal(sys, tol)
```

Entradas esperadas:

- `sys`: modelo dinamico `ss`, `tf` o similar.
- `tol`: tolerancia numerica para cancelar polos y ceros casi iguales.

Salida:

- `sysr`: modelo reducido equivalente en la practica numerica.

En el taller se usa para limpiar modelos despues de multiplicar plantas, controladores y lazos cerrados.

### 26.4 `feedback` - Control System Toolbox

Documentacion oficial: <https://www.mathworks.com/help/control/ref/inputoutputmodel.feedback.html>

Uso tipico:

```matlab
T = feedback(G*K, 1)
S = feedback(1, G*K)
```

Entradas esperadas:

- primer argumento: modelo directo.
- segundo argumento: modelo de realimentacion o ganancia.
- por defecto usa realimentacion negativa.

Salidas:

- modelo de lazo cerrado.

En el taller:

- `feedback(G*K, 1)` calcula `T = GK/(1+GK)`.
- `feedback(1, G*K)` calcula `S = 1/(1+GK)`.

### 26.5 `sigma` - Control System Toolbox

Documentacion oficial: <https://www.mathworks.com/help/control/ref/dynamicsystem.sigma.html>

Uso tipico:

```matlab
sigma(sys)
sigma(sys, {wmin, wmax})
[sv, wout] = sigma(sys, w)
```

Entradas esperadas:

- `sys`: sistema SISO o MIMO.
- `{wmin, wmax}`: rango de frecuencia en `rad/s`.
- `w`: vector de frecuencias en `rad/s`.

Salidas:

- si no se piden salidas, dibuja valores singulares.
- si se piden salidas, entrega magnitudes `sv` y frecuencias `wout`.

En SISO coincide con la magnitud de Bode. En MIMO muestra la maxima amplificacion posible por frecuencia.

### 26.6 `makeweight` - Robust Control Toolbox

Documentacion oficial: <https://www.mathworks.com/help/robust/ref/makeweight.html>

Uso tipico:

```matlab
W = makeweight(g0, wc, ginf)
```

Entradas esperadas:

- `g0`: ganancia deseada a baja frecuencia.
- `wc`: frecuencia de cruce aproximada en `rad/s`.
- `ginf`: ganancia deseada a alta frecuencia.

Salida:

- `W`: filtro estable de primer orden que conecta esas ganancias.

En el taller se usa para construir pesos con sentido fisico: exigente en baja frecuencia para `W1`, exigente en alta frecuencia para `W3`.

### 26.7 `augw` - Robust Control Toolbox

Documentacion oficial: <https://www.mathworks.com/help/robust/ref/dynamicsystem.augw.html>

Uso tipico:

```matlab
P = augw(G, W1, W2, W3)
```

Entradas esperadas:

- `G`: planta nominal.
- `W1`: peso sobre sensibilidad `S`.
- `W2`: peso sobre esfuerzo `KS`.
- `W3`: peso sobre sensibilidad complementaria `T`.

Salida:

- `P`: planta generalizada aumentada para sintesis H_inf.

En el taller se deja esta funcion en `construir_planta_generalizada_hinf.m` para mostrar de donde sale la formulacion que despues usa `mixsyn`.

### 26.8 `mixsyn` - Robust Control Toolbox

Documentacion oficial: <https://www.mathworks.com/help/robust/ref/dynamicsystem.mixsyn.html>

Uso tipico:

```matlab
[K, CL, gamma, info] = mixsyn(G, W1, W2, W3)
```

Entradas esperadas:

- `G`: planta nominal, SISO o MIMO.
- `W1`, `W2`, `W3`: pesos compatibles en dimensiones con las salidas/entradas de `G`.

Salidas:

- `K`: controlador sintetizado.
- `CL`: lazo cerrado ponderado desde entradas externas hacia salidas de desempeno.
- `gamma`: norma H_inf alcanzada.
- `info`: informacion adicional del proceso de sintesis.

En el taller se usa como interfaz directa para sensibilidad mixta:

```math
min_K ||
[ W_1 S; W_2 K S; W_3 T ]
||_\infty
```

### 26.9 `hinfsyn` - Robust Control Toolbox

Documentacion oficial: <https://www.mathworks.com/help/robust/ref/dynamicsystem.hinfsyn.html>

Uso tipico:

```matlab
[K, CL, gamma] = hinfsyn(P, nmeas, ncont)
```

Entradas esperadas:

- `P`: planta generalizada.
- `nmeas`: numero de mediciones que entran al controlador.
- `ncont`: numero de senales de control que salen del controlador.

Salidas:

- `K`: controlador H_inf.
- `CL`: sistema cerrado ponderado.
- `gamma`: desempeno H_inf obtenido.

`mixsyn` usa esta idea por debajo: primero arma una planta aumentada y luego resuelve un problema H_inf.

### 26.10 `exportgraphics` - MATLAB

Documentacion oficial: <https://www.mathworks.com/help/matlab/ref/exportgraphics.html>

Uso tipico:

```matlab
exportgraphics(fig, "figura.png", ...
    "BackgroundColor", "white", ...
    "Resolution", 200)
```

Entradas esperadas:

- `fig` o `axes`: objeto grafico a exportar.
- nombre de archivo.
- opciones como color de fondo y resolucion.

Salida:

- archivo exportado en disco.

En el taller se usa para que todas las figuras guardadas tengan fondo blanco, rejilla legible y colores distinguibles.
