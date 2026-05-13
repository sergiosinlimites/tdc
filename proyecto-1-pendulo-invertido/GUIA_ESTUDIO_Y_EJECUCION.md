# Guia de Estudio y Ejecucion

Esta guia esta pensada para entender el proyecto en orden, sin abrir todos los
archivos a la vez. La idea es que primero entiendas el sistema fisico, luego el
modelo lineal, despues el LQR, y al final la simulacion no lineal, Simulink y el
observador.

## 0. Mapa rapido del proyecto

Empieza por estos archivos, en este orden:

1. `parametros_pendulo.m`
2. `modelo_lineal_pendulo.m`
3. `diseno_lqr_pendulo.m`
4. `simulacion_lineal_pendulo.m`
5. `simulacion_no_lineal_pendulo.m`
6. `diseno_observador_pendulo.m`
7. `simulacion_observador_lineal_pendulo.m`
8. `analisis_resultados.m`
9. `main_pendulo.m`
10. `animacion_pendulo_interactiva.m`
11. `pendulo_invertido.slx`

Si quieres solo correr todo:

```matlab
cd /home/sergio/Escritorio/tdc/proyecto-1-pendulo-invertido
main_pendulo
```

Si quieres reconstruir el modelo Simulink desde cero:

```matlab
build_pendulo_simulink
```

Si quieres abrir una simulacion visual interactiva:

```matlab
animacion_pendulo_interactiva
```

En esa ventana puedes mover la fuerza con el deslizador. Con `LQR activo`, el
deslizador funciona como una perturbacion externa que se suma al controlador.
Si desactivas `LQR activo`, el deslizador queda como la entrada manual completa
al carro.

## 0.5. Dibujo mental del sistema y del vector de estado

El pendulo invertido de este proyecto es un pendulo montado sobre un carro:

```text
                 theta
                  /
                 /   pendulo
                o    centro de masa
                |
                |
        -----------------
        |     carro     |  ---> x positivo
        -----------------
              suelo
```

Hay dos movimientos principales:

1. El carro se mueve horizontalmente.
2. El pendulo gira alrededor del punto donde esta unido al carro.

Por eso el estado tiene cuatro componentes:

```math
x_s =
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
\text{posicion horizontal del carro [m]} \\
\text{velocidad horizontal del carro [m/s]} \\
\text{angulo del pendulo respecto a la vertical hacia arriba [rad]} \\
\text{velocidad angular del pendulo [rad/s]}
\end{bmatrix}
```

Respuesta directa a la duda:

- `x` es la posicion del carro, no del pendulo.
- `x_dot` o `\dot{x}` es la velocidad horizontal del carro.
- `theta` es el angulo del pendulo.
- `theta_dot` o `\dot{\theta}` es la velocidad angular del pendulo.

En MATLAB se suele llamar `x` al vector de estado completo, pero aqui eso puede
confundir porque tambien hay una variable fisica llamada `x`. Para evitarlo,
piensa asi:

```text
xs o x_s  = vector completo de estado
xs(1)     = x             = posicion del carro
xs(2)     = x_dot         = velocidad del carro
xs(3)     = theta         = angulo del pendulo
xs(4)     = theta_dot     = velocidad angular del pendulo
```

La entrada del sistema es:

```math
u = \text{fuerza horizontal aplicada al carro [N]}
```

Entonces el controlador no empuja directamente el pendulo. Empuja el carro. Al
mover el carro, se genera la aceleracion que corrige el angulo del pendulo.

## 1. `parametros_pendulo.m`

Este archivo define los valores fisicos del sistema:

- masa del carro `M`;
- masa del pendulo `m`;
- longitud al centro de masa `l`;
- friccion viscosa `b`;
- gravedad `g`;
- limite de fuerza `umax`;
- condicion inicial base;
- matrices `Q` y `R` para el LQR.

Concepto importante:

En control moderno se trabaja mucho con modelos parametrizados. No conviene
escribir numeros sueltos en todos lados porque despues no sabes que simulacion
uso que valores. Este archivo es el "centro de verdad" del proyecto.

La convencion usada es:

```matlab
x_s = [x; x_dot; theta; theta_dot]
```

Donde `theta = 0` significa que el pendulo esta vertical hacia arriba. Eso es
clave: el signo de todas las ecuaciones depende de esta convencion.

La condicion inicial base es:

```matlab
p.x0_default = [0; 0; deg2rad(5); 0];
```

Visualmente:

```math
x_s(0) =
\begin{bmatrix}
0 \\
0 \\
5^\circ \\
0
\end{bmatrix}
```

Eso significa:

- el carro arranca en `x = 0 m`;
- el carro arranca quieto, `x_dot = 0 m/s`;
- el pendulo arranca inclinado `5 grados`;
- el pendulo arranca sin velocidad angular, `theta_dot = 0 rad/s`.

La matriz `Q` tambien sigue ese mismo orden:

```matlab
p.Q = diag([10, 1, 300, 20]);
```

```math
Q =
\begin{bmatrix}
10 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 300 & 0 \\
0 & 0 & 0 & 20
\end{bmatrix}
```

Interpretacion:

- peso `10` para posicion del carro;
- peso `1` para velocidad del carro;
- peso `300` para angulo del pendulo;
- peso `20` para velocidad angular.

El peso del angulo es grande porque el objetivo principal es que el pendulo no
se caiga.

## 2. `modelo_lineal_pendulo.m`

Este archivo construye las matrices:

```matlab
dx/dt = A*x + B*u
y    = C*x + D*u
```

Concepto importante:

El sistema real del pendulo es no lineal porque aparecen terminos como:

```matlab
sin(theta)
cos(theta)
theta_dot^2
```

Pero cerca de `theta = 0`, podemos aproximarlo con un sistema lineal. Esta
aproximacion es local: sirve para angulos pequenos, no para cualquier posicion
del pendulo.

Las ecuaciones no lineales usadas como punto de partida son:

```math
(M+m)\ddot{x} + b\dot{x}
+ ml(\ddot{\theta}\cos\theta - \dot{\theta}^2\sin\theta) = u
```

```math
l\ddot{\theta} = g\sin\theta - \ddot{x}\cos\theta
```

### De donde sale la primera ecuacion

La primera ecuacion es la ecuacion de fuerzas horizontales del sistema completo
carro + pendulo:

```math
(M+m)\ddot{x} + b\dot{x}
+ ml(\ddot{\theta}\cos\theta - \dot{\theta}^2\sin\theta) = u
```

No aparece de la nada. Sale de aplicar la segunda ley de Newton:

```math
\sum F_x = ma_x
```

pero teniendo cuidado con algo importante: la masa del pendulo no tiene la misma
aceleracion horizontal que el carro. El carro se mueve con posicion `x`, pero el
centro de masa del pendulo tambien se mueve porque el pendulo gira.

La posicion horizontal del centro de masa del pendulo es:

```math
x_p = x + l\sin\theta
```

Visualmente:

```text
posicion horizontal del pendulo = posicion del carro + aporte por inclinacion

x_p = x + l*sin(theta)
      |     |
      |     desplazamiento horizontal del centro de masa respecto al pivote
      posicion del carro
```

Ahora derivamos dos veces para obtener aceleracion horizontal.

Primera derivada:

```math
\dot{x}_p = \dot{x} + l\dot{\theta}\cos\theta
```

Segunda derivada:

```math
\ddot{x}_p =
\ddot{x} + l(\ddot{\theta}\cos\theta - \dot{\theta}^2\sin\theta)
```

Ese termino largo:

```math
l(\ddot{\theta}\cos\theta - \dot{\theta}^2\sin\theta)
```

es la parte de la aceleracion horizontal causada por el giro del pendulo.

Desglose:

- `l*theta_ddot*cos(theta)` viene de la aceleracion angular del pendulo;
- `-l*theta_dot^2*sin(theta)` viene de que el pendulo gira con velocidad angular,
  y al derivar `cos(theta)` aparece otro `theta_dot`;
- por eso aparece `theta_dot^2`: no es un invento del control, sale de derivar
  dos veces la posicion `l*sin(theta)`.

Ahora se escribe la suma de fuerzas horizontales externas. Sobre el conjunto
carro + pendulo actuan horizontalmente:

```math
u - b\dot{x}
```

La fuerza `u` empuja el carro. La friccion viscosa `b*x_dot` se opone al
movimiento.

La masa del carro aporta:

```math
M\ddot{x}
```

La masa del pendulo aporta:

```math
m\ddot{x}_p
```

Entonces:

```math
u - b\dot{x} = M\ddot{x} + m\ddot{x}_p
```

Sustituyendo la aceleracion del centro de masa del pendulo:

```math
u - b\dot{x}
=
M\ddot{x}
+
m\left[
\ddot{x} + l(\ddot{\theta}\cos\theta - \dot{\theta}^2\sin\theta)
\right]
```

Distribuyendo:

```math
u - b\dot{x}
=
(M+m)\ddot{x}
+
ml(\ddot{\theta}\cos\theta - \dot{\theta}^2\sin\theta)
```

Pasando la friccion al lado derecho queda la forma usada en el codigo:

```math
(M+m)\ddot{x} + b\dot{x}
+ ml(\ddot{\theta}\cos\theta - \dot{\theta}^2\sin\theta) = u
```

Lectura fisica de cada termino:

```text
(M+m)*x_ddot
    aceleracion horizontal base de todo el conjunto

b*x_dot
    fuerza de friccion del carro

m*l*theta_ddot*cos(theta)
    aceleracion horizontal por aceleracion angular del pendulo

-m*l*theta_dot^2*sin(theta)
    efecto no lineal por velocidad angular del pendulo

u
    fuerza aplicada al carro
```

### De donde sale la segunda ecuacion

La segunda ecuacion relaciona la aceleracion angular del pendulo con gravedad y
aceleracion del carro:

```math
l\ddot{\theta} = g\sin\theta - \ddot{x}\cos\theta
```

No sale directamente de una sumatoria de fuerzas en `Y`. Sale de proyectar las
fuerzas sobre la direccion tangencial del movimiento del pendulo, es decir, la
direccion en la que el pendulo puede girar.

El pendulo esta unido al carro por una barra. Esa barra impone una restriccion:
el centro de masa del pendulo no se mueve libremente en `X` y `Y`, sino sobre un
arco alrededor del pivote.

```text
                    direccion tangencial
                         \
                 theta    \
                  /        \
                 o --------- direccion radial, a lo largo de la barra
                /
        pivote o
```

La fuerza de tension de la barra va en direccion radial. Como esa fuerza va a lo
largo de la barra, no produce giro alrededor del pivote. Por eso conviene mirar
solo la direccion tangencial.

Con la convencion del proyecto:

```text
theta = 0  pendulo vertical hacia arriba
theta > 0  pendulo inclinado hacia la derecha
```

La posicion del centro de masa del pendulo puede pensarse como:

```math
x_p = x + l\sin\theta
```

```math
y_p = l\cos\theta
```

Aqui `y_p` se toma positivo hacia arriba, medido desde el pivote. Si `theta = 0`,
entonces `y_p = l`, que corresponde al pendulo arriba.

El vector tangencial unitario, en la direccion en la que aumenta `theta`, es:

```math
\hat{t} =
\begin{bmatrix}
\cos\theta \\
-\sin\theta
\end{bmatrix}
```

Ese vector apunta aproximadamente hacia la derecha cuando `theta = 0`, que es
justo la direccion inicial de caida si el pendulo se inclina hacia la derecha.

Ahora hacemos Newton en la direccion tangencial:

```math
\sum F_t = m a_t
```

Como la aceleracion tangencial por giro es:

```math
a_t = l\ddot{\theta}
```

entonces:

```math
\frac{\sum F_t}{m} = l\ddot{\theta}
```

Hay dos aportes importantes en esa direccion tangencial.

Primero, la gravedad. El vector aceleracion de gravedad es:

```math
\vec{g} =
\begin{bmatrix}
0 \\
-g
\end{bmatrix}
```

Proyectamos gravedad sobre la direccion tangencial:

```math
\vec{g}\cdot\hat{t}
=
\begin{bmatrix}
0 \\
-g
\end{bmatrix}
\cdot
\begin{bmatrix}
\cos\theta \\
-\sin\theta
\end{bmatrix}
=
g\sin\theta
```

Por eso aparece:


```math
g\sin\theta
```

Segundo, el pivote no esta quieto: el carro puede acelerar horizontalmente con
`\ddot{x}`. Si miras el pendulo desde el marco que se mueve con el carro, aparece
una aceleracion ficticia opuesta a la aceleracion del carro:

```math
\vec{a}_{ficticia} =
\begin{bmatrix}
-\ddot{x} \\
0
\end{bmatrix}
```

Proyectamos esa aceleracion sobre la direccion tangencial:

```math
\vec{a}_{ficticia}\cdot\hat{t}
=
\begin{bmatrix}
-\ddot{x} \\
0
\end{bmatrix}
\cdot
\begin{bmatrix}
\cos\theta \\
-\sin\theta
\end{bmatrix}
=
-\ddot{x}\cos\theta
```

Por eso aparece:

```math
-\ddot{x}\cos\theta
```

Sumando los aportes tangenciales por unidad de masa:

```math
l\ddot{\theta} = g\sin\theta - \ddot{x}\cos\theta
```

Esa es la segunda ecuacion.

Lectura fisica:

- si `theta` crece, `g*sin(theta)` tiende a aumentar mas el angulo, porque el
  equilibrio invertido es inestable;
- si el carro acelera, el termino `-x_ddot*cos(theta)` puede ayudar a corregir
  o empeorar la caida, segun el sentido de la aceleracion.

Estas ecuaciones dicen, en palabras:

- la fuerza `u` acelera al carro;
- la friccion `b*x_dot` se opone al movimiento del carro;
- el pendulo afecta al carro porque su masa tambien se mueve;
- el carro afecta al pendulo porque la base del pendulo se acelera.

Para linealizar alrededor de `theta = 0`, se usan estas aproximaciones:

```math
\sin\theta \approx \theta,
\qquad
\cos\theta \approx 1,
\qquad
\dot{\theta}^2\sin\theta \approx 0
```

El ultimo termino se elimina porque es de orden no lineal: multiplica velocidad
angular al cuadrado por angulo.

Aplicando esas aproximaciones a las dos ecuaciones no lineales queda:

```math
(M+m)\ddot{x} + b\dot{x} + ml\ddot{\theta} = u
```

```math
l\ddot{\theta} = g\theta - \ddot{x}
```

La segunda ecuacion permite escribir:

```math
\ddot{\theta} = \frac{g\theta - \ddot{x}}{l}
```

Ahora sustituimos esa expresion en la primera:

```math
(M+m)\ddot{x} + b\dot{x}
+ ml\left(\frac{g\theta - \ddot{x}}{l}\right) = u
```

Se cancela `l`:

```math
(M+m)\ddot{x} + b\dot{x}
+ m(g\theta - \ddot{x}) = u
```

Distribuyendo:

```math
(M+m)\ddot{x} + b\dot{x} + mg\theta - m\ddot{x} = u
```

Agrupando los terminos con `xddot`:

```math
M\ddot{x} + b\dot{x} + mg\theta = u
```

Despejando:

```math
\ddot{x} = \frac{u - b\dot{x} - mg\theta}{M}
```

Con esa aceleracion del carro, volvemos a:

```math
\ddot{\theta} = \frac{g\theta - \ddot{x}}{l}
```

Sustituyendo `xddot`:

```math
\ddot{\theta}
=
\frac{
g\theta - \frac{u - b\dot{x} - mg\theta}{M}
}{l}
```

Llevando todo a un solo denominador:

```math
\ddot{\theta}
=
\frac{
Mg\theta - u + b\dot{x} + mg\theta
}{Ml}
```

Finalmente:

```math
\ddot{\theta}
=
\frac{(M+m)g\theta + b\dot{x} - u}{Ml}
```

Despues de linealizar, quedan dos aceleraciones:

```math
\ddot{x} = \frac{u - b\dot{x} - mg\theta}{M}
```

```math
\ddot{\theta} =
\frac{(M+m)g\theta + b\dot{x} - u}{Ml}
```

Ahora se acomodan en forma de primer orden. Como:

```math
\frac{d}{dt}
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
\dot{x} \\
\ddot{x} \\
\dot{\theta} \\
\ddot{\theta}
\end{bmatrix}
```

entonces:

```math
\begin{bmatrix}
\dot{x} \\
\ddot{x} \\
\dot{\theta} \\
\ddot{\theta}
\end{bmatrix}
=
\underbrace{
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -b/M & -mg/M & 0 \\
0 & 0 & 0 & 1 \\
0 & b/(Ml) & (M+m)g/(Ml) & 0
\end{bmatrix}}_{A}
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
+
\underbrace{
\begin{bmatrix}
0 \\
1/M \\
0 \\
-1/(Ml)
\end{bmatrix}}_{B}
u
```

Observa las filas:

- fila 1: `d/dt(x) = x_dot`;
- fila 2: `d/dt(x_dot) = x_ddot`;
- fila 3: `d/dt(theta) = theta_dot`;
- fila 4: `d/dt(theta_dot) = theta_ddot`.

En este proyecto, el modelo lineal sale como:

```matlab
A =
         0    1.0000         0         0
         0   -0.2000   -3.9240         0
         0         0         0    1.0000
         0    0.6667   45.7800         0

B =
         0
    2.0000
         0
   -6.6667
```

Como hay un polo abierto positivo, el pendulo vertical hacia arriba es
inestable sin control.

Donde se ve eso:

```matlab
eig(A)
```

En `analisis_resultados.m` aparece como:

```matlab
results.polos_abiertos = eig(A);
```

Con los parametros del proyecto, los polos abiertos son aproximadamente:

```text
0
6.7382
-0.1428
-6.7953
```

El polo importante aqui es:

```text
6.7382
```

porque es positivo.

Por que un polo positivo significa inestabilidad:

En un sistema lineal, cada polo genera un modo parecido a:

```math
e^{\lambda t}
```

Si `lambda` es negativo:

```math
e^{-6.7953t}
```

ese modo se va apagando con el tiempo.

Si `lambda` es positivo:

```math
e^{6.7382t}
```

ese modo crece con el tiempo. Eso significa que una inclinacion pequena del
pendulo no desaparece sola: aumenta. Fisicamente, es lo que esperas de un
pendulo parado hacia arriba. Si lo sueltas apenas inclinado, se cae.

Ese analisis es "en abierto" porque todavia no se ha aplicado el controlador:

```math
\dot{x}_s = Ax_s
```

Despues del LQR se revisan los polos de:

```matlab
eig(A - B*K)
```

Si el controlador esta bien disenado, esos polos ya quedan con parte real
negativa.

La matriz `C = eye(4)` significa que, en la simulacion lineal basica, se sacan
como salida los cuatro estados:

```math
y =
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
```

Mas adelante, para el observador, se cambia a medir solo `x` y `theta`.

## 3. Controlabilidad

Esto se revisa en `main_pendulo.m` y `analisis_resultados.m` con:

```matlab
rank(ctrb(A,B))
```

Concepto importante:

Un sistema es controlable si la entrada `u` puede mover todos los estados
importantes. En este caso necesitamos rango `4`, porque hay cuatro estados.

Si el rango fuera menor que `4`, habria alguna combinacion de posicion,
velocidad, angulo o velocidad angular que no podriamos controlar con la fuerza
del carro. En ese caso, disenar un LQR completo no tendria sentido.

Matematicamente se arma esta matriz:

```math
\mathcal{C} =
\begin{bmatrix}
B & AB & A^2B & A^3B
\end{bmatrix}
```

Cada bloque responde una pregunta:

- `B`: que estados mueve directamente la fuerza `u`;
- `AB`: que estados se empiezan a mover despues por la dinamica;
- `A^2B`, `A^3B`: como la influencia de `u` se propaga con el tiempo.

Si estas cuatro columnas independientes alcanzan rango `4`, la fuerza aplicada
al carro puede influir en los cuatro estados del sistema.

## 4. `diseno_lqr_pendulo.m`

Este archivo calcula la ganancia:

```matlab
u = -K*x
```

usando:

```matlab
[K, S, polos_cerrados] = lqr(A, B, Q, R);
```

Concepto importante:

LQR significa `Linear Quadratic Regulator`. Busca una ganancia `K` que minimiza:

```math
J = \int_0^\infty (x_s^T Q x_s + u^T R u)\,dt
```

La matriz `Q` penaliza estados grandes. En este proyecto se penaliza fuerte
`theta` porque dejar crecer el angulo es peligroso.

La matriz `R` penaliza esfuerzo de control. Si `R` es pequeno, el controlador
se vuelve mas agresivo. Si `R` es grande, usa menos fuerza, pero normalmente
responde mas lento.

La ley de control tiene esta forma:

```math
u = -Kx_s
```

Como hay una sola entrada, `K` es una fila:

```math
K =
\begin{bmatrix}
k_1 & k_2 & k_3 & k_4
\end{bmatrix}
```

Por tanto:

```math
u =
-(k_1x + k_2\dot{x} + k_3\theta + k_4\dot{\theta})
```

Es decir, la fuerza que se aplica al carro se calcula mezclando:

- cuanto se movio el carro;
- que tan rapido va el carro;
- cuanto se inclino el pendulo;
- que tan rapido esta girando el pendulo.

El signo menos significa retroalimentacion negativa: si el estado se aleja del
equilibrio, el controlador intenta empujarlo de vuelta.

Despues de aplicar `u = -K*x`, el sistema cerrado queda:

```matlab
dx/dt = (A - B*K)*x
```

Con nombres mas explicitos:

```math
\dot{x}_s = Ax_s + Bu
```

```math
u = -Kx_s
```

Sustituyendo:

```math
\dot{x}_s = Ax_s + B(-Kx_s)
```

```math
\dot{x}_s = (A - BK)x_s
```

Por eso miramos los polos de:

```matlab
eig(A - B*K)
```

Si todos tienen parte real negativa, el sistema lineal cerrado es estable.

## 5. `simulacion_lineal_pendulo.m`

Este archivo simula directamente:

```matlab
dx/dt = (A - B*K)*x
```

Es la simulacion mas limpia para estudiar la respuesta ideal del diseno LQR.

Concepto importante:

La simulacion lineal no incluye todavia:

- saturacion de actuador;
- efectos no lineales grandes;
- error de estimacion;
- ruido.

Por eso sirve como primer laboratorio, pero no como prueba final.

En esta simulacion no hay una entrada externa nueva. Solo se le da una condicion
inicial y se mira como el sistema vuelve al equilibrio:

```math
x_s(0) =
\begin{bmatrix}
0 \\
0 \\
5^\circ \\
0
\end{bmatrix}
\quad \Longrightarrow \quad
x_s(t)
```

La salida `sim.x` queda organizada por columnas:

```text
sim.x(:,1) = x
sim.x(:,2) = x_dot
sim.x(:,3) = theta
sim.x(:,4) = theta_dot
```

Si haces:

```matlab
plot(sim.t, sim.x(:,1))
```

estas viendo la posicion del carro. Si haces:

```matlab
plot(sim.t, rad2deg(sim.x(:,3)))
```

estas viendo el angulo del pendulo en grados.

## 6. `simulacion_no_lineal_pendulo.m`

Este archivo simula la planta no lineal con `ode45`.

Aqui se usan las ecuaciones completas, con `sin(theta)` y `cos(theta)`. El
controlador sigue siendo el mismo:

```matlab
u = -K*x
```

pero ahora se puede activar saturacion:

```matlab
u = min(max(u, -umax), umax)
```

Concepto importante:

El LQR fue disenado con el modelo lineal, pero lo probamos contra la planta no
lineal para ver si todavia funciona cerca del equilibrio. Esto es lo que se
llama validar la region local de operacion.

En los casos probados, `3`, `5`, `8` y `10` grados, el sistema se estabiliza.
Pero eso no significa estabilidad global. Si arrancas muy lejos, el modelo
lineal deja de representar bien la fisica.

Dentro del archivo, el vector `xs` se lee asi:

```matlab
x_dot = xs(2);
theta = xs(3);
theta_dot = xs(4);
```

No se define `x = xs(1)` en esa parte porque para calcular las aceleraciones no
hace falta la posicion absoluta del carro; hacen falta la velocidad del carro,
el angulo y la velocidad angular.

La dinamica no lineal calcula:

```math
\dot{x}_s =
\begin{bmatrix}
\dot{x} \\
\ddot{x} \\
\dot{\theta} \\
\ddot{\theta}
\end{bmatrix}
```

con:

```math
\ddot{x} =
\frac{
u - b\dot{x} - mg\sin\theta\cos\theta
+ ml\dot{\theta}^2\sin\theta
}{
M + m\sin^2\theta
}
```

```math
\ddot{\theta} =
\frac{g\sin\theta - \ddot{x}\cos\theta}{l}
```

Comparalo con el modelo lineal:

```text
no lineal: usa sin(theta), cos(theta), theta_dot^2
lineal:    usa theta en lugar de sin(theta), 1 en lugar de cos(theta)
```

Por eso el modelo lineal es una aproximacion cerca de `theta = 0`.

## 7. Saturacion del actuador

La saturacion aparece en la simulacion no lineal y en Simulink.

Concepto importante:

El LQR ideal puede pedir cualquier fuerza. Un actuador real no. Por eso se
limita:

```matlab
|u| <= 10 N
```

Esto cambia el comportamiento del sistema, porque ya no se cumple exactamente:

```matlab
u = -K*x
```

Ahora es:

```matlab
u = sat(-K*x)
```

La saturacion puede hacer que el sistema responda mas lento o incluso pierda
estabilidad si la condicion inicial es demasiado grande.

Graficamente, la saturacion convierte una fuerza ideal grande en una fuerza
fisicamente posible:

```text
si -K*x_s =  18 N  ->  u =  10 N
si -K*x_s =   4 N  ->  u =   4 N
si -K*x_s = -25 N  ->  u = -10 N
```

## 8. `diseno_observador_pendulo.m`

Este archivo disena un observador de Luenberger.

Situacion:

En el LQR basico asumimos que conocemos todos los estados:

```matlab
x
x_dot
theta
theta_dot
```

Pero en la practica muchas veces solo mides:

```matlab
x
theta
```

Las velocidades no siempre se miden directamente. El observador sirve para
estimar los estados que faltan.

La ecuacion del observador es:

```matlab
xhat_dot = A*xhat + B*u + L*(y - C*xhat)
```

La medicion usada es:

```math
y =
C_{meas}x_s =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0
\end{bmatrix}
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
x \\
\theta
\end{bmatrix}
```

Es decir: el sensor mide posicion del carro y angulo del pendulo, pero no mide
directamente las velocidades.

Donde:

- `xhat` es el estado estimado;
- `y` es lo que mides;
- `C*xhat` es lo que el observador cree que deberias medir;
- `y - C*xhat` es el error de medicion;
- `L` decide que tan fuerte corriges la estimacion.

La idea intuitiva:

El observador corre una copia del modelo dentro del controlador. Si la copia
predice mal las mediciones, se corrige usando el error `y - C*xhat`.

Por eso Luenberger no es magia rara: es un modelo interno mas una correccion
proporcional al error de salida.

Visualmente:

```text
medicion real y = [x; theta]
             |
             v
      error = y - C*xhat
             |
             v
modelo + correccion con L  --->  xhat = [xhat; xhat_dot; thetahat; thetahat_dot]
             |
             v
        controlador LQR usa u = -K*xhat
```

## 9. Observabilidad

Antes de usar el observador, se revisa:

```matlab
rank(obsv(A, C_meas))
```

Concepto importante:

Un sistema es observable si, mirando las salidas medidas durante el tiempo,
puedes reconstruir los estados internos.

En este proyecto:

```matlab
C_meas = [1 0 0 0;
          0 0 1 0]
```

Eso significa que medimos posicion del carro y angulo del pendulo. El rango de
observabilidad da `4`, asi que desde esas dos mediciones se pueden estimar los
cuatro estados del modelo lineal.

La matriz que se revisa es:

```math
\mathcal{O} =
\begin{bmatrix}
C \\
CA \\
CA^2 \\
CA^3
\end{bmatrix}
```

La idea es parecida a controlabilidad, pero al reves:

- controlabilidad pregunta si la entrada `u` puede afectar todos los estados;
- observabilidad pregunta si las mediciones `y` contienen informacion suficiente
  para reconstruir todos los estados.

Aunque no midas `x_dot` ni `theta_dot`, esas velocidades dejan huella en como
cambian `x` y `theta` con el tiempo. Por eso el observador puede reconstruirlas.

## 10. `simulacion_observador_lineal_pendulo.m`

Este archivo prueba el sistema lineal usando estados estimados:

```matlab
u = -K*xhat
```

No usa el estado real completo para controlar. Usa la estimacion `xhat`.

Concepto importante:

Esto es una separacion clasica en control moderno:

- disenas `K` para controlar;
- disenas `L` para estimar;
- combinas ambos usando `u = -K*xhat`.

Este principio se conoce como principio de separacion para sistemas lineales:
puedes disenar el controlador y el observador por separado, siempre que el
sistema sea controlable y observable.

En el archivo se construye un sistema aumentado:

```math
z =
\begin{bmatrix}
x_s \\
\hat{x}_s
\end{bmatrix}
```

```math
\dot{z} =
\begin{bmatrix}
A & -BK \\
LC_{meas} & A - BK - LC_{meas}
\end{bmatrix}
z
```

La primera mitad de `z` es el estado real. La segunda mitad es el estado
estimado. Despues se calcula el error:

```math
e = x_s - \hat{x}_s
```

Si el observador esta bien disenado, ese error debe tender a cero.

## 11. `analisis_resultados.m`

Este archivo junta las pruebas:

- polos abiertos;
- polos cerrados;
- controlabilidad;
- observabilidad;
- casos con `3`, `5`, `8` y `10` grados;
- comparacion lineal contra no lineal;
- variantes de `Q` y `R`;
- simulacion con observador;
- generacion de graficas.

Las graficas quedan en:

```text
figures/
```

Mira especialmente:

- `comparacion_lineal_no_lineal.png`;
- `barrido_condiciones_iniciales.png`;
- `variantes_lqr.png`;
- `observador_lineal.png`.

## 12. `main_pendulo.m`

Este es el archivo que debes ejecutar cuando quieras ver todo el resumen:

```matlab
main_pendulo
```

Te imprime:

- parametros;
- matrices `A`, `B`, `C`, `D`;
- polos abiertos;
- controlabilidad;
- observabilidad;
- ganancia `K`;
- polos cerrados;
- matriz del observador `L`;
- desempeno no lineal con saturacion.

Si estas estudiando, una buena practica es ejecutar `main_pendulo`, mirar una
seccion del resultado y despues abrir el archivo que produjo esa parte.

## 13. Que ver en Simulink

Abre:

```matlab
open_system('pendulo_invertido')
```

O haz doble clic en:

```text
pendulo_invertido.slx
```

El modelo tiene este flujo:

```text
Planta_Lineal -> Controlador_LQR -> Saturacion_Actuador -> Planta_Lineal
```

Bloques importantes:

- `Planta_Lineal`: bloque `State-Space` con `A_lin`, `B_lin`, `C_lin`, `D_lin`.
- `Controlador_LQR`: bloque `Gain` con ganancia `-K_lqr`.
- `Saturacion_Actuador`: limita la fuerza entre `-umax` y `umax`.
- `Estados_To_Workspace`: guarda los estados en `simout_x`.
- `Control_To_Workspace`: guarda la fuerza en `simout_u`.
- `Scope_Estados`: visualiza los estados.
- `Scope_Control`: visualiza la fuerza.

Antes de simular, corre:

```matlab
init_pendulo_simulink
```

Luego simula:

```matlab
out = sim('pendulo_invertido');
```

Para mirar el angulo:

```matlab
simout_x = out.get('simout_x');
plot(simout_x.time, rad2deg(simout_x.signals.values(:,3)))
grid on
xlabel('t [s]')
ylabel('theta [deg]')
```

Para mirar la fuerza:

```matlab
simout_u = out.get('simout_u');
plot(simout_u.time, simout_u.signals.values)
grid on
xlabel('t [s]')
ylabel('u [N]')
```

## 14. Ruta recomendada de estudio

Primera pasada:

1. Lee `parametros_pendulo.m`.
2. Lee `modelo_lineal_pendulo.m`.
3. Ejecuta `main_pendulo`.
4. Mira los polos abiertos y confirma que hay uno positivo.
5. Mira `K` y los polos cerrados.

Segunda pasada:

1. Lee `diseno_lqr_pendulo.m`.
2. Cambia `Q` o `R` en `parametros_pendulo.m`.
3. Vuelve a ejecutar `main_pendulo`.
4. Compara `variantes_lqr.png`.

Tercera pasada:

1. Lee `simulacion_no_lineal_pendulo.m`.
2. Cambia los angulos iniciales en `analisis_resultados.m`.
3. Prueba valores como `12`, `15` o `20` grados.
4. Observa cuando la saturacion empieza a dominar.

Cuarta pasada:

1. Lee `diseno_observador_pendulo.m`.
2. Lee `simulacion_observador_lineal_pendulo.m`.
3. Mira `observador_lineal.png`.
4. Cambia los polos del observador y observa si estima mas rapido o mas lento.

Quinta pasada:

1. Abre `pendulo_invertido.slx`.
2. Sigue la senal desde la planta hasta el controlador y de vuelta.
3. Corre la simulacion.
4. Mira `simout_x` y `simout_u`.

## 15. Experimentos pequenos para aprender

Prueba 1:

Cambia `p.R = 0.05` a:

```matlab
p.R = 0.25;
```

Efecto esperado: menos esfuerzo de control, respuesta menos agresiva.

Prueba 2:

Cambia el peso de `theta` en `Q`:

```matlab
p.Q = diag([10, 1, 900, 20]);
```

Efecto esperado: el controlador se preocupa mas por enderezar el pendulo.

Prueba 3:

Cambia la saturacion:

```matlab
p.umax = 5;
```

Efecto esperado: el actuador se queda corto antes; puede aumentar el tiempo de
recuperacion o fallar para angulos iniciales mas grandes.

Prueba 4:

Cambia los polos del observador:

```matlab
polos_obs = [-6, -7, -8, -9];
```

Efecto esperado: estimacion mas lenta, pero menos agresiva.

Prueba 5:

Haz los polos del observador mucho mas rapidos:

```matlab
polos_obs = [-30, -31, -32, -33];
```

Efecto esperado: convergencia rapida en el modelo ideal, pero en sistemas reales
esto puede amplificar ruido.

## 16. Idea mental para llevarte

El proyecto tiene esta cadena conceptual:

```text
Fisica no lineal
    -> linealizacion local
    -> espacio de estados
    -> controlabilidad
    -> LQR
    -> lazo cerrado A - B*K
    -> simulacion lineal
    -> validacion no lineal
    -> saturacion
    -> observabilidad
    -> observador de Luenberger
    -> control usando estados estimados
```

Si entiendes esa cadena, entiendes el corazon del proyecto.
