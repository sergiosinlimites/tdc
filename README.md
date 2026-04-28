# Control moderno, LQR, Riccati, Lyapunov, LMI y control estocástico

> Guía secuencial para alguien que ya vio control clásico, pero apenas está entrando a técnicas de control moderno y control óptimo.
>
> Fuente base de estudio: presentación `Modern_Optimal_Control.pdf` cargada en el proyecto.

---

## 0. Objetivo de esta guía

Esta guía conecta los temas en este orden:

1. Control clásico vs. control moderno.
2. Espacio de estados.
3. Realimentación de estados.
4. LQR: control óptimo lineal cuadrático.
5. Matrices `Q`, `R`, `P`, `A`, `B`, `B^T` y ganancia `K`.
6. Ecuación de Riccati.
7. Lyapunov y estabilidad.
8. Matrices positivas definidas.
9. Operador `vec` o `stack`.
10. Producto de Kronecker.
11. LMI: desigualdades matriciales lineales.
12. Complemento de Schur.
13. Optimización convexa y algoritmos de punto interior.
14. Control estocástico y sistemas con incertidumbre.
15. Ideas de proyectos en Python y MATLAB.

La idea es que cada concepto nuevo se apoye en el anterior. No se asume que ya domines control moderno.

---

# 1. De control clásico a control moderno

En control clásico normalmente se trabaja con funciones de transferencia:

```math
G(s)=\frac{Y(s)}{U(s)}
```

El enfoque está en la relación entre entrada y salida:

```math
u(t) \longrightarrow \text{sistema} \longrightarrow y(t)
```

Ahí se estudian polos, ceros, respuesta al escalón, error en estado estacionario, diagramas de Bode, Nyquist y lugar geométrico de las raíces.

En control moderno el sistema se representa con variables de estado:

```math
\dot{x}=Ax+Bu
```

```math
y=Cx+Du
```

Donde:

- `x`: vector de estados internos del sistema.
- `u`: vector de entradas o señales de control.
- `y`: vector de salidas medidas o de interés.
- `A`: matriz de dinámica interna.
- `B`: matriz que dice cómo entra el control al sistema.
- `C`: matriz que relaciona estados con salidas.
- `D`: matriz de paso directo entre entrada y salida.

## Analogía

En control clásico miras principalmente lo que sale del sistema, como si solo vieras el velocímetro de un carro.

En control moderno miras también lo que pasa por dentro: posición, velocidad, aceleración, ángulo, corriente, temperatura interna, etc.

---

# 2. Espacio de estados

Un sistema lineal continuo se escribe como:

```math
\dot{x}=Ax+Bu
```

Si el sistema tiene `n` estados y `m` entradas:

```math
x\in\mathbb{R}^{n}
```

```math
u\in\mathbb{R}^{m}
```

```math
A\in\mathbb{R}^{n\times n}
```

```math
B\in\mathbb{R}^{n\times m}
```

## ¿Qué significa `A`?

La matriz `A` describe cómo evoluciona el sistema por sí solo, sin control externo.

Si `u=0`, queda:

```math
\dot{x}=Ax
```

Entonces `A` contiene la dinámica natural del sistema.

## ¿Qué significa `B`?

La matriz `B` indica cómo la entrada de control afecta los estados.

Ejemplo: si tienes una masa, un resorte y un amortiguador, el control puede ser una fuerza. Esa fuerza no afecta igual a la posición que a la velocidad. La matriz `B` codifica esa relación.

---

# 3. Realimentación de estados

Una ley de control típica en control moderno es:

```math
u=-Kx
```

Donde:

- `u`: entrada de control.
- `K`: matriz de ganancias.
- `x`: vector de estados.

El signo negativo significa realimentación negativa: si el estado se aleja del equilibrio, el control lo empuja de vuelta.

Sustituyendo en el sistema:

```math
\dot{x}=Ax+B(-Kx)
```

```math
\dot{x}=(A-BK)x
```

La matriz importante del sistema cerrado es:

```math
A_{cl}=A-BK
```

El objetivo es elegir `K` para que `A-BK` sea estable.

## Conexión con control clásico

En control clásico mirabas los polos del sistema cerrado. En espacio de estados también: los polos son los valores propios de:

```math
A-BK
```

Si esos valores propios están en el semiplano izquierdo para tiempo continuo, el sistema cerrado es estable.

---

# 4. Controlabilidad

Antes de diseñar `K`, hay que saber si el sistema se puede controlar.

Un sistema es controlable si, mediante la entrada `u`, puedes llevar los estados desde una condición inicial hasta otra condición deseada.

Para un sistema continuo:
    
```math
\dot{x}=Ax+Bu
```

la matriz de controlabilidad es:

```math
\mathcal{C}=\begin{bmatrix}B & AB & A^2B & \cdots & A^{n-1}B\end{bmatrix}
```

El sistema es controlable si:

```math
rank(\mathcal{C})=n
```

## Analogía

Tener un sistema no controlable es como tener un carro con volante, pero sin frenos o sin acelerador para ciertas direcciones. Hay estados que simplemente no puedes modificar.

---

# 5. LQR: Linear Quadratic Regulator

LQR significa:

- `Linear`: el modelo es lineal.
- `Quadratic`: la función de costo es cuadrática.
- `Regulator`: busca regular el sistema hacia el equilibrio.

El problema LQR busca una ley:

```math
u=-Kx
```

que minimice un costo:

```math
J=\int_0^\infty \left(x^TQx+u^TRu\right)dt
```

## ¿Qué mide ese costo?

Tiene dos partes.

### 1. Costo de error de estado

```math
x^TQx
```

Penaliza que los estados se alejen del objetivo.

### 2. Costo de esfuerzo de control

```math
u^TRu
```

Penaliza usar demasiada señal de control.

## Idea central

LQR busca equilibrio entre:

1. Corregir rápido y bien.
2. No gastar demasiada energía ni saturar actuadores.

## Analogía

Si conduces una bicicleta y te desvías:

- Corregir muy suave puede ser lento.
- Corregir muy agresivo puede ser brusco.
- LQR encuentra una corrección óptima según qué tanto te importan el error y el esfuerzo.

---

# 6. Matriz Q

`Q` es la matriz que penaliza los estados.

```math
x^TQx
```

Si un estado es muy importante, se le asigna un peso grande en `Q`.

Ejemplo con dos estados:

```math
x=\begin{bmatrix}x_1\\x_2\end{bmatrix}
```

```math
Q=\begin{bmatrix}100 & 0\\0 & 1\end{bmatrix}
```

Esto significa que el estado `x1` pesa mucho más que `x2`.

## Interpretación práctica

En un péndulo invertido:

- puedes penalizar mucho el ángulo, porque si el péndulo se cae, el sistema falla;
- puedes penalizar menos la posición del carro, si no es tan crítica.

---

# 7. Matriz R

`R` penaliza el uso de la entrada de control.

```math
u^TRu
```

Si `R` es grande, controlar sale caro. El controlador será más suave.

Si `R` es pequeño, controlar sale barato. El controlador será más agresivo.

## Caso escalar

Si tienes una sola entrada:

```math
R=r
```

Entonces:

- `r` grande: menos control.
- `r` pequeño: más control.

## Analogía

`Q` dice cuánto te duele equivocarte.

`R` dice cuánto te duele corregir.

---

# 8. Matriz P: función de valor

En LQR aparece una matriz `P` asociada a una función de valor:

```math
V(x)=x^TPx
```

Esta función representa el costo futuro esperado desde el estado actual.

## Interpretación

`P` dice qué tan caro es estar en cierto estado.

Si estás en una dirección del espacio de estados donde `V(x)` crece mucho, significa que esa condición es peligrosa o costosa.

## Analogía

`P` es como un mapa de relieve del problema:

- zonas bajas: estados poco costosos;
- zonas altas: estados muy costosos.

---

# 9. Ganancia óptima K

Para el LQR continuo estándar, la ganancia óptima es:

```math
K=R^{-1}B^TP
```

Donde:

- `R^{-1}`: inversa de la matriz que penaliza el esfuerzo de control.
- `B^T`: transpuesta de la matriz de entrada.
- `P`: matriz de costo futuro.

## ¿Qué significa `B^T`?

Si:

```math
B\in\mathbb{R}^{n\times m}
```

entonces:

```math
B^T\in\mathbb{R}^{m\times n}
```

`B^T` reorganiza la información de cómo los estados se conectan con las entradas.

## Interpretación de la fórmula

```math
K=R^{-1}B^TP
```

significa:

1. `P` dice qué estados son costosos.
2. `B^T` dice qué parte de ese costo puede ser atacada por la entrada.
3. `R^{-1}` ajusta la acción según qué tan caro es usar control.

Frase clave:

> `P` dice qué duele, `B` dice qué puedes mover y `R` dice cuánto cuesta moverlo.

---

# 10. Ecuación algebraica de Riccati

Para encontrar `P`, se resuelve la ecuación algebraica de Riccati:

```math
A^TP+PA-PBR^{-1}B^TP+Q=0
```

Esta ecuación aparece al resolver el problema de optimización LQR.

## ¿Por qué es difícil?

Porque contiene el término:

```math
PBR^{-1}B^TP
```

Ahí `P` aparece multiplicándose con `P`. Por eso la ecuación es no lineal.

## Comparación

Una ecuación de Lyapunov es lineal en `P`:

```math
A^TP+PA+Q=0
```

Una ecuación de Riccati es no lineal en `P`:

```math
A^TP+PA-PBR^{-1}B^TP+Q=0
```

---

# 11. Riccati diferencial vs. Riccati algebraica

## Riccati diferencial

Aparece en problemas de horizonte finito:

```math
-\dot{P}=A^TP+PA-PBR^{-1}B^TP+Q
```

Aquí `P` cambia con el tiempo.

## Riccati algebraica

Aparece en problemas de horizonte infinito o régimen estacionario:

```math
A^TP+PA-PBR^{-1}B^TP+Q=0
```

Aquí se busca una `P` constante.

## Idea práctica

- Horizonte finito: maniobra específica, por ejemplo aterrizaje de un dron.
- Horizonte infinito: regulación permanente, por ejemplo mantener estable un motor o un robot.

---

# 12. Lyapunov

Lyapunov es una forma de analizar estabilidad sin resolver explícitamente la trayectoria del sistema.

Se propone una función:

```math
V(x)=x^TPx
```

con:

```math
P=P^T>0
```

Para que el sistema sea estable, se busca que:

```math
V(x)>0
```

para todo `x ≠ 0`, y que:

```math
\dot{V}(x)<0
```

## Interpretación física

`V(x)` se parece a una energía.

Si la energía siempre disminuye, el sistema tiende al equilibrio.

## Analogía

Una bolita dentro de un tazón:

- Si la energía baja, la bolita termina en el fondo.
- Si la energía sube o no baja, la bolita puede escaparse o no estabilizarse.

---

# 13. Ecuación de Lyapunov

Para el sistema:

```math
\dot{x}=Ax
```

se toma:

```math
V(x)=x^TPx
```

La derivada es:

```math
\dot{V}=x^T(A^TP+PA)x
```

Si queremos que:

```math
\dot{V}=-x^TQx
```

entonces:

```math
A^TP+PA=-Q
```

O equivalentemente:

```math
A^TP+PA+Q=0
```

Esta es la ecuación de Lyapunov.

## Uso

Sirve para verificar estabilidad. Si para un `Q>0` existe una `P>0`, entonces el sistema es estable.

---

# 14. Matriz positiva definida

Una matriz simétrica `P` es positiva definida si:

```math
z^TPz>0 \quad \forall z\neq 0
```

Donde `z` es un vector cualquiera distinto de cero.

## ¿Qué es z?

`z` es un vector de prueba. No es una señal de control ni un estado especial. Sirve para verificar si la matriz produce una cantidad positiva en cualquier dirección.

## Ojo importante

Que una matriz sea positiva definida no significa que todos sus elementos sean positivos.

Significa que la forma cuadrática:

```math
z^TPz
```

es positiva para todo vector no nulo.

## Ejemplo

```math
P=\begin{bmatrix}2 & 0\\0 & 3\end{bmatrix}
```

```math
z=\begin{bmatrix}z_1\\z_2\end{bmatrix}
```

Entonces:

```math
z^TPz=2z_1^2+3z_2^2>0
```

siempre que `z` no sea cero.

---

# 15. Operador vec o stack

El operador `vec` o `stack` toma una matriz y la convierte en un vector apilando sus columnas.

Si:

```math
A=\begin{bmatrix}a_{11} & a_{12}\\a_{21} & a_{22}\end{bmatrix}
```

entonces:

```math
vec(A)=\begin{bmatrix}a_{11}\\a_{21}\\a_{12}\\a_{22}\end{bmatrix}
```

## ¿Para qué sirve?

Sirve para convertir ecuaciones matriciales en ecuaciones vectoriales.

Esto es útil porque muchas herramientas numéricas trabajan mejor con sistemas tipo:

```math
Mx=b
```

---

# 16. Producto de Kronecker

El producto de Kronecker entre dos matrices `A` y `B` se escribe:

```math
A\otimes B
```

Si:

```math
A=\begin{bmatrix}a_{11} & a_{12}\\a_{21} & a_{22}\end{bmatrix}
```

entonces:

```math
A\otimes B=\begin{bmatrix}a_{11}B & a_{12}B\\a_{21}B & a_{22}B\end{bmatrix}
```

## Identidad clave

Una identidad muy usada es:

```math
vec(ABC)=(C^T\otimes A)vec(B)
```

## ¿Para qué sirve en control?

Permite reescribir ecuaciones como Lyapunov de forma lineal.

Por ejemplo:

```math
A^TP+PA+Q=0
```

puede transformarse en una ecuación vectorial usando `vec` y Kronecker.

## Advertencia

En general:

```math
A\otimes B \neq B\otimes A
```

No se debe asumir conmutatividad.

---

# 17. Por qué Kronecker ayuda en Lyapunov, pero no resuelve todo Riccati

La ecuación de Lyapunov es lineal en `P`:

```math
A^TP+PA+Q=0
```

Al usar `vec`, puede escribirse como un sistema lineal:

```math
M\,vec(P)=vec(-Q)
```

Pero Riccati tiene un término cuadrático:

```math
PBR^{-1}B^TP
```

Aunque vectorices, la no linealidad sigue existiendo.

Entonces:

- `vec` y Kronecker ordenan el problema;
- pero no eliminan automáticamente la dificultad de Riccati.

Para eso aparece el enfoque LMI.

---

# 18. LMI: Linear Matrix Inequality

Una LMI es una desigualdad matricial lineal.

Tiene forma:

```math
F(x)=F_0+x_1F_1+x_2F_2+\cdots+x_nF_n>0
```

Donde:

- `F0, F1, ..., Fn` son matrices conocidas.
- `x1, x2, ..., xn` son variables de decisión.
- `F(x)>0` significa que la matriz es positiva definida.

## ¿Qué significa lineal?

Significa que las variables aparecen sin productos entre ellas.

Por ejemplo, esto es lineal:

```math
F(x)=F_0+x_1F_1+x_2F_2
```

Pero esto no es lineal:

```math
F(x)=F_0+x_1x_2F_1
```

porque aparece el producto `x1 x2`.

## ¿Por qué las LMIs son importantes?

Porque muchas condiciones de estabilidad y desempeño pueden expresarse como LMIs.

Además, las LMIs pertenecen a la optimización convexa, lo que permite resolverlas con algoritmos confiables.

---

# 19. LMI de estabilidad por Lyapunov

Para el sistema:

```math
\dot{x}=Ax
```

la estabilidad puede verificarse buscando una matriz:

```math
P=P^T>0
```

que cumpla:

```math
A^TP+PA<0
```

Esto es una LMI en la variable `P`.

## Interpretación

- `P>0`: la energía es positiva.
- `A^TP+PA<0`: la energía disminuye.

Si ambas condiciones se cumplen, el sistema es estable.

---

# 20. Síntesis de control mediante LMI

Para diseñar un controlador:

```math
u=Kx
```

el sistema cerrado es:

```math
\dot{x}=(A+BK)x
```

La condición de Lyapunov sería:

```math
(A+BK)^TP+P(A+BK)<0
```

El problema es que aquí aparece el producto entre incógnitas:

```math
KP
```

Eso no es LMI.

## Cambio de variable

Se usa una sustitución típica:

```math
Y=KP
```

Así se transforma el problema en una forma lineal en las variables `P` y `Y`.

Después de resolver:

```math
K=YP^{-1}
```

## Idea importante

Muchas veces no diseñamos directamente `K`. Diseñamos variables auxiliares que hacen el problema convexo, y al final recuperamos `K`.

---

# 21. Complemento de Schur

El complemento de Schur es una herramienta para convertir ciertas desigualdades no lineales en LMIs por bloques.

Una forma típica es:

```math
Q-SR^{-1}S^T>0
```

Bajo condiciones adecuadas, esto puede escribirse como:

```math
\begin{bmatrix}
Q & S\\
S^T & R
\end{bmatrix}>0
```

## ¿Por qué sirve?

Porque elimina inversas y expresiones no lineales difíciles, convirtiéndolas en condiciones matriciales por bloques.

## Analogía

Es como convertir una ecuación difícil en una tabla de bloques más fácil de revisar por un solver.

---

# 22. Optimización convexa

Un problema convexo tiene una geometría favorable: no tiene mínimos locales falsos separados del mínimo global.

## Analogía

Es como una taza lisa: si buscas el punto más bajo, no te quedas atrapado en huecos secundarios.

En cambio, un problema no convexo se parece a una montaña con muchos valles: puedes caer en un mínimo que no es el mejor.

## Por qué importa

Las LMIs son útiles porque se pueden resolver mediante optimización convexa.

Eso da:

- solución numérica confiable;
- criterios claros de factibilidad;
- posibilidad de incluir restricciones adicionales;
- uso de solvers robustos.

---

# 23. Algoritmos de punto interior

Los algoritmos de punto interior resuelven problemas convexos moviéndose por dentro de la región factible.

## Idea básica

En vez de caminar por el borde de las restricciones, el método se mantiene dentro de la zona permitida y se acerca progresivamente al óptimo.

## Barrera

Se usan funciones de barrera para evitar tocar zonas prohibidas. Por ejemplo, si una restricción exige que una cantidad sea positiva, la barrera castiga acercarse demasiado a cero.

## En LMIs

La restricción típica es:

```math
F(x)>0
```

Los métodos de punto interior buscan mantener `F(x)` positiva definida durante las iteraciones.

## Qué debes saber como estudiante

No necesitas programar desde cero un método de punto interior al inicio. Lo importante es saber que:

- las LMIs son resolubles numéricamente;
- CVXPY, YALMIP, SeDuMi, MOSEK, SDPT3 y otros solvers usan ideas de optimización convexa;
- el solver te dice si el problema es factible y entrega las matrices buscadas.

---

# 24. Control estocástico

Control estocástico significa control de sistemas con incertidumbre aleatoria.

En un sistema determinista:

```math
\dot{x}=Ax+Bu
```

si conoces `x(0)`, puedes predecir la trayectoria ideal.

En un sistema estocástico hay ruido o cambios aleatorios:

```math
\dot{x}=Ax+Bu+w(t)
```

Donde `w(t)` representa perturbaciones o ruido.

## Ejemplos reales

- Sensores con ruido.
- Motores con variación de carga.
- Robots con fricción cambiante.
- Drones con viento.
- Procesos industriales con perturbaciones.

## Idea

No se busca que una trayectoria perfecta se comporte bien, sino que el sistema sea estable y tenga buen desempeño en promedio o bajo incertidumbre.

---

# 25. LQG: LQR + estimador de Kalman

Un problema clásico de control estocástico es LQG:

```text
LQG = LQR + Filtro de Kalman
```

## ¿Por qué se necesita?

LQR asume que conoces todos los estados `x`.

Pero en la vida real muchas veces no mides todos los estados, o los mides con ruido.

Entonces se usa un estimador:

```math
\hat{x}
```

y la ley de control queda:

```math
u=-K\hat{x}
```

## Interpretación

- LQR calcula cómo controlar si conocieras los estados.
- Kalman estima los estados a partir de mediciones ruidosas.
- LQG combina ambos.

---

# 26. Sistemas con saltos de Markov

Un sistema con saltos de Markov puede cambiar aleatoriamente entre varios modos:

```math
\dot{x}=A_i x+B_i u
```

Donde `i` es el modo actual.

Ejemplo:

- Modo 1: operación normal.
- Modo 2: carga alta.
- Modo 3: falla parcial.

El cambio entre modos se modela con una cadena de Markov.

## Por qué son difíciles

Ya no hay una sola ecuación de Riccati, sino varias ecuaciones acopladas, una por cada modo.

Por eso las formulaciones LMI se vuelven útiles.

---

# 27. Estabilidad en media cuadrática

En sistemas estocásticos se usa una noción como:

```math
\mathbb{E}[x^Tx]
```

Donde `E` representa valor esperado.

Un sistema es estable en media cuadrática si, en promedio, la energía del estado disminuye o permanece acotada según la definición usada.

## Analogía

No preguntas:

> ¿Esta trayectoria particular salió bien?

Preguntas:

> ¿En promedio, el sistema se comporta de forma estable frente a muchas realizaciones aleatorias?

---

# 28. Relación entre todos los conceptos

La cadena completa es:

```text
Control clásico
    ↓
Espacio de estados
    ↓
Realimentación de estados u = -Kx
    ↓
LQR: escoger K minimizando error y esfuerzo
    ↓
Riccati: ecuación para encontrar P
    ↓
K = R^{-1}B^T P
    ↓
Lyapunov: base energética para estabilidad
    ↓
vec y Kronecker: herramientas para manipular ecuaciones matriciales
    ↓
LMI: formulación convexa de estabilidad y control
    ↓
Schur: puente para convertir desigualdades difíciles en LMIs
    ↓
Solvers de punto interior
    ↓
Control robusto/estocástico y sistemas más realistas
```

---

# 29. Errores comunes

## Error 1: creer que `F(x)>0` significa elementos positivos

No. Significa matriz positiva definida.

## Error 2: creer que `R` es una resistencia física

No necesariamente. En LQR `R` es una matriz de peso del esfuerzo de control. Puede estar relacionada con energía, pero matemáticamente es una penalización.

## Error 3: escoger `Q` y `R` sin pensar

`Q` y `R` definen el comportamiento del controlador. No son decoración.

## Error 4: olvidar la controlabilidad

Si el sistema no es controlable o estabilizable, no puedes esperar que LQR arregle todo.

## Error 5: asumir que Kronecker vuelve lineal cualquier cosa

Kronecker ayuda a organizar ecuaciones matriciales, pero no elimina automáticamente términos no lineales como los de Riccati.

## Error 6: confundir LQR con PID

PID usa error, integral y derivada de salida.

LQR usa estados y minimiza una función de costo.

---

# 30. Ruta recomendada de estudio

## Semana 1: espacio de estados

- Repasar matrices `A`, `B`, `C`, `D`.
- Simular sistemas con `ss`, `lsim`, `step`.
- Ver polos como valores propios de `A`.

## Semana 2: realimentación de estados

- Diseñar `K` por asignación de polos.
- Ver cómo cambian los polos de `A-BK`.

## Semana 3: LQR

- Escoger `Q` y `R`.
- Resolver Riccati.
- Comparar control suave vs. agresivo.

## Semana 4: Lyapunov

- Verificar estabilidad con `P`.
- Resolver ecuaciones de Lyapunov.

## Semana 5: LMI

- Formular estabilidad como LMI.
- Resolver con CVXPY o YALMIP.

## Semana 6: control estocástico

- Agregar ruido.
- Implementar filtro de Kalman.
- Comparar LQR ideal vs. LQG.

---

# 31. Herramientas recomendadas

## Python

Librerías útiles:

```bash
pip install numpy scipy matplotlib control cvxpy
```

Opcionales:

```bash
pip install slycot osqp ecos scs clarabel
```

Funciones importantes:

- `numpy.array`
- `numpy.linalg.eig`
- `numpy.linalg.inv`
- `scipy.linalg.solve_continuous_are`
- `scipy.linalg.solve_discrete_are`
- `scipy.linalg.solve_continuous_lyapunov`
- `control.ss`
- `control.step_response`
- `control.forced_response`
- `control.lqr`
- `control.place`
- `cvxpy.Variable`
- `cvxpy.Problem`
- `cvxpy.Minimize`
- `cvxpy.bmat`

## MATLAB

Funciones útiles:

- `ss`
- `step`
- `lsim`
- `eig`
- `rank`
- `ctrb`
- `obsv`
- `place`
- `acker`
- `lqr`
- `care`
- `dare`
- `lyap`
- `kalman`
- `lqe`
- `dlqr`

Para LMIs:

- LMI Lab: `setlmis`, `lmivar`, `lmiterm`, `getlmis`, `feasp`, `mincx`.
- YALMIP: `sdpvar`, `optimize`, `sdpsettings`.
- Solvers: SeDuMi, SDPT3, MOSEK.

---

# 32. Ideas de proyectos sencillos

A continuación tienes ideas ordenadas desde básicas hasta avanzadas. Varias pueden hacerse en Python o MATLAB.

---

## Ruta recomendada de 3 proyectos

Si no quieres abrir demasiados frentes, la ruta más eficiente para cubrir casi todo el temario es esta:

### Proyecto A: péndulo invertido sobre carro

Punto de entrada recomendado.

Qué cubre:

- modelado no lineal;
- linealización alrededor del equilibrio;
- espacio de estados;
- controlabilidad y observabilidad;
- `LQR`;
- observador de estados o filtro de Kalman;
- saturación;
- comparación entre modelo lineal y planta no lineal.

Qué lo hace valioso:

- es más exigente que masa-resorte-amortiguador;
- obliga a pensar en estabilidad de un sistema inestable;
- conecta muy bien con Riccati, Lyapunov y LQG.

Plan detallado:

- [Proyecto 1: sistema y arquitectura](docs/specs/algorithms/proyecto-1-pendulo-invertido/proyecto-1-pendulo-invertido-system.md)
- [Proyecto 1: implementación y pruebas](docs/specs/algorithms/proyecto-1-pendulo-invertido/proyecto-1-pendulo-invertido-implementation-plan.md)

### Proyecto B: suspensión activa de un cuarto de vehículo

Segundo proyecto recomendado.

Qué cubre:

- modelado mecánico con perturbación de carretera;
- realimentación multivariable;
- desempeño vs. esfuerzo de control;
- robustez ante incertidumbre paramétrica;
- comparación entre `LQR`, `H∞` o diseño robusto equivalente;
- análisis de sensibilidad y rechazo de perturbaciones.

Qué lo hace valioso:

- introduce el lenguaje de control robusto con una planta físicamente intuitiva;
- permite conectar Lyapunov y LMI con un problema realista.

### Proyecto C: ACC + mantenimiento de carril + cambio de carril

Proyecto final grande y completo.

Qué cubre:

- control longitudinal y lateral;
- restricciones explícitas;
- `MPC`, `adaptive MPC` o `nonlinear MPC`;
- estimación de estados;
- lógica supervisora;
- validación por escenarios.

Qué lo hace valioso:

- es una aplicación industrial moderna;
- te permite integrar la mayor parte del curso en un solo sistema;
- es muy fuerte como proyecto de portafolio o proyecto final.

---

## Proyecto 1: masa-resorte-amortiguador con LQR

### Objetivo

Controlar la posición de una masa usando una fuerza externa.

### Modelo

```math
m\ddot{x}+b\dot{x}+kx=u
```

Estados:

```math
x_1=x
```

```math
x_2=\dot{x}
```

Modelo:

```math
\dot{x}=Ax+Bu
```

```math
A=\begin{bmatrix}0 & 1\\-k/m & -b/m\end{bmatrix}
```

```math
B=\begin{bmatrix}0\\1/m\end{bmatrix}
```

### Conceptos que practica

- Espacio de estados.
- Realimentación `u=-Kx`.
- LQR.
- Selección de `Q` y `R`.
- Comparación de respuesta rápida vs. suave.

### Python

Librerías:

- `numpy`
- `scipy.linalg`
- `control`
- `matplotlib`

Funciones:

- `control.ss(A,B,C,D)`
- `control.lqr(A,B,Q,R)`
- `control.step_response(sys)`
- `control.forced_response(sys,T,U,X0)`
- `numpy.linalg.eig(A-B@K)`

### MATLAB

Funciones:

- `ss(A,B,C,D)`
- `lqr(A,B,Q,R)`
- `eig(A-B*K)`
- `step(sys)`
- `lsim(sys,u,t,x0)`

### Extensión realista

Simular saturación de actuador:

```math
u_{max}=10N
```

Comparar qué pasa cuando el LQR pide más fuerza de la disponible.

---

## Proyecto 2: motor DC controlado por voltaje

### Objetivo

Controlar la velocidad angular de un motor DC.

### Variables típicas

- Velocidad angular `ω`.
- Corriente de armadura `i`.
- Entrada: voltaje `V`.

### Conceptos que practica

- Modelo electromecánico.
- Control de velocidad.
- LQR vs. PID.
- Penalización de corriente o voltaje.

### Python

Librerías:

- `numpy`
- `scipy`
- `control`
- `matplotlib`

Funciones:

- `control.ss`
- `control.lqr`
- `control.step_response`
- `scipy.integrate.solve_ivp`

### MATLAB

Funciones:

- `ss`
- `lqr`
- `step`
- `lsim`
- `pidtune` para comparar con PID.

### Extensión realista

Agregar ruido de medición en velocidad y aplicar un filtro simple o un filtro de Kalman.

---

## Proyecto 3: diseño LQR para un péndulo invertido linealizado

### Objetivo

Mantener un péndulo invertido en posición vertical.

### Conceptos que practica

- Sistema inestable.
- Estados múltiples.
- LQR en un sistema más exigente.
- Elección crítica de `Q`.

### Estados típicos

```math
x=\begin{bmatrix}
\text{posición del carro}\\
\text{velocidad del carro}\\
\text{ángulo del péndulo}\\
\text{velocidad angular}
\end{bmatrix}
```

### Python

Librerías:

- `numpy`
- `control`
- `matplotlib`
- `scipy.integrate`

Funciones:

- `control.lqr`
- `control.ss`
- `control.forced_response`
- `solve_ivp` para simular el modelo no lineal.

### MATLAB

Funciones:

- `lqr`
- `ss`
- `lsim`
- `ode45` para simular el modelo no lineal.

### Extensión realista

Diseñar el LQR con el modelo lineal, pero probarlo en el modelo no lineal.

---

## Proyecto 4: verificar estabilidad con Lyapunov

### Objetivo

Dado un sistema:

```math
\dot{x}=Ax
```

verificar estabilidad resolviendo:

```math
A^TP+PA=-Q
```

### Conceptos que practica

- Función de Lyapunov.
- Matriz positiva definida.
- Valores propios de `P`.
- Energía decreciente.

### Python

Librerías:

- `numpy`
- `scipy.linalg`
- `matplotlib`

Funciones:

- `scipy.linalg.solve_continuous_lyapunov(A.T, -Q)`
- `numpy.linalg.eigvals(P)`
- `scipy.integrate.solve_ivp`

### MATLAB

Funciones:

- `lyap(A',Q)` o `lyap(A', Q)` según convención de signos.
- `eig(P)`
- `ode45`

### Extensión realista

Graficar elipses de energía:

```math
x^TPx=c
```

para visualizar cómo el sistema se mueve hacia el origen.

---

## Proyecto 5: LMI básica de estabilidad

### Objetivo

Encontrar una matriz `P>0` tal que:

```math
A^TP+PA<0
```

### Conceptos que practica

- LMI.
- Positividad definida.
- Factibilidad.
- Solver convexo.

### Python

Librerías:

- `cvxpy`
- `numpy`

Funciones:

- `cvxpy.Variable((n,n), symmetric=True)`
- `P >> eps*np.eye(n)`
- `A.T@P + P@A << -eps*np.eye(n)`
- `cvxpy.Problem(cvxpy.Minimize(0), constraints)`
- `problem.solve(solver="SCS")`

### MATLAB con YALMIP

Funciones:

- `sdpvar(n,n,'symmetric')`
- `optimize([P >= eps*eye(n), A'*P+P*A <= -eps*eye(n)])`
- `value(P)`

### MATLAB con LMI Lab

Funciones:

- `setlmis([])`
- `lmivar`
- `lmiterm`
- `getlmis`
- `feasp`
- `dec2mat`

### Extensión realista

Comparar el resultado de Lyapunov con la estabilidad observada por simulación temporal.

---

## Proyecto 6: diseño de controlador por LMI

### Objetivo

Diseñar `K` para que:

```math
\dot{x}=(A+BK)x
```

sea estable.

### Cambio de variable

Usar:

```math
Y=KP
```

Resolver para `P` y `Y`, luego recuperar:

```math
K=YP^{-1}
```

### Conceptos que practica

- Síntesis por LMI.
- Cambio de variable.
- Recuperación de la ganancia.
- Comparación con asignación de polos o LQR.

### Python

Librerías:

- `cvxpy`
- `numpy`
- `control`

Funciones:

- `cvxpy.Variable`
- `cvxpy.bmat`
- `problem.solve()`
- `numpy.linalg.inv`
- `numpy.linalg.eigvals(A+B@K)`

### MATLAB

Con YALMIP:

- `sdpvar`
- `optimize`
- `value`
- `eig(A+B*K)`

Con MATLAB básico para comparar:

- `place`
- `lqr`

### Extensión realista

Agregar una restricción de decaimiento:

```math
(A+BK)^TP+P(A+BK)+2\alpha P<0
```

Esto fuerza una velocidad mínima de convergencia.

---

## Proyecto 7: comparación PID vs. LQR en un motor DC

### Objetivo

Comparar un PID clásico con un LQR moderno.

### Conceptos que practica

- Control clásico vs. moderno.
- Respuesta temporal.
- Sobreimpulso.
- Tiempo de establecimiento.
- Esfuerzo de control.

### Python

Librerías:

- `control`
- `numpy`
- `matplotlib`

Funciones:

- `control.tf`
- `control.ss`
- `control.feedback`
- `control.step_response`
- `control.lqr`

### MATLAB

Funciones:

- `tf`
- `ss`
- `feedback`
- `step`
- `pidtune`
- `lqr`

### Extensión realista

Comparar energía de control:

```math
\int u^2(t)dt
```

No solo mirar quién responde más rápido.

---

## Proyecto 8: LQG con ruido de sensores

### Objetivo

Controlar un sistema cuando no todos los estados se miden perfectamente.

### Conceptos que practica

- Ruido de proceso.
- Ruido de medición.
- Filtro de Kalman.
- LQR con estados estimados.
- Control estocástico básico.

### Python

Librerías:

- `numpy`
- `scipy`
- `control`
- `matplotlib`

Funciones:

- `control.lqr`
- `scipy.linalg.solve_continuous_are`
- `numpy.random.normal`
- `scipy.integrate.solve_ivp`

Nota: para Kalman en Python, puedes implementarlo manualmente o buscar funciones disponibles según la versión de `python-control`.

### MATLAB

Funciones:

- `lqr`
- `kalman`
- `lqe`
- `ss`
- `lsim`

### Extensión realista

Aplicarlo al motor DC con medición ruidosa de velocidad.

---

## Proyecto 9: sistema de dos tanques acoplados

### Objetivo

Controlar niveles de líquido en dos tanques conectados.

### Conceptos que practica

- Sistema MIMO o SISO según configuración.
- Linealización alrededor de un punto de operación.
- Control LQR.
- Perturbaciones de caudal.

### Python

Librerías:

- `numpy`
- `scipy.integrate`
- `control`
- `matplotlib`

Funciones:

- `solve_ivp`
- `control.lqr`
- `control.ss`
- `control.forced_response`

### MATLAB

Funciones:

- `ode45`
- `ss`
- `lqr`
- `lsim`

### Extensión realista

Agregar límites de bomba:

```math
0\leq u\leq u_{max}
```

Esto permite ver que el LQR ideal no siempre respeta límites físicos.

---

## Proyecto 10: dron simplificado en un eje

### Objetivo

Controlar la altura o el ángulo de inclinación de un dron en un modelo simplificado.

### Conceptos que practica

- Modelo físico simple.
- LQR.
- Ruido de viento como perturbación.
- Saturación de actuadores.

### Python

Librerías:

- `numpy`
- `scipy.integrate`
- `control`
- `matplotlib`

Funciones:

- `control.lqr`
- `solve_ivp`
- `numpy.clip` para saturación.

### MATLAB

Funciones:

- `lqr`
- `ode45`
- `saturation` en Simulink si se usa modelo gráfico.

### Extensión realista

Probar diferentes valores de `R` para representar ahorro de batería.

---

## Proyecto 11: sistema con cambio aleatorio de modo

### Objetivo

Simular un sistema que cambia entre dos dinámicas diferentes.

### Ejemplo

Modo 1:

```math
\dot{x}=A_1x+B_1u
```

Modo 2:

```math
\dot{x}=A_2x+B_2u
```

El modo cambia aleatoriamente.

### Conceptos que practica

- Control estocástico.
- Sistemas con saltos.
- Cadenas de Markov básicas.
- Estabilidad promedio.

### Python

Librerías:

- `numpy`
- `scipy.integrate`
- `matplotlib`

Funciones:

- `numpy.random.choice`
- `solve_ivp`
- `numpy.mean`

### MATLAB

Funciones:

- `rand`
- `randsample`
- `ode45`
- `mean`

### Extensión realista

Un motor con dos cargas posibles: carga normal y carga pesada.

---

## Proyecto 12: LMI robusta con incertidumbre polytópica

### Objetivo

Diseñar un controlador que estabilice varios modelos posibles del mismo sistema.

### Idea

Si el sistema puede ser:

```math
A_1, A_2, A_3
```

se busca una misma matriz `P` que cumpla:

```math
A_i^TP+PA_i<0
```

para todos los modelos.

### Conceptos que practica

- Robustez.
- LMI múltiple.
- Incertidumbre paramétrica.
- Diseño conservador.

### Python

Librerías:

- `cvxpy`
- `numpy`

Funciones:

- `cvxpy.Variable`
- listas de restricciones en un ciclo `for`
- `problem.solve()`

### MATLAB

Con YALMIP:

- `sdpvar`
- `optimize`
- ciclo `for` con restricciones.

### Extensión realista

Masa-resorte-amortiguador con masa incierta:

```math
m\in[m_{min},m_{max}]
```

---

# 33. Proyecto recomendado para empezar

El mejor proyecto inicial es:

## Masa-resorte-amortiguador con LQR + Lyapunov + LMI

Porque con un solo sistema puedes estudiar:

1. Espacio de estados.
2. Polos de `A`.
3. Controlabilidad.
4. LQR.
5. Efecto de `Q` y `R`.
6. Matriz `P` de Riccati.
7. Verificación de estabilidad con Lyapunov.
8. Formulación LMI básica.
9. Simulación temporal.
10. Saturación de actuador.

## Orden de implementación

1. Definir `m`, `b`, `k`.
2. Construir `A` y `B`.
3. Verificar controlabilidad.
4. Elegir `Q` y `R`.
5. Calcular `K` con LQR.
6. Simular sistema abierto.
7. Simular sistema cerrado.
8. Cambiar `Q` y `R`.
9. Graficar estado y señal de control.
10. Verificar estabilidad con Lyapunov.
11. Repetir la estabilidad como LMI.

---

# 34. Plantilla mínima en Python para LQR

```python
import numpy as np
import control as ct
import matplotlib.pyplot as plt

# Parámetros físicos
m = 1.0
b = 0.5
k = 2.0

# Modelo en espacio de estados
A = np.array([[0, 1],
              [-k/m, -b/m]])

B = np.array([[0],
              [1/m]])

C = np.array([[1, 0]])
D = np.array([[0]])

# Pesos LQR
Q = np.diag([10, 1])
R = np.array([[1]])

# Controlador LQR
K, P, eig_cl = ct.lqr(A, B, Q, R)

print("K =", K)
print("P =", P)
print("Polos cerrados =", eig_cl)

# Sistema cerrado: xdot = (A-BK)x
Acl = A - B @ K
sys_cl = ct.ss(Acl, B, C, D)

# Respuesta desde condición inicial
T = np.linspace(0, 10, 500)
X0 = [1, 0]
T, y = ct.initial_response(sys_cl, T, X0)

plt.plot(T, y)
plt.xlabel("Tiempo [s]")
plt.ylabel("Posición [m]")
plt.title("Respuesta con LQR")
plt.grid(True)
plt.show()
```

---

# 35. Plantilla mínima en MATLAB para LQR

```matlab
clear; clc; close all;

% Parámetros físicos
m = 1.0;
b = 0.5;
k = 2.0;

% Modelo en espacio de estados
A = [0 1;
    -k/m -b/m];

B = [0;
     1/m];

C = [1 0];
D = 0;

% Pesos LQR
Q = diag([10 1]);
R = 1;

% Controlador LQR
[K,P,eig_cl] = lqr(A,B,Q,R);

K
P
eig_cl

% Sistema cerrado
Acl = A - B*K;
sys_cl = ss(Acl,B,C,D);

% Respuesta desde condición inicial
x0 = [1;0];
t = 0:0.01:10;
initial(sys_cl,x0,t)
grid on
```

---

# 36. Plantilla mínima en Python para LMI de estabilidad

```python
import numpy as np
import cvxpy as cp

A = np.array([[0, 1],
              [-2, -0.5]])

n = A.shape[0]
eps = 1e-6

P = cp.Variable((n, n), symmetric=True)

constraints = [
    P >> eps*np.eye(n),
    A.T @ P + P @ A << -eps*np.eye(n)
]

problem = cp.Problem(cp.Minimize(0), constraints)
problem.solve(solver=cp.SCS)

print("Estado del problema:", problem.status)
print("P encontrada:")
print(P.value)
```

---

# 37. Plantilla mínima en MATLAB/YALMIP para LMI de estabilidad

```matlab
clear; clc;

A = [0 1;
    -2 -0.5];

n = size(A,1);
eps = 1e-6;

P = sdpvar(n,n,'symmetric');

Constraints = [P >= eps*eye(n), A'*P + P*A <= -eps*eye(n)];
Options = sdpsettings('solver','sedumi','verbose',1);

sol = optimize(Constraints, [], Options);

if sol.problem == 0
    disp('LMI factible. P =')
    disp(value(P))
else
    disp('No se encontró solución factible.')
    sol.info
end
```

---

# 38. Qué deberías dominar después de esta guía

Al terminar, deberías poder explicar:

1. Qué diferencia hay entre función de transferencia y espacio de estados.
2. Qué significa `u=-Kx`.
3. Qué hacen `Q` y `R` en LQR.
4. Qué es `P` y por qué aparece Riccati.
5. Por qué Riccati es no lineal.
6. Qué significa estabilidad por Lyapunov.
7. Qué es una matriz positiva definida.
8. Qué es `z` en `z^TPz>0`.
9. Qué hacen `vec` y Kronecker.
10. Qué es una LMI.
11. Por qué las LMIs se resuelven con optimización convexa.
12. Qué hace un algoritmo de punto interior.
13. Qué cambia cuando el sistema tiene ruido o incertidumbre.
14. Cómo iniciar proyectos prácticos en Python o MATLAB.

---

# 39. Sentencia final honesta

Control moderno puede parecer pesado porque junta matrices, optimización y estabilidad. Pero el hilo lógico es muy claro:

```text
Quiero controlar estados.
Necesito una ganancia K.
LQR me dice cómo escoger K de forma óptima.
Riccati me da la matriz P para construir K.
Lyapunov me explica por qué el sistema es estable.
LMI me permite resolver versiones más generales y difíciles del problema.
Control estocástico aparece cuando la vida real mete ruido, incertidumbre y cambios aleatorios.
```

Si entiendes esa cadena, ya no estás memorizando fórmulas: estás entendiendo la arquitectura del control moderno.
