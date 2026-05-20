# Plan de trabajo: Taller 1 - Control robusto H_inf en UAV

Fuente principal:

- `drive/TDC/02. TAREAS/T1/taller1_2022.pdf`

Recursos de apoyo revisados:

- `drive/TDC/02. TAREAS/T1/modelo_lin.mat`
- `drive/TDC/01. NOTAS DE CLASE/H_inf/Sofrony_c.pdf`
- `drive/TDC/01. NOTAS DE CLASE/H_inf/Presentation - POLES_ZEROS.pdf`
- `drive/TDC/04. Otros Recursos/H_inf/Ejemplo/Hinf/C_Hinf.m`
- `drive/TDC/04. Otros Recursos/UAV_SIM_AEM/Simulation_Lin/`
- `proyecto-1-pendulo-invertido/`

## 1. Objetivo del taller

El taller pide disenar y comparar dos controladores para un UAV:

1. Un controlador clasico tipo PID organizado como `SAS/CAS`.
2. Un controlador robusto `H_inf`.

La comparacion debe hacerse con analisis frecuencial y con simulacion en MATLAB/Simulink. El objetivo de control es seguimiento de referencias tipo set-point para los angulos `theta` y `phi` en el rango `[-40, 40] deg`, rechazando ruido de medicion e incertidumbre o perturbaciones de entrada.

## 2. Decisiones tomadas antes de implementar

Estas decisiones quedan congeladas como punto de partida:

1. Se habilitara Robust Control Toolbox y cualquier otra toolbox necesaria.

   Ruta principal: usar MATLAB con `hinfsyn`, `mixsyn`, `augw`, `makeweight`, `connect`, `sumblk`, `sigma` y, si estan disponibles, herramientas de analisis robusto.

2. No se hara todavia la version en Python.

   Primero se completa el taller en MATLAB/Simulink. Despues de tener visto bueno sobre la version MATLAB, se abrira una solicitud separada para portar o replicar el flujo en Python.

3. Si el taller no exige explicitamente un diseno MIMO desde el inicio, se avanzara de simple a complejo.

   Se empieza con disenos por eje y por canales dominantes. Luego se valida el acoplamiento con el modelo completo. Si la sintesis robusta queda estable y clara, se agrega una version MIMO como extension o comparacion.

4. La simulacion final obligatoria usara el modelo lineal acoplado.

   El modelo desacoplado se usa para diseno. El modelo completo `linmodel` se usa para comprobar desempeno realista con acoplamiento. La simulacion no lineal de los recursos UAV queda como extension opcional si el tiempo y las dependencias no introducen ruido innecesario.

5. El entregable principal sera un archivo Markdown explicando todo el proceso.

   El informe debe explicar teoria, ecuaciones, decisiones de diseno, pesos H_inf, interconexion, sensibilidad, simulaciones, resultados y como revisar/probar/verificar los archivos. Debe entrar a las matematicas necesarias sin desviarse a un "rabbit hole" teorico.

## 3. Datos confirmados de la planta

El archivo `modelo_lin.mat` contiene:

- `linmodel`: modelo completo acoplado.
- `latmod`: modelo lateral/direccional.
- `longmod`: modelo longitudinal.

Datos observados con MATLAB:

```text
linmodel:
  Estados: 13
  Entradas: 8
  Salidas: 14

latmod:
  Estados: 5
  Entradas: aileron, rudder
  Salidas: beta, p, r, phi, psi

longmod:
  Estados: 6
  Entradas: elevator, throttle
  Salidas: V, alpha, q, theta, h, ax, az
```

Canales candidatos:

```text
Longitudinal:
  theta <- elevator
  q     <- elevator

Lateral/direccional:
  phi <- aileron
  p   <- aileron
  r   <- rudder
```

El taller pide controlar principalmente rotaciones y velocidades angulares:

- Longitudinal: `theta` y `q` con `elevator`.
- Lateral/direccional: `phi`, `psi`, `p`, `r` con `aileron` y `rudder`.

Para seguimiento de referencia, el foco sera:

- `theta_ref -> theta`
- `phi_ref -> phi`

Las velocidades angulares `q`, `p`, `r` se usaran para amortiguamiento y analisis.

## 4. Especificaciones tecnicas del taller

El enunciado indica:

```text
Seguimiento:
  theta, phi en [-40, 40] deg

Ancho de banda:
  minimo indicado: 8 Hz

Perturbaciones de entrada:
  hasta 6 Hz

Ruido de medicion:
  lateral/direccional: potencia promedio 0.001
  longitudinal:        potencia promedio 0.0001

Esfuerzo de control:
  |u| <= 30 deg

Comportamiento deseado:
  ejes criticamente amortiguados
  respuesta lo mas rapida posible
  esfuerzo de control pequeno
  ejes lo mas desacoplados posible
```

Conversion importante:

```matlab
u_max = deg2rad(30);
wb = 2*pi*8;
wp = 2*pi*6;
```

## 5. Estructura de archivos propuesta

Se seguira el estilo modular usado en `proyecto-1-pendulo-invertido`.

```text
taller1/
|-- PLAN_TALLER1_HINF.md
|-- main_taller1.m
|-- parametros_taller1.m
|-- cargar_modelo_uav.m
|-- seleccionar_canales_uav.m
|-- analisis_planta_uav.m
|-- diseno_pid_sas_cas.m
|-- diseno_hinf_taller1.m
|-- construir_planta_generalizada_hinf.m
|-- analisis_sensibilidades.m
|-- simulacion_taller1.m
|-- crear_graficas_taller1.m
|-- init_taller1_simulink.m
|-- build_taller1_simulink.m
|-- taller1_uav.slx
|-- figures/
`-- reporte_taller1.md
```

Responsabilidad de cada archivo:

- `main_taller1.m`: ejecuta todo el flujo reproducible.
- `parametros_taller1.m`: guarda especificaciones, unidades, limites, pesos iniciales y casos de simulacion.
- `cargar_modelo_uav.m`: carga `modelo_lin.mat` y normaliza nombres de entradas/salidas.
- `seleccionar_canales_uav.m`: crea plantas SISO/SIMO/MIMO candidatas para diseno.
- `analisis_planta_uav.m`: calcula polos, ceros, `dcgain`, `sigma`, controlabilidad y observaciones de acoplamiento.
- `diseno_pid_sas_cas.m`: implementa SAS/CAS clasico.
- `diseno_hinf_taller1.m`: sintetiza el controlador H_inf usando Robust Control Toolbox.
- `construir_planta_generalizada_hinf.m`: arma la interconexion con pesos `W1`, `W2`, `W3`.
- `analisis_sensibilidades.m`: calcula y grafica `S`, `T`, `KS` para PID y H_inf.
- `simulacion_taller1.m`: ejecuta casos temporales con ruido, perturbacion y saturacion.
- `crear_graficas_taller1.m`: exporta figuras reproducibles.
- `init_taller1_simulink.m`: prepara variables para Simulink.
- `build_taller1_simulink.m`: genera o actualiza `taller1_uav.slx`.
- `reporte_taller1.md`: documento final con teoria, desarrollo, resultados y verificacion.

## 6. Teoria minima que debe aparecer en el informe

### 6.1 Norma H_inf

Para un sistema estable `G(s)`, la norma H_inf mide la mayor amplificacion inducida en frecuencia:

```math
||G||_\infty = sup_\omega \bar{\sigma}(G(j\omega))
```

En SISO esto coincide con el pico del Bode de magnitud. En MIMO se usa el maximo valor singular `sigma_bar`, porque la ganancia depende tambien de la direccion del vector de entrada.

Esta es la razon por la cual el taller pide usar `sigma` en MATLAB.

### 6.2 Lazo abierto, sensibilidad y sensibilidad complementaria

Para una planta `G` y un controlador `K`:

```math
L = GK
```

La sensibilidad de salida es:

```math
S = (I + GK)^{-1}
```

La sensibilidad complementaria es:

```math
T = GK(I + GK)^{-1}
```

Y se cumple:

```math
S + T = I
```

Interpretacion:

- `S` pequeno a bajas frecuencias mejora seguimiento y rechazo de perturbaciones lentas.
- `T` pequeno a altas frecuencias ayuda a rechazar ruido de medicion.
- `K*S` pequeno limita accion de control y ayuda frente a incertidumbre aditiva.

### 6.3 Compromiso fundamental

Como:

```math
S + T = I
```

no se puede hacer `S` y `T` pequenos en todas las frecuencias al mismo tiempo. Por eso se busca:

- baja frecuencia: `S` pequeno;
- alta frecuencia: `T` pequeno;
- esfuerzo de control: `K*S` moderado.

### 6.4 Problema H_inf de sensibilidad mixta

El diseno H_inf se puede formular como:

```math
min_K || Fl(P,K) ||_\infty
```

donde `P` es la planta generalizada y `Fl(P,K)` es la transformacion lineal fraccional inferior que conecta la planta con el controlador.

Para sensibilidad mixta, el objetivo tipico es:

```math
min_K
||
[ W1*S
  W2*K*S
  W3*T ]
||_\infty
= gamma
```

Donde:

- `W1` penaliza error de seguimiento y perturbaciones.
- `W2` penaliza esfuerzo de control.
- `W3` penaliza ruido de medicion y alta frecuencia.

La condicion deseada es que:

```math
|| [ W1*S ; W2*K*S ; W3*T ] ||_\infty < 1
```

En la practica, se acepta el menor `gamma` viable y luego se revisa desempeno temporal y saturacion.

### 6.5 Pesos iniciales

Se usaran pesos iniciales inspirados en las notas y el ejemplo `C_Hinf.m`.

Forma base:

```math
W1(s) = (s/M1 + wb)/(s + A1*wb)
```

`W1` debe ser grande a baja frecuencia para forzar `S` pequeno. Por tanto, `1/W1` define la frontera admisible para `S`.

Forma para control:

```math
W2(s) = constante o filtro suave
```

`W2` crece si el controlador pide demasiado esfuerzo.

Forma para ruido/robustez:

```math
W3(s) = (s/M3 + wh)/(s + A3*wh)
```

`W3` penaliza `T` en alta frecuencia.

Valores iniciales tentativos:

```matlab
s = tf('s');
wb = 2*pi*8;
wp = 2*pi*6;

W1 = makeweight(10, wb, 0.01);
W2 = 0.05;
W3 = makeweight(0.01, wp, 10);
```

Estos valores no son finales. Se ajustan segun `sigma(S)`, `sigma(T)`, `sigma(K*S)`, saturacion y respuesta temporal.

### 6.6 Robustez e incertidumbre

El enunciado menciona incertidumbre aditiva inversa. Una forma de expresarla es:

```math
G_delta = (I - G0*Delta_a)^{-1}G0
```

La interpretacion con ganancia pequena consiste en encontrar el bloque de transferencia que ve la incertidumbre y exigir que el producto de ganancias sea menor que uno:

```math
||M||_\infty ||Delta||_\infty < 1
```

En las notas de clase, para incertidumbre aditiva aparece naturalmente `K*S` como funcion critica. Por eso el taller pide revisar `K(s)S(s)`.

## 7. Metodologia de implementacion

### Fase 0. Preparar entorno

Objetivo:

- Confirmar toolboxes.
- Confirmar rutas.
- Cargar modelos.

Comandos MATLAB esperados:

```matlab
ver
which hinfsyn
which mixsyn
which augw
which makeweight
```

Criterio de avance:

- `hinfsyn`, `mixsyn`, `augw` y `makeweight` deben aparecer disponibles.

### Fase 1. Cargar y analizar planta

Objetivo:

- Cargar `linmodel`, `latmod`, `longmod`.
- Verificar nombres de estados, entradas y salidas.
- Calcular polos, ceros y valores singulares.

Funciones MATLAB:

```matlab
load
pole
zero
tzero
dcgain
sigma
rank
ctrb
obsv
```

Entregables:

- Tabla de estados, entradas y salidas.
- Polos abiertos.
- Ceros de transmision.
- Comentario de ceros no minimos o restricciones de desempeno.
- Graficas `sigma` de plantas relevantes.

### Fase 2. Definir canales de diseno

Objetivo:

- Partir simple.
- Disenar por eje.
- Luego validar con modelo acoplado.

Canales iniciales:

```matlab
G_theta = longmod('theta','elevator');
G_phi   = latmod('phi','aileron');
G_q     = longmod('q','elevator');
G_p     = latmod('p','aileron');
G_r     = latmod('r','rudder');
```

Despues:

```matlab
G_ang = linmodel({'theta','phi'}, {'elevator','aileron'});
```

Criterio:

- Si el diseno SISO por ejes cumple, se mantiene como base.
- Si hay acoplamiento fuerte, se agrega diseno MIMO 2x2 para `theta/phi`.

### Fase 3. Diseno PID/SAS-CAS

Arquitectura clasica:

```text
SAS:
  realimentacion de velocidades angulares q, p, r

CAS:
  PI sobre errores de angulo theta_ref - theta y phi_ref - phi
```

Forma conceptual:

```math
u_elevator = PI_theta(theta_ref - theta) - Kq*q
```

```math
u_aileron = PI_phi(phi_ref - phi) - Kp*p
```

```math
u_rudder = -Kr*r
```

Se debe revisar el signo real de cada canal con `dcgain`, respuesta escalon y polaridad de realimentacion.

Entregables:

- Ganancias SAS.
- Ganancias CAS.
- Respuesta temporal.
- `S`, `T`, `K*S`.
- Esfuerzo de control con saturacion.

### Fase 4. Diseno H_inf SISO por eje

Objetivo:

- Obtener un primer controlador robusto claro y verificable.

Ruta:

```matlab
P = augw(G, W1, W2, W3);
[K, CL, gamma] = hinfsyn(P, nmeas, ncon);
```

o:

```matlab
[K, CL, gamma] = mixsyn(G, W1, W2, W3);
```

Se hara para:

```text
theta/elevator
phi/aileron
```

Luego se combinan controladores por bloques para simular sobre `linmodel`.

Entregables:

- Pesos usados.
- Controlador obtenido.
- `gamma`.
- Orden del controlador.
- Polos del controlador.
- `S`, `T`, `K*S`.
- Comparacion contra `1/W1`, `1/W2`, `1/W3`.

### Fase 5. Diseno H_inf MIMO como extension si aplica

Objetivo:

- Mejorar desacople si el diseno por ejes no basta.

Modelo candidato:

```matlab
G_mimo = linmodel({'theta','phi'}, {'elevator','aileron'});
```

Tambien puede evaluarse:

```matlab
G_lat_mimo = latmod({'phi','r'}, {'aileron','rudder'});
```

Criterio:

- Solo se avanza a MIMO si el diseno SISO deja acoplamiento relevante o si el informe queda mas fuerte con una comparacion multivariable.
- Si el MIMO se vuelve inestable, de alto orden o poco interpretable, se deja como analisis y se entrega el diseno SISO validado en planta acoplada.

### Fase 6. Analisis de sensibilidad

Para cada controlador:

```matlab
L = G*K;
S = feedback(eye(size(L)), L);
T = feedback(L, eye(size(L)));
KS = K*S;
```

Revisar:

```matlab
sigma(S)
sigma(T)
sigma(KS)
norm(S, inf)
norm(T, inf)
norm(KS, inf)
```

En MIMO, `sigma` es obligatorio porque muestra valores singulares y no solo magnitud canal a canal.

Entregables:

- Figura `S` PID vs H_inf.
- Figura `T` PID vs H_inf.
- Figura `KS` PID vs H_inf.
- Tabla de normas H_inf aproximadas.
- Comentario de desempeno nominal, estabilidad robusta y desempeno robusto.

### Fase 7. Simulacion final MATLAB/Simulink

El modelo `taller1_uav.slx` debe incluir:

- Generador de referencias `theta_ref` y `phi_ref`.
- Selector PID/H_inf.
- Controlador PID/SAS-CAS.
- Controlador H_inf.
- Saturacion de actuadores `+-30 deg`.
- Ruido blanco de medicion.
- Perturbacion de entrada hasta `6 Hz`.
- Planta acoplada `linmodel`.
- Logging de referencia, salida, error y control.

Casos minimos:

```text
1. theta_ref =  10 deg, phi_ref = 0
2. theta_ref = -10 deg, phi_ref = 0
3. theta_ref = 0, phi_ref =  10 deg
4. theta_ref = 0, phi_ref = -10 deg
5. theta_ref =  30 o 40 deg, phi_ref = 0
6. theta_ref = 0, phi_ref =  30 o 40 deg
7. theta_ref y phi_ref simultaneos
8. ruido de medicion activado
9. perturbacion de entrada activada
10. comparacion PID vs H_inf
```

Metricas:

- Error maximo.
- Error estacionario.
- Tiempo de establecimiento.
- Sobreimpulso.
- Maximo valor de control.
- Porcentaje de tiempo saturado.
- Sensibilidad a ruido.
- Acoplamiento entre `theta` y `phi`.

## 8. Figuras esperadas

Minimo:

- `figures/planta_sigma.png`
- `figures/planta_polos_ceros.png`
- `figures/pid_sensibilidades.png`
- `figures/hinf_sensibilidades.png`
- `figures/comparacion_s_pid_hinf.png`
- `figures/comparacion_t_pid_hinf.png`
- `figures/comparacion_ks_pid_hinf.png`
- `figures/seguimiento_theta_pid_hinf.png`
- `figures/seguimiento_phi_pid_hinf.png`
- `figures/control_theta_pid_hinf.png`
- `figures/control_phi_pid_hinf.png`
- `figures/ruido_medicion_pid_hinf.png`
- `figures/perturbacion_entrada_pid_hinf.png`
- `figures/simulacion_acoplada_pid_hinf.png`

## 9. Informe final esperado

Archivo:

```text
taller1/reporte_taller1.md
```

Estructura sugerida:

1. Objetivo.
2. Interpretacion del enunciado.
3. Planta UAV y modelos disponibles.
4. Convenciones, unidades y senales.
5. Analisis de planta: polos, ceros, valores singulares.
6. Diseno clasico PID/SAS-CAS.
7. Teoria H_inf necesaria.
8. Planta generalizada.
9. Seleccion de pesos `W1`, `W2`, `W3`.
10. Sintesis H_inf.
11. Analisis de `S`, `T`, `K*S`.
12. Simulacion MATLAB/Simulink.
13. Comparacion PID vs H_inf.
14. Conclusiones.
15. Como reproducir, probar y verificar.

## 10. Como revisar, probar y verificar los archivos

### 10.1 Revision rapida de entorno

Desde MATLAB:

```matlab
cd('/home/sergio/Escritorio/tdc/taller1')
ver
which hinfsyn
which mixsyn
which augw
which makeweight
```

Debe confirmarse que Robust Control Toolbox esta activo.

### 10.2 Ejecutar flujo completo

```matlab
cd('/home/sergio/Escritorio/tdc/taller1')
main_taller1
```

El script debe:

- cargar modelos;
- disenar PID/SAS-CAS;
- disenar H_inf;
- ejecutar analisis frecuencial;
- correr simulaciones;
- exportar figuras;
- mostrar resumen numerico.

### 10.3 Verificar modelo Simulink

```matlab
cd('/home/sergio/Escritorio/tdc/taller1')
build_taller1_simulink
open_system('taller1_uav')
sim('taller1_uav')
```

Verificar:

- bloques conectados correctamente;
- saturacion en `+-30 deg`;
- ruido conectado a mediciones, no a referencias;
- perturbacion conectada a entrada de planta;
- logging disponible en workspace.

### 10.4 Verificar resultados numericos

Se debe comprobar:

```matlab
isstable(CL_pid)
isstable(CL_hinf)
norm(S_pid, inf)
norm(T_pid, inf)
norm(KS_pid, inf)
norm(S_hinf, inf)
norm(T_hinf, inf)
norm(KS_hinf, inf)
```

Para H_inf:

```matlab
gamma
```

debe ser finito y razonable. Idealmente cercano o menor que `1`, pero el criterio final depende de la factibilidad de pesos y saturacion.

### 10.5 Verificar graficas

Revisar la carpeta:

```text
taller1/figures/
```

Debe contener las figuras del analisis y la simulacion. Si falta alguna, el flujo no esta completamente reproducible.

### 10.6 Verificacion conceptual

Preguntas que el informe debe responder:

- Que planta se uso para diseno?
- Que planta se uso para simulacion?
- Cuales son los canales de entrada/salida?
- Que polos y ceros limitan el desempeno?
- Por que `S` debe ser pequeno a baja frecuencia?
- Por que `T` debe ser pequeno a alta frecuencia?
- Que significa `K*S` fisicamente?
- Como se escogieron `W1`, `W2`, `W3`?
- Que valor de `gamma` se obtuvo?
- El controlador respeta `+-30 deg`?
- Cual controlador rechaza mejor ruido?
- Cual controlador usa menos esfuerzo?
- Hay acoplamiento entre `theta` y `phi`?
- Que pasa cerca de `40 deg`?

## 11. Criterios de aceptacion

El taller se considera completo cuando:

- El flujo MATLAB corre desde `main_taller1.m` sin pasos manuales ocultos.
- El PID/SAS-CAS produce seguimiento estable.
- El H_inf se sintetiza con Robust Control Toolbox.
- Se reportan `S`, `T`, `K*S` para ambos controladores.
- Se usa `sigma` para analisis de valores singulares.
- La simulacion final incluye ruido, perturbacion y saturacion.
- La planta acoplada `linmodel` se usa para validar.
- Las figuras se exportan automaticamente.
- `reporte_taller1.md` explica teoria, ecuaciones, diseno, resultados y verificacion.
- El usuario puede reproducir todo siguiendo los comandos del informe.

## 12. Riesgos y mitigaciones

Riesgo: Robust Control Toolbox no queda activo.

Mitigacion:

- No avanzar a Python todavia por decision del usuario.
- Primero resolver instalacion/habilitacion de toolbox.

Riesgo: pesos H_inf demasiado exigentes.

Mitigacion:

- Empezar con pesos suaves.
- Ajustar hasta obtener `gamma` finito.
- No exigir simultaneamente seguimiento muy rapido, poco control y alto rechazo de ruido si la planta no lo permite.

Riesgo: controlador H_inf de orden alto.

Mitigacion:

- Documentar orden.
- Evaluar `minreal`.
- Considerar reduccion de orden solo si no degrada las graficas `S`, `T`, `K*S`.

Riesgo: saturacion rompe el desempeno ideal.

Mitigacion:

- La sintesis H_inf es lineal, pero la simulacion debe incluir saturacion.
- Ajustar `W2` para penalizar esfuerzo de control.
- Reportar claramente cuando el actuador satura.

Riesgo: el diseno por ejes no desacopla suficiente.

Mitigacion:

- Validar siempre en `linmodel`.
- Si el acoplamiento es fuerte, agregar diseno MIMO 2x2 como fase adicional.

## 13. Siguiente paso

El siguiente paso tecnico, despues de habilitar Robust Control Toolbox, es implementar:

```text
parametros_taller1.m
cargar_modelo_uav.m
seleccionar_canales_uav.m
analisis_planta_uav.m
```

Con eso se congela la interfaz de senales y se evita disenar controladores sobre canales mal interpretados.
