%% Procedimiento A
clc, clear, close all

H = tf([1 5],[1 5 6 1]);    %Sistema a simular
pp = pole(H);               %Polos del sistema. 
dt = 0.2;                   %Tamaño de muestreo
t = 0:dt:30;                %Tiempo de muestre
y = step(H,t);              %Respuesta al escalon unitario
dy = diff(y)/dt;            %Primera derivada para encontrar el punto de inflexión
[m,p] = max(dy);            %Se encuentra el valor máximo y el valor en el tiempo de dicho máximo
yi = y(p);                  %Valor del máximo
ti = t(p);                  %Tiempo en que ocurre el máximo
L = ti-yi/m;                %Retardo
Tau = (y(end)-yi)/m+ti-L;   %Constante de tiempo
plot(t,y,'b',[0 L L+Tau t(end)],[0 0 y(end) y(end)],'k');

%Calculo de las constantes del controlador PID
Kp = 1.2*(Tau/L);
Ki = Kp/2*L;
Kd = Kp*(0.5*L);

%% Procedimiento B
clc, clear, close all

H = tf([1 10],[1 5 6 1]);    %Sistema a simular
pp = pole(H);               %Polos del sistema. 
dt = 0.2;                   %Tamaño de muestreo
t = 0:dt:30;                %Tiempo de muestre
y = step(H,t);              %Respuesta al escalon unitario
dy = diff(y)/dt;            %Primera derivada para encontrar el punto de inflexión
[m,p] = max(dy);            %Se encuentra el valor máximo y el valor en el tiempo de dicho máximo
yi = y(p);                  %Valor del máximo
ti = t(p);                  %Tiempo en que ocurre el máximo
L = ti-yi/m;                %Retardo
Tau = (y(end)-yi)/m+ti-L;   %Constante de tiempo
plot(t,y,'b',[0 L L+Tau t(end)],[0 0 y(end) y(end)],'k');

%Calculo de las constantes del controlador PID
Kp = 1.2*(Tau/L);
Ki = Kp/(2*L);
Kd = Kp*(0.5*L);

%% Suma de sistema PID y planta
clc, clear, close all
PID = tf([53.4177 187.565 1],[12.83 0]);
Planta = tf([1 10],[1 5 6 1]);

Gc = series(PID,Planta);
G = feedback(Gc,1,-1);


