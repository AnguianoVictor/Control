clc, clear, close all

s = tf('s');                %Variable complejo
t=0:0.1:20;                 %Vector independiente
parabola=(1*t).^2;               %Funcion PARABOLA
Ga = (30*(s+3)*(s+2)*(s+1))/((s)*((s+5)*(s+5)));   
Galc = feedback(Ga,1,-1);   %Sistema en lazo cerrado
[y,t]=lsim(Galc,parabola,t);%Respuesta a la PARABOLA UNITARIA
e = parabola'-y;            %Vector de error
figure
plot(t,y)                   %Grfica RESPUESTA AL PARABOLA UNITARIO (SALIDA)
hold on
plot(t,parabola)            %Grafica PARABOLA UNITARIO (ENTRADA)
plot(t,e);                  %Grafica Error PARABOLA UNITARIO (Error)
legend('Respuesta a la parabola unitaria','Entrada','Error')
title('Respuesta a la parabola unitario')
axis([0 7 0 15])