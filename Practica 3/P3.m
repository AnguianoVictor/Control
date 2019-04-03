clc, clear, close all

%% II.- Polos Dominantes
s = tf('s');
G = (6.75*s^3+102.5*s^2+318.75*s+750)/((s^2+2*s+5)*(s+15)*(s+10));
G = minreal(G);             % Reduccion de terminos. 
% Respuesta al escalon unitario
I_G = step(G);
% En forma analitica
% Y(s) = G(s)*R(s)
Y = G*1/s;
% Transformada inversa de Y(s)
% Definiendo Y(s), se limpia la variable s y se declara como simbolica. 
clear s
syms s t
G = (6.75*s^3+102.5*s^2+318.75*s+750)/((s^2+2*s+5)*(s+15)*(s+10));
Y = G*1/s;
y = ilaplace(Y);
% Graficar y(t)
subplot(2,2,1)
plot(I_G), title('Respuesta al escalon usando step')
subplot(2,2,2)
ezplot(y,[0,10])
axis([0 10 0 12])
subplot(2,2,[3,4])
t2 = 0:0.001:10;
y1 = 1-exp(-15.*t2)/4;
plot(t2,y1);
hold on;
y2 = 1-(cos(2.*t2).*exp(-t2))/2;
plot(t2,y2)
y3 = 1-exp(-10.*t2)/4;
plot(t2,y3);
plot(I_G)
legend('P1=-15','P2,3=-1+-2','P4=-10','Resp 2do orden');

%% IIA
clc, clear, close all
s = tf('s');

T1 = (24.542)/((s^2+4*s+24.542));
T2 = (73.626)/((s+3)*(s^2+4*s+24.542));
T3 = (245.42)/((s+10)*(s^2+4*s+24.542));
T4 = (490.84)/((s+20)*(s^2+4*s+24.542));
T5 = (736.26)/((s+30)*(s^2+4*s+24.542));
figure,step(T1)
figure,step(T2)
figure,step(T3)
figure,step(T4)
figure,step(T5)
