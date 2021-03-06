clc, clear, close all

% A.- Genere el sistema dado. 
clc, clear, close all
s = tf('s');
for a=0:4:20
    G = (1.5/(s+a));
    step(G)
    hold on
    axis([0 1.2 0 0.5])
    legend('a = 0','a = 4','a = 8', 'a = 12', 'a = 16', 'a = 20');
end
G1 = (1.5/(s));
G2 = (1.5/(s+4));
G3 = (1.5/(s+8));
G4 = (1.5/(s+12));
G5 = (1.5/(s+16));
G6 = (1.5/(s+20));
sistemas = [G1 G2 G3 G4 G5 G6];
%Calculo de tr y ts (tiempo de crecimiento y tiempo de asentamiento)
for j=1:length(sistemas)
    Gf=tf([cell2mat(sistemas(j).Numerator)],[cell2mat(sistemas(j).Denominator)]);
    [Wn,Z] = damp(Gf);
    Wd = Wn*sqrt(1-Z^2);
    tr(j) = (pi - acos(Z))/Wd;
    ts(j) =  4/(Z*Wn);
end
clear Gf

%% B.- Genere las funciones de transferencia siguientes
clc, clear, close all
s = tf('s');
vs = [20 10 5 0];

for i=1:length(vs)
    G=(25)/(s^2+vs(i)*s+25);
    step(G);
    hold on
    axis([0 4 0 2.2])
    legend('b = 20','b = 10','b = 5', 'b = 0');
    [Omegan,Zeta] = damp(G);
    Omegan = Omegan(1);
    Zeta = Zeta(1);
    Wn(i) = Omegan;
    Z(i) = Zeta;
    Wd = Omegan*sqrt(1-Zeta^2);
    tr(i) = (pi-acos(Zeta))/Wd;
    ts(i) = 4/(Zeta*Omegan);
    tp(i) = pi/(Wd);
    SP(i) = 100*(exp(-((Zeta*Omegan*pi)/Wd)));
    clear Omegan Zeta Wd
end
G1=(25)/(s^2+20*s+25);
pG1 = pole(G1);
figure,pzmap(G1)
title('Sobreamortiguado')
legend('P1=-18.6, P2=-1.3')
G2=(25)/(s^2+10*s+25);
pG2 = pole(G2);
figure,pzmap(G2)
title('Criticamente amortiguado')
legend('P1,P2=-5')
G3=(25)/(s^2+5*s+25);
pG3 = pole(G3);
figure,pzmap(G3)
title('Subamortiguado')
legend('P1=-2.5+4.33i,P2=-2.5-4.33i')
G4=(25)/(s^2+25);
pG4 = pole(G4);
figure,pzmap(G4)
title('Criticamente Estable')
legend('P1=+5i,P2=-5i')

%% C. Crear la funcion de transferencia. 
clc, clear, close all
s = tf('s');
Wn = 1.5;
Z = 0:0.1:1;
for i=1:length(Z)
    G=(Wn^2)/(s^2+2*Wn*Z(i)*s+Wn^2);
    step(G);
    hold on
    axis([0 15 0 2.1])
    %legend('Z=0','Z=0.1','Z=0.2','Z=0.3','Z=0.4','Z=0.5','Z=0.6','Z=0.7','Z=0.8','Z=0.9','Z=1.0')
    [Omega,Zeta] = damp(G);
    Zeta = Zeta(1);
    Omegan= Wn;
    Z(i) = Zeta;
    Wd = Omegan*sqrt(1-Zeta^2);
    tr(i) = (pi-acos(Zeta))/Wd;
    ts(i) = 4/(Zeta*Omegan);
    tp(i) = pi/(Wd);
    SP(i) = 100*(exp(-((Zeta*Omegan*pi)/Wd)));
    clear Omegan Zeta Wd
end

%% D.- Crear la funcion de transferencia. 
clc, clear, close all
s= tf('s');
Z = 0.5;
Wn = 0:2:10;
Wn(1) = 1;
figure
subplot(2,1,1)
for i=1:length(Wn)
    G=(Wn(i)^2)/(s^2+2*Wn(i)*Z*s+Wn(i)^2);
    step(G);
    hold on
    axis([0 7 0 1.3])
    legend('\omega_{n} = 1','\omega_{n} = 2','\omega_{n} = 4','\omega_{n} = 6','\omega_{n} = 8','\omega_{n} = 10')
    Zeta = Z;
    Omegan= Wn(i);
    Wd = Omegan*sqrt(1-Zeta^2);
    tr(i) = (pi-acos(Zeta))/Wd;
    ts(i) = 4/(Zeta*Omegan);
    tp(i) = pi/(Wd);
    SP(i) = 100*(exp(-((Zeta*Omegan*pi)/Wd)));
    polos(i,:) = pole(G);
end
hold off

subplot(2,1,2)
for i=1:length(Wn)
    G=(Wn(i)^2)/(s^2+2*Wn(i)*Z*s+Wn(i)^2);
    pzmap(G);
    hold on
end
hold off

%% E.- Crear la funcion de transferencia
clc, clear, close all
s = tf('s');
a = 5:5:30;
Wd = 5;             % Wd= Wn*sqrt(1-Z^2)
figure
subplot(2,1,1)
for i=1:length(a)
    syms Omegan Zeta
    eqns = [Omegan*sqrt(1-Zeta^2) == 5,Zeta*Omegan == a(i)];
    vars = [Omegan Zeta];
    [solv, solu] = solve(eqns,vars);
    Wn(i) = double(solv);
    Z(i) = double(solu);
    G = (double(solv)^2)/(s^2+2*double(solv)*double(solu)*s+double(solv)^2);
    tr(i) = (pi-acos(double(solu)))/Wd;
    ts(i) = 4/(double(solu)*double(solv));
    tp(i) = pi/(Wd);
    SP(i) = 100*(exp(-((double(solu)*double(solv)*pi)/Wd)));
    step(G)
    hold on
    axis([0 1.2 0 1.1])
    legend('\zeta\omega_{n} = 5','\zeta\omega_{n} = 10','\zeta\omega_{n} = 15','\zeta\omega_{n} = 20','\zeta\omega_{n} = 25','\zeta\omega_{n} = 30')
    clear Omegan Zeta
end
hold off

subplot(2,1,2)
for j=1:length(Wn)
    Gs=(Wn(j)^2)/(s^2+2*Wn(j)*Z(j)*s+Wn(j)^2);
    pzmap(Gs);
    hold on
    axis([-31 0 -6 6])
end
hold off

%% F.- Crear la funcion de transferencia
clc, clear, close all
s = tf('s');
a = 0:5:30;
a(1) = 1;
             % Wd= Wn*sqrt(1-Z^2)
figure
subplot(2,1,1)
for i=1:length(a)
    syms Omegan Zeta
    eqns = [Omegan*sqrt(1-Zeta^2) == a(i),Zeta*Omegan == -1];
    vars = [Omegan Zeta];
    [solv, solu] = solve(eqns,vars);
    Wn(i) = double(solv);
    Z(i) = double(solu);
    Wd = double(solv)*sqrt(1-double(solu)^2);
    G = (double(solv)^2)/(s^2+2*double(solv)*double(solu)*s+double(solv)^2);
    tr(i) = (pi-acos(double(solu)))/Wd;
    ts(i) = 4/(double(solu)*double(solv));
    tp(i) = pi/(Wd);
    SP(i) = 100*(exp(-((double(solu)*double(solv)*pi)/Wd)));
    step(G)
    hold on
    axis ([53 55 -0.1*10^25 0.1*10^25])
    legend('\omega_{d} = 1','\omega_{d} = 5','\omega_{d} = 10','\omega_{d} = 15','\omega_{d} = 20','\omega_{d} = 25','\omega_{d} = 30')
    clear Omegan Zeta
end
hold off

subplot(2,1,2)
for j=1:length(Wn)
    Gs=(Wn(j)^2)/(s^2+2*Wn(j)*Z(j)*s+Wn(j)^2);
    pzmap(Gs);
    hold on
end
hold off

%% II.- Polos Dominantes
clc, clear, close all
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
axis([0 700 0 1.2])
%Graficar respuesta en el tiempo
subplot(2,2,2)
ezplot(y,[0,10])
axis([0 5 0 1.2])

%Respuesta individuales
subplot(2,2,[3,4])
t2 = 0:0.01:5;
y1 = 1-exp(-15.*t2)/4;
plot(t2,y1);
hold on;

y2 = 1-(cos(2.*t2).*exp(-t2))/2;
plot(t2,y2)

y3 = 1-exp(-10.*t2)/4;
plot(t2,y3);

ezplot(y,[0,5]);
axis([0 4 0 1.25])
title('Respuestas individuales')

%% IIA
clc, clear, close all
s = tf('s');

T1 = (24.542)/((s^2+4*s+24.542));
T2 = (73.626)/((s+3)*(s^2+4*s+24.542));
T3 = (245.42)/((s+10)*(s^2+4*s+24.542));
T4 = (490.84)/((s+20)*(s^2+4*s+24.542));
T5 = (736.26)/((s+30)*(s^2+4*s+24.542));
figure
step(T1);
hold on
axis([0 3 0 1.3])
legend('T1','T2','T3','T4','T5')
step(T2);
step(T3);
step(T4);
step(T5);

figure
pzmap(T1);
hold on
pzmap(T2);
pzmap(T3);
pzmap(T4);
pzmap(T5);
legend('T1','T2','T3','T4','T5')

%% Procedimiento B
clc, clear, close all
% % Polos Adicionales
s = tf('s');
Gp = 4/(s^2+2*s+4); %Lazo Abierto
% % Calculo de función en lazo cerrado
Gplc = feedback(Gp,1,-1);    % Retroalimentación unitaria.

figure
subplot(2,1,1)
step(Gp)
title('Sistema en lazo abierto')
subplot(2,1,2)
step(Gplc)
title('Sistema en lazo cerrado.')


% % Lazo Abierto con ceros adicionales.
 k = [0 0.5 1 3 15 -3 -15];
 figure
 for i=1:length(k)
     Gpca =  (4*(s+k(i)))/(k(i)*(s^2+2*s+4));
     step(Gpca);
     hold on
     legend('k=0','k=0.5','k=1','k=3','k=15','k=-3','k=-15')
 end
 title('Sistema lazo abierto con ceros adicionales')
hold off
% Lazo cerrado con ceros adicionales
figure
for i=1:length(k)
     Gldz =  (4*(s+k(i)))/(k(i)*(s^2+2*s+4));
     step(feedback(Gldz,1,-1),10);
     hold on
     legend('k=0','k=0.5','k=1','k=3','k=15','k=-3','k=-15')
 end
 title('Sistema lazo cerrado con ceros adicionales')
 hold off
%  
 % Sea la funcion de transferencia en lazo abierto. 
 clc, clear
% % Polos Adicionales
 s = tf('s');
 Gp = 4.4*(s+1)/(s^2+2*s+4); %Lazo Abierto
% % Calculo de función en lazo cerrado
Gplc = feedback(Gp,1,-1);    % Retroalimentación unitaria. 
figure
subplot(2,1,1)
step(Gp)
title('Sistema en lazo abierto')
subplot(2,1,2)
step(Gplc)
title('Sistema en lazo cerrado.')

% % Lazo Abierto con ceros adicionales.
 k = [0 1.1 1 3 15 25];
 figure
 for i=1:length(k)
     Gpca =  (4.4*((s+1)*k(i))/((s+k(i))*(s^2+2*s+4)));
     step(Gpca);
     hold on
     legend('k=0','k=1.1','k=1','k=3','k=15','k=25')
 end
title('Sistema lazo abierto con ceros adicionales')
hold off

figure
for i=1:length(k)
     Gldz =  (4.4*((s+1)*k(i)))/((s+k(i))*(s^2+2*s+4));
     step(feedback(Gldz,1,-1),10);
     hold on
     legend('k=0','k=1.1','k=1','k=3','k=15','k=25')
 end
 title('Polos adicionales en lazo cerrado')
 hold off