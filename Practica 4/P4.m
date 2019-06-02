%% Procedimiento A
clc, clear, close all
s = 0:0.1:40;             %Vector s
Gp = 1./((s+1).*(s+1));   %Funcion de transferencia
Kp = Gp(1);               %Aplicacion del teorema de valor final
essp = 1/(1+Kp);          %Error de posicion

Gv = s./((s+1).*(s+1));   %Funcion de transferencia
Kv = Gv(1);               %Aplicacion del teorema de valor final
essv = 1/(Kv);            %Error de velocidad

Ga = s.^2./((s+1).*(s+1));%Funcion de transferencia
Ka = Ga(1);               %Aplicacion del teorema de valor final
essa = 1/(Ka);            %Error de aceleracion

% Comportamiento del sistema en lazo cerrado para ESCALON UNITARIO
s = tf('s');                %Variable complejo
t=0:0.1:10;                 %Vector independiente
[~,tam] = size(t);          %Dimensionamiento
t1 = ones(1,tam);           %Funcion ESCALON UNITARIO
Gp = 1/((s+1)*(s+1));       %Sistema 
Gplc = feedback(Gp,1,-1);   %Sistema en lazo cerrado
[y,t]=lsim(Gplc,t1,t);      %Respuesta al ESCALON UNITARIO
e = t1'-y;                  %Vector de error
figure
plot(t,y)                   %Grfica RESPUESTA AL ESCALON UNITARIO (SALIDA)
hold on
plot(t,t1)                  %Grafica ESCALON UNITARIO (ENTRADA)
plot(t,e);                  %Grafica Error ESCALON UNITARIO (Error)
legend('Respuesta al escalon','Entrada','Error')
title('Respuesta al escalon unitario')
axis([0 10 0 1.1])

% Comportamiento del sistema en lazo cerrado para RAMPA
s = tf('s');                %Variable complejo
t=0:0.1:10;                 %Vector independiente
alpha=2;                    %Pendiente de la funcion RAMPA
ramp=alpha*t;               %Funcion RAMPA
Gv = s/((s+1)*(s+1));
Gvlc = feedback(Gv,1,-1);  %Sistema en lazo cerrado
[y,t]=lsim(Gvlc,ramp,t);      %Respuesta a la RAMPA UNITARIA
e = ramp'-y;                  %Vector de error
figure
plot(t,y)                   %Grfica RESPUESTA A LA RAMPA UNITARIA (SALIDA)
hold on
plot(t,ramp)                  %Grafica RAMPA UNITARIA (ENTRADA)
plot(t,e);                  %Grafica Error RAMPA UNITARIA (Error)
legend('Respuesta a la rampa unitaria','Entrada','Error')
title('Respuesta a la rampa unitario')
axis([0 10 0 10])

% Comportamiento del sistema en lazo cerrado para PARABOLA
s = tf('s');                %Variable complejo
t=0:0.1:10;                 %Vector independiente
parabola=t.^2;               %Funcion PARABOLA
Ga = s^2/((s+1)*(s+1));
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
axis([0 10 0 10])


%% Procedimiento B
clc, clear, close all
s = 0:0.1:40;           %Vector s
Gp = 1./(s.*((s+1).*(s+1)));  %Funcion de transferencia
Kp = Gp(1);              %Aplicacion del teorema de valor final
essp = 1/(1+Kp);        %Error de posicion

Gv = 1./((s+1).*(s+1));  %Funcion de transferencia
Kv = Gv(1);              %Aplicacion del teorema de valor final
essv = 1/(Kv);           %Error de velocidad

Ga = s./(((s+1).*(s+1)));  %Funcion de transferencia
Ka = Ga(1);              %Aplicacion del teorema de valor final
essa = 1/(Ka);         %Error de aceleracion

% Comportamiento del sistema en lazo cerrado para ESCALON UNITARIO
s = tf('s');                %Variable complejo
t=0:0.1:20;                 %Vector independiente
[~,tam] = size(t);          %Dimensionamiento
t1 = ones(1,tam);           %Funcion ESCALON UNITARIO
Gp = 1/(s.*(s+1)*(s+1));       %Sistema 
Gplc = feedback(Gp,1,-1);   %Sistema en lazo cerrado
[y,t]=lsim(Gplc,t1,t);      %Respuesta al ESCALON UNITARIO
e = t1'-y;                  %Vector de error
figure
plot(t,y)                   %Grfica RESPUESTA AL ESCALON UNITARIO (SALIDA)
hold on
plot(t,t1)                  %Grafica ESCALON UNITARIO (ENTRADA)
plot(t,e);                  %Grafica Error ESCALON UNITARIO (Error)
legend('Respuesta al escalon','Entrada','Error')
title('Respuesta al escalon unitario')
axis([0 20 -0.5 1.5])

% Comportamiento del sistema en lazo cerrado para RAMPA
s = tf('s');                %Variable complejo
t=0:0.1:20;                 %Vector independiente
alpha=2;                    %Pendiente de la funcion RAMPA
ramp=alpha*t;               %Funcion RAMPA
Gv = 1/(((s+1)*(s+1)));
Gvlc = feedback(Gv,1,-1);  %Sistema en lazo cerrado
[y,t]=lsim(Gvlc,ramp,t);      %Respuesta a la RAMPA UNITARIA
e = ramp'-y;                  %Vector de error
figure
plot(t,y)                   %Grfica RESPUESTA A LA RAMPA UNITARIA (SALIDA)
hold on
plot(t,ramp)                  %Grafica RAMPA UNITARIA (ENTRADA)
plot(t,e);                  %Grafica Error RAMPA UNITARIA (Error)
legend('Respuesta a la rampa unitaria','Entrada','Error')
title('Respuesta a la rampa unitario')
axis([0 10 0 15])

% Comportamiento del sistema en lazo cerrado para PARABOLA
s = tf('s');                %Variable complejo
t=0:0.1:20;                 %Vector independiente
parabola=t.^2;               %Funcion PARABOLA
Ga = s/(((s+1)*(s+1)));
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
axis([0 10 0 15])

%% Procedimiento C
clc, clear, close all
s = 0:0.1:40;           %Vector s
Gp = (30.*(s+3).*(s+4).*(s+8))./((s.^2).*((s+19).*(s+15)));  %Funcion de transferencia
Kp = Gp(1);              %Aplicacion del teorema de valor final
essp = 1/(1+Kp);        %Error de posicion

Gv = (30.*(s+3).*(s+4).*(s+8))./((s).*((s+19).*(s+15)));  %Funcion de transferencia
Kv = Gv(1);              %Aplicacion del teorema de valor final
essv = 1/(Kv);           %Error de velocidad

Ga = (30.*(s+3).*(s+4).*(s+8))./((1).*((s+19).*(s+15)));  %Funcion de transferencia
Ka = Ga(1);              %Aplicacion del teorema de valor final
essa = 1/(Ka);         %Error de aceleracion

% Comportamiento del sistema en lazo cerrado para ESCALON UNITARIO
s = tf('s');                %Variable complejo
t=0:0.1:20;                 %Vector independiente
[~,tam] = size(t);          %Dimensionamiento
t1 = ones(1,tam);           %Funcion ESCALON UNITARIO
Gp = (30*(s+3)*(s+4)*(s+8))/((s^2)*((s+19)*(s+15)));       %Sistema 
Gplc = feedback(Gp,1,-1);   %Sistema en lazo cerrado
[y,t]=lsim(Gplc,t1,t);      %Respuesta al ESCALON UNITARIO
e = t1'-y;                  %Vector de error
figure
plot(t,y)                   %Grfica RESPUESTA AL ESCALON UNITARIO (SALIDA)
hold on
plot(t,t1)                  %Grafica ESCALON UNITARIO (ENTRADA)
plot(t,e);                  %Grafica Error ESCALON UNITARIO (Error)
legend('Respuesta al escalon','Entrada','Error')
title('Respuesta al escalon unitario')
axis([0 20 -0.5 1.5])

% Comportamiento del sistema en lazo cerrado para RAMPA
s = tf('s');                %Variable complejo
t=0:0.1:20;                 %Vector independiente
alpha=2;                    %Pendiente de la funcion RAMPA
ramp=alpha*t;               %Funcion RAMPA
Gv = (30*(s+3)*(s+4)*(s+8))/((s)*((s+19)*(s+15))); 
Gvlc = feedback(Gv,1,-1);  %Sistema en lazo cerrado
[y,t]=lsim(Gvlc,ramp,t);      %Respuesta a la RAMPA UNITARIA
e = ramp'-y;                  %Vector de error
figure
plot(t,y)                   %Grfica RESPUESTA A LA RAMPA UNITARIA (SALIDA)
hold on
plot(t,ramp)                  %Grafica RAMPA UNITARIA (ENTRADA)
plot(t,e);                  %Grafica Error RAMPA UNITARIA (Error)
legend('Respuesta a la rampa unitaria','Entrada','Error')
title('Respuesta a la rampa unitario')
axis([0 10 0 15])

% Comportamiento del sistema en lazo cerrado para PARABOLA
s = tf('s');                %Variable complejo
t=0:0.1:20;                 %Vector independiente
parabola=t.^2;               %Funcion PARABOLA
Ga = (30*(s+3)*(s+4)*(s+8))/((1)*((s+19)*(s+15)));
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
axis([0 10 0 15])

%% Procedimiento D
clc, clear, close all
s = 0:0.1:40;           %Vector s
Gp = (30.*(s+3).*(s+2).*(s+1))./((s.^3).*((s+5).*(s+5)));  %Funcion de transferencia
Kp = Gp(1);              %Aplicacion del teorema de valor final
essp = 1/(1+Kp);        %Error de posicion

Gv = (30.*(s+3).*(s+2).*(s+1))./((s.^2).*((s+5).*(s+5)));  %Funcion de transferencia
Kv = Gv(1);              %Aplicacion del teorema de valor final
essv = 1/(Kv);           %Error de velocidad

Ga = (30.*(s+3).*(s+2).*(s+1))./((s).*((s+5).*(s+5)));  %Funcion de transferencia
Ka = Ga(1);              %Aplicacion del teorema de valor final
essa = 1/(Ka);         %Error de aceleracion

% Comportamiento del sistema en lazo cerrado para ESCALON UNITARIO
s = tf('s');                %Variable complejo
t=0:0.1:20;                 %Vector independiente
[~,tam] = size(t);          %Dimensionamiento
t1 = ones(1,tam);           %Funcion ESCALON UNITARIO
Gp = (30*(s+3)*(s+2)*(s+1))/((s^3)*((s+5)*(s+5)));       %Sistema 
Gplc = feedback(Gp,1,-1);   %Sistema en lazo cerrado
[y,t]=lsim(Gplc,t1,t);      %Respuesta al ESCALON UNITARIO
e = t1'-y;                  %Vector de error
figure
plot(t,y)                   %Grfica RESPUESTA AL ESCALON UNITARIO (SALIDA)
hold on
plot(t,t1)                  %Grafica ESCALON UNITARIO (ENTRADA)
plot(t,e);                  %Grafica Error ESCALON UNITARIO (Error)
legend('Respuesta al escalon','Entrada','Error')
title('Respuesta al escalon unitario')
axis([0 20 -0.5 1.5])

% Comportamiento del sistema en lazo cerrado para RAMPA
s = tf('s');                %Variable complejo
t=0:0.1:20;                 %Vector independiente
alpha=2;                    %Pendiente de la funcion RAMPA
ramp=alpha*t;               %Funcion RAMPA
Gv = (30*(s+3)*(s+2)*(s+1))/((s^2)*((s+5)*(s+5)));   
Gvlc = feedback(Gv,1,-1);  %Sistema en lazo cerrado
[y,t]=lsim(Gvlc,ramp,t);      %Respuesta a la RAMPA UNITARIA
e = ramp'-y;                  %Vector de error
figure
plot(t,y)                   %Grfica RESPUESTA A LA RAMPA UNITARIA (SALIDA)
hold on
plot(t,ramp)                  %Grafica RAMPA UNITARIA (ENTRADA)
plot(t,e);                  %Grafica Error RAMPA UNITARIA (Error)
legend('Respuesta a la rampa unitaria','Entrada','Error')
title('Respuesta a la rampa unitario')
axis([0 10 0 15])

% Comportamiento del sistema en lazo cerrado para PARABOLA
s = tf('s');                %Variable complejo
t=0:0.01:20;                 %Vector independiente
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
axis([0 20 0 15])
