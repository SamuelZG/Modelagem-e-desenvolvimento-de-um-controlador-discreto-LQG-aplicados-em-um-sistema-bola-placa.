
close all
%clear all
format short g


%% Variaveis do sistema
dm = 0.0174; %m    distance from motor shaft to pole
dc = 0.1; %m    distance between center pole and side pole
fc = 0.017;%Ns/m  roll friction coefficient 0.001045  fc = 0.02
%Eta = 1.00;%Damping ratio of the Servo motor 1.0087  Eta = 1.00
%Wn = 18;%rad/s Servo natural Frequency 18.915        Wn = 18
Eta = 1.570142671;%Damping ratio of the Servo motor 1.0087  Eta = 1.00
Wn = 23.49468025;
g = 9.81;%m/s^2 gravitational acceleration
K = dc/dm;
%L = 0.193;
% Ball Type    |  Radius |  Mass
% Hollow steel |  0.016  |  0.013
% Massive Iron |  0.01   |  0.046
% Ping Pong    |  0.02   |  0.001
% Marble       |  0.007  |  0.0049

rb = 15.86/2000;%m   radius of the ball 7,5mm 0.02m
mb = 16.7/1000;%kg  mass of the ball 16,7g 0.0028Kg

AngMax = deg2rad(6);
conv = -AngMax/0.25;

A = [ 0 1 0 0; 0 (-5/7)*(fc/mb) (-5/7)*g 0; 0 0 0 1; 0 0 -(Wn^2) -2*Eta*Wn];
B = [ 0 ; 0 ; 0 ; Wn^2];
C = [ 1 0 0 0];
D = zeros(1,1);


stname = {'Posicao', 'Velocidade', 'Angulo','Velocidade Angular'};
sys = ss(A,B,C,'stname',stname); %Sistema Continuo

Q = diag([64, 4, 52, 1]);
R = 10;
K = lqr(A,B,Q,R);

AA = [A-B*K];
BB = [B*K(1)];

CC = C;

sys2 = ss(AA,BB,CC);
x0 = [0.1;0;0;0];
[y,t,x] = initial(sys2, x0,5);

u = -(K(1)*x(:,1)+K(2)*x(:,2)+K(3)*x(:,3)+K(4)*x(:,4));%Ação de Controle

figure
subplot(1,2,1)
plotyy(t,rad2deg(u),t,x(:,1))
title("acao e angulo Continuo")
legend("acao","posicao")
ylim([-3 3])
xlim([0 2])
grid on
subplot(4,2,2)
plot(t,x(:,1))
title("Posicao")
grid on
subplot(4,2,4)
plot(t,rad2deg(x(:,2)))
grid on
title("Velocidade")
subplot(4,2,6)
plot(t,x(:,3))
grid on
title("Angulo")
subplot(4,2,8)
plot(t,x(:,4))
grid on
title("Vel.Angular")
%
%

%discreto
T = 0.05; %Tempo de aquisiçãoo
sysD = c2d(sys,T); %Sistema Discreto
Ad = sysD.a;
Bd = sysD.b;
Cd = sysD.c;
F = Q;
N = 10000;
%R = 1;
for k=1:N
  W = R +Bd'*F*Bd;
  P = F- F*Bd*inv(W)*Bd'*F;
  F = Ad'*P*Ad + Q;
end
K = inv(W)*Bd'*F*Ad

AA = [Ad-Bd*K];
BB = [Bd*K(1)];
CC = C;

sysDC = ss(AA,BB,CC,0,T,'stname',stname); %Sistema Discreto com Controle

[y,t,x] = initial(sys2, x0,5);
u = -(K(1)*x(:,1)+K(2)*x(:,2)+K(3)*x(:,3)+K(4)*x(:,4));%Açãoo de Controle

figure
subplot(1,2,1)
plotyy(t,rad2deg(u),t,x(:,1))
title("acao e angulo Discreto")
legend("acao","posicao")
ylim([-3 3])
xlim([0 2])
grid on
subplot(4,2,2)
plot(t,x(:,1))
title("Posicao")
grid on
subplot(4,2,4)
plot(t,rad2deg(x(:,2)))
grid on
title("Velocidade")
subplot(4,2,6)
plot(t,x(:,3))
grid on
title("Angulo")
subplot(4,2,8)
plot(t,x(:,4))
grid on
title("Vel.Angular")
%
%% Discreto implementado com FOR
N = 5/T;
n = 0:N-1;

x1 = [0.1;0;0;0];
u1 = 0;
lim = 6;

for i =2:N
  x1(:,i) = Ad*x1(:,i-1) + Bd*u1(i-1);
  u1(i) = -(K(1)*x1(1,i)+K(2)*x1(2,i)+K(3)*x1(3,i)+K(4)*x1(4,i));
  if(rad2deg(u1(i))>lim)
  u1(i) = deg2rad(lim);
  end
  if(rad2deg(u1(i))<-lim)
  u1(i) = deg2rad(-lim);
  end
end;

figure
subplot(1,2,1)
plotyy(n*T,rad2deg(u1),n*T,x1(1,:))
title("acao e angulo Discreto com FOR")
legend("acao","posicao")
ylim([-3 3])
xlim([0 2])
grid on
subplot(4,2,2)
plot(n*T,x1(1,:))
title("Posicao")
grid on
subplot(4,2,4)
plot(n*T,rad2deg(x1(2,:)))
grid on
title("Velocidade")
subplot(4,2,6)
plot(n*T,x1(3,:))
grid on
title("Angulo")
subplot(4,2,8)
plot(n*T,x1(4,:))
grid on
title("Vel.Angular")
%
%
%%Implementaçãoo do observador de ordem reduzida
Rv = 1;
Rw = 10;%diag([1, 1, 2000, 2000]);
N = 5000;
M =eye(4);
G_ext = zeros(4,N);
Adm = Ad;
Bdm = Bd;
Cdm = Cd;
for k=1:N
    G = M * Cdm' * inv(Cdm * M * Cdm' + Rv);%% Calculo do ganho de Kalman
    P = M - G * Cdm * M;
    M = Adm * P * Adm' + Bdm * Rw * Bdm';
    G_ext(:,k) = G;
end
G

tempo = 5;
xO = zeros(4,tempo/T);
x1 = [0.1;0;0;0];
uO = zeros(tempo/T,1);

for i = 2:(tempo/T+1)
  %planta real
  uO(i-1) = -(K(1)*xO(1,i-1)+K(2)*xO(2,i-1)+K(3)*xO(3,i-1)+K(4)*xO(4,i-1));
  if(rad2deg(uO(i-1))>lim)
    uO(i-1) = deg2rad(lim);
  end
  if(rad2deg(uO(i-1))<-lim)
    uO(i-1) = deg2rad(-lim);
  end
  x1(:,i) = Ad*x1(:,i-1) + Bd*uO(i-1);
  yObs(:,i) = Cd*x1(:,i); %% sensores
  %% Simulador
   %Saturaçãoo do Motor
  erro_y(:,i) = yObs(:,i) - Cdm * Adm * xO(:,i-1);
  xO(:,i) = Adm*xO(:,i-1) + [Bdm - G * Cdm * Bdm] * uO(i-1) + G*erro_y(:,i);
end

n = 1:(tempo/T+1);

figure
subplot(1,2,1)
plotyy((n:tempo/T)*T,rad2deg(uO),n*T,xO(1,:))
title("acao e angulo Discreto com FOR e Observador")
legend("acao","posicao")
ylim([-8 8])
xlim([0 3])
grid on
subplot(4,2,2)
plot(n*T,x1(1,:),n*T,xO(1,:))
%legend("Planta","Observavel")
title("Posicao")
grid on
subplot(4,2,4)
plot(n*T,rad2deg(x1(2,:)),n*T,rad2deg(xO(2,:)))
%legend("Planta","Observavel")
grid on
title("Velocidade")
subplot(4,2,6)
plot(n*T,x1(3,:),n*T,xO(3,:))
%legend("Planta","Observavel")
grid on
title("Angulo")
subplot(4,2,8)
plot(n*T,x1(4,:),n*T,xO(4,:))
%legend("Planta","Observavel")
grid on
title("Vel.Angular")
figure
step(-sysDC)
