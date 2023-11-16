%% Coche tracción delantera%%
% Robótica Móvil%
% Julio Alexis González Villa%
close all
clear 
clc

%Valores de las variables
L = 0.25;
t = 0.01;
s = 5;
N = s/t;
p = [0.0 0.0 0.0]';
pp = [0.2 0.2 0.8]';
v = zeros(3,N);

%Variables para poder graficar
p_plot = zeros(3,N);
pp_plot = zeros(3,N);
t_plot = t:t:5;
pd_plot = zeros(3,N);
ppd_plot = zeros(3,N);

for i=1:N
    
    R = [-sin(p(3)) cos(p(3)) L;
        -sin(p(3)+120) cos(p(3)+120) L;
        -sin(p(3)+240) cos(p(3)+240) L];

    %Cinemática inversa
    v = R*pp;
    p = p+pp*t;

    %Cinemática directa
    ppd = inv(R)*v;
    pd = p+ppd*t;

%     cla
%     Dibujar_Omnidireccional_3(p,L)
%     drawnow

    p_plot(:,i) = p;
    pp_plot(:,i) = pp;

    pd_plot(:,i) = pd;
end 


%Gráfica de la posición
figure 
f1 = subplot(2,2,1);
grid on
hold on
plot(f1,t_plot,p_plot(1,:),'r-','LineWidth',2)
plot(f1,t_plot,p_plot(2,:),'b-','LineWidth',2)
plot(f1,t_plot,p_plot(3,:),'g-','LineWidth',2)
legend('x', 'y', 'theta')
xlabel('Tiempo')
ylabel('Posición')

%Gráfica de las velocidades 
f2 = subplot(2,2,2); 
grid on
hold on
plot(f2,t_plot,pp_plot(1,:),'r-','LineWidth',2)
plot(f2,t_plot,pp_plot(2,:),'b-','LineWidth',2)
plot(f2,t_plot,pp_plot(3,:),'g-','LineWidth',2)
legend('x dot', 'y dot', 'theta dot')
xlabel('Tiempo')
ylabel('Velocidades')

% Gráfica de la trayectoria inversa
f3 = subplot(2,2,3);
title cinemática inversa
grid on
hold on
plot(f3,p_plot(1,:), p_plot(2,:), 'y-','LineWidth',2)
Dibujar_Omnidireccional_3(p,L)

% Gráfica de la trayectoria directa
f4 = subplot(2,2,4);
title cinemática directa
grid on
hold on
plot(f4,pd_plot(1,:), pd_plot(2,:), 'm-','LineWidth',2)
Dibujar_Omnidireccional_3(p,L)

