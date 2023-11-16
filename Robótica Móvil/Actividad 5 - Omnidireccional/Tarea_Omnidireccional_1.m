%Daniel Alejandro Claustro León 217535102
close all
clear all
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
    cla
    R = [-sin(p(3)) cos(p(3)) L;
        -sin(p(3)+120) cos(p(3)+120) L;
        -sin(p(3)+240) cos(p(3)+240) L];

    %Cinemática inversa
    v = R*pp;
    p = p+pp*t;

    %Cinemática directa
    ppd = inv(R)*v;
    pd = p+ppd*t;

    Dibujar_Omnidireccional_3(p,L)
    drawnow

    p_plot(:,i) = p;
    pp_plot(:,i) = pp;

    pd_plot(:,i) = pd;
end 

%Gráfica de la pose del diferencial
figure  
grid on
hold on
plot(t_plot,p_plot(1,:),'m-','LineWidth',2)
plot(t_plot,p_plot(2,:),'b-','LineWidth',2)
plot(t_plot,p_plot(3,:),'g-','LineWidth',2)
legend('X', 'Y', 'Theta')
xlabel('Tiempo')
ylabel('Pose')

%Gráfica de las velocidades 
figure  
grid on
hold on
plot(t_plot,pp_plot(1,:),'m-','LineWidth',2)
plot(t_plot,pp_plot(2,:),'b-','LineWidth',2)
plot(t_plot,pp_plot(3,:),'g-','LineWidth',2)
legend('X.punto', 'Y.punto', 'Theta.punto')
xlabel('Tiempo')
ylabel('Velocidades')

% Gráfica de la trayectoria del móvil - Inversa
figure
grid on
hold on
plot(p_plot(1,:), p_plot(2,:), 'g-','LineWidth',2)
Dibujar_Omnidireccional_3(p,L)

% Gráfica de la trayectoria del móvil - directa
figure
grid on
hold on
plot(pd_plot(1,:), pd_plot(2,:), 'y-','LineWidth',2)
Dibujar_Omnidireccional_3(p,L)

