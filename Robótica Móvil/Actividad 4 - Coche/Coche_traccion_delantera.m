%% Coche tracción delantera%%
% Robótica Móvil%
% Julio Alexis González Villa%
clear all
close all
clc

%Valores de variables y posiciones iniciales
d = 0.3;
vs = 0.3;
ws = -0.4;
p = [0 0 0 0]';
pp = [0 0 0 0]';
t = 0.01;
s = 10;
N = s/t;

%Variables para poder graficar
p_plot = zeros(4,N);
pp_plot = zeros(4,N);
t_plot = t:t:s;

for i=1:N
 
    pp(1) = vs*cos(p(4))*cos(p(3));
    pp(2) = vs*cos(p(4))*sin(p(3));
    pp(3) = (vs/d)*sin(p(4));
    pp(4) = ws;

    p = p+pp*t; 

%     cla
%     Dibujar_Coche(p,d)
%     drawnow

    p_plot(:,i) = p;
    pp_plot(:,i) = pp;
end

%Gráfica de la posición del robot
figure  
f1 = subplot(2,2,1);
grid on
hold on
plot(f1,t_plot,p_plot(1,:),'m-','LineWidth',2)
plot(f1,t_plot,p_plot(2,:),'b-','LineWidth',2)
plot(f1,t_plot,p_plot(3,:),'g-','LineWidth',2)
plot(f1,t_plot,p_plot(4,:),'y-','LineWidth',2)
legend('x', 'y', 'theta', 'alpha')
xlabel('Tiempo')
ylabel('Pose')

%Gráfica de las velocidades 
f2 = subplot(2,2,2); 
grid on
hold on
plot(f2,t_plot,pp_plot(1,:),'m-','LineWidth',2)
plot(f2,t_plot,pp_plot(2,:),'b-','LineWidth',2)
plot(f2,t_plot,pp_plot(3,:),'g-','LineWidth',2)
plot(f2,t_plot,pp_plot(4,:),'y-','LineWidth',2)
legend('x dot', 'y dot', 'theta dot', 'alpha dot')
xlabel('Tiempo')
ylabel('Velocidades')

% Gráfica de la trayectoria del diferencial
f3 = subplot(2,2,3);
grid on
hold on
plot(f3,p_plot(1,:), p_plot(2,:), 'r-','LineWidth',2)
Dibujar_Coche(p,d)

