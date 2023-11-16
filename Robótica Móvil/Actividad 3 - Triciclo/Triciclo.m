%% Triciclo %%
% Robótica Móvil%
% Julio Alexis González Villa%
close all
clear all
clc

%Velocidad
vs = 0.5;
alpha = pi/2;

%Parámetros
d = 0.15;
R = 0.1;
t = 0.05;
S = 10;
I = S/t;

%Condiciones iniciales
p = [0.0 0.0 0.0]';
pp = [0.0 0.0 0.0]';

%Registro de posición, velocidad y tiempo (Gráficas)
p_plot = zeros(3,I);
pp_plot = zeros(3,I);
t_plot = t:t:10;


for i=1:I
  
    pp(1) = vs*cos(alpha)*cos(p(3));
    pp(2) = vs*cos(alpha)*sin(p(3));
    pp(3) = (vs/d)*sin(alpha);

    p = p+pp*t; 

    cla
    Dibujar_Triciclo(p,alpha,d)
    drawnow

    p_plot(:,i) = p;
    pp_plot(:,i) = pp;
end

%Gráfica de la posición del triciclo
figure
f1 = subplot(2,2,1);
grid on
hold on
plot(f1,t_plot,p_plot(1,:),'g-','LineWidth',2)
plot(f1,t_plot,p_plot(2,:),'y-','LineWidth',2)
plot(f1,t_plot,p_plot(3,:),'m-','LineWidth',2)
legend('x', 'y', 'theta')
xlabel('Tiempo')
ylabel('Posición')

%Gráfica de las velocidades 
f2 = subplot(2,2,2);
grid on
hold on
plot(f2,t_plot,pp_plot(1,:),'g-','LineWidth',2)
plot(f2,t_plot,pp_plot(2,:),'y-','LineWidth',2)
plot(f2,t_plot,pp_plot(3,:),'m-','LineWidth',2)
legend('x dot', 'y dot', 'theta dot')
xlabel('Tiempo')
ylabel('Velocidades')

% Trayectoria del triciclo
f3 = subplot(2,2,3);
grid on
hold on
plot(f3,p_plot(1,:), p_plot(2,:), 'r-','LineWidth',2)
Dibujar_Triciclo(p,alpha,d)




