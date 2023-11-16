%% Control Coche tracción delantera%%
% Robótica Móvil%
% Julio Alexis González Villa%
clear all
close all
clc

%% Tipo coche tracción delantera

%Valores de variables y posiciones iniciales
d = 0.3;
p = [0 0 0 0]';
pp = [0 0 0 0]';
t = 0.01;
s = 10;
N = s/t;

%Ganancias 
Kv = 2.5;
K_alpha = 2.0;

%Trayectoria deseada
td = [1.5 1.5]';

%Variables para poder graficar
p_plot = zeros(4,N);
pp_plot = zeros(4,N);
t_plot = t:t:s;
x_plot = [];
xd_plot = [];

for i=1:N
  
    ev = sqrt((td(1)-p(1))^2+(td(2)-p(2))^2);
    ew = atan2(td(2)-p(2),td(1)-p(1))-(p(3)+p(4));
    ew = atan2(sin(ew),cos(ew));

    vs = Kv*ev;
    ws = K_alpha*ew;

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
    x_plot = [x_plot p(1:2)];
    xd_plot = [xd_plot td];
end

%Gráfica de la pose del diferencial
figure  
grid on
hold on
plot(t_plot,p_plot(1,:),'m-','LineWidth',2)
plot(t_plot,p_plot(2,:),'b-','LineWidth',2)
plot(t_plot,p_plot(3,:),'g-','LineWidth',2)
plot(t_plot,p_plot(4,:),'y-','LineWidth',2)
legend('X', 'Y', 'Theta', 'Alpha')
xlabel('Tiempo')
ylabel('Posición')

%Gráfica de las velocidades 
figure  
grid on
hold on
plot(t_plot,pp_plot(1,:),'m-','LineWidth',2)
plot(t_plot,pp_plot(2,:),'b-','LineWidth',2)
plot(t_plot,pp_plot(3,:),'g-','LineWidth',2)
plot(t_plot,pp_plot(4,:),'y-','LineWidth',2)
legend('X.punto', 'Y.punto', 'Theta.punto', 'Alpha.punto')
xlabel('Tiempo')
ylabel('Velocidad')

% Gráfica de la trayectoria del diferencial
figure
grid on
hold on
plot(p_plot(1,:), p_plot(2,:), 'g-','LineWidth',2)
plot(td(1), td(2), 'rp')
Dibujar_Coche(p,d)

%Gráfica trayectoria real vs deseada
figure
hold on
grid on 
plot(t_plot, x_plot, 'r-', 'LineWidth', 2)
plot(t_plot, xd_plot, 'b--', 'LineWidth', 2)
legend('x', 'y', 'xd', 'yd')
