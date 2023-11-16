%% Control Triciclo%%
% Robótica Móvil%
% Julio Alexis González Villa%

close all
clear 
clc

%Valores de variables y posiciones iniciales
d = 0.25;
alpha = 0;
p = [0 0 pi]';
pp = [0 0 0]';
R = 0.05;
t = 0.05;
s = 5;
N = s/t;
alpha_sup = pi/2;
alpha_inf = -pi/2;

%Ganancias 
kv = 1.0;
k_alpha = 0.2;

%Trayectoria deseada
td = [1.5 1.5]';

%Variables para poder graficar
p_plot = zeros(3,N);
pp_plot = zeros(3,N);
t_plot = t:t:s;
x_plot = [];
xd_plot = [];

for i=1:N
   
    ev = sqrt((td(1)-p(1))^2+(td(2)-p(2))^2);
    alpha = atan2(td(2)-p(2),td(1)-p(1))-p(3);

    if alpha>alpha_sup
        alpha = alpha_sup;
    end

    if alpha<alpha_inf
        alpha = alpha_inf;
    end 

    vs = kv*ev;

    pp(1) = vs*cos(alpha)*cos(p(3));
    pp(2) = vs*cos(alpha)*sin(p(3));
    pp(3) = (vs/d)*sin(alpha);

    p = p+pp*t; 
%     cla
%     Dibujar_Triciclo(p,alpha,d)
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
plot(t_plot,p_plot(1,:),'y-','LineWidth',2)
plot(t_plot,p_plot(2,:),'b-','LineWidth',2)
plot(t_plot,p_plot(3,:),'g-','LineWidth',2)
legend('x', 'y', 'theta')
xlabel('Tiempo')
ylabel('Posición')

%Gráfica de las velocidades 
figure  
grid on
hold on
plot(t_plot,pp_plot(1,:),'y-','LineWidth',2)
plot(t_plot,pp_plot(2,:),'b-','LineWidth',2)
plot(t_plot,pp_plot(3,:),'g-','LineWidth',2)
legend('x.', 'y.', 'Theta.')
xlabel('Tiempo')
ylabel('Velocidades')

% Gráfica de la trayectoria del diferencial
figure
grid on
hold on
plot(p_plot(1,:), p_plot(2,:), 'r--','LineWidth',2)
plot(td(1), td(2), 'rp','LineWidth',2)
Dibujar_Triciclo(p,alpha,d)

%Gráfica trayectoria real vs deseada
figure
hold on
grid on 
plot(t_plot, x_plot, 'b-', 'LineWidth', 2)
plot(t_plot, xd_plot, 'r--', 'LineWidth', 2)
legend('x', 'y', 'xd', 'yd')

