%% Diferencial %%
% Robótica Móvil%
% Julio Alexis González Villa%

close all
clear var
clc

%Configuración
L = 0.25;
p = [0 0 0]';
pp = [0 0 0]';
R = 0.05;
t = 0.05;
s = 5;
N = s/t;

%Ganancias
kv = 0.9;
kw = 2.5;

%Trayectoria deseada
td = [1.5 -1.5]';

%Variables para poder graficar
p_plot = zeros(3,N);
pp_plot = zeros(3,N);
t_plot = t:t:5;
x_plot = [];
xd_plot = [];

for i=1:N
    
    ev = sqrt((td(1)-p(1))^2+(td(2)-p(2))^2);
    ew = atan2(td(2)-p(2),td(1)-p(1))-p(3);
    ew = atan2(sin(ew),cos(ew));

    vs = kv*ev;
    ws = kw*ew;
    
    wr=(vs+ws*L)/R;
    wl=(vs-ws*L)/R;

    pp(1) = 0.5*R*(wr+wl)*cos(p(3));
    pp(2) = 0.5*R*(wr+wl)*sin(p(3));
    pp(3) = (R/(2*L))*(wr-wl);

    p = p+pp*t; 

%     cla
%     Dibujar_Diferencial(p,L)
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
plot(t_plot,p_plot(2,:),'m-','LineWidth',2)
plot(t_plot,p_plot(3,:),'g-','LineWidth',2)
legend('x', 'y', 'theta')
xlabel('Tiempo')
ylabel('Posición')

%Gráfica de las velocidades 
figure  
grid on
hold on
plot(t_plot,pp_plot(1,:),'y-','LineWidth',2)
plot(t_plot,pp_plot(2,:),'m-','LineWidth',2)
plot(t_plot,pp_plot(3,:),'g-','LineWidth',2)
legend('x.', 'y.', 'theta.')
xlabel('Tiempo')
ylabel('Velocidades')

% Gráfica de la trayectoria del diferencial
figure
grid on
hold on
plot(p_plot(1,:), p_plot(2,:), 'g-','LineWidth',2)
plot(td(1), td(2), 'rp','LineWidth',2)
Dibujar_Diferencial(p,L)

%Gráfica trayectoria real vs deseada
figure
hold on
grid on 
plot(t_plot, x_plot, 'r-', 'LineWidth', 2)
plot(t_plot, xd_plot, 'k--', 'LineWidth', 2)
legend('x', 'y', 'xd', 'yd')

