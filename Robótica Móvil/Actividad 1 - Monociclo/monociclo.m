%% Monociclo %%
% Robótica Móvil%
% Julio Alexis González Villa%
clear var
close all
clc

%Vector de variables de actuación
v = 0.0;
w = -0.8;

%Condiciones iniciales
p = [0.0 0.0 0.0]';
pp = [0.0 0.0 0.0]';

%Estimación en el tiempo en iteraciones
t = 0.05;
s = 5;
I = s/t;

%Registro de la posición en cada iteración
p_plot = zeros(3,I);
pp_plot = zeros(3,I);

for i=1:I
    %Animación del robot
    cla
    Dibujar_Movil(p)
    drawnow

    %Jacobiano
    J = [cos(p(3)) 0; sin(p(3)) 0; 0 1];
    
    pp = J*[v;w];

    %Paso de Integración
    p = p + pp*t;
    %Guardamos la posición y velocidad en cada iteración
    p_plot(:,i) = p;
    pp_plot(:,i) = pp;

   
end

t_plot = t:t:s;

% Gráfica de posición
figure
hold on 
grid on
plot(t_plot,p_plot(1,:),'r-','LineWidth', 2)
plot(t_plot,p_plot(2,:),'g-','LineWidth', 2)
plot(t_plot,p_plot(3,:),'b-','LineWidth', 2)
legend('x','y','theta')

% Gráfica de velocidad
figure
hold on 
grid on
plot(t_plot,pp_plot(1,:),'r-','LineWidth', 2)
plot(t_plot,pp_plot(2,:),'g-','LineWidth', 2)
plot(t_plot,pp_plot(3,:),'b-','LineWidth', 2)
legend('x dot','y dot','theta dot')

% Gráfica de la trayectoria realizada 
figure
hold on 
grid on
plot(p_plot(1,:),p_plot(2,:),'r-','LineWidth', 2)
Dibujar_Movil(p)
