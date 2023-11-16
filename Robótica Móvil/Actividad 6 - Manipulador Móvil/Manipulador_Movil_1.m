%% Manipulador Móvil %%
% Robótica Móvil%
% Julio Alexis González Villa%
close all 
clear 
clc

%variables de inicio del móvil
q_arm = [0 pi/4 -pi/4]';
p_base = [0 0 0]';
q = [p_base;q_arm];
qp = [0 0 0 0 0 0]';

td = [-1.0 1.0 0.5]';
s = 5;
t = 0.05;
N = s/t;

K = diag([1.5 1.5 1.5])'; %Ganancias

%Jacobiano del manipulador móvil
Jacob = @(theta_b,theta_1,theta_2,theta_3)[1, 0, (sin(theta_1 + theta_b)*sin(theta_2)*sin(theta_3))/4 - (3*sin(theta_1 + theta_b)*cos(theta_2))/10 - (sin(theta_1 + theta_b)*cos(theta_2)*cos(theta_3))/4 - sin(theta_b)/4, -(sin(theta_1 + theta_b)*(5*cos(theta_2 + theta_3) + 6*cos(theta_2)))/20, -(cos(theta_1 + theta_b)*(5*sin(theta_2 + theta_3) + 6*sin(theta_2)))/20, -(cos(theta_1 + theta_b)*sin(theta_2 + theta_3))/4;...
                                           0, 1, cos(theta_b)/4 + (3*cos(theta_1 + theta_b)*cos(theta_2))/10 + (cos(theta_1 + theta_b)*cos(theta_2)*cos(theta_3))/4 - (cos(theta_1 + theta_b)*sin(theta_2)*sin(theta_3))/4,  (cos(theta_1 + theta_b)*(5*cos(theta_2 + theta_3) + 6*cos(theta_2)))/20, -(sin(theta_1 + theta_b)*(5*sin(theta_2 + theta_3) + 6*sin(theta_2)))/20, -(sin(theta_2 + theta_3)*sin(theta_1 + theta_b))/4;...
                                           0, 0,                                                                                                                                                                     0,                                                                        0,                           cos(theta_2 + theta_3)/4 + (3*cos(theta_2))/10,                           cos(theta_2 + theta_3)/4];

wTp = @(x_b,y_b,theta_b)[cos(theta_b) -sin(theta_b) 0 x_b; sin(theta_b) cos(theta_b) 0 y_b; 0 0 1 0; 0 0 0 1];
pTb = [1 0 0 0.25; 0 1 0 0; 0 0 1 0.25; 0 0 0 1];
T01 = @(theta_1)[cos(theta_1) 0 sin(theta_1) 0; sin(theta_1) 0 -cos(theta_1) 0; 0 1 0 0.35; 0 0 0 1];
T12 = @(theta_2)[cos(theta_2) -sin(theta_2) 0 0.3*cos(theta_2); sin(theta_2) cos(theta_2) 0 0.3*sin(theta_2); 0 0 1 0; 0 0 0 1];
T23 = @(theta_3)[cos(theta_3) -sin(theta_3) 0 0.25*cos(theta_3); sin(theta_3) cos(theta_3) 0 0.25*sin(theta_3); 0 0 1 0; 0 0 0 1];

%Variables para gráficar
q_plot = [];
qp_plot = [];
x_plot = [];
xd_plot = [];
t_plot = [];

for i=1:N
    
    bTe = T01(q_arm(1))*T12(q_arm(2))*T23(q_arm(3));
    wTe = wTp(q(1), q(2), q(3))*pTb*bTe; 

    X = wTe(1:3,4); 
    e = td - X;
    
    J = Jacob(q(3), q(4), q(5), q(6));
    qp = pinv(J)*K*e;

    q = q+qp*t;
    p_base = q(1:3);
    q_arm = q(4:6);

    cla
    Dibujar_MM(p_base, q_arm)
    plot3(td(1), td(2), td(3), 'rp','LineWidth', 4)
    drawnow

    q_plot = [q_plot q];
    qp_plot = [qp_plot qp];
    x_plot = [x_plot wTe(1:3,4)];
    xd_plot = [xd_plot td];
    t_plot = [t_plot i*t];
end

%Gráfica pose
figure 
f1 = subplot(2,2,1);
hold on
grid on 
plot(f1,t_plot, q_plot, '-', 'LineWidth', 2)
legend('x_b', 'y_b', 'theta_b', 'theta_1', 'theta_2', 'theta_3')

%Gráfica velocidades
f2 = subplot(2,2,2); 
hold on 
grid on
plot(f2,t_plot, qp_plot, '-', 'LineWidth', 2)
legend('X_b dot', 'y_b dot', 'theta_b dot', 'theta_1 dot', 'theta_2 dot', 'theta_3 dot')

%Gráfica trayectoria real vs deseada
f3 = subplot(2,2,3);
hold on
grid on 
plot(f3,t_plot, x_plot, 'b-', 'LineWidth', 2)
plot(f3,t_plot, xd_plot, 'c--', 'LineWidth',2)
legend('x', 'y', 'z', 'xd', 'yd', 'zd')