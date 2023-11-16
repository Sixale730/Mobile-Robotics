clear all
close all
clc

%% Cargar trayectoria suavizada
load('spath.mat')

R = 0.195/2;
L = 0.381/2;
D = 0.025;
i = 1;

bot = Bot_Pioneer();

%% Values
S = 100;

wr = 0.0;
wl = 0.0;

k = diag([10 10]);

p_plot = [];
t_plot = [];
vw_plot = [];
pd_plot = [];

tic;
while toc<=S
    p = bot.Get_Pose();
    % pg = bot.Get_Goal_Position()
    % po = bot.Get_Obs_Positions()

    xp = p(1,:) + D*cos(p(3,:));
    yp = p(2,:) + D*sin(p(3,:));
    
    pp = [xp;yp];
    pd = [spath(1,i), spath(2,i)]';

    xd = pd(1);
    yd = pd(2);

    td = [xd;yd];

    ex = td(1)-p(1);
    ey = td(2)-p(2);
    e = [ex;ey];

    Cin = [(1/2)*cos(p(3))-(D/L)*sin(p(3)) (1/2)*cos(p(3))+(D/L)*sin(p(3));
           (1/2)*sin(p(3))+(D/L)*cos(p(3)) (1/2)*sin(p(3))-(D/L)*cos(p(3))];
    
    vw = (1/R)*inv(Cin)*k*e;

    if norm(e) < 0.08
        i = i+1;
    end
    if i>size(spath)
        break 
    end 
    
    bot.Set_Joint_Velocity(vw);

    p_plot = [p_plot p];
    t_plot = [t_plot toc];
    vw_plot= [vw_plot vw];
    pd_plot = [pd_plot pd];
    bot.Set_Joint_Velocity([wr wl]');
end

%Gráfica de las velocidades 
figure  
grid on
hold on
plot(t_plot,vw_plot(1,:),'y-','LineWidth',2)
plot(t_plot,vw_plot(2,:),'g-','LineWidth',2)
legend('WR', 'WL')
xlabel('Tiempo')
ylabel('Velocidades')

% Gráfica de la trayectoria del diferencial
figure
grid on
hold on
plot(p_plot(1,:), p_plot(2,:), 'r-','LineWidth',2)
plot(pd(1), pd(2), 'm*')
Dibujar_Diferencial(p,L)

%Gráfica trayectoria real vs deseada
figure
hold on
grid on 
plot(t_plot, p_plot(1,:), 'g', 'LineWidth', 2)
plot(t_plot, p_plot(2,:), 'r', 'LineWidth', 2)
plot(t_plot, pd_plot(1,:), 'g--', 'LineWidth', 2)
plot(t_plot, pd_plot(2,:), 'r--', 'LineWidth', 2)
legend('x', 'y', 'xd', 'yd')


%% Terminar simulacion
bot.Stop_Simulation();



