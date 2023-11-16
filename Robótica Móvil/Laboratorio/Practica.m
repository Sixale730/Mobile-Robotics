%% Julio Alexis González Villa
clear all
close all
clc

load('path_1.mat')
%load('path_2.mat')

%Distancias
R = 0.195/2;
L = 0.381/2;
D = 0.05;
i=1;

bot = TurtleBot3_WafflePI('http://192.168.50.113:11311');

bot.Reset_Odometry();

%Graficas
p_plot = [];
pp_plot = [];
pd_plot = [];
t_plot = [];
vw_plot = [];


k = diag([1.5 1.5]);


%Tiempo
S = 360;

tic;

Robot = figure();
while toc<=S
    
    
    p = bot.Get_Pose()
    
    xp = p(1,:) + D*cos(p(3,:));
    yp = p(2,:) + D*sin(p(3,:));
    
    pp = [xp;yp];
    pd = [PH(1,i), PH(2,i)]';

    xd = pd(1);
    yd = pd(2);

    ex = xd - xp;
    ey = yd - yp;
    e = [ex;ey];
    
    pp = [xp;yp];

    Cin = [cos(p(3,:)) -D*sin(p(3,:));
          sin(p(3,:))  D*cos(p(3,:))];

    vw = inv(Cin)*k*e;
    
    if norm(e) < 0.05
        i = i+1;
    end
    if i>size(PH)
        break 
    end 
  
    
    bot.Set_Velocity(vw(1),vw(2));
    disp(p')

    p_plot = [p_plot p];
    pp_plot =[pp_plot pp];
    pd_plot = [pd_plot pd];
    t_plot = [t_plot toc];
    vw_plot= [vw_plot vw];
    
    cla
    Dibujar_Diferencial(p,L)
    plot(p_plot(1,:),p_plot(2,:),'r','LineWidth',2);
    hold on
    plot(p_plot(1,:), p_plot(2,:), 'g-.','LineWidth',2)
    plot(pd_plot(1,:),pd_plot(2,:),'b','LineWidth',2);
    xlim([-0.5 3])
    drawnow    
   
end

bot.Set_Velocity(0,0);

bot.Stop_Connection()



% Terminar simulacion
% bot.Stop_Simulation();

%% 
%Gráfica de las velocidades 
figure  
grid on
hold on
plot(t_plot,vw_plot(1,:),'m-','LineWidth',2)
plot(t_plot,vw_plot(2,:),'b-','LineWidth',2)
legend('WR', 'WL')
xlabel('Tiempo')
ylabel('Velocidades')

% Gráfica de la trayectoria del diferencial
figure
grid on
hold on
plot(p_plot(1,:), p_plot(2,:), 'g-','LineWidth',2)
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
