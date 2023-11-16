%% Julio Alexis González Villa
clear all
close all
clc

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

%Robot

%Posicion Deseada y Ganancias
dx = 0.5;
dy = 0.5;
r = 0.7;
k = diag([.6 .6]);


%Tiempo
S = 60;

% %Velocidad de Llantas
% wr = 1;
% wl = 1;

tic;

Robot = figure();
while toc<=S
    
    
    p = bot.Get_Pose();
    

    %Trayectoria
%     r = 0.8*cos(.15.*toc);
%     xd = r.*cos(0.05.*toc);
%     yd = r.*sin(0.05.*toc);

    xd = 0.05*toc;    
    r = sin(xd*pi);

    if r >=0.6
        yd = 0.6;
    elseif r<=-0.6
        yd=-0.6;
    else
        yd=r;
    end

    Pd=[xd yd 0]';

    xp = p(1,:) + D*cos(p(3,:));
    yp = p(2,:) + D*sin(p(3,:));
    
    pp = [xp;yp];
  

    ex = xd - xp;
    ey = yd - yp;
    e = [ex;ey];
    
    pp = [xp;yp];

    Cin = [cos(p(3,:)) -D*sin(p(3,:));
          sin(p(3,:))  D*cos(p(3,:))];

    vw = inv(Cin)*k*e;

  
    
    bot.Set_Velocity(vw(1),vw(2));
    disp(p')

    p_plot = [p_plot p];
    pp_plot =[pp_plot pp];
    pd_plot = [pd_plot Pd];
    cla
    Dibujar_Diferencial(p,L)
    plot(p_plot(1,:),p_plot(2,:),'r','LineWidth',2);
    hold on
    plot(pd_plot(1,:),pd_plot(2,:),'b','LineWidth',2);
    xlim([-0.5 10])
    drawnow
    
   

end

bot.Set_Velocity(0,0);

bot.Stop_Connection()



% Terminar simulacion

%bot.Stop_Simulation();

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