clear all
close all
clc

%%
L = 0.471/2;
l = 0.3/2;

p_plot = [];
t_plot = [];
v_plot = [];
pd_plot = [];

bot = Bot_youBot_Platform();

%% 
S = 12;

pd = [-2; 1; -pi/4];
disp(pd)

k = [2 0 0; 0 4 0; 0 0 3];

tic;
while toc<=S
    img = bot.Get_Image();
    p = bot.Get_Pose();

    e = [pd(1)-p(1); pd(2)-p(2); pd(3)-p(3)];

    alpha = p(3)+(pi/4);

    T = [sqrt(2)*sin(alpha) -sqrt(2)*cos(alpha) -(L+l);
        sqrt(2)*cos(alpha) sqrt(2)*sin(alpha) (L+l);
        sqrt(2)*cos(alpha) sqrt(2)*sin(alpha) -(L+l);
        sqrt(2)*sin(alpha) -sqrt(2)*cos(alpha) (L+l)];

    v = T*k*e;

    [Angles,Ranges] = bot.Get_LaserScans();
    [X,Y] = pol2cart(Angles,Ranges);

    bot.Set_Joint_Velocity(v);
    
    
    v_plot = [v_plot v];
    p_plot = [p_plot p];
    t_plot = [t_plot toc];
    pd_plot = [pd_plot pd];
end

disp(p)

%%
figure
title('Trayectoria')
hold on
grid on
plot(p_plot(1,:),p_plot(2,:),'r','LineWidth',2)
Dibujar_Omnidireccional_4(p,L,l)
xlabel('x')
ylabel('y')

figure
title('P')
hold on
grid on
plot(t_plot,p_plot(1,:),'m','LineWidth',2)
plot(t_plot,p_plot(2,:),'g','LineWidth',2)
plot(t_plot,p_plot(3,:),'c','LineWidth',2)
ylabel('m, rad')
xlabel('t')
legend('x','y','\theta','Location','best')

figure
title('Velocidades')
hold on
grid on
plot(t_plot,v_plot(1,:),'m','LineWidth',2)
plot(t_plot,v_plot(2,:),'g','LineWidth',2)
plot(t_plot,v_plot(3,:),'y','LineWidth',2)
plot(t_plot,v_plot(4,:),'c','LineWidth',2)
ylabel('m')
xlabel('t')
legend('v1','v2','v3', 'v4')

figure
title('Posición ')
hold on
grid on
plot(t_plot,p_plot(1,:),'m','LineWidth',2)
plot(t_plot,p_plot(2,:),'g','LineWidth',2)
plot(t_plot,pd_plot(1,:),'y','LineWidth',2)
plot(t_plot,pd_plot(2,:),'c','LineWidth',2)
ylabel('m')
xlabel('t')
legend('x','y','xd','yd')



%%
bot.Stop_Simulation();

