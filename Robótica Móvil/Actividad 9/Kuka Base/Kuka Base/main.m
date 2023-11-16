clear all
close all
clc

%%
L = 0.471/2;
l = 0.3/2;

p_plot = [];
t_plot = [];

bot = Bot_youBot_Platform();

%%
S = 10;

v = [0.2 -0.2 0.2 -0.2]';


tic;
while toc<=S
    img = bot.Get_Image();
    p = bot.Get_Pose();

    [Angles,Ranges] = bot.Get_LaserScans();
    [X,Y] = pol2cart(Angles,Ranges);


    subplot(2,1,1)
    cla
    imshow(img)
    
    subplot(2,1,2)
    cla
    plot(X,Y,'*')
    axis equal
    
    drawnow
    

    bot.Set_Joint_Velocity(v);
    disp(p')

    p_plot = [p_plot p];
    t_plot = [t_plot toc];
end

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
title('p')
hold on
grid on
plot(t_plot,p_plot(1,:),'m','LineWidth',2)
plot(t_plot,p_plot(2,:),'g','LineWidth',2)
plot(t_plot,p_plot(3,:),'k','LineWidth',2)
ylabel('m, rad')
xlabel('t')
legend('x','y','\theta','Location','best')

%%
bot.Stop_Simulation();

