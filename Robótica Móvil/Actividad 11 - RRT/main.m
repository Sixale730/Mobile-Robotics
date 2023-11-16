clear all
close all
clc

%%
R = 0.195/2;
L = 0.381/2;

bot = Bot_Pioneer();

%%
S = 10;

wr = 0.0;
wl = 0.0;

tic;
while toc<=S
    p = bot.Get_Pose();
    pg = bot.Get_Goal_Position()
    po = bot.Get_Obs_Positions()

    bot.Set_Joint_Velocity([wr wl]');
end

%% Terminar simulacion
bot.Stop_Simulation();



