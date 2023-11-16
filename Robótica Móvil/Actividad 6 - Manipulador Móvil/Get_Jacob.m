clear all
close all
clc

x_b = sym('x_b');
y_b = sym('y_b');
theta_b = sym('theta_b');
theta_1 = sym('theta_1');
theta_2 = sym('theta_2');
theta_3 = sym('theta_3');

wTp = [cos(theta_b) -sin(theta_b) 0 x_b; sin(theta_b) cos(theta_b) 0 y_b; 0 0 1 0; 0 0 0 1];
pTb = [1 0 0 0.25; 0 1 0 0; 0 0 1 0.25; 0 0 0 1];
T01 = [cos(theta_1) 0 sin(theta_1) 0; sin(theta_1) 0 -cos(theta_1) 0; 0 1 0 0.35; 0 0 0 1];
T12 = [cos(theta_2) -sin(theta_2) 0 0.3*cos(theta_2); sin(theta_2) cos(theta_2) 0 0.3*sin(theta_2); 0 0 1 0; 0 0 0 1];
T23 = [cos(theta_3) -sin(theta_3) 0 0.25*cos(theta_3); sin(theta_3) cos(theta_3) 0 0.25*sin(theta_3); 0 0 1 0; 0 0 0 1];

T1 = simplify(wTp);
T2 = simplify(T1*pTb);
T3 = simplify(T2*T01);
T4 = simplify(T3*T12);
T5 = simplify(T4*T23);  % T5 = wTe

tx = T5(1,4);
ty = T5(2,4);
tz = T5(3,4);

Jv = [diff(tx,x_b) diff(tx,y_b) diff(tx,theta_b) diff(tx,theta_1) diff(tx,theta_2) diff(tx,theta_3); ...
      diff(ty,x_b) diff(ty,y_b) diff(ty,theta_b) diff(ty,theta_1) diff(ty,theta_2) diff(ty,theta_3); ...
      diff(tz,x_b) diff(tz,y_b) diff(tz,theta_b) diff(tz,theta_1) diff(tz,theta_2) diff(tz,theta_3)];

Jv = simplify(Jv);
