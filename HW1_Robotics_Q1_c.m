clear all;clc;close all;
i=1;                    %array counter
x_c = 0; x_f = 2;       %initial & final pos
xdot_c = 0; xdot_f = 0; %initial & final vel
xdotdot_c = 0;          %initial acc
t_togo = 2;             %total time
t_delta = 0.01;         %time increment
while(t_togo > 0)
    i=i+1;
    pos(i)= x_c; 
    vel(i) = xdot_c; 
    acc(i) = xdotdot_c;
    [x_c,xdot_c,xdotdot_c]=Incremental_Plan_Fun(x_c,xdot_c,x_f,xdot_f,t_togo,t_delta)
    t_togo = t_togo - 0.01;
end
%----Plots
t=0:0.01:2;     % x-axis steps
hold on; grid on; ylim([-3.1 3.1]);xlim([-0.1 2.1]);
plot(t,pos,'-r','LineWidth',1);
plot(t,vel,'-b','LineWidth',1);
plot(t,acc,'-g','LineWidth',1);
title('Position, Velocity & Acceleration in Incremental Path Planning');
ylabel('Magnitude'); xlabel('time(sec)');legend('Position','Velocity','Acceleration');

%----Incremental Plannning Function
function [pos,vel,acc]= Incremental_Plan_Fun(x_c,xdot_c,x_f,xdot_f,t_togo,t_delta)
%----coefficients
c0 = x_c;
c1 = xdot_c;
c2 = (3*(x_f - x_c)/(t_togo^2)) - ((xdot_f + 2*xdot_c)/t_togo);
c3 = ((xdot_f+xdot_c)/(t_togo^2)) - (2*(x_f-x_c)/(t_togo^3));
%----measurements
pos = c0 + c1*t_delta + c2*t_delta^2 + c3*t_delta^3;
vel = c1 + 2*c2*t_delta + 3*c3*t_delta^2;
acc = 2*c2 + 6*c3*t_delta;
end


