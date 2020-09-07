clc;
clear all;
close all;
%% Parameter Values
g = 9.8;
Mu = 0.1;
alpha = 0.2186;
T = 250;
Cx = 0.4;
Cz = 0.05;
Ro = 1.225;
A = 2.9;
n = 20;
d = 0.25;
M = 195;
F_br = 4000;
m_LT = 20;
a3 = (81.032*8/n);
a2 = (-617.95*4/n);
a1 = (1609*2/n);

a33 = (67.707*8/n);
a22 = (-546.449*4/n);
a11 = (1735*2/n);
%% Before UAV Take off
tspan = 0:0.001:0.48;
[t,x] = ode45(@Dynamics,tspan,[8 0]);
figure(1)
plot(t,x(:,1),'b','linewidth',2);grid on;hold on
plot(t(end),x(end,1),'*r','linewidth',2)
xlabel('t');ylabel('x');
title('Displacement x(t) before UAV take off')

figure(2)
plot(t,x(:,2),'b','linewidth',2);grid on;hold on
plot(t(end),x(end,2),'*r','linewidth',2)
xlabel('t');ylabel('v');
title('Velocity v(t) before UAV take off')

for k=1:length(x(:,1))
    X31 = ((Cx*Ro*A+Mu*Cz*Ro*A*sign(x(k,2)))/(2*M))*x(k,2)^2;
    X32 = (abs(x(k,1))+Mu*d)/(2*M*sqrt(x(k,1)^2+d^2));
    X33_1 = ((n*a3/8)*x(k,1)^3)+(((n*a2)/4)*x(k,1)^2)+((n*a1/2)*x(k,1));
    X33_2 = ((n*a33/8)*x(k,1)^3)+(((n*a22)/4)*x(k,1)^2)+((n*a11/2)*x(k,1)); 
    if x(k,1)>=0
        X33 = X33_1;
    else
        X33 = X33_2;
    end
    X34 = (T/M)+Mu*g*cos(alpha)*sign(x(k,2));
    X35 = (Mu*F_br/m_LT)*sign(x(k,2));
   Acc(k) = X31-X32*X33-X34+g*sin(alpha)-X35; 
end

figure(3)
plot(t,Acc,'b','linewidth',2);grid on;hold on
plot(t(end),Acc(end),'*r','linewidth',2)
xlabel('t');ylabel('a');
title('Acceleration a(t) before UAV take off')

figure(4)
plot(t,x(:,1),'g',t,x(:,2),'b',t,Acc,'r','linewidth',2);hold on
plot(t(end),x(end,1),'*g',t(end),x(end,2),'*b',t(end),Acc(end),'*r','linewidth',2)
xlabel('t');ylabel('x(t),v(t),a(t)');grid on
legend('Position','Velocity','Acceleration')
title('Overall trajectories of x(t),v(t),a(t) before UAV take off')
%% After UAV Take off
length(x(:,1))
tspan = 0.481:0.001:2.5;
[t1,x_new] = ode45(@Dynamics,tspan,[x(end,1) 30]);
figure(5)
plot([t;t1],[x(:,1);x_new(:,1)],'b','linewidth',2);grid on;hold on
plot(t(end),x(end,1),'*r','linewidth',2)
xlabel('t');ylabel('x(t)');
title('Displacement x(t) before and after UAV take off')

figure(6)
plot([t;t1],[x(:,2);x_new(:,2)],'b','linewidth',2);grid on;hold on
plot(t(end),x(end,2),'*r','linewidth',2)
xlabel('t');ylabel('v(t)');
title('Velocity v(t) before and after UAV take off')

for k=1:length(x_new(:,1))
    X31 = ((Cx*Ro*A+Mu*Cz*Ro*A*sign(x_new(k,2)))/(2*M))*x_new(k,2)^2;
    X32 = (abs(x_new(k,1))+Mu*d)/(2*M*sqrt(x_new(k,1)^2+d^2));
    X33_1 = ((n*a3/8)*x_new(k,1)^3)+(((n*a2)/4)*x_new(k,1)^2)+((n*a1/2)*x_new(k,1));
    X33_2 = ((n*a33/8)*x_new(k,1)^3)+(((n*a22)/4)*x_new(k,1)^2)+((n*a11/2)*x_new(k,1)); 
    if x_new(k,1)>=0
        X33 = X33_1;
    else
        X33 = X33_2;
    end
    X34 = (T/M)+Mu*g*cos(alpha)*sign(x_new(k,2));
    X35 = (Mu*F_br/m_LT)*sign(x_new(k,2));
   Acc(k+481) = X31-X32*X33-X34+g*sin(alpha)-X35; 
end

figure(7)
plot([t;t1],Acc,'b','linewidth',2);grid on;hold on
plot(t(end),Acc(480),'*r','linewidth',2)
xlabel('t');ylabel('a(t)');
title('Acceleration a(t) before and after UAV take off')

figure(8)
plot([t;t1],[x(:,1);x_new(:,1)],'g',[t;t1],[x(:,2);x_new(:,2)],'b',[t;t1],Acc,'r','linewidth',2);hold on
plot(t(end),x(end,1),'*g',t(end),x(end,2),'*b',t(end),Acc(480),'*r','linewidth',2)
xlabel('t');ylabel('x(t),v(t),a(t)');grid on
legend('Position','Velocity','Acceleration')
title('Overall trajectories of x(t),v(t),a(t) before and after UAV take off')
%-----------------------------end------------------------------