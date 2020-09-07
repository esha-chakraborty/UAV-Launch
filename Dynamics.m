function [ dx ] = Dynamics( t,x )
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
if t>0.48
    M = 175;
end
F_br = 4000;
m_LT = 20;
a3 = (81.032*8/n);
a2 = (-617.95*4/n);
a1 = (1609*2/n);

a33 = (67.707*8/n);
a22 = (-546.449*4/n);
a11 = (1735*2/n);

    X31 = ((Cx*Ro*A+Mu*Cz*Ro*A*sign(x(2)))/(2*M))*x(2)^2;
    X32 = (abs(x(1))+Mu*d)/(2*M*sqrt(x(1)^2+d^2));
    X33_1 = ((n*a3/8)*x(1)^3)+(((n*a2)/4)*x(1)^2)+((n*a1/2)*x(1));
    X33_2 = ((n*a33/8)*x(1)^3)+(((n*a22)/4)*x(1)^2)+((n*a11/2)*x(1)); 
    if x(1)>=0
        X33 = X33_1;
    else
        X33 = X33_2;
    end
    X34 = (T/M)+Mu*g*cos(alpha)*sign(x(2));
    X35 = (Mu*F_br/m_LT)*sign(x(2));
    dx1 = x(2);
    dx2 = X31-X32*X33-X34+g*sin(alpha)-X35;
    dx = [dx1;dx2];
end

