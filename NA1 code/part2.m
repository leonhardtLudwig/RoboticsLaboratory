Ta = 2;
Tc = 3;
vc = 1/(Ta+Tc);
%a = vc/Ta;


xf = 1;
yf = 1;
xi = -1;
yi = -1;

thetaf= pi/2;
thetai= pi/2;

qi = [xi;yi;thetai];
qf = [xf;yf;thetaf];


kf = 1;
ki = 10;

a = [xf,xi,kf*cos(thetaf)-3*xf, ki*cos(thetai)+3*xi ];
b = [yf,yi,kf*sin(thetaf)-3*yf, ki*sin(thetai)+3*yi ];
