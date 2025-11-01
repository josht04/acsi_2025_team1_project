params.mq = 0.029; % kg
params.mb = 0.01; % kg
params.ms = 0.0025; % kg
params.Ixx = 6.410179e-06; % kg m2
params.g = 9.81; % kg/m2
params.r = 0.05665/2; % m


mq = params.mq;
mb = params.mb;
ms = params.ms;
L = 0.3;
Ixx = params.Ixx;
g = params.g;
r = params.r;

syms y z phi theta y_d z_d phi_d theta_d Fl Fr

state = [y,z,phi,theta,y_d,z_d,phi_d,theta_d];
inputs = [Fl,Fr];

Ip = ms*L^2/3+mb*L^2;
Lcg = L*(ms/2+mb)/(ms+mb);

% A*x_dd = B

% Dynamics derived from the pdf
A = [ms+mb+mq 0 (ms+mb)*Lcg*cos(phi+theta) (ms+mb)*Lcg*cos(phi+theta);
    0 ms+mb+mq (ms+mb)*Lcg*sin(phi+theta) (ms+mb)*Lcg*sin(phi+theta);
    Lcg*cos(phi+theta)*(ms+mb) Lcg*sin(phi+theta)*(ms+mb) Ip Ip;
    0 0 Ixx 0];

B = [-(Fl+Fr)*sin(phi)+(ms+mb)*Lcg*sin(phi+theta)*(phi_d+theta_d)^2 ;
    (Fl+Fr)*cos(phi)-(ms+mb)*Lcg*cos(phi+theta)*(phi_d+theta_d)^2-g*(ms+mb+mq) ;
    -g*Lcg*(ms+mb)*sin(phi+theta) ; 
    (Fr-Fl)*r];

state_dd = A\B;

dstate_dt = [y_d ; z_d ;phi_d ;theta_d; state_dd]

A = jacobian(dstate_dt,state)
B = jacobian(dstate_dt,inputs)