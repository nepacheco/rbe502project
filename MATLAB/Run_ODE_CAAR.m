clc; clear; close all;
%% Implement the PD+ GRAVITY COMPENSATION control for set point tracking.
x0= [-0.5,0.2,0.1,0.1];
tf = 75;
params = [];
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) ode_caar(t,x,params),[0 tf],x0, options);

%% PD Gravity comp using L for single notch
x0= [1,1];
tf = 75;
params = [];
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4]);
[T,X] = ode45(@(t,x) ode_caar_l(t,x,params),[0 tf],x0, options);
