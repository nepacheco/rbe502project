clc; clear; close all;
%% Implement the PD+ GRAVITY COMPENSATION control for set point tracking.
x0= [-0.5,0.2,0.1,0.1];
tf = 75;
params = [];
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) ode_caar(t,x,params),[0 tf],x0, options);

%% PD Gravity comp using L for single notch
x0= [0.000,0.000];
tf = 40;
params = [];
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4]);
[T,X] = ode45(@(t,x) ode_caar_l(t,x,params),[0 tf],x0, options);

figure()
hold on;
subplot(211)
plot(T,X(:,1)*1000,'r-');
title("Tendon Displacement and velocity over time");
xlabel("Time (s)");
ylabel("Tendon Displacement (mm)")
subplot(212)
plot(T,X(:,2)*1000,'b-');
xlabel("Time (s)");
ylabel("Tendon Rate of Change (mm/s)")
hold off;