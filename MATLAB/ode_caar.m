function dx = ode_caar(t,x,params)
% Initial parameters
qd = [5;0]; qd_dot = [0;0];
Kp = 1*eye(2,2);
Kd = 1*eye(2,2);

% state variables 
q = x(1:2);
q_dot = x(3:4);
% Dynamic Model
[D, C, G] = generate_dynamic_model(false,5,q,q_dot,[1,1,1,1]);
%input torque
u = -Kp*(q-qd) - Kd*(q_dot - qd_dot) + G;
q_dotdot = D\(u-G - C*q_dot);
% updating states
dx = zeros(4,1);
dx(1) = x(3);
dx(2) = x(4);
dx(3) = q_dotdot(1);
dx(4) = q_dotdot(2);
end