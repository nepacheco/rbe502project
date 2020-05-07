function dx = ode_caar_l(t,x,params)
% Initial parameters
qd = 2; qd_dot = 0;
Kp = 1;
Kd = 1;

% state variables 
q = x(1);
q_dot = x(2);
% Dynamic Model
[D, C, G] = generate_full_model_v2(q,qd);
%input torque
u = -Kp*(q-qd) - Kd*(q_dot - qd_dot) + G;
q_dotdot = D\(u-G - C*q_dot);
% updating states
dx = zeros(2,1);
dx(1) = x(2);
dx(2) = q_dotdot(1);

end