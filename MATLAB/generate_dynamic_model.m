function [D,C,G] = generate_dynamic_model(is_sym,s0,q,q_dot,spring_params)
%generate_dynamic_model - generates a symbolic dynamic model 
%   currently it does only a single notch 4/30 11:14pm
%   Also can do variables 5/3 4:22pm
if is_sym
    syms s0 s s_dot t t_dot 
    syms F1 F2 m1 g1 
    syms k1 c1
    syms I1
else
    s = q(1); t = q(2);
    s_dot = q_dot(1); t_dot = q_dot(2);
    k1 = spring_params(1); c1 = spring_params(2);
    m1 = spring_params(3); I1 = spring_params(4);
    g1 = 9.8;
end
    

T = 1/2*m1*s_dot^2 + 1/2*I1*t_dot^2;
U = m1*g1*s + 1/2*k1*(s + 1/2*t - s0)^2;
R = 0.5*c1*(s_dot + 1/2*t_dot)^2;

D = [m1 0; 0 I1];
C = [c1 1/2*c1; 1/2*c1 1/4*c1];
G = [m1*g1 + k1*(s+1/2*t-s0); 1/2*k1*(s + 1/2*t - s0)];

end