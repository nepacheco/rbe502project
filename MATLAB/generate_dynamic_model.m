function [D,C,G] = generate_dynamic_model()
%generate_dynamic_model - generates a symbolic dynamic model 
%   currently it does only a single notch 4/30 11:14pm
syms s0 s9t) s_dot(t) th(t) th_dot(t) 
syms F1 F2 m1 g1 
syms k1 c1
syms I1

T = 1/2*m1*s_dot^2 + 1/2*I1*th_dot^2;
U = m1*g1*s + 1/2*k1*(s + 1/2*th - s0)^2;
R = 0.5*c1*(s_dot + 1/2*th_dot^2);


D = [];
C = [c1 1/2*c1; 1/2*c1 1/4*c1];
G = [diff(U,s);diff(U,th)];

end