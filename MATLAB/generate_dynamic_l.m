function [D,C,G] = generate_dynamic_l(is_sym,s0,state,spring_params)
%generate_dynamic_model - generates a symbolic dynamic model 
%   currently it does only a single notch 4/30 11:14pm
%   Also can do variables 5/3 4:22pm
if is_sym
    syms y2 y1 h
    syms m1 g1 
    syms k1 c1
    syms I1
else
    L = state(1); L_dot = state(2);
    k1 = spring_params(1); c1 = spring_params(2);
    m1 = spring_params(3); I1 = spring_params(4);
    % params from single notch tube
    h = 5;
    y1 = 2.9564;
    y2 = 1.6423;
    g1 = 9.8;
end
    
D = m1*(y1/(y1 + y2))^2 + I1*(1/(y1 + y2))^2;
C = c1*((0.5 - y1)/(y1 + y2))^2;
G = -m1*g1*(y1)/(y1 + y2) + k1*((0.5 - y1)/(y1 + y2)*L-s0 + h)*(0.5 - y1)/(y1 + y2);

end