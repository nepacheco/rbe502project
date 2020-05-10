%% Setup
syms t real
syms l l_dot l_dotdot real
% syms y1 y2 h n
% these conditions based on robot in MQP
h = 5/1000;
y1 = 2.9564/1000;
y2 = 1.6423/1000;
g = 9.8;
n = 6;
a = 1/(n*(y1 + y2));
b = y1*a;

P1 = [(h-b*l)*sin(a*l/2);0;(h-b*l)*cos(a*l/2)];
P2 = P1 + [(2*h-b*l)*sin(1.5*a*l);0;(2*h-b*l)*cos(1.5*a*l)];
P3 = P2 + [(2*h-b*l)*sin(2.5*a*l);0;(2*h-b*l)*cos(2.5*a*l)];
P4 = P3 + [(2*h-b*l)*sin(3.5*a*l);0;(2*h-b*l)*cos(3.5*a*l)];
P5 = P4 + [(2*h-b*l)*sin(4.5*a*l);0;(2*h-b*l)*cos(4.5*a*l)];
P6 = P5 + [(2*h-b*l)*sin(5.5*a*l);0;(2*h-b*l)*cos(5.5*a*l)];

P1_dot = [(-b*l)*cos(a*l/2)*(a/2*l_dot) + (-b*l_dot)*sin(a*l/2);
            0;
            (b*l)*sin(a/2*l)*(a/2*l_dot) + (-b*l_dot)*cos(a/2*l)];
P2_dot = P1_dot + [(2*h-b*l)*cos(1.5*a*l)*(1.5*a*l_dot) + (-b*l_dot)*sin(1.5*a*l);
                    0;
                   (-2*h + b*l)*sin(1.5*a*l)*(1.5*a*l_dot) + (-b*l_dot)*cos(1.5*a*l)];
P3_dot = P2_dot + [(2*h-b*l)*cos(2.5*a*l)*(2.5*a*l_dot) + (-b*l_dot)*sin(2.5*a*l);
                    0;
                   (-2*h + b*l)*sin(2.5*a*l)*(2.5*a*l_dot) + (-b*l_dot)*cos(2.5*a*l)];
P4_dot = P3_dot + [(2*h-b*l)*cos(3.5*a*l)*(3*a*l_dot) + (-b*l_dot)*sin(3.5*a*l);
                    0;
                   (-2*h + b*l)*sin(3.5*a*l)*(3.5*a*l_dot) + (-b*l_dot)*cos(3.5*a*l)];
P5_dot = P4_dot + [(2*h-b*l)*cos(4.5*a*l)*(4.5*a*l_dot) + (-b*l_dot)*sin(4.5*a*l);
                    0;
                   (-2*h + b*l)*sin(4.5*a*l)*(4.5*a*l_dot) + (-b*l_dot)*cos(4.5*a*l)];
P6_dot = P5_dot + [(2*h-b*l)*cos(5.5*a*l)*(5.5*a*l_dot) + (-b*l_dot)*sin(5.5*a*l);
                    0;
                   (-2*h + b*l)*sin(5.5*a*l)*(5.5*a*l_dot) + (-b*l_dot)*cos(5.5*a*l)];
P3_dot = simplify(P3_dot,'Steps',20);
P4_dot = simplify(P4_dot,'Steps',20);
P5_dot = simplify(P5_dot,'Steps',20);
P6_dot = simplify(P6_dot,'Steps',20);
syms m

%% Energy equations
syms k c real % spring and damper constants
s0 = .005;

T = 1/2*m*(P1_dot'*P1_dot + P2_dot'*P2_dot + P3_dot'*P3_dot +...
    P4_dot'*P4_dot + P5_dot'*P5_dot + P6_dot'*P6_dot);

U = -m*[0 0 g]*(P1 + P2 + P3 + P4 + P5 + P6) + 1/2*k*(6*(h-b*l + 1/2*0.0025*a*l-s0)^2);

R = 1/2*c*(6*(-b*l_dot + 0.0025*a*l_dot)^2);

%% Dynamic Model
syms l l_dot
G = simplify(diff(U,l),'Steps',20);
T_dot = simplify(diff(T,l_dot),'Steps',20);
T_part_diff = simplify(diff(T,l),'Steps',20);
R_diff = simplify(diff(R,l_dot),'Steps',20);

D = simplify(subs(T_dot,l_dot,1),'Steps',20);
G = simplify(diff(U,l),'Steps',20);

syms l(t)
temp = subs(T_dot,[l],[l(t)]);
temp_diff = diff(temp,t);
C = subs(temp_diff,diff(l(t),t),l_dot);
syms L real
C = vpa(subs(C,l(t),L),2);
vpa(subs(C,[l_dot,L],[1,1]),2);
syms l real
C = vpa(subs(C,L,l),2);
vpa(subs(C,[l_dot,l],[1,1]),2);

C = C - T_part_diff + R_diff;
%%
vpa(subs(G,[l,l_dot,m,c,k],[0,0,1,1,1]),4)

