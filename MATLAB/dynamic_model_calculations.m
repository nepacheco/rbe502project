%% Setup
syms t real
syms l l_dot l_dotdot real
% syms y1 y2 h n
% these conditions based on robot in MQP
h = 5;
y1 = 2.9564;
y2 = 1.6423;
g = 9.8;
n = 6;

a = 1/(n*(y1 + y2));
b = y1*a;

P1 = [0;0;h-b*l];
P2 = P1 + [(-b*l)*sin(a*l);0;(-b*l)*cos(a*l)];
P3 = P2 + [(-b*l)*sin(2*a*l);0;(-b*l)*cos(2*a*l)];
P4 = P3 + [(-b*l)*sin(3*a*l);0;(-b*l)*cos(3*a*l)];
P5 = P4 + [(-b*l)*sin(4*a*l);0;(-b*l)*cos(4*a*l)];
P6 = P5 + [(-b*l)*sin(5*a*l);0;(-b*l)*cos(5*a*l)];

P1_dot = [0;0;b*l_dot];
P2_dot = P1_dot + [(-b*l)*cos(a*l)*(a*l_dot) + (-b*l_dot)*sin(a*l);
                    0;
                   (b*l)*sin(a*l)*(a*l_dot) + (-b*l_dot)*cos(a*l)];
P3_dot = P2_dot + [(-b*l)*cos(2*a*l)*(2*a*l_dot) + (-b*l_dot)*sin(2*a*l);
                    0;
                   (b*l)*sin(2*a*l)*(2*a*l_dot) + (-b*l_dot)*cos(2*a*l)];
P4_dot = P3_dot + [(-b*l)*cos(3*a*l)*(3*a*l_dot) + (-b*l_dot)*sin(3*a*l);
                    0;
                   (b*l)*sin(3*a*l)*(3*a*l_dot) + (-b*l_dot)*cos(3*a*l)];
P5_dot = P4_dot + [(-b*l)*cos(4*a*l)*(4*a*l_dot) + (-b*l_dot)*sin(4*a*l);
                    0;
                   (b*l)*sin(4*a*l)*(4*a*l_dot) + (-b*l_dot)*cos(4*a*l)];
P6_dot = P5_dot + [(-b*l)*cos(5*a*l)*(5*a*l_dot) + (-b*l_dot)*sin(5*a*l);
                    0;
                   (b*l)*sin(5*a*l)*(5*a*l_dot) + (-b*l_dot)*cos(5*a*l)];
P3_dot = simplify(P3_dot,'Steps',20);
P4_dot = simplify(P4_dot,'Steps',20);
P5_dot = simplify(P5_dot,'Steps',20);
P6_dot = simplify(P6_dot,'Steps',20);
m = 1;

%% Energy equations
syms k c real % spring and damper constants
s0 = 5;

T = 1/2*m*(P1_dot'*P1_dot + P2_dot'*P2_dot + P3_dot'*P3_dot +...
    P4_dot'*P4_dot + P5_dot'*P5_dot + P6_dot'*P6_dot);

U = m*[0 0 g]*(P1 + P2 + P3 + P4 + P5 + P6) + 1/2*k*(6*(h-b*l + 1/2*a*l-s0)^2);

R = 1/2*c*(6*(-b*l_dot + a*l_dot)^2);

%% Dynamic Model
G = simplify(diff(U,l),'Steps',20);
T_dot = simplify(diff(T,l_dot),'Steps',20);
T_part_diff = simplify(diff(T,l),'Steps',20);
R_diff = simplify(diff(R,l_dot),'Steps',20);

D = subs(T_dot,l_dot,1);
C = 0
G = simplify(diff(U,l),'Steps',20);