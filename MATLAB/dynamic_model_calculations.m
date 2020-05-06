syms l l_dot l_dotdot
% syms y1 y2 h n
h = 5;
y1 = 2.9564;
y2 = 1.6423;
g1 = 9.8;
n = 6

a = 1/(n*(y1 + y2));
b = y1*a;

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