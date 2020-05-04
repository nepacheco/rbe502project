function robot = create_robot(n)
%create_robot creates a robot with the specified number of notches
r_o = 7.2/2; % mm - the outer r of tube
r_i = 5.2/2; % mm - the inner r of tube
c = ones(1,n)*5; % mm - the length of non-cut sections
h = ones(1,n)*5; % mm - the length of notched section
g = ones(1,n)*6.1; % mm - the depth of the cut
phi = ones(1,n)*0;
robot1 = Wrist(c, h, g, phi, r_i, r_o, n);

r_o = 4.2/2; % mm - the outer r of tube
r_i = 3.2/2; % mm - the inner r of tube
g = ones(1,n)*3.3; % mm - the depth of the cut
robot2 = Wrist(c, h, g, phi, r_i, r_o, n);
robot = CAAR(robot1,robot2,n);

end
