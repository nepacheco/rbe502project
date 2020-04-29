function path = get_robot_fwKin(robot, l, theta, d)
 

[k, s] = get_arc_params(robot, l);
transformation = zeros(4,4,2*(robot.n+1));
% Plotting multiple frames from multiple sections of tube
T_tip = get_arc_fwdkin(0,0,0);
transformation(:,:,1) = get_arc_fwdkin(0,0,0);
% Add section of un-notched tube
T_straight = get_arc_fwdkin(0, theta, d);
transformation(:,:,2) = T_straight;
%plot_section(f,T_tip,T_straight,0,0,outer_diameter/2);
T_tip = T_tip*T_straight;
%plot_section_frame(f,T_tip);
notch_disp = l/robot.n; % mm - the amount the tendon is pulled.

% For loop to go through the c, h, and g, matrices to create
[k, s] = robot.get_arc_params(notch_disp);
for i = 1:robot.n
    T_arc = get_arc_fwdkin(k(i), 0, s(i));
    transformation(:,:,2*i + 1) = T_arc;
    T_tip = T_tip*T_arc;
    T_straight = get_arc_fwdkin(0,robot.phi(i),robot.c(i));
    transformation(:,:,2*i+2) = T_straight;
    T_tip = T_tip*T_straight;
end
path = transformation;
end