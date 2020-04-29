function plot_section(f,T_init,T_change, k, s, r_o)
%PLOT_SECTION Plots a section of the tube
%   f - the figure to plot on
%   T_init - the initial transformation matrix
%   T_change - the next transformation matrix
%   k - curvature of the arc
%   r_o - the outer diameter of the tube

figure(f);
%hold off

T = T_init*T_change;
init_pos = T_init(1:3,4); % initial position of arc
final_pos = T(1:3,4); % ending position of arc

if (k ~= 0) % if there is curvature
    T_radius = T_init*[1/k; 0; 0; 1];
    T_radius_matrix = [T_init(:,1:3) T_radius];
    
    u = T(1:3,3); % vector of (x,y) orientation of center of circle
    u = u./norm(u); % unit vector
    xc = T_radius(1); % get the center x of circle
    yc = T_radius(2); % get the center y of circle
    zc = T_radius(3); % center z of the circle
    
    theta = k*s;
    theta_diff = linspace(0,theta,100);
    x_diff = -1/k*cos(theta_diff);
    z_diff = 1/k*sin(theta_diff);
    circle = T_radius_matrix*[x_diff;zeros(1,100);z_diff;ones(1,100)];
    plot3(circle(1,:),circle(2,:),circle(3,:));
   % plot_cylinder(f,circle(1,:),circle(2,:),circle(3,:),r_o,true,T_init,T);
    
else
    pathx = linspace(init_pos(1),final_pos(1),100); % x path traveled
    pathy = linspace(init_pos(2),final_pos(2),100); % y path traveled
    pathz = linspace(init_pos(3),final_pos(3),100);
    plot3(pathx,pathy,pathz);
    %plot_cylinder(f,pathx,pathy,pathz,r_o,false,1,1);
end

