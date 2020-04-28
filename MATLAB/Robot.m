classdef (Abstract) Robot
    %ROBOT handles robot independent math and plotting
    %   Detailed explanation goes here
    
    properties
        n {mustBeNumeric} % number of nocthes
        c {mustBeNumeric} % length of uncut section of tube
        h {mustBeNumeric} % length of cut section of tube
        phi {mustBeNumeric} % rotation between notches
        r_o {mustBeNumeric} % the outer radius of robot
        max_trans = 50 % maximum translation of the robot
        p % The current plot where everything is stored
    end
    
    methods (Abstract)
        [k,s] = get_arc_params(obj,l)
        l = get_l(obj,s)
        y_bar = get_bending_plane(obj,i)
        k_max = get_max_k(obj)
        check_bounds(obj)
        set_boundary_conditions(obj)
        L = get_max_l(obj)
        M = get_arc_actuator_matrix(obj,l)
        J = get_robot_dependent_jacobian(obj,l,theta,d)
        actuator_val = check_actuator_bounds(obj,actuator_val)
        actuator_pos = run_analytic_ikin(obj, desired_pos,tolerance,max_count,plot_on)
        actuator_pos = get_ikin_guess(obj,desired_pos);
    end
    methods
        
        function [path,T_tip] = get_robot_fwKin(obj, l, theta, d)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % checking boundary conditions
            %             maxL = obj.get_max_l();
            %             if (l > maxL)
            %                 fprintf("The maximum value for L is %f\n",maxL);
            %                 fprintf("Using the maximum value instead of %f\n", l);
            %                 l = maxL;
            %             end
%             if (d > obj.max_trans)
%                 fprintf("The maximum value for d is %f\n",obj.max_trans);
%                 fprintf("Using the maximum value instead of %f\n", d);
%                 d = obj.max_trans;
%             end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Initial frame and translation section
            transformation = zeros(4,4,2*(obj.n+1));
            % Plotting multiple frames from multiple sections of tube
            T_tip = obj.get_arc_fwdkin(0,0,0);
            transformation(:,:,1) = obj.get_arc_fwdkin(0,0,0);
            % Add section of un-notched tube
            T_straight = obj.get_arc_fwdkin(0, theta, d);
            transformation(:,:,2) = T_straight;
            %plot_section(f,T_tip,T_straight,0,0,outer_diameter/2);
            T_tip = T_tip*T_straight;
            %plot_section_frame(f,T_tip);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % getting position of notches
            % For loop to go through the c, h, and g, matrices to create
            [k, s] = obj.get_arc_params(l);
            for i = 1:obj.n
                T_arc = obj.get_arc_fwdkin(k(i), 0, s(i));
                transformation(:,:,2*i + 1) = T_arc;
                T_tip = T_tip*T_arc;
                T_straight = obj.get_arc_fwdkin(0,obj.phi(i),obj.c(i));
                transformation(:,:,2*i+2) = T_straight;
                T_tip = T_tip*T_straight;
            end
            path = transformation;
        end
        
        function transform = get_arc_fwdkin(obj,kappa,phi,arc_length)
            % Define transformation matrices based on arc parameters
            % theta - rotation about the base
            % k - 1/r where r is the radius of curvature
            % s - arc length of the constant curvature section
            
            % rot is the twist rotation matrix as part of
            % creating the transformation matrix between
            % sections of the tube
            % phi - (radians)the base angle change around the z-axis between sections
            rot = @(theta) theta*[0 -1 0 0;...
                1  0 0 0;...
                0  0 0 0;...
                0  0 0 0];
            
            % inp is the linear translation described between
            % the two points of a tube
            % k - 1/r (mm) which is the radius of curvatature of the section
            % l - (mm) the arclength of the section of tube
            inp = @(k,s) s*[0 0 k 0;...
                0 0 0 0;...
                -k 0 0 1;...
                0 0 0 0];  % Webster says this matrix should be transposed
            % but it seems to produce the wrong matrix
            % when transposed
            
            T = @(k,theta,s) expm(rot(theta))*expm(inp(k,s));
            transform = T(kappa, phi,arc_length);
        end
        
        function J = get_notch_jacobian(obj, K, delta_phi, s)
            %GET_NOTCH_JACOBIAN - gives jacobian for one notch
            %   Returns the Robot Jacobian for a single notch given a
            %   curvature(k) rotation (phi) and arc length (s)
            J = [ (cos(delta_phi) * (cos(K * s) - 1))/(K^2) 0 0;...
                (sin(delta_phi) * (cos(K * s) - 1))/(K^2)   0 0;...
                -(sin(K * s) - (K * s))/ (K^2)                     0 1;...
                -s*sin(delta_phi)                                  0 -K*sin(delta_phi);...
                s*cos(delta_phi)                                   0  K*cos(delta_phi);...
                0                                                  1 0;
                ];
            
            if K == 0
                J = [ -(s^2*cos(delta_phi)/2) 0 0;...
                    -(s^2*sin(delta_phi)/2)   0 0;...
                    0                                0 1;...
                    -s*sin(delta_phi)                0 0;...
                    s*cos(delta_phi)                 0 0;...
                    0                                1 0;
                    ];
            end
        end
        
        function J = get_robot_jacobian(obj, l, theta, d)
            %GET_ROBOT_JACOBIAN - gets the full robot independent jacobian
            %   Given a tendon displacement, rotation, and linear
            %   translation, the function outputs the full robot jacobian
            %   for all notches of the robot.
            
            full_jacobian = [];
            transform = obj.get_robot_fwKin(l,theta,d);
            curr_transform = eye(4);
            [k,s] = obj.get_arc_params(l);
            temp_J = [];
            for i = 1:(obj.n*2) + 1
                curr_transform = curr_transform*transform(:,:,i);
                if(mod(i,2) == 0) % Notch Transform
                    b = int32(i/2);
                    temp_J = obj.get_notch_jacobian(k(b),0,s(b));
                elseif(i == 1) % First straight section transformation
                    temp_J = obj.get_notch_jacobian(0,theta,d);
                else
                    b = int32((i-1)/2); % Straight sections
                    temp_J = obj.get_notch_jacobian(0,obj.phi(b),obj.c(b));
                end
                % Add the tube section to the full jacobian
                full_jacobian = [full_jacobian adj(curr_transform)*temp_J];
            end
            J = full_jacobian;
        end
        
        function [actuator_pos,count] = run_ikin(obj,desired_pos,tolerance,max_count,plot_on)
            %RUN_IKIN runs the inverse kinematics of the wrist
            %   Function takes in the current actuator variables (l,theta,
            %   d) and the desired tip position (x,y,z) of the wrist. The
            %   function will plot if the variable plot on is true
            curr_actuator = obj.get_ikin_guess(desired_pos);
%             curr_actuator = [0;atan2(desired_pos(2),desired_pos(1));0];
%             curr_actuator = [0;0;0];
            G = eye(3,3);%.*[0.5;0.5;0.5]; % gain matrix
            lambda = 1;
            % obj.check_bounds(desired_pos); % Make sure pos is in bounds
            count = 0;
            error = [0;0;60] - desired_pos;
            if plot_on
                obj.plot_robot(0,0,0);
                str = sprintf("Tip Error\nX: %f\nY: %f\nZ: %f",error);
                dim = [0.6,0.3,0.3,0.3];
                a = annotation('textbox',dim,'String',str,'FitBoxToText','on');
                w = waitforbuttonpress;
                pause(1)
            end
            [~,T_tip] = obj.get_robot_fwKin(curr_actuator(1),curr_actuator(2),curr_actuator(3));
            tip_pos = T_tip(1:3,4);
            error = tip_pos - desired_pos;
            if plot_on
                obj.plot_robot(curr_actuator(1),curr_actuator(2),curr_actuator(3));
                str = sprintf("Tip Error\nX: %f\nY: %f\nZ: %f",error);
                a.String = str;
                pause(0.5)
            end
            while(norm(error) > tolerance)
                % Getting the jacobian and its inverse
                J = obj.get_robot_dependent_jacobian(curr_actuator(1),curr_actuator(2),curr_actuator(3));
                [Jp, Jp_inv] = skewm(J,tip_pos);
                if plot_on
                    fprintf("\nActuator l:%f, theta:%f, d: %f\n",curr_actuator);
                    disp(J)
                    fprintf("Error:x: %f, y: %f, z: %f\n", error);
                    disp(Jp_inv)
                end
                % Perform Damped Least Squares Calculation
%                 DLS = damped_least_squares(Jp,lambda);
                % updating actuator velocities
                %actuator_vel = -DLS*error;
                actuator_vel = -Jp_inv*G*error;
                curr_actuator = curr_actuator + actuator_vel;
                % Maintain boundaries
                curr_actuator = obj.check_actuator_bounds(curr_actuator);
                if plot_on
                    fprintf("Actuator vel, l:%f, theta:%0.20f, d:%f\n",actuator_vel);
                end
                % update error
                [~,T_tip] = obj.get_robot_fwKin(curr_actuator(1),curr_actuator(2),curr_actuator(3));
                tip_pos = T_tip(1:3,4);
                error = tip_pos - desired_pos;
                count = count + 1;
                if plot_on
                    obj.plot_robot(curr_actuator(1),curr_actuator(2),curr_actuator(3));
                    str = sprintf("Tip Error\nX: %f\nY: %f\nZ: %f",error);
                    a.String = str;
                    pause(0.5)
                end
                if count >= max_count
%                     fprintf("max count\n");
                    break
                end
            end
            if plot_on
                fprintf("It took %d number of iterations\n", count);
                fprintf("L: %f, Theta: %f, D: %f\n",curr_actuator);
            end
            curr_actuator = obj.check_actuator_bounds(curr_actuator);
            actuator_pos = curr_actuator;
        end
        
        function plot_robot(obj, l, theta, d, varargin)
            %PLOT_ROBOT - plots the sections of the robot
            %   Takes in the actuator variables of the tube and will plot
            %   the tube. Can also plot multiple tubes by giving the
            %   transformation of the tip of the previous tube as the last
            %   argument
            if (d > obj.max_trans)
                fprintf("The maximum value for d is %f\n",obj.max_trans);
                fprintf("Using the maximum value instead of %f\n", d);
                d = obj.max_trans;
            end
            % If we are doing multiple tubes we put the tip of the previous
            % tube as the the argument
            if ~isempty(varargin)
                %                 disp(varargin)
                T_start = varargin(1);
            else
                %                 disp("NO START")
                T_start = eye(4);
            end
            T = obj.get_robot_fwKin(l,theta,d);
            
            % clearing previous plots data
            obj.p.XData = 0;
            obj.p.YData = 0;
            obj.p.ZData = 0;
            % Gets the position of the base of the robot
            T_tip = T_start*eye(4)*T(:,:,1);
            % plots the initial translated section of the tube
            obj.plot_section(T_tip, T(:,:,2), 0, 0, obj.r_o);
            % update the tip of the robot
            T_tip = T_tip*T(:,:,2);
            
            [k, s] = obj.get_arc_params(l);
            
            
%             children = get(obj.p.Parent,'Children');
%             j = length(children);
%             for i=1:j
%                 if isa(children(i), 'matlab.graphics.chart.primitive.Quiver')
%                      j = j - 1;
%                     delete(children(i));
%                 end
%             end
            % for loop for the notches and their respective straight
            % sections
            for i = 2:length(T)/2
                obj.plot_section(T_tip, T(:,:,2*i-1),...
                    k(i-1), s(i-1), obj.r_o);
                T_tip = T_tip*T(:,:,2*i-1);
                obj.plot_section(T_tip, T(:,:,2*i), 0, 0, obj.r_o);
                T_tip = T_tip*T(:,:,2*i);
            end
            obj.p.Color = 'b';
            obj.p.LineWidth = 4;
            drawnow()
            max_length = sum(obj.c) + sum(obj.h) + obj.max_trans;
            axis equal
            xlim([-2,max_length/2]);
            ylim([-max_length/2,max_length/2]);
            zlim([0,70]); % should be max length, changed to record video
            xlabel('X Axis (mm)');
            ylabel('Y Axis (mm)');
            zlabel('Z Axis (mm)');

            grid on
        end
        
        function plot_section(obj,T_init,T_change, k, s, r_o)
            %PLOT_SECTION Plots a section of the tube
            %   f - the figure to plot on
            %   T_init - the initial transformation matrix
            %   T_change - the next transformation matrix
            %   k - curvature of the arc
            %   r_o - the outer diameter of the tube
%             hold on
            T = T_init*T_change;
            %plot_section_frame(obj.p.Parent, T);
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
                %                 plot3(circle(1,:),circle(2,:),circle(3,:));
                obj.p.XData = [obj.p.XData circle(1,:)];
                obj.p.YData = [obj.p.YData circle(2,:)];
                obj.p.ZData = [obj.p.ZData circle(3,:)];
                %plot_cylinder(fig,circle(1,:),circle(2,:),circle(3,:),r_o,true,T_init,T);
                
            else
                pathx = linspace(init_pos(1),final_pos(1),100); % x path traveled
                pathy = linspace(init_pos(2),final_pos(2),100); % y path traveled
                pathz = linspace(init_pos(3),final_pos(3),100);
                obj.p.XData = [obj.p.XData pathx];
                obj.p.YData = [obj.p.YData pathy];
                obj.p.ZData = [obj.p.ZData pathz];
                %plot_cylinder(pathx,pathy,pathz,r_o,false,1,1);
            end
%             hold off
        end
    end
end
