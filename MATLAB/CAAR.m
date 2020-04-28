classdef CAAR < Robot
    
    properties
        robot_1  % external wrist
        robot_2  % internal wrist
        boundary_conditions % contains boundary for bending forward and backwards
        height_eqns
        %         r_o {mustBeNumeric}
        %         c {mustBeNumeric}
        %         h {mustBeNumeric}
        %         phi {mustBeNumeric}
        %         n {mustBeNumeric} % number of notches in caar
    end
    
    methods
        function obj = CAAR(robot1, robot2, n)
            obj.robot_1 = robot1;
            obj.robot_2 = robot2;
            obj.n = n;
            obj.r_o = robot1.r_o;
            obj.c = robot1.c;
            obj.h = robot1.h;
            obj.phi = robot1.phi;
            [obj.boundary_conditions, obj.height_eqns] =...
                obj.set_boundary_conditions();
            obj.p = plot3(0,0,0);
        end
        
        function [ybar_o,ybar_i] = get_bending_plane(obj,i)
            %GET_BENDING_PLANE - gets the neutral axis of the tubes
            %   Gets the neutral bending plane of both notched tubes of the
            %   car.
            %       i - the notch you want to bend on
            ybar_o = obj.robot_1.get_bending_plane(i);
            ybar_i = obj.robot_2.get_bending_plane(i);
        end
        
        function [k, s] = get_arc_params(obj, l)
            %GET_ARC_PARAMS - gets the curvature and arc length of outer
            %tube
            %   Given a specific displacement of the tubes this function
            %   will determine the curvature and arc length of each notch
            %   in the outer tube.
            k = zeros(1,obj.n);
            s = zeros(1,obj.n);
            h_notch = obj.robot_1.h;
            l = l/obj.n; % divide the tendon equally among the notches
            for i = 1:obj.n
                % Equations from Swaney 2010
                [ybar_o, ybar_i] = get_bending_plane(obj,i);
                gamma = l/(ybar_o + ybar_i);
                s(i) = h_notch(i) - (ybar_o*gamma);
                k(i) = gamma/s(i);
            end
        end
        
        function l = get_l(obj, s)
            %GET_L gets the tendon displacement given an arc length
            %   Given a specific arc length, this function
            %   will output the amount of tendon displacement
            [ybar_o, ybar_i] = obj.get_bending_plane(1);
            s = s - sum(obj.c); % subtract the sections of the tube that are not cut
            s = s/obj.n; % divide the remaining by the number of notches
            gamma = -(s - obj.h(1))/ybar_o;
            l = gamma*(ybar_o + ybar_i)*obj.n;
        end
        
        function [k_max, theta_max] = get_max_k(obj)
            %GET_MAX_K - gets the maximum curvature possible for each notch
            %   Each section of the tube has a maximum curvature and this
            %   function will determine that number for each notch. The
            %   equations differ from Webster 2010 slightly for our
            %   purposes.
            theta_max = zeros(1,obj.n);
            k_max = zeros(1, obj.n);
            for i = 1:obj.n
                y_bar = obj.get_bending_plane(i);
                theta_max(i) = obj.h(i)/(obj.r_o + y_bar);
                p = obj.r_o;
                k_max(i) = 1/p;
            end
        end
        
        function [boundary_conditions, height_eqns] = set_boundary_conditions(obj)
            %SET_BOUNDARY_CONDITIONS - determines workspace of tubes
            %   this function fills in an array with (radius,height) pairs
            %   to determine where the upper boundary of the robot is. To
            %   find the lower bound just subtract that maximum allowed
            %   translation
            disp("Setting boundary conditions")
            numPoints = 100;
            obj.boundary_conditions = zeros(numPoints, 2,2);
            maxL = obj.get_max_l();
            increment = maxL/(numPoints - 1);
            deltaL = 0;
            % Forward direction boundary
            for i = 1:numPoints % Looping from 0-MaxL
                [~, T_tip] = obj.get_robot_fwKin(deltaL, 0, obj.max_trans);
                obj.boundary_conditions(i,1,1) = T_tip(1,4);% radius is x distance
                obj.boundary_conditions(i,2,1) = T_tip(3,4);% height is z distance
                deltaL = deltaL + increment;
            end
            poly_fwd = polyfit(obj.boundary_conditions(:,1,1),...
                obj.boundary_conditions(:,2,1)-ones(numPoints,1)*obj.max_trans,5);
            deltaL = 0;
            % backwards direction boundary
            for i = 1:numPoints % Looping from 0-MaxL
                [~, T_tip] = obj.get_robot_fwKin(deltaL, 0, obj.max_trans);
                obj.boundary_conditions(i,1,2) = abs(T_tip(1,4)); % radius is x distance
                obj.boundary_conditions(i,2,2) = T_tip(3,4); % height is z distance
                deltaL = deltaL - increment;
            end
            poly_bkwd = polyfit(obj.boundary_conditions(:,1,2),...
                obj.boundary_conditions(:,2,2)-ones(numPoints,1)*obj.max_trans,5);
            height_eqns = [poly_fwd;poly_bkwd];
            boundary_conditions = obj.boundary_conditions;
        end
        
        function check_bounds(obj, pos)
            %CHECK_BOUNDS - determines if a position is within bounds
            %   Given a position (x,y,Z) this function uses the previously
            %   set boundary conditions to determine if the position is
            %   within the workspace
            radius = sqrt(pos(1)^2 + pos(2)^2);
            numPoints = size(obj.boundary_conditions,1);
            direction = 1;
            if (atan2(pos(2),pos(1)) > pi/2 || atan2(pos(2),pos(1)) < -pi/2)
                direction = 2;
            end
            if (radius > ...
                    obj.boundary_conditions(numPoints,1,direction))
                ME = MException(...
                    "Radius:OOB","radius given: %f\nMaximum radius:%f\n",...
                    radius,obj.boundary_conditions(numPoints,1,direction));
                throw(ME);
            else
                for i = 1:(numPoints-1)
                    % Determining which points to use for interpolation
                    min_radius = obj.boundary_conditions(i,1,direction);
                    max_radius = obj.boundary_conditions(i+1,1,direction);
                    if (radius >= min_radius && radius <= max_radius)
                        ratio = (radius - min_radius)/(max_radius - min_radius);
                        % lower bound of height for interpolation based on
                        % lower radius
                        min_height = obj.boundary_conditions(i,2,direction);
                        % upper bound of height for interpolation based on
                        % larger radius
                        max_height = obj.boundary_conditions(i+1,2,direction);
                        % determining the actually upper and lower bound
                        % that the given radius can be in between
                        top_height = (max_height - min_height)*ratio + min_height;
                        bot_height = (top_height - obj.max_trans);
                        
                        if ~(pos(3) <= top_height && pos(3) >= bot_height)
                            ME = MException("Height:OOB","height is out of bounds");
                            throw(ME);
                        end
                        break
                    end
                end
            end
        end
        
        function L = get_max_l(obj)
            %GET_MAX_L - gets the maximum amount of delta in displacement
            %   function that gets the maximum amount of tube separation
            %   based on the largest amount of arc_length for each notch
            %   Capping the arc length at 90° or pi/2 radians
            L = 0;
            [~, gamma_max] = obj.get_max_k();
            if sum(gamma_max) > pi/2
                gamma_max = pi/2;
            else
                gamma_max = sum(gamma_max);
            end
            for i = 1:obj.n
                [y_baro, y_bari] = obj.get_bending_plane(i);
                L = L + (gamma_max/obj.n)*(y_baro + y_bari);
            end
        end
        
        function M = get_arc_actuator_matrix(obj,l)
            %GET_ARC_ACTUATOR_MATRIX - converts actuator vel to arc vel
            %   Function creates a matrix that when right multiplied by
            %   actuator velocities will produce the necessary arc
            %   velocities for each section of the tube.
            
            % equations are based on Swaney 2010
            M = [ 0 0 0; 0 1 0; 0 0 1];
            for i = 1:obj.n
                [y_baro, y_bari] = obj.get_bending_plane(i);
                [k,s] = obj.get_arc_params(l);
                dtheta_dt = 1/(obj.n*(y_baro + y_bari));
                theta = k(i)*s(i);
                ds_dt = -y_baro/(obj.n*(y_baro + y_bari));
                dk_dt = (dtheta_dt*s(i) - ds_dt*theta)/s(i)^2;
                M = [M;dk_dt 0 0; 0 0 0; ds_dt 0 0; zeros(3,3)];
            end
        end
        
        function J = get_robot_dependent_jacobian(obj,l,theta,d)
            %GET_ROBOT_DEPENDENT_JACOBIAN - gets robot dependent jacobian
            %   gets the robot dependent jacobian by multiplying the robot
            %   independent jacobian by the matrix that converts actuator
            %   velocities sto arc velocities.
            M = obj.get_arc_actuator_matrix(l);
            Jrobot = obj.get_robot_jacobian(l,theta,d);
            J = Jrobot*M;
        end
        
        function actuator_val = check_actuator_bounds(obj, actuator_val)
            %CHECK_ACTUATOR_BOUNDS - bounds the actuator values
            %   Determines if the actuators will put the robot in an
            %   out-of-bounds position and adjusts them to keep them in
            %   bounds
            % Making sure L doesn't get to large in magnitude
            l = actuator_val(1);
            max_l = obj.get_max_l();
            if abs(l) > max_l
                actuator_val(1) = max_l*l/abs(l);
            end
            % Keeping theta between -pi/2 and pi/2
            theta = actuator_val(2);
            theta = mod(theta,pi);
            if theta > pi/2
                theta = theta - pi;
            end
            actuator_val(2) = theta;
            % Limiting d to be between 0 and max_trans
            d = actuator_val(3);
            if d < 0
                d = 0;
            elseif d > obj.max_trans
                d = obj.max_trans;
            end
            actuator_val(3) = d;
        end
        
        function [actuator_pos,count] = run_analytic_ikin(obj,...
                desired_pos, tolerance,max_count, plot_on, use_guess)
            %%RUN_ANALYTIC_IKIN runs inverse kinematics through a geometric
            %%approach
            %   Uses law of sines and properties of isosceles triangles to
            %   determine the arc length and curvature necessary to create
            %   a circle that reachs a desired point from the origin
%             obj.check_bounds(desired_pos);
            l = 0; theta = 0; d = 0;
            % theta is based on angle between y and x
            theta = atan2(desired_pos(2),desired_pos(1));
            if use_guess
                actuator_pos = obj.get_ikin_guess(desired_pos);
            else
                actuator_pos = [l;theta;d];
            end
            tip_error = [0;0;60] - desired_pos;
            if plot_on
                obj.plot_robot(0,0,0);
                str = sprintf("Tip Error\nX: %f\nY: %f\nZ: %f",tip_error);
                dim = [0.6,0.3,0.3,0.3];
                a = annotation('textbox',dim,'String',str,'FitBoxToText','on');
                w = waitforbuttonpress;
                pause(1)
                obj.plot_robot(actuator_pos(1),actuator_pos(2),...
                    actuator_pos(3));
                pause(0.5)
            end
            actuator_pos = obj.check_actuator_bounds(actuator_pos);
            % Convert it to 2D perspective by combining x and y
            
            if plot_on
                fprintf("Initial l: %f\n",actuator_pos(1));
                fprintf("theta: %f\n",theta);
                fprintf("actuator_pos(2): %f\n",actuator_pos(2));
                fprintf("Initial d: %f\n",actuator_pos(3));
            end
            
            tip_error = eye(3,1);
            if norm(actuator_pos(2) - theta) > eps % Bending Backwards
                direction = -1;
            else % bending forwards
                direction = 1;
            end
            if plot_on
                fprintf("Direction: %f\n",direction);
            end
            % this needs to have the negative offset
            % to guarentee that it doesn't try negative tendon displacement
            count = 0;
            l_gain = 0.19*direction;
            d_gain = 0.7;
            % Get Initial Error
            [~, T_tip] = obj.get_robot_fwKin(...
                actuator_pos(1),actuator_pos(2),actuator_pos(3));
            tip_pos = T_tip(1:3,4);
            tip_error = desired_pos - tip_pos;
            x_error = norm(desired_pos(1:2)) - norm(tip_pos(1:2));
            while norm(tip_error) > tolerance
                % update actuators based on error
                actuator_pos(1) = actuator_pos(1) + l_gain*x_error;
                actuator_pos(3) = actuator_pos(3) + d_gain*tip_error(3);
                actuator_pos = obj.check_actuator_bounds(actuator_pos);
                if plot_on
                    pause(0.5)
                    fprintf("Tip Pos, x: %f, y: %f, z: %f\n",tip_pos);
                    fprintf("tip_error: %f, %f, %f\n",tip_error);
                    fprintf("radial error: %f\n",x_error);
                    fprintf("Change in s: %f\n",l_gain*x_error);
                    fprintf("Translation Value: %f\n\n",actuator_pos(3));
                end
                % recalculate error
                [~, T_tip] = obj.get_robot_fwKin(...
                actuator_pos(1),actuator_pos(2),actuator_pos(3));
                tip_pos = T_tip(1:3,4);
                tip_error = desired_pos - tip_pos;
                x_error = norm(desired_pos(1:2)) - norm(tip_pos(1:2));
                count = count + 1;
                if plot_on
                    obj.plot_robot(actuator_pos(1),actuator_pos(2),...
                        actuator_pos(3));
                    str = sprintf("Tip Error\nX: %f\nY: %f\nZ: %f",tip_error);
                    a.String = str; 
                end
                if count >= max_count
                    break;
                end
            end
            if plot_on
                fprintf("Count: %d",count);
            end
            actuator_pos = obj.check_actuator_bounds(actuator_pos);
        end
        
        function actuator_pos = get_ikin_guess(obj,desired_pos)
            l = 0; theta = 0; d = 0;
            % theta is based on angle between y and x
            theta = atan2(desired_pos(2),desired_pos(1));
            actuator_pos = [l;theta;d];
            actuator_pos = obj.check_actuator_bounds(actuator_pos);
            if norm(actuator_pos(2) - theta) > 0.001
                direction = 2; % Bending backwards
            else
                direction = 1; % Bending forward
            end
            poly_height = obj.height_eqns(direction,:);
            % Convert it to 2D perspective by combining x and y
            x = sqrt(desired_pos(1)^2 + desired_pos(2)^2);
%             d = desired_pos(3) - polyval(poly_height,x);
            if d < 0
                d = 0;
            end
            z = desired_pos(3) - d;
            % Geometry
            hyp = sqrt(x^2 + z^2);
            alpha = atan2(z,x);
            psi = pi - 2*alpha;
            r = hyp*sin(alpha)/sin(psi);
            s = psi*r;
            if r == Inf
                actuator_pos(1) = 0;
            else
                actuator_pos(1) = obj.get_l(s);
            end
            %%%% brute force fix %%%%%
            % this is necessary because the current polynomial over
            % estimates how much d is necessary sometimes
            if direction == 2
                actuator_pos(1) = -abs(actuator_pos(1));
            else
                actuator_pos(1) = abs(actuator_pos(1));
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%
            actuator_pos(3) = d;
            actuator_pos = obj.check_actuator_bounds(actuator_pos);
        end
    end
end
