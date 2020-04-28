classdef Wrist < Robot
    
    properties
        % from parent class
        %       c - non cut height
        %       h - notched cut height
        %       r_i - inner radius of tube
        %       r_o - outer radius of tube
        %       g - cut depth
        g {mustBeNumeric}
        r_i {mustBeNumeric}
        boundary_conditions % holds radius height pairs of robot
        height_eqns
    end
    
    methods
        function obj = Wrist(c,h,g,phi,r_i,r_o,n)
            obj.c = c;
            obj.h = h;
            obj.g = g;
            obj.phi = phi;
            obj.r_i = r_i;
            obj.r_o = r_o;
            obj.n = n;
            [obj.boundary_conditions, obj.height_eqns] =...
                obj.set_boundary_conditions();
            obj.p = plot3(0,0,0);
        end
        
        function y_bar = get_bending_plane(obj,i)
            %GET_BENDING_PLANE - gets the neutral bending plane of tube
            %   gets the netural bending plane of the specific notched
            %   section as indicated by (i)
            phi_o = 2*acos((obj.g(i)-obj.r_o)/obj.r_o);
            phi_i = 2*acos((obj.g(i)-obj.r_o)/obj.r_i);
            y_bar_o = 4*obj.r_o*sin(1/2*phi_o)^3/(3*(phi_o - sin(phi_o)));
            y_bar_i = 4*obj.r_i*sin(1/2*phi_i)^3/(3*(phi_i - sin(phi_i)));
            A_o = (obj.r_o^2)*(phi_o - sin(phi_o))/2;
            A_i = (obj.r_i^2)*(phi_i - sin(phi_i))/2;
            y_bar = (y_bar_o*A_o - y_bar_i*A_i)/(A_o - A_i);
            
        end
        
        function [k, s] = get_arc_params(obj, l)
            %GET_ARC_PARAMS - gets the curvature and arc length of outer
            %tube
            %   Given a specific tnedon displacement this function
            %   will determine the curvature and arc length of each notch
            %   in the outer tube.
            % assumes the l given is the total tendon displacement
            k = zeros(1,obj.n);
            s = zeros(1,obj.n);

            l = l/obj.n; % divide the tendon equally among the notches
            for i = 1:obj.n
                y_bar = get_bending_plane(obj,i);
                k(i) = l/(obj.h(i)*(obj.r_i + y_bar) - l*y_bar);
                s(i) = obj.h(i)/(1 + y_bar*k(i));
            end
        end
        
        function l = get_l(obj, s)
            %GET_L gets the tendon displacement given an arc length
            %   Given a specific arc length, this function
            %   will output the amount of tendon displacement
            y_bar = obj.get_bending_plane(1);
            s = s - sum(obj.c); % subtract the sections of the tube that are not cut
            s = s/obj.n; % divide the remaining by the number of notches
            k = (obj.h(1)-s)/(y_bar*s);
            l = obj.n*(k*obj.h(1)*(obj.r_i + y_bar)/(1 - y_bar*k));
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
                p = obj.r_o; %+ (obj.n-1)*obj.c(i)/theta_max(i);
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
            obj.boundary_conditions = zeros(numPoints, 2);
            maxL = obj.get_max_l();
            increment = maxL/(numPoints - 1);
            deltaL = 0;
            for i = 1:numPoints
                [~, T_tip] = obj.get_robot_fwKin(deltaL, 0, obj.max_trans);
                obj.boundary_conditions(i,1) = T_tip(1,4);
                obj.boundary_conditions(i,2) = T_tip(3,4);
                deltaL = deltaL + increment;
            end
            poly_fwd = polyfit(obj.boundary_conditions(:,1),...
                obj.boundary_conditions(:,2)-ones(numPoints,1)*obj.max_trans,5);
            height_eqns = [poly_fwd];
            boundary_conditions = obj.boundary_conditions;
        end
        
        function check_bounds(obj, pos)
            %CHECK_BOUNDS - determines if a position is within bounds
            %   Given a position (x,y,Z) this function uses the previously
            %   set boundary conditions to determine if the position is
            %   within the workspace
            radius = sqrt(pos(1)^2 + pos(2)^2);
            numPoints = size(obj.boundary_conditions,1);
            if (radius > ...
                    obj.boundary_conditions(numPoints,1))
                fprintf("radius given: %f\nMaximum radius:%f\n",radius,obj.boundary_conditions(numPoints,1))
                error("Out of bounds: radius too large")
            else
                for i = 1:(numPoints-1)
                    % Determining which points to use for interpolation
                    min_radius = obj.boundary_conditions(i,1);
                    max_radius = obj.boundary_conditions(i+1,1);
                    if (radius >= min_radius && radius <= max_radius)
                        ratio = (radius - min_radius)/(max_radius - min_radius);
                        % lower bound of height for interpolation based on
                        % lower radius
                        min_height = obj.boundary_conditions(i,2);
                        % upper bound of height for interpolation based on
                        % larger radius
                        max_height = obj.boundary_conditions(i+1,2);
                        % determining the actually upper and lower bound
                        % that the given radius can be in between
                        top_height = (max_height - min_height)*ratio + min_height;
                        bot_height = (top_height - obj.max_trans);
                        
                        if ~(pos(3) <= top_height && pos(3) >= bot_height)
                            error("Out of bounds: height is incorrect")
                        end
                        break
                    end
                end
            end
        end
        
        function J = get_robot_dependent_jacobian(obj,l,theta,d)
            %GET_ROBOT_DEPENDENT_JACOBIAN - gets robot dependent jacobian
            %   gets the robot dependent jacobian by multiplying the robot
            %   independent jacobian by the matrix that converts actuator
            %   velocities sto arc velocities.
            J_robot = obj.get_robot_jacobian(l, theta, d);
            M = obj.get_arc_actuator_matrix(l);
            J = J_robot*M;
            
        end
        
        function M = get_arc_actuator_matrix(obj,l)
            %GET_ARC_ACTUATOR_MATRIX - converts actuator vel to arc vel
            %   Function creates a matrix that when right multiplied by
            %   actuator velocities will produce the necessary arc
            %   velocities for each section of the tube.
            M = [ 0 0 0; 0 1 0; 0 0 1];
            for i = 1:obj.n
                y_bar = obj.get_bending_plane(i);
                [k,s] = obj.get_arc_params(l);
                r = obj.r_i;
                h = obj.h(i);
                dk_dt = h*(r + y_bar)/...
                    (obj.n*(h*(r + y_bar) + l/obj.n * y_bar)^2);
                ds_dt = -h^2*y_bar*(r + y_bar)/...
                    (obj.n*(1 + y_bar*k(i))^2 * (h*(r + y_bar) + l/obj.n*y_bar)^2);
                M = [M;dk_dt 0 0; 0 0 0; ds_dt 0 0; zeros(3,3)];
            end
        end
        
        function L = get_max_l(obj)
            %GET_MAX_L - gets the maximum amount of delta in displacement
            %   function that gets the maximum amount of tube separation
            %   based on the largest amount of arc_length for each notch
            %   Capping the arc length at 90° or pi/2 radians
            L = 0;
            [~, theta_max] = obj.get_max_k();
            if sum(theta_max) > pi/2
                theta_max = pi/2;
            else
                theta_max = sum(theta_max);
            end
            for i = 1:obj.n
                y_bar = obj.get_bending_plane(i);
                L = L + (obj.r_i + y_bar)*theta_max/obj.n;
            end
        end
        
        function actuator_val = check_actuator_bounds(obj, actuator_val)
            %CHECK_ACTUATOR_BOUNDS - bounds the actuator values
            %   Determines if the actuators will put the robot in an
            %   out-of-bounds position and adjusts them to keep them in
            %   bounds
            % Making sure L doesn't get to large in magnitude
            l = actuator_val(1);
            max_l = obj.get_max_l();
            if l >= max_l
                l = max_l - 10*eps;
            elseif l <= 0
                l = 0.0001; % don't want to be at a singularity
            end
            actuator_val(1) = l;
            % Keeping theta between 2*pi and 0
            actuator_val(2) = mod(actuator_val(2),2*pi);
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
                desired_pos,tolerance,max_count,plot_on,use_guess)
            %%RUN_ANALYTIC_IKIN runs inverse kinematics through a geometric
            %%approach
            %   Uses law of sines and properties of isosceles triangles to
            %   determine the arc length and curvature necessary to create
            %   a circle that reachs a desired point from the origin
            l = 0; theta = 0; d = 0;
            if use_guess
                actuator_pos = obj.get_ikin_guess(desired_pos);
            else
                actuator_pos = [l;theta;d];
            end
            % theta is based on angle between y and x
            actuator_pos(2) = atan2(desired_pos(2),desired_pos(1));
            % Convert it to 2D perspective by combining x and y
            
            tip_error = eye(3,1);
            if plot_on
                fprintf("theta: %f\n",theta);
                fprintf("actuator_pos(2): %f\n",actuator_pos(2));
            end
            l_gain = 0.2;
            d_gain = 0.5;
            count = 0;
            while norm(tip_error) > tolerance
                actuator_pos = obj.check_actuator_bounds(actuator_pos);
                [~, T_tip] = obj.get_robot_fwKin(...
                    actuator_pos(1),actuator_pos(2),actuator_pos(3));
                tip_pos = T_tip(1:3,4);
                tip_error = desired_pos - tip_pos;
                x_error = norm(desired_pos(1:2)) - norm(tip_pos(1:2));
                actuator_pos(1) = actuator_pos(1) + l_gain*x_error;
                actuator_pos(3) = actuator_pos(3) + d_gain*tip_error(3);
                if plot_on
                    fprintf("tip_error: %f\n",tip_error);
                    fprintf("Change in s: %f\n",s_gain*x_error);
                end
                count = count + 1;
                if count > max_count
                    break;
                end
            end
            actuator_pos = obj.check_actuator_bounds(actuator_pos);
        end
        
        function actuator_pos = get_ikin_guess(obj,desired_pos)
            l = 0; theta = 0; d = 0;
            % theta is based on angle between y and x
            theta = atan2(desired_pos(2),desired_pos(1));
            actuator_pos = [l;theta;d];
            actuator_pos = obj.check_actuator_bounds(actuator_pos);
            % Convert it to 2D perspective by combining x and y
            x = sqrt(desired_pos(1)^2 + desired_pos(2)^2);
            z = desired_pos(3)- d;
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
%             actuator_pos(3) = actuator_pos(3) + d_gain*tip_error(3);
            actuator_pos = obj.check_actuator_bounds(actuator_pos);
        end
    end
end