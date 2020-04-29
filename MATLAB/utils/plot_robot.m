function plot_robot(robot, f, l, theta, d, varargin)
            if length(varargin) > 0
%                 disp(varargin)
                T_start = varargin(1);
            else 
%                 disp("NO START")
                T_start = eye(4);
            end
            T = get_robot_fwKin(robot,l,theta,d);            
            figure(f)
            l = l/robot.n;
            cla
            hold on
            T_tip = T_start*eye(4)*T(:,:,1);
            plot_section(f,T_tip, T(:,:,2), 0, 0, robot.r_o);
            T_tip = T_tip*T(:,:,2);

            [k, s] = robot.get_arc_params(l);
            for i = 2:length(T)/2
                plot_section(f,T_tip, T(:,:,2*i-1), k(i-1), s(i-1), robot.r_o);
                T_tip = T_tip*T(:,:,2*i-1);
                plot_section(f,T_tip, T(:,:,2*i), 0, 0, robot.r_o);
                T_tip = T_tip*T(:,:,2*i);
            end
            hold off
            
%             tendon = uicontrol('Parent',f,'Style','slider','Position',[200,50,300,23],...
%                 'value',l, 'min',-10, 'max',10);
%             tendon.Callback = @(es,ed) plot_robot(robot,f,theta,es.Value, d);
%             tenmin = uicontrol('Parent',f,'Style','text','Position',[170,50,23,23],...
%                 'String','-10');
%             tenmax = uicontrol('Parent',f,'Style','text','Position',[500,50,23,23],...
%                 'String','10');
%             tenlabel = uicontrol('Parent',f,'Style','text','Position',[1,50,150,23],...
%                 'String','Tendon Displacement (mm)');
%             
%             
%             rotator = uicontrol('Parent',f,'Style','slider','Position',[200,30,300,23],...
%                 'value',theta, 'min',0, 'max',6.28);
%             rotator.Callback = @(es,ed) plot_robot(robot,f,es.Value,l, d);
%             rotmin = uicontrol('Parent',f,'Style','text','Position',[170,30,23,23],...
%                 'String','0');
%             rotmax = uicontrol('Parent',f,'Style','text','Position',[500,30,23,23],...
%                 'String','2pi');
%             rotlabel = uicontrol('Parent',f,'Style','text','Position',[1,25,150,23],...
%                 'String','Rotation Angle (rad)');
%             
%             
%             translator = uicontrol('Parent',f,'Style','slider','Position',[200,10,300,23],...
%                 'value',d, 'min',-10, 'max',10);
%             translator.Callback = @(es,ed) plot_robot(robot,f,theta,l,es.Value);
%             transmin = uicontrol('Parent',f,'Style','text','Position',[170,10,23,23],...
%                 'String','-10');
%             transmax = uicontrol('Parent',f,'Style','text','Position',[500,10,23,23],...
%                 'String','10');
%             translabel = uicontrol('Parent',f,'Style','text','Position',[1,1,150,23],...
%                 'String','Translation Distance (mm)');
            axis equal
        end