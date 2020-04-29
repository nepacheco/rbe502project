function keyboard_control(robot, handles)
    height = 0;
    for i=1:length(robot.c)
        height=height+robot.c(i);
    end
    for i=1:length(robot.h)
        height=height+robot.h(i);
    end
    robot.run_ikin([0;0;0],[0;0;height],true)
    global x y z curr_pos plot_on
    while(1)
        w = waitforbuttonpress();
        value = get(ancestor(robot.p, 'figure'),'CurrentCharacter');
        vel_plot = false;
        if w == 1
            if value == 'w'
                x = x + .1;
                temp = robot.run_ikin([curr_pos(1);curr_pos(2);curr_pos(3)],[x;y;z],vel_plot);
                curr_pos = temp;
                 set(handles.text15, 'String', curr_pos(1));
                 set(handles.text18, 'String', curr_pos(2));
                 set(handles.text19, 'String', curr_pos(3));
                 set(handles.text29, 'String', x);
                 set(handles.text32, 'String', y);
                 set(handles.text33, 'String', z);
                 set(handles.slider1, 'value', x);
                 set(handles.slider4, 'value', y);
                 set(handles.slider5, 'value', z);
                  if plot_on == 1
                    robot.plot_robot(curr_pos(1),curr_pos(2),curr_pos(3));
                 end
            end
            if value == 's'
                x = x - .1;
                temp = robot.run_ikin([curr_pos(1);curr_pos(2);curr_pos(3)],[x;y;z],vel_plot);
                curr_pos = temp;
                 set(handles.text15, 'String', curr_pos(1));
                 set(handles.text18, 'String', curr_pos(2));
                 set(handles.text19, 'String', curr_pos(3));
                 set(handles.text29, 'String', x);
                 set(handles.text32, 'String', y);
                 set(handles.text33, 'String', z);
                 set(handles.slider1, 'value', x);
                 set(handles.slider4, 'value', y);
                 set(handles.slider5, 'value', z);
                  if plot_on == 1
                    robot.plot_robot(curr_pos(1),curr_pos(2),curr_pos(3));
                 end
            end
            if value == 'a'
                y = y + .1;
                temp = robot.run_ikin([curr_pos(1);curr_pos(2);curr_pos(3)],[x;y;z],vel_plot);
                curr_pos = temp;
                 set(handles.text15, 'String', curr_pos(1));
                 set(handles.text18, 'String', curr_pos(2));
                 set(handles.text19, 'String', curr_pos(3));
                 set(handles.text29, 'String', x);
                 set(handles.text32, 'String', y);
                 set(handles.text33, 'String', z);
                 set(handles.slider1, 'value', x);
                 set(handles.slider4, 'value', y);
                 set(handles.slider5, 'value', z);
                  if plot_on == 1
                    robot.plot_robot(curr_pos(1),curr_pos(2),curr_pos(3));
                 end
            end
            if value == 'd'
                y = y - .1;
                temp = robot.run_ikin([curr_pos(1);curr_pos(2);curr_pos(3)],[x;y;z],vel_plot);
                curr_pos = temp;
                 set(handles.text15, 'String', curr_pos(1));
                 set(handles.text18, 'String', curr_pos(2));
                 set(handles.text19, 'String', curr_pos(3));
                 set(handles.text29, 'String', x);
                 set(handles.text32, 'String', y);
                 set(handles.text33, 'String', z);
                 set(handles.slider1, 'value', x);
                 set(handles.slider4, 'value', y);
                 set(handles.slider5, 'value', z);
                  if plot_on == 1
                    robot.plot_robot(curr_pos(1),curr_pos(2),curr_pos(3));
                 end
            end
            if value == 'q'
                z = z + .1;
                temp = robot.run_ikin([curr_pos(1);curr_pos(2);curr_pos(3)],[x;y;z],vel_plot);
                curr_pos = temp;
                 set(handles.text15, 'String', curr_pos(1));
                 set(handles.text18, 'String', curr_pos(2));
                 set(handles.text19, 'String', curr_pos(3));
                 set(handles.text29, 'String', x);
                 set(handles.text32, 'String', y);
                 set(handles.text33, 'String', z);
                 set(handles.slider1, 'value', x);
                 set(handles.slider4, 'value', y);
                 set(handles.slider5, 'value', z);
                  if plot_on == 1
                    robot.plot_robot(curr_pos(1),curr_pos(2),curr_pos(3));
                 end
            end
             if value == 'e'
                z = z - .1;
                temp = robot.run_ikin([curr_pos(1);curr_pos(2);curr_pos(3)],[x;y;z],vel_plot);
                curr_pos = temp;
                 set(handles.text15, 'String', curr_pos(1));
                 set(handles.text18, 'String', curr_pos(2));
                 set(handles.text19, 'String', curr_pos(3));
                 set(handles.text29, 'String', x);
                 set(handles.text32, 'String', y);
                 set(handles.text33, 'String', z);
                 set(handles.slider1, 'value', x);
                 set(handles.slider4, 'value', y);
                 set(handles.slider5, 'value', z);
                 if plot_on == 1
                    robot.plot_robot(curr_pos(1),curr_pos(2),curr_pos(3));
                 end
             end
        end
        if w == 0
        end
    end
end