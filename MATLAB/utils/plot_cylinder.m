function plot_cylinder(X, Y, Z, r_o, isCurved, T_init, T_final)

curve = [X',Y',Z'];

if(isCurved)
    
%     disp("Transformation matrix")
%     disp(T_init)
%     test = [X(2)-X(1);Y(2)-Y(1);Z(2)-Z(1)];
%     test = test/norm(test);
%     disp("test vector orthogonality")
%     null(test')
%     disp(test)
    
    
    th = 3*pi/4:pi/50:(5*pi)/4;
    %th = 0:pi/50:2*pi;
    zunit = r_o * cos(th);
    yunit = r_o * sin(th);
    
    circle = [zunit;
        yunit;
        zeros(1,length(zunit));
        ones(1,length(zunit))];
    
    for i = 1:10:length(X)
        circle_init = [];
        circle_end = [];
        for col = 1:length(zunit)
            circle_init = [circle_init T_init*circle(:,col)];
            circle_end = [circle_end T_final*circle(:,col)];
        end

        plot3(circle_init(1,:), circle_init(2,:),circle_init(3,:), '-b' );
        plot3(circle_end(1,:), circle_end(2,:),  circle_end(3,:),'-b');

        surf([circle_init(1,:);circle_end(1,:)],...
                [circle_init(2,:);circle_end(2,:)] ,...
                [circle_init(3,:);circle_end(3,:)])
    end
    
else
    tubeplot(curve', r_o, 8, 20);
end



end
