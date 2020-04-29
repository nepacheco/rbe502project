function plot_section_frame(f,T)
%plot_section Plots one section of the tube based on transformation matrix
%       figure - the figure to plot on
%       T - the transformation matrix
%figure(f)
%hold on
%grid on
axes(f);
%this plots the frame of the first link based on transformation matrix
q = quiver3(ones(3,1)*T(1,4),ones(3,1)*T(2,4),ones(3,1)*T(3,4),...
    T(1,1:3)',T(2,1:3)',T(3,1:3)',0.5);
q.AutoScaleFactor = 3;

end

