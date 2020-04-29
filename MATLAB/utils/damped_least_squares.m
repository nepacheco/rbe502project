function DLS = damped_least_squares(J, lambda)
%DAMPED_LEAST_SQUARES returns the damped least squares inverse kinematics matrix
%   Determines the damped least squares matrix using the given damping
%   coefficient (lambda) for the given input matrix
DLS = J'/(J*J' + lambda^2*eye(size(J,2)));
end

