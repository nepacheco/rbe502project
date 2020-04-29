function transform = get_arc_fwdkin(kappa,phi,arc_length)
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