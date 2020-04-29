function [Jp,Jp_inv] = skewm(J,pos)
%SKEWM takes the skew of a matrix relative to a given position
%   J is 6xn matrix, pos is 3x1 vector
Jp = -skew(pos)*J(4:6,:) + J(1:3,:);
Jp_inv = pinv(Jp);
end

