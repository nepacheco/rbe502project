
%Hey it's me Joe I switched this up because Fichera told me to
function A = adj(T)
    A = [T(1:3,1:3)          skew(T(1:3,4))*T(1:3,1:3);
          zeros(3,3)         T(1:3,1:3)];
end