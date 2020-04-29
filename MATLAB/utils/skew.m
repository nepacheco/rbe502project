function skew_mat = skew(w)
mat = @(q)[ 0  -q(3)  q(2); 
           q(3)    0  -q(1);
          -q(2)  q(1)    0];

skew_mat = mat(w);
end

