function Adg = ADJOINT(g)
%ADJOINT(g) takes 4-by-4 homogeneous matrix and generates corresponding 
%6-by-6 adjoint matrix
%Ad(g) = [R (p^)R;0 R]
[rows, cols] = size(g);
    if ((rows ~= 4) || (cols ~= 4))
        error('ADJOINT requires a 4x4 matrix argument. Check your dimensions.');
    end

R = g(1:3,1:3);
p = g(1:3,4);
p_hat = HATOPT(p);

Adg = [R p_hat*R;
       zeros(3) R];

end

