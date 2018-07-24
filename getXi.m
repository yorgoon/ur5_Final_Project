function xi = getXi( g )
%Take a homogenous transformation matrix and extract the unscaled twist
%
%      [ 0   -a3    a2 ]
% a^ = [ a3   0    -a1 ]
%      [-a2   a1    0  ]
%
[rows, cols] = size(g);
if ((rows ~= 4) || (cols ~= 4))
    error('getXi requires a 4-by-4 homogeneous matrix argument. Check your dimensions.');
end

xi_hat = logm(g);
xi = [xi_hat(1:3,4);xi_hat(3,2);xi_hat(1,3);xi_hat(2,1)];

end

