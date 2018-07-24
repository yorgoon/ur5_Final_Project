function g = XF( xi )
%g = XF(xi) accepts a single 6  1 vector containing [x; y; z; theta1; theta2; theta3] 
%(angles in radians) and returns the corresponding 4-by-4 homogeneous transformation.

[rows, cols] = size(xi);
if ((rows ~= 6) || (cols ~= 1))
  error('XF requires a 6x1 vector argument. Check your dimensions.');
end

x = xi(1);
y = xi(2);
z = xi(3);
theta1 = xi(4);
theta2 = xi(5);
theta3 = xi(6);

R = EULERXYZ([theta1, theta2, theta3]');
% 4X4 homogeneous transformation
g = [R [x y z]';0 0 0 1];

end

