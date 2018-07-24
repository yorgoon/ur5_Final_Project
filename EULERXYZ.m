function R = EULERXYZ(q)
%R = EULERXYZ(q) accepts one input a 3-by-1 vector of angles (in radians), 
%and returns the corresponding 3-by-3 rotation matrix.

% check argument dimension
[rows, cols] = size(q);
if ((rows ~= 3) || (cols ~= 1))
  error('EULERXYZ requires a 3x1 vector argument. Check your dimensions.');
end

phi = q(1);
theta = q(2);
psi = q(3);

R = ROTZ(psi)*ROTY(theta)*ROTX(phi);

