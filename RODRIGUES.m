function R = RODRIGUES(w, theta)
%RODRIGUES(w, theta) takes one 3-by-1 vector and one scalar and output
%corresponding rotation matrix
[rows, cols] = size(w);
if ((rows ~= 3) || (cols ~= 1))
  error('RODRIGUES requires a 3x1 vector argument. Check your dimensions.');
end
if abs(norm(w)-1) > 0.001
    error('3-by-1 vector needs to be a unit vecor.')
end
[rows, cols] = size(theta);
if ((rows ~= 1) || (cols ~= 1))
  error('RODRIGUES requires a scalar argument. Check your dimensions.');
end

R = eye(3) + HATOPT(w) * sin(theta) + (HATOPT(w))^2 * (1 - cos(theta));

end

