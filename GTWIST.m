function g = GTWIST(xi, theta)
%GTWIST(xi, theta) takes twist and theta to create homogeneous
%transformation matrix

v = xi(1:3);
w = xi(4:6);
if w == 0
    g = [eye(3) theta*v;0 0 0 1];
else
    g = [RODRIGUES(w,theta) (eye(3)-RODRIGUES(w,theta))*cross(w,v)+w*w'*v*theta;0 0 0 1];
end

end

