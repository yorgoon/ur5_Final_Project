function ROTY = ROTY(radians)
% R = ROTY(rad) creates a 3-by-3 matrix used to rotate a 3-by-1 vector
% around the y-axis by rad radians.
    [a,b] = size(radians);
    if a~=1 || b~=1
        error('Error. Input must be a scalar, not a vector nor a matrix form.')
    else
        ROTY = [cos(radians)  0 sin(radians)
                0             1 0
                -sin(radians) 0 cos(radians)];
    end

end
