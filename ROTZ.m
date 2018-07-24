function ROTZ = ROTZ(radians)
% R = ROTZ(rad) creates a 3-by-3 matrix used to rotate a 3-by-1 vector
% around the z-axis by rad radians.
    [a,b] = size(radians);
    if a~=1 || b~=1
        error('Error. Input must be a scalar, not a vector nor a matrix form.')
    else
        ROTZ = [cos(radians)    -sin(radians)   0
                sin(radians)    cos(radians)    0
                0               0               1];
    end

end
