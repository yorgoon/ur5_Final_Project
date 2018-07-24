function ROTX = ROTX(radians)
% R = ROTX(rad) creates a 3-by-3 matrix used to rotate a 3-by-1 vector
% around the x-axis by rad radians.
    [a,b] = size(radians);
    if a~=1 || b~=1
        error('Error. Input must be a scalar, not a vector nor a matrix form.')
    else
        ROTX = [   1      0              0
                   0      cos(radians)   -sin(radians)
                   0      sin(radians)   cos(radians)  ];
    end
end
