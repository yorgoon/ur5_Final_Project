function q = EULERXYZINV(R)
%q = EULERXYZ(R) accepts one input a 3-by-3 rotation matrix, 
%and returns the corresponding 3-by-1 vector of angles.

% check argument dimension
[rows, cols] = size(R);
if ((rows ~= 3) || (cols ~= 3))
  error('EULERXYZINV requires a 3x3 matrix argument. Check your dimensions.');
end

if ((abs(R(3,2)) <= 1e-10) && (abs(R(3,3)) <= 1e-10))
    disp('The calculation is going to be numerically ill-defined since cos(theta) is equal to or close to zero.');
    if (R(3,1) < 0)
        theta = pi/2;
        % EULERXYZ reduced into
        % [0  sin(phi + psi)    cos(phi - psi)]
        % [0  cos(phi - psi)   -sin(phi + psi)]
        % [-1 0                 0             ]
        % So we can set one angle whatever we want.
        psi = 0;
        phi = atan2(R(1,2),R(2,2));
        q = [phi, theta, psi]';
    else
        theta = -pi/2;
        % EULERXYZ reduced into
        % [0  -sin(phi + psi)   -cos(phi + psi)]
        % [0  cos(phi + psi)    -sin(phi + psi)]
        % [1  0                  0             ]
        % So we can set one angle whatever we want.
        phi = 0;
        psi = atan2(-R(1,2),R(2,2));
        q = [phi, theta, psi]';
    end
else
    % R23^2 + R33^2 = (cos(theta))^2. This fact leads to the duality of the
    % value. We don't know the sign of cos(theta). Thus, we have two
    % candidates of cos(theta).
    % Candidate 1,2 for Theta
    theta(1) = atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2));
    theta(2) = atan2(-R(3,1),-sqrt(R(3,2)^2 + R(3,3)^2));
    % Corresponding candidate 1,2 for phi
    phi(1) = atan2(R(3,2)/cos(theta(1)),R(3,3)/cos(theta(1)));
    phi(2) = atan2(R(3,2)/cos(theta(2)),R(3,3)/cos(theta(2)));
    % Corresponding candidate 1,2 for psi
    psi(1) = atan2(R(2,1)/cos(theta(1)),R(1,1)/cos(theta(1)));
    psi(2) = atan2(R(2,1)/cos(theta(2)),R(1,1)/cos(theta(2)));
    
    q_candidate = zeros(3,2);
    q_candidate(:,1) = [phi(1), theta(1), psi(1)]';
    q_candidate(:,2) = [phi(2), theta(2), psi(2)]';

    % Use Frobinius norm to see the distance between calculated EULERXYZ and
    % argument R
    normm = zeros(2,1);
    for i=1:2
        normm(i) = norm(EULERXYZ(q_candidate(:,i)) - R, 'fro');
        % calculate norm, choose the one has the least distance
        %if normm < 0.1
        %    q = q_candidate(:,i);
        %end
    end
    [~, b] = min(normm);
    q = q_candidate(:,b);
end



