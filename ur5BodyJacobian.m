function J = ur5BodyJacobian( Q )
%Compute the Jacobian matrix for the UR5. All necessary parameters are to be
%defined inside the function. Again, parameters such as twists and gst(0) (if needed) should
%be defined in the function.
L = [425 392.25 109.15 94.65 82.3]*0.001;

% w
w = zeros(3,6);
w(:,1) = [0 0 1]';
w(:,2) = [1 0 0]';
w(:,3) = w(:,2);
w(:,4) = w(:,2);
w(:,5) = w(:,1);
w(:,6) = w(:,2);

% q
q = zeros(3,6);
q(:,1) = [0; 0; 0];
q(:,2) = q(:,1);
q(:,3) = [0; 0; L(1)]; % v3
q(:,4) = [0; 0; L(1)+L(2)];
q(:,5) = [L(3); 0; 0];
q(:,6) = [0; 0; L(1)+L(2)+L(4)];


% xi
xi = zeros(6);
for i=1:length(xi)
    xi(:,i) = [-cross(w(:,i),q(:,i));w(:,i)];
    xi(:,i) = ADJOINT([ROTZ(pi/2) [0 0 0.0892]';0 0 0 1])*xi(:,i); % to adjust its orientation to match with Rviz
end
% gst(0)
gstO = [ROTX(-pi/2) [0 L(3)+L(5) L(1)+L(2)+L(4)+0.0892]';0 0 0 1]; % Rviz!!!!!!!!!!

gst = gstO;
for i=length(xi):-1:1
    gst = GTWIST(xi(:,i),Q(i)) * gst;
end

% Spatial manipulator jacobian
Js = zeros(6,length(Q));
Js(:,1) = xi(:,1);
for i=2:length(Q)
    g = eye(4);
    for j=1:i-1
        g = g * GTWIST(xi(:,j),Q(j));
    end
    Js(:,i) = ADJOINT(g) * xi(:,i);
end

% Body manipulator jacobian
J = ADJOINT(gst)\Js;

end

