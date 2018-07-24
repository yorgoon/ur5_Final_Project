function gst = ur5FwdKin( Q )
% input: joints is 6*1 vector where joints (i) correspond to joint i in
% gazebo setting.
% output: g is 4*4 transformation matrix relative to base_link
[rows, cols] = size(Q);
    if ((rows ~= 6) || (cols ~= 1))
        error('ur5FwdKin requires a 6-by-1 joint vector argument. Check your dimensions.');
    end

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
end
