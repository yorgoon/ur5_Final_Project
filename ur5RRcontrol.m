function finalerr = ur5RRcontrol( gdesired, K, ur5 )
%ur5RRcontrol takes gdesired, K, and ur5 object produces final error

% Step size
Tstep = 1;

% qk needs to be adjusted because of difference between the convention used
% to calculate forward kinematics ins ur5FwdKin.m and the convention being 
% used in ROS
% current joint configuration, qk in ROS
qk_ur5 = ur5.get_current_joints;
if qk_ur5 == 0
    disp('get_current_joints didnt work...')
    return
end
real_home = [0.000130098245176
  -1.570796239845220
   0.000000088209105
  -1.570796363734165
  -0.000000580666784
  -0.000000230519685];
% create q0
qk = qk_ur5 - real_home;

% create gst0
gst = ur5FwdKin(qk);
%Frame_qk = tf_frame('base_link','Frame_qk',gst);

% exp_xiHat_k = g(t*t) = gdesired^-1 * gst
xi_k = getXi(gdesired\gst);
v = xi_k(1:3);
w = xi_k(4:6);
%nn = 10;
while norm(v) > 0.0003 || norm(w) > 0.0005
    
    % Test manipulability
    J = ur5BodyJacobian(qk);
    disp(['{\sigma}_min = ',num2str(manipulability(J,'sigmamin'))])
    
    if abs(manipulability(J,'sigmamin')) < 0.0001
        finalerr = -1;
        return
    end
    norm_tt = norm(gdesired\gst);
    disp(num2str(norm_tt))
    K = 4.5/0.4*norm_tt - 9.75;
    disp(num2str(K))
    % update joint configuration
    qk = qk - K * Tstep * J \ xi_k;    
    
    % update configuration
    gst = ur5FwdKin(qk);
    % Display frame
    %Frame_qk.move_frame('base_link',gst);
    
    % move to updated joint configuration for ROS
    qk_ur5 = qk + real_home;
    
    qk_ur5 = rem(qk_ur5,2*pi);
    
    for i=1:length(qk_ur5)
        if qk_ur5(i) > pi
            qk_ur5(i) = qk_ur5(i) - 2*pi;
        elseif qk_ur5(i) < -pi
            qk_ur5(i) = qk_ur5(i) + 2*pi;
        end
    end
    
    % Time to get to qk
    time_int = 5;

    %if nn == 10

   % end
    
   % nn = nn - 1;
    
   % if nn == 0
   %     nn = 10;
   % end
    % update xi_k
    xi_k = getXi(gdesired\gst);
    v = xi_k(1:3);
    w = xi_k(4:6);
    

end
ur5.move_joints(qk_ur5,5);
pause(4.5)
gst_final = ur5FwdKin(qk);
finalerr = norm(gst_final(1:3,4));
disp(['Final error = ',num2str(finalerr)])
end

