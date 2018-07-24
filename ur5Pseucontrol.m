function finalerr = ur5Pseucontrol( gdesired, K, ur5 )
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
    qk_ur5 = ur5.get_current_joints;
end
real_home = [0.000130098245176
  -1.570796239845220
   0.000000088209105
  -1.570796363734165
  -0.000000580666784
  -0.000000230519685];
% create q0
qk = qk_ur5 - real_home;
q_ini = qk_ur5 - real_home;

% create gst0
gst = ur5FwdKin(qk);
%Frame_qk = tf_frame('base_link','Frame_qk',gst);

% exp_xiHat_k = g(t*t) = gdesired^-1 * gst
xi_k = getXi(gdesired\gst);
v = xi_k(1:3);
w = xi_k(4:6);

%nn = 20;

while norm(v) > 0.00005 || norm(w) > 0.0002
    
    % Test manipulability
    J = ur5BodyJacobian(qk);
    disp(['sigma_min = ',num2str(manipulability(J,'sigmamin'))])
%     
%     if abs(manipulability(J,'sigmamin')) < 0.0001
%         finalerr = -1;
%         return
%     end
    
    norm_tt = norm(gdesired\gst);
    %disp(['norm_tt* = ',num2str(norm_tt)])
    a = log(0.1)/(-0.46);
    K = exp(-a*norm_tt + a) * 0.7;
    %K = .05;
    %K = -5/4*norm_tt+1.85;
    %disp(num2str(K))
    % update joint configuration
    qk = qk - K * Tstep * pinv(J) * xi_k + (eye(6) - pinv(J) * J) * (q_ini - qk);    
    
    
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
    
     if qk_ur5(2) < -pi/2*1.3 &&  qk_ur5(2) > -pi*0.8
         qk_ur5(3) = -abs(qk_ur5(3));
     elseif qk_ur5(2) > -pi/2*0.8 &&  qk_ur5(2) < 0-0.15
         qk_ur5(3) = abs(qk_ur5(3));
     %else
     %    qk_ur5(2) = -pi/2*1.1;
     end   
    
     qk = qk_ur5 - real_home;
     if qk_ur5(2) < -pi || qk_ur5(2) > 0
        qk = [0 pi/6 pi/2 -pi/6 -pi/2 0]';
    end
    
    % update configuration
    gst = ur5FwdKin(qk);
    
    % Time to get to qk
    %%time_int = 8;
    time_int = 20*norm(gdesired-gst) +4;
    %ur5.move_joints(qk_ur5,time_int);
    
    
%     if nn == 10
%         ur5.move_joints(qk_ur5,time_int);
%         pause(time_int*.95)
%     end
%     
%     nn = nn - 1;
%     
%     if nn == 0
%         nn = 20;
%     end
    
    
    % update xi_k
    xi_k = getXi(gdesired\gst);
    v = xi_k(1:3);
    w = xi_k(4:6);
    

end
ur5.move_joints(qk_ur5,time_int);
pause(time_int)

gst_final = ur5FwdKin(qk);
finalerr = norm(gst_final(1:3,4));
disp(['Final error = ',num2str(finalerr)])
end

