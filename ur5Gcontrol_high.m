function finalerr = ur5Gcontrol_high( gdesired,q_desired, K, ur5 )
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

% create gst0
gst = ur5FwdKin(qk);
%Frame_qk = tf_frame('base_link','Frame_qk',gst);

% exp_xiHat_k = g(t*t) = gdesired^-1 * gst
xi_k = getXi(gdesired\gst);
v = xi_k(1:3);
w = xi_k(4:6);

%nn = 200;
while norm(v) > 0.0015 || norm(w) > 0.0035
    
    % Test manipulability
    J = ur5BodyJacobian(qk);
    %disp(['\sigma_m_i_n = ',num2str(manipulability(J,'sigmamin'))])
    
    if abs(manipulability(J,'sigmamin')) < 0.0001
        finalerr = -1;
        return
    end
    norm_tt = norm(gdesired\gst);
    %disp(['norm_tt* = ',num2str(norm_tt)])
    %K = 1/(8/0.4*norm_tt - 18);
    
    %K = -5/4*norm_tt+1.85;
    %disp(['K = ',num2str(K)])
    n = 1;
    q_normk = norm(q_desired(1:3) - qk(1:3));
    norm_dst = norm(gdesired(1:3,4) - gst(1:3,4));
    for K=0.1:0.1:1
        % update joint configuration
        qk1 = qk - K * Tstep * transpose(J) * xi_k;
        q_normk1 = norm(q_desired(1:4) - qk1(1:4));
        q_norm_diff(n) = q_normk1 - q_normk;
        gst_K = ur5FwdKin(qk1);
        norm_ts = norm(gdesired(1:3,4) - gst_K(1:3,4));
        norm_tst(n) = norm_ts - norm_dst;
        n = n+1;
    end
    norm_weighted = 0.01*norm_tst + q_norm_diff;
    %disp(['norm_dist =',num2str(min(norm_tst))]);
    %if norm(gdesired\gst)-1 < 0.01
    %    norm_weighted = q_norm_diff;
    %end
    [~, B] = min(norm_weighted);
    K = 0.1 + 0.1*(B-1);
    disp(K)
    qk = qk - K * Tstep * transpose(J) * xi_k;
    norm_qkk1 = norm(K * Tstep * transpose(J) * xi_k);
    
    %disp(['norm_Qkk1 = ',num2str(norm_qkk1)])
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
    time_int = 20*norm(gdesired-gst) +2.5;
    
    disp(['time_intv = ',num2str(time_int)])
    
% 
%     if nn == 2
%         ur5.move_joints(qk_ur5,time_int);
%         pause(time_int*.9)
%     end
%     
%     nn = nn - 1;
%     
%     if nn == 0
%         nn = 200;
%     end
    
    % update xi_k
    xi_k = getXi(gdesired\gst);
    v = xi_k(1:3);
    w = xi_k(4:6);
   
end

ur5.move_joints(qk_ur5,time_int);
pause(time_int*.95)

gst_final = ur5FwdKin(qk);
finalerr = norm(gst_final(1:3,4));
disp(['Final error = ',num2str(finalerr)])
end
