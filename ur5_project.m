%% UR5 Final Project
% Team 8: Amrita, Soowon, Yingtian(Paul)

%%% Initialization %%%
% Initialize ur5
ur5 = ur5_interface();
% initialize gripper
ur5.init_gripper();
%%
% Recorded start and target configuration
% * This part can be modified as your own configuration
Q_start = [0.6380 -1.0037 1.4073 -2.0149 -1.5603 -0.9498]';

Q_target = [1.4779 -1.1515 1.6429 -2.0496 -1.5588 -1.6428]';

% Start, Target Work space
g_start = ur5FwdKin(Q_start - ur5.home);
g_target = ur5FwdKin(Q_target - ur5.home);

% Above
g_start1 = g_start;
g_start1(3,4) = g_start(3,4) - 0.05;
g_target1 = g_target;
g_target1(3,4) = g_target(3,4) + 0.1;

% Different home for control methods
Q_home_ctrl = [0.9818 -1.8475 1.6436 -1.3615 -1.5274 -0.4957]';
g_home_ctrl = ur5FwdKin(Q_home_ctrl-ur5.home);

%% IK-based %%
% Above start
% Get joint configuration using IK
Q_start1_all = ur5InvKin(g_start1);

% Collision free configuration
Q_start1_valid = zeros(6,1);

n = 1;
for i=1:length(Q_start1_all)
    Q1 = Q_start1_all(:,i);
    % Guarantee the 2nd joint is above the table
    if Q1(2) > -pi && Q1(2) < 0
        % Calculate 4th joint config (to get height)
        g_joint4 = ur5FwdKin_q3(Q1-ur5.home);
        % Guarantee the gripper is above the table
        if g_joint4(3,4) > 0.05
            Q_start1_valid(:,n) = Q1;
            n = n + 1;
        end
    end
end

% Get Q_start1 with minimal norm
norm_start1_point = zeros(size(Q_start1_valid,2),1);
for k = 1:size(Q_start1_valid,2)
    norm_start1_point(k) = norm(Q_start1_valid(:,k)-ur5.home);
end
[a, indx] = min(norm_start1_point);
Q_start1 = Q_start1_valid(:,indx);

% Above Target
% Get joint configuration using IK
Q_target1_all = ur5InvKin(g_target1);

% Collision free configuration
Q_target1_valid = zeros(6,1);

n = 1;
for i=1:length(Q_target1_all)
    Q2 = Q_target1_all(:,i);
    % Guarantee the 2nd joint is above the table
    if Q2(2) > -pi && Q2(2) < 0
        % Calculate 4th joint config (to get height)
        g_joint4 = ur5FwdKin_q3(Q2-ur5.home);
        % Guarantee the gripper is above the table
        if g_joint4(3,4) > 0.05
            Q_target1_valid(:,n) = Q2;
            n = n + 1;
        end
    end
end

% Get Q_start1 with minimal norm
norm_target1_point = zeros(size(Q_target1_valid,2),1);
for k = 1:size(Q_target1_valid,2)
    norm_target1_point(k) = norm(Q_target1_valid(:,k)-Q_start1);
end
[a, indx] = min(norm_target1_point);
Q_target1 = Q_target1_valid(:,indx);

%% IK-based %%
tic
% Go home
ur5.move_joints(ur5.home,7)
pause(7)
% Go above start
ur5.move_joints(Q_start1,8);
pause(8)
% Open gripper
ur5.open_gripper();
pause(1)
% Go start
ur5.move_joints(Q_start,5);
pause(5)
ur5.close_gripper();
pause(2)
% Go up again
ur5.move_joints(Q_start1,3);
pause(4)
% above target
ur5.move_joints(Q_target1,6);
pause(7)
% Go target
ur5.move_joints(Q_target,3);
pause(4)
ur5.open_gripper();
pause(2)
% Go up target
ur5.move_joints(Q_target1,3);
pause(4)
% Go back home
ur5.move_joints(ur5.home,8)
toc
%////////////////////////////////////////////////////////////////////////%
%////////////////////////////////////////////////////////////////////////%
%////////////////////////////////////////////////////////////////////////%
%% Rate Control (DK-based)
tic
% Move to home
ur5.move_joints(Q_home_ctrl,7);
pause(7)
% Go above start
ur5RRcontrol(g_start1, 1, ur5);
pause(0.5)
% Open gripper
ur5.open_gripper();
pause(1)
% Go start
ur5RRcontrol(g_start, 1, ur5);
pause(1)
% Close gripper
ur5.close_gripper();
pause(2)
% Go up again
ur5RRcontrol(g_start1, 1, ur5);
pause(0.5)
% Go above target
ur5RRcontrol(g_target1, 1, ur5);
pause(0.5)
% Go target
ur5RRcontrol(g_target, 1, ur5);
pause(1)
% Open gripper
ur5.open_gripper
pause(2)
% Go up again
ur5RRcontrol(g_target1, 1, ur5);
pause(0.5)
% Go back to home
ur5RRcontrol(g_home_ctrl, 1, ur5);
toc
%////////////////////////////////////////////////////////////////////////%
%////////////////////////////////////////////////////////////////////////%
%////////////////////////////////////////////////////////////////////////%
%% Gradient-based Control
tic
% Move to home
ur5.move_joints(Q_home_ctrl,7);
pause(7)
% Go above start
ur5Gcontrol_low(g_start1, Q_start1, 1, ur5);
pause(0.5)
% Open gripper
ur5.open_gripper
pause(1)
% Go start
ur5Gcontrol_high(g_start,Q_start, 1, ur5);
pause(0.5)
% Close gripper
ur5.close_gripper
pause(2)
% Go up again
ur5Gcontrol_low(g_start1, Q_start1, 1, ur5);
pause(0.5)
% Go above target
ur5Gcontrol_low(g_target1,Q_target1, 1, ur5);
pause(0.5)
% Go target
ur5Gcontrol_high(g_target, Q_target, 1, ur5);
pause(1)
% Open gripper
ur5.open_gripper
pause(2)
% Go up again
ur5Gcontrol_low(g_target1, Q_target1, 1, ur5);
pause(0.5)
% Go back to home
ur5Gcontrol_low(g_home_ctrl, Q_home_ctrl, 1, ur5);
toc
%////////////////////////////////////////////////////////////////////////%
%////////////////////////////////////////////////////////////////////////%
%////////////////////////////////////////////////////////////////////////%
%% Pseudo Control
tic
% Go home
ur5.move_joints(ur5.home +[0 0 0 0 0 0]',7);
pause(7)
%
g_home_pseudo = ur5FwdKin([0 0 0 0 0 0]');
% Go above start
ur5Pseucontrol(g_start1, 1, ur5);
pause(0.5)
% Open gripper
ur5.open_gripper
pause(1)
% Go start
ur5Pseucontrol(g_start, 1, ur5);
pause(1)
% Close gripper
ur5.close_gripper
pause(2)
% Go up again
ur5Pseucontrol(g_start1, 1, ur5);
pause(0.5)
% Go above target
ur5Pseucontrol(g_target1, 1, ur5);
pause(0.5)
% Go to target
ur5Pseucontrol(g_target, 1, ur5);
pause(1)
% Open gripper
ur5.open_gripper
pause(2)
% Go up again
ur5Pseucontrol(g_target1, 1, ur5);
pause(0.5)
% Go back to home
ur5Pseucontrol(g_home_pseudo, 1, ur5);
toc
%/////////////////////////////////////////////////////////////////////////%
%/////////////////////////////////////////////////////////////////////////%
%/////////////////////////////////////////////////////////////////////////%
%% WRITING WRITING
% J

Q_letter_entry = [1.0286 -1.2760 2.0147 -2.2899 -1.5283 -0.4625]';
g_target = ur5FwdKin(Q_letter_entry - ur5.home);

J1 = [76.94 106.4 0]'*1e-3;
J2 = [106.94 106.4 0]'*1e-3;
J3 = [106.94 94.31 0]'*1e-3;
J4 = [95 94.31 0]'*1e-3;
g_J1 = g_target;
g_J1(1:3,4) = g_J1(1:3,4) + J1;
g_J2 = g_target;
g_J2(1:3,4) = g_J2(1:3,4) + J2;
g_J3 = g_target;
g_J3(1:3,4) = g_J3(1:3,4) + J3;
g_J4 = g_target;
g_J4(1:3,4) = g_J4(1:3,4) + J4;

H1 = [76.94 128.39 0]'*1e-3;
H2 = [116.03 128.39 0]'*1e-3;
H3 = [96.44 128.39 0]'*1e-3;
H4 = [96.44 143.39 0]'*1e-3;
H5 = [76.94 143.39 0]'*1e-3;
H6 = [116.03 143.39 0]'*1e-3;

g_H1 = g_target;
g_H2 = g_target;
g_H3 = g_target;
g_H4 = g_target;
g_H5 = g_target;
g_H6 = g_target;
g_H1(1:3,4) = g_target(1:3,4) + H1;
g_H2(1:3,4) = g_target(1:3,4) + H2;
g_H3(1:3,4) = g_target(1:3,4) + H3;
g_H4(1:3,4) = g_target(1:3,4) + H4;
g_H5(1:3,4) = g_target(1:3,4) + H5;
g_H6(1:3,4) = g_target(1:3,4) + H6;

U1 = [76.94 169.74 0]'*1e-3;
U2 = [116.03 164.74 0]'*1e-3;
U3 = [116.03 179.58 0]'*1e-3;
U4 = [76.94 179.58 0]'*1e-3;

g_U1 = g_target;
g_U2 = g_target;
g_U3 = g_target;
g_U4 = g_target;
g_U1(1:3,4) = g_target(1:3,4) + U1;
g_U2(1:3,4) = g_target(1:3,4) + U2;
g_U3(1:3,4) = g_target(1:3,4) + U3;
g_U4(1:3,4) = g_target(1:3,4) + U4;

margin_up = 0.05;
%%
ur5Pseucontrol(g_target, .1, ur5);
%% above J entry
g_J1up = g_J1;
g_J1up(3,4) = g_J1up(3,4) + margin_up;
ur5Pseucontrol(g_J1up, .1, ur5);
pause(1)

% J entry
ur5Pseucontrol(g_J1, .1, ur5);
pause(.1)
%
ur5Pseucontrol(g_J2, .1, ur5);
pause(.1)
%
ur5Pseucontrol(g_J3, .1, ur5);
pause(.1)
%
ur5Pseucontrol(g_J4, .1, ur5);
pause(.1)

% H
% Above H entry
g_H1up = g_H1;
g_H1up(3,4) = g_H1up(3,4) + margin_up;
ur5Pseucontrol(g_H1up, .1, ur5);
pause(.1)

% H1
ur5Pseucontrol(g_H1, .1, ur5);
pause(.1)
% H2
ur5Pseucontrol(g_H2, .1, ur5);
pause(.1)
% H3
ur5Pseucontrol(g_H3, .1, ur5);
pause(.1)
% H4
ur5Pseucontrol(g_H4, .1, ur5);
pause(.1)
% H5
ur5Pseucontrol(g_H5, .1, ur5);
pause(.1)
% H6
ur5Pseucontrol(g_H6, .1, ur5);
pause(.1)

% Above U entry
g_U1up = g_U1;
g_U1up(3,4) = g_U1up(3,4) + margin_up;
ur5Pseucontrol(g_U1up, .1, ur5);
pause(.1)

% U1
ur5Pseucontrol(g_U1, .1, ur5);
pause(.1)
% U2
ur5Pseucontrol(g_U2, .1, ur5);
pause(.1)
% U3
ur5Pseucontrol(g_U3, .1, ur5);
pause(.1)
% U4
ur5Pseucontrol(g_U4, .1, ur5);
pause(.1)

%% Blue Jay
Q_letter_entry = [1.0331 -1.2124 1.9165 -2.2451 -1.5479 -0.5308]';
g_target = ur5FwdKin(Q_letter_entry - ur5.home);
unit_conv = 0.2/1440;

% 1st segment
first = zeros(3,27);
first(:,1) = [597 185 0]'*0.2/1440;
first(:,2) = [785 195 0]'*0.2/1440;
first(:,3) = [933 207 0]'*0.2/1440;
first(:,4) = [1071 227 0]'*0.2/1440;
first(:,5) = [1142 240 0]'*0.2/1440;
first(:,6) = [1167 250 0]'*0.2/1440;
first(:,7) = [1177 259 0]'*0.2/1440;
first(:,8) = [1185 276 0]'*0.2/1440;
first(:,9)= [1189 399 0]'*0.2/1440;
first(:,10) = [1181 551 0]'*0.2/1440;
first(:,11) = [1161 671 0]'*0.2/1440;
first(:,12) = [1132 773 0]'*0.2/1440;
first(:,13)= [1115 825 0]'*0.2/1440;
first(:,14) = [1092 802 0]'*0.2/1440;
first(:,15) = [1078 793 0]'*0.2/1440;
first(:,16) = [1066 787 0]'*0.2/1440;
first(:,17)= [1052 736 0]'*0.2/1440;
first(:,18)= [1035 693 0]'*0.2/1440;
first(:,19)= [1013 650 0]'*0.2/1440;
first(:,20)= [981 611 0]'*0.2/1440;
first(:,21) = [945 575 0]'*0.2/1440;
first(:,22) = [906 541 0]'*0.2/1440;
first(:,23) = [855 497 0]'*0.2/1440;
first(:,24) = [796 445 0]'*0.2/1440;
first(:,25) = [723 365 0]'*0.2/1440;
first(:,26)= [651 275 0]'*0.2/1440;
first(:,27) = [597 185 0]'*0.2/1440;

% Entry point
g_first_entry = g_target;
g_first_entry(1:3,4) = g_first_entry(1:3,4) + first(:,1);
%
g_first_up = g_first_entry;
g_first_up(3,4) = g_first_up(3,4) + .05;
ur5Pseucontrol(g_first_up, .1, ur5);
pause(1)
%
for k=1:length(first)
    g_first = g_target;
    g_first(1:3,4) = g_first(1:3,4) + first(:,k);
    ur5Pseucontrol(g_first, .1, ur5);
    pause(.1)
end

% 2nd
second = [372 197;213 216;83 239;58 246;46 254;37 265;33 278;31 381;34 469;...
          41 546;50 591;100 555;155 522;199 502;240 489;271 482;302 479;...
          295 444;294 425;297 385;306 351;320 322;342 291;356 270;367 243;...
          373 215;372 197]*unit_conv; 
second = [second';zeros(1,length(second))];

% 2nd Entry point
g_2nd_entry = g_target;
g_2nd_entry(1:3,4) = g_2nd_entry(1:3,4) + second(:,1);
%
g_2nd_up = g_2nd_entry;
g_2nd_up(3,4) = g_2nd_up(3,4) + .05;
ur5Pseucontrol(g_2nd_up, .1, ur5);
pause(1)
%
for k=1:length(second)
    g_2nd = g_target;
    g_2nd(1:3,4) = g_2nd(1:3,4) + second(:,k);
    ur5Pseucontrol(g_2nd, .1, ur5);
    pause(.1)
end


% 3rd BIG FACE
third = [271 36;305 32;346 37;381 50;421 71;460 100;500 137;542 185;...
         572 227;605 279;607 281;663 360;731 440;783 489;862 553;...
         908 590;929 609;958 642;990 688;1011 733;1029 788;1034 812;...
         1067 832;1087 849;1102 868;1184 904;1256 943;1308 980;1342 1011;...
         1371 1046;1389 1075;1402 1109;1407 1135;1365 1101;1332 1078;...
         1283 1050;1215 1019;1153 998;1081 981;1015 974;952 973;903 978;...
         887 984;904 991;1044 1010;1137 1030;1206 1051;1257 1075;1306 1104;...
         1342 1134;1206 1118;1084 1099;962 1078;896 1062;865 1052;853 1064;...
         837 1074;817 1074;803 1069;793 1062;764 1077;737 1098;709 1126;...
         686 1159;663 1206;646 1263;638 1348;640 1410;609 1426;537 1384;...
         478 1338;443 1304;409 1250;393 1203;389 1154;394 1120;412 1079;...
         384 1112;367 1148;355 1188;338 1149;334 1104;343 1069;365 1024;...
         392 985;357 1001;330 1028;309 1064;295 1107;286 1162;199 1036;...
         149 940;113 853;72 715;55 638;137 574;200 540;272 517;361 511;...
         345 479;337 446;334 404;342 369;365 319;391 279;406 251;409 216;...
         397 158;372 115;343 84;299 52;271 36]*unit_conv;
     
third = [third';zeros(1,length(third))];

% 3rd Entry point
g_3rd_entry = g_target;
g_3rd_entry(1:3,4) = g_3rd_entry(1:3,4) + third(:,1);
%
g_3rd_up = g_3rd_entry;
g_3rd_up(3,4) = g_3rd_up(3,4) + .05;
ur5Pseucontrol(g_3rd_up, .1, ur5);
pause(1)
%
for k=1:length(third)
    g_3rd = g_target;
    g_3rd(1:3,4) = g_3rd(1:3,4) + third(:,k);
    ur5Pseucontrol(g_3rd, .1, ur5);
    pause(.1)
end

% Fourth

fourth = [970 1111;914 1187;846 1261;779 1319;680 1387;675 1340;680 1285;691 1238;...
          709 1191;734 1149;760 1120;794 1099;811 1107;827 1110;850 1104;863 1095;873 1087;...
          913 1099;970 1111]*unit_conv;
     
fourth = [fourth';zeros(1,length(fourth))];

% 4th Entry point
g_4th_entry = g_target;
g_4th_entry(1:3,4) = g_4th_entry(1:3,4) + fourth(:,1);
%
g_4th_up = g_4th_entry;
g_4th_up(3,4) = g_4th_up(3,4) + .05;
ur5Pseucontrol(g_4th_up, .1, ur5);
pause(1)
%
for k=1:length(fourth)
    g_4th = g_target;
    g_4th(1:3,4) = g_4th(1:3,4) + fourth(:,k);
    ur5Pseucontrol(g_4th, .1, ur5);
    pause(.1)
end

% Fifth

fifth = [739 988;641 1004;569 1028;532 1048;510 1070;489 1111;477 1171;478 1243;...
         488 1285;505 1318;531 1347;565 1372;604 1392;606 1305;619 1239;639 1182;...
         669 1131;708 1087;764 1044;739 988]*unit_conv;
     
fifth = [fifth';zeros(1,length(fifth))];

% 5th Entry point
g_5th_entry = g_target;
g_5th_entry(1:3,4) = g_5th_entry(1:3,4) + fifth(:,1);
%
g_5th_up = g_5th_entry;
g_5th_up(3,4) = g_5th_up(3,4) + .05;
ur5Pseucontrol(g_5th_up, .1, ur5);
pause(1)
%
for k=1:length(fifth)
    g_5th = g_target;
    g_5th(1:3,4) = g_5th(1:3,4) + fifth(:,k);
    ur5Pseucontrol(g_5th, .1, ur5);
    pause(.1)
end

% Sixth
sixth = [761 733;728 679;691 651;658 634;621 623;592 622;559 630;536 647;...
         523 673;524 706;536 728;554 748;579 765;601 775;539 755;492 768;
         463 780;444 798;437 818;438 841;448 862;470 889;504 912;548 932;598 946;...
         650 952;722 950;738 946;746 938;748 924;712 909;687 884;671 851;667 821;...
         671 793;690 758;718 735;741 725;761 733]*unit_conv;
     
sixth = [sixth';zeros(1,length(sixth))];

% 6th Entry point
g_6th_entry = g_target;
g_6th_entry(1:3,4) = g_6th_entry(1:3,4) + sixth(:,1);
%
g_6th_up = g_6th_entry;
g_6th_up(3,4) = g_6th_up(3,4) + .05;
ur5Pseucontrol(g_6th_up, .1, ur5);
pause(1)
%
for k=1:length(sixth)
    g_6th = g_target;
    g_6th(1:3,4) = g_6th(1:3,4) + sixth(:,k);
    ur5Pseucontrol(g_6th, .1, ur5);
    pause(.1)
end

     
% Seventh

seventh = [776 755;816 801;802 814;783 818;765 811;755 796;755 775;765 761;776 755]*unit_conv;
     
seventh = [seventh';zeros(1,length(seventh))];

% 7th Entry point
g_7th_entry = g_target;
g_7th_entry(1:3,4) = g_7th_entry(1:3,4) + seventh(:,1);
%
g_7th_up = g_4th_entry;
g_7th_up(3,4) = g_7th_up(3,4) + .05;
ur5Pseucontrol(g_7th_up, .1, ur5);
pause(1)
%
for k=1:length(seventh)
    g_7th = g_target;
    g_7th(1:3,4) = g_7th(1:3,4) + seventh(:,k);
    ur5Pseucontrol(g_7th, .1, ur5);
    pause(.1)
end

%% Dancing
pose = zeros(6,14);
pose(:,1) = [0.9699 -1.5380 -0.0385 -1.5853 -0.0029 -0.4954];
pose(:,6) = [0.8525 -2.6068 1.0701 -1.5847 -0.0030 -0.4953];
pose(:,2) = [0.7299 -1.5362 -0.0173 -1.6817 -1.6037 -0.4954];
pose(:,3) = [0.9699 -1.5380 -0.0385 -1.5853 -0.0029 -0.4954];
pose(:,4) = pose(:,2);
pose(:,5) = pose(:,3);
pose(:,7) = [0.9690 -1.1979 -0.3511 -1.5852 -0.0030 -0.4954];
pose(:,8) = pose(:,6);
pose(:,9) = pose(:,7);
pose(:,10) = [0.9948 -1.8453 0.3229 -1.5851 -0.0030 -0.4953];
pose(:,11) = [0.8652 -0.5222 -1.0505 -1.5851 -0.0029 -0.4954];
pose(:,12) = [0.8525 -2.6068 1.0701 -1.5847 -0.0030 -0.4953];
pose(:,13) = [0.7339 -1.4831 2.3915 -1.6831 0.0991 -0.4954];
pose(:,14)= [0.7303 -2.0042 0.0077 -1.6830 0.0990 -0.4954];
%%
% go home
ur5.move_joints(ur5.home,7)
pause(1)
% dance
for k=1:size(pose,2)
    ur5.move_joints(pose(:,k),4)
    pause(4)
end
ur5.move_joints(ur5.home,7)