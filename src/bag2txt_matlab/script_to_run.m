clc;close all; clear all
% [names, types] = bagInfo('razor2_syn.bag')

for i = 1:length(file_name)
    vicon_pose = bagReader('razor2_syn.bag','/razor/vicon_republish/pose/');
end
x = vicon_pose.vicon_position_W_x;
y = vicon_pose.vicon_position_W_y;
psi = vicon_pose.vicon_euler_angles_z;

fileID = fopen('vicon.txt','a');
fprintf(fileID,'%10s %10s %10s\n','x','y','psi');
fprintf(fileID,'%10.4f %10.4f %10.4f\n',x,y,psi);
fclose(fileID);
% type vicon.txt
