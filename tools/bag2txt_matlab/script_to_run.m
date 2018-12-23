clc;close all; clear all
file_name = {'razor2_syn'}
% [names, types] = bagInfo('razor2_syn.bag')

for i = 1:length(file_name)
    vicon_pose = bagReader('razor2_syn.bag','/razor/vicon_republish/pose/');
end
x = vicon_pose.vicon_position_W_x;
y = vicon_pose.vicon_position_W_y;
psi = vicon_pose.vicon_euler_angles_z;

figure()
plot(x,y)

%%
A = [x y psi]
fileID = fopen('vicon.txt','a');
fprintf(fileID,'%10s %10s %10s\n','x','y','psi');
for i=1:length(A(:,1))
    fprintf(fileID,'%10.4f %10.4f %10.4f \r\n',A(i,1),A(i,2),A(i,3));
end
fclose(fileID);
% type result.txt
