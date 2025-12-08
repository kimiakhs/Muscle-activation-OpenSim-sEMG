% this code is for sending force data to mocap global frame
clear,clc,close all
%%
%import org.opensim.modeling.*
%%
%markers on the robot
marker_R = [-116.20076,803.813904,792.801514];
%marker_L = [-295.368073,792.885071,795.631104]; Original
marker_L = [-295.368073,803.813904,792.801514];
marker_F = [-202.868332,803.433044,847.367859];
%marker_D = [-206.136719,739.192688,750.402954]; Original
marker_U = [-208.338287,853.621277,747.274475];
marker_D = [-208.338287,739.192688,747.274475];

%Getting the rotation from mocap ground
Y_axis = (marker_R-marker_L)/norm(marker_R-marker_L);
temp = (marker_D-marker_U)/norm(marker_D-marker_U);
Z_axis = cross(temp,Y_axis)/norm(cross(temp,Y_axis));
X_axis = cross(Y_axis,Z_axis);
Rotation = [X_axis',Y_axis',Z_axis'];

%%
%read ros bag and rotate forces
FilePath_read = '2025-11-13-AmandaFtData\rosbag-data-200-7-1-1\';
data = ros2bagreader(FilePath_read);
msg= data.readMessages;
offset_fx = -39;
offset_fy = 32;
offset_fz = -3.7;
offset_tx = 0.41;
offset_ty = 0.62;
offset_tz = 0.37;
offset_time_ns= double(msg{1}.header.stamp.nanosec) * 10^-9;
offset_time_s= double(msg{1}.header.stamp.sec);

for i=1:length(msg)
    force(1,1) = msg{i}.wrench.force.x-offset_fx;
    force(2,1) = msg{i}.wrench.force.y-offset_fy;
    force(3,1) = msg{i}.wrench.force.z-offset_fz;
    force_temp = Rotation * force;
    force2sim(i,:) = force_temp';

    torque(1,1) = msg{i}.wrench.torque.x-offset_tx;
    torque(2,1) = msg{i}.wrench.torque.y-offset_ty;
    torque(3,1) = msg{i}.wrench.torque.z-offset_tz;
    torque_temp = Rotation * torque;
    torque2sim(i,:) = torque_temp';

    time_ns= double(msg{i}.header.stamp.nanosec) * 10^-9-offset_time_ns; %time in seconds
    time_s= double(msg{i}.header.stamp.sec)-offset_time_s;
    time(i,1) = time_s+time_ns;

end
tableOsim = [time,-force2sim,zeros(length(time),3),-torque2sim];

%%
%saving force and torque in .sto
FilePath_save= '2025-11-13-AmandaFtData\';
header = {'time','ground_force_r_vx','ground_force_r_vy','ground_force_r_vz','ground_force_r_px','ground_force_r_py','ground_force_r_pz','ground_torque_r_x','ground_torque_r_y','ground_torque_r_z'};
writeSTO(tableOsim,header,FilePath_save,'Force_sim_Iso3');