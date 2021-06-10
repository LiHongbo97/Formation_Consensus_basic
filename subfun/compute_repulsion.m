function [ repulsion] = compute_repulsion(robot_pose,obs_pose,detect_R)
% computer the repulsion using artificial potential field method
% obs_pose=[x1 y1;x2; y2;....]
[M,N]=size(obs_pose);
repulsion(1)=0; %x direction
repulsion(2)=0; %y direction
for i=1:M
    distance=sqrt((robot_pose(1)-obs_pose(i,1))^2+(robot_pose(2)-obs_pose(i,2))^2);
    if distance<=detect_R
        temp=(1/distance-1/detect_R)/(distance^3);
        repulsion(1)=repulsion(1)+temp*(robot_pose(1)-obs_pose(i,1));
        repulsion(2)=repulsion(2)+temp*(robot_pose(2)-obs_pose(i,2));
    end
end
end

