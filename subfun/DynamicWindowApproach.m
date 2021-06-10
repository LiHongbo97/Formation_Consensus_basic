function [vx,vy]=DynamicWindowApproach(pose_x,pose_y,ob_pose,V_x,V_y)
%A very simplified DWA path planning
pre_px=[pose_x];
pre_py=[pose_y];
pre_p=[pose_x pose_y];
delta_t=0.1;
for i=1:15
    pre_px(i+1)=pre_px(i)+delta_t*V_x;
    pre_py(i+1)=pre_py(i)+delta_t*V_y;
    pre_p=[pre_p;pre_px(i+1) pre_py(i+1)];
end
ob_num=size(ob_pose);
v_x=V_x;
v_y=V_y;
for i=1:ob_num(1)
%     for j=2:16
        dist = sqrt((ob_pose(i,1)-pre_px(16))^2+(ob_pose(i,2)-pre_py(16))^2);
        if dist<0.6
            theta=atan2(ob_pose(i,2)-pre_py(16),ob_pose(i,1)-pre_px(16));
            v_x=V_x*cos(-theta)-V_y*sin(-theta);
            v_y=V_x*sin(-theta)+V_y*cos(-theta);
            break
        end
%     end
end
vx=v_x;
vy=v_y;
end