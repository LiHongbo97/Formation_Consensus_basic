function [ A] = communication_range(i,N,pose_x,pose_y,max_dist, A1)
% if i agent goes out of the j's communication range, then A(i,j)=A(j,i)=0
for m=1:N
    distance = 0;
    if m~=i && A1(i,m)==1
        distance=sqrt((pose_x(i)-pose_x(m))^2+(pose_y(i)-pose_y(m))^2);
        if distance>max_dist
            A1(i,m)=0;
        end
    end 
end
    
A=A1;
end

