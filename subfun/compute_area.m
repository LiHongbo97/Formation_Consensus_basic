function [ area ] = compute_area(x,y,range)
% compute the formation windows area
% 
if(nargin==2)
    range=10;
end

if(x>=0)
    num_x=floor(x(1)/range);
    x1=num_x*range;
    x2=(num_x+1)*range;
else
    num_x=floor(x(1)/-range);
    x1=-range*(num_x+1);
    x2=-range*num_x;
end
if(y>=0)
    num_y=floor(y(1)/range);
    y1=num_y*range;
    y2=(num_y+1)*range;
else
    num_y=floor(y(1)/-range);
    y1=-range*(num_y+1);
    y2=-range*num_y;
end
area=[x1-3 x2+3 y1-3 y2+3];
end

