function [ output_args ] = draw_circle (x,y,r,state)
%Use circles to represent real time robot
%  
if(nargin==3)
    color='-k';
end
if(state==1)
    color='-k';
    line=1;
elseif (state==2)
    color='g';
    line=2;
elseif (state==3)
    color='r';
    line=1;
elseif(state==4)
    color='b';
    line=1;
elseif (state==0)
    color='-k';
    line=1;
end
t=0:0.01:2*pi;
X=x+r*cos(t);
Y=y+r*sin(t);
plot(X,Y,color,'LineWidth',line);
end

