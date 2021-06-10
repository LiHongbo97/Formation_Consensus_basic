function [ output_args ] = consensus_APF( input_args )
%%% Created: 2020-10-2
%%% Last modified: 2020-10-6
%%% Author: Hongbo Li

close all;
fol_num=5;       % 5 followers
N=6;             % 5 followers and 1 leader
countmax=500;   % Largest cycle count number
dt=0.1;          % Control and communication period
gama=0.5;
beta=10;         
K0=1;
KN=0.2;         % Four control parameters
goal=[12 12];   % Destination point

% x and y speed :m/s, angular velocity: rad/s, acceleration: m/s^2 and rad/s^2
Kinematic=[0.5;0.5;0.5;0.5;0.5;0.5];
%% Adjacent matrix
% A=[0 0 1 1 1;     % a(ij)
%    0 0 0 0 1;
%    0 0 0 1 1;
%    0 0 1 0 1;
%    0 0 0 0 0];

A=[0 1 1 0 0 1;     % a(ij)
   1 0 0 0 0 1;
   1 0 0 1 0 0;
   0 0 1 0 1 1;
   0 0 0 1 0 0;
   1 1 0 1 0 0];
 %% Initial position matrix
        init_f=[-4 -1.5 0;%%%[x y th]
                -2 -2.5 pi/4; 
                -6 -2.4 -pi/4;
                -2.5 -4 pi/2;
                -1 -4.5 -pi/2;
                -2.5 -3 0];
    pose_x=init_f(:,1);
    pose_y=init_f(:,2);
    pose_th=init_f(:,3);
    pose_x(:,2)=init_f(:,1);
    pose_y(:,2)=init_f(:,2);
    pose_th(:,2)=init_f(:,3);

    ob_temp=[3 2.3;
             5 4;
             4 8;
             7 5;
             14 16]; %%%obstacle position

    %% consensus relative position 
    delta_x=[-1.5 1.5 -3 -1.5 1.5 0];    
    delta_y=[1.5 1.5 0 -1.5 -1.5 0];  %relative position between leader and follwers
    V_x(:,1)=[0;0;0;0;0;0];
    V_y(:,1)=[0;0;0;0;0;0]; % velocity matrix
    k=1;    % counting number
    d_max=2;
    detect_R=1; %%obstacle detection range

    %% edge weighted matrix
    edge_w=[];
    sum_weight=[0;0;0;0;0;0];
                
    %% Begin to run
    
    for count=1:countmax
        k=k+1;
%         store the last time velocity
        %%%Calculate attraction from goal and put it on leader velocity
        distance=sqrt((goal(1)-pose_x(N,k))^2+(goal(2)-pose_y(N,k))^2);
        th=atan2(goal(2)-pose_y(N,k),goal(1)-pose_x(N,k));
        if distance>d_max
            distance=d_max;
        end
        V_x(N,k+1)=KN*distance*cos(th);
        V_y(N,k+1)=KN*distance*sin(th);

        %% Calculate the agents velocity
        ob_pose=ob_temp;
        repulsion=compute_repulsion([pose_x(N,k),pose_y(N,k)],ob_pose,detect_R);        
        % Put obstacle repulsion to leader
        V_x(N,k+1)=V_x(N,k+1)+beta*repulsion(1);
        V_y(N,k+1)=V_y(N,k+1)+beta*repulsion(2);
%         [V_x(N,k+1),V_y(N,k+1)]=DynamicWindowApproach(pose_x(N,k),pose_y(N,k),ob_pose,V_x(N,k+1),V_y(N,k+1));
        % When the local minimum appears, give an random error
        if(distance>1&&abs(V_x(N,k+1))<=0.1&&abs(V_y(N,k+1))<=0.1)
            V_x(N,k+1)=-1+2*rand(1);
            V_y(N,k+1)=-1+2*rand(1);
        end

        % Calculate followers' velocity
        for i=1:fol_num        
            sum_delta_x=0;
            sum_delta_y=0;
            sum_edge_weight=0;
            for j=1:N       %%One-order Consensus control
                if A(i,j)==1
                    w_ij=2-exp(-((pose_x(j,k-1)-pose_x(i,k)-(delta_x(j)-delta_x(i)))^2+(pose_y(j,k-1)-pose_y(i,k)-(delta_y(j)-delta_y(i)))^2)); %edge weighted calculation 
                    sum_delta_x=sum_delta_x+A(i,j)*w_ij*((pose_x(j,k-1)-pose_x(i,k))-(delta_x(j)-delta_x(i)));
                    sum_delta_y=sum_delta_y+A(i,j)*w_ij*((pose_y(j,k-1)-pose_y(i,k))-(delta_y(j)-delta_y(i)));
                    sum_edge_weight=sum_edge_weight+w_ij;
                end
            end
            edge_w(i,k)=sum_edge_weight;
            % Store the weight error into sum_weight matrix
            if edge_w(i,k)>edge_w(i,k-1)&&k>2
                sum_weight(i,k)=sum_weight(i,k-1)+abs(edge_w(i,k)-edge_w(i,k-1));
            else
                sum_weight(i,k)=sum_weight(i,k-1);
            end
            if mod(k,100)==1 % reset matrix to 0 after period 10s
                sum_weight(i,k)=0;
            end
            
            distance=sqrt(sum_delta_x^2+ sum_delta_y^2);
            th=atan2(sum_delta_y, sum_delta_x);
            if distance>d_max
                distance=d_max;
            end
            V_x(i,k+1)=K0*V_x(N,k)+gama*distance*cos(th); % ideal velocity without repulsion from obstalcle
            V_y(i,k+1)=K0*V_y(N,k)+gama*distance*sin(th);
            
            out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic,0.1);
%             out=[V_x(i,k+1) V_y(i,k+1)];
            V_x(i,k+1)=out(1);
            V_y(i,k+1)=out(2);

           %%%Consider repulsion among agents and store the pose to obs_pose
            kk=0;
            for j=1:N
                if j~=i
                    kk=kk+1;
                    obs_pose(kk,1)=pose_x(j,k);
                    obs_pose(kk,2)=pose_y(j,k);
                end
            end
            ob_pose=[obs_pose;ob_temp];
            repulsion=compute_repulsion([pose_x(i,k),pose_y(i,k)],ob_pose,detect_R);        
            % put repulsion from obstacle on the robot velocity
            V_x(i,k+1)=V_x(i,k+1)+beta*repulsion(1);
            V_y(i,k+1)=V_y(i,k+1)+beta*repulsion(2);
 
        end
        
        % update the position and calculate error between prediction and
        % real position
        for i=1:N
            out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic,0.1);
%             out=[V_x(i,k+1) V_y(i,k+1)];
            V_x(i,k+1)=out(1);
            V_y(i,k+1)=out(2);
            pose_x(i,k+1)=pose_x(i,k)+dt*V_x(i,k+1);
            pose_y(i,k+1)=pose_y(i,k)+dt*V_y(i,k+1);
            pose_th(i,k+1)=atan2(V_y(i,k+1),V_x(i,k+1));
        end
        
        %% ====Animation====
        area = compute_area(pose_x(N,k+1),pose_y(N,k+1),N+1);
        hold off;
        ArrowLength=0.5;% 
        for j=1:N
            quiver(pose_x(j,k+1),pose_y(j,k+1),ArrowLength*cos(pose_th(j,k+1)),ArrowLength*sin(pose_th(j,k+1)),'*k');hold on;
            if j==N
                state=2;
            else
                state=1;
            end
            draw_circle (pose_x(j,k+1),pose_y(j,k+1),0.25,state);hold on;
        end
        for i=1:N
            for j=1:N
                if A(i,j)==1
                    draw_arrow([pose_x(j,k+1),pose_y(j,k+1)],[pose_x(i,k+1),pose_y(i,k+1)], .2);hold on;
                end
            end
        end
        if size(ob_temp)~=[0 0]
            plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);hold on;
        end
        axis(area);
        grid on;
        drawnow;    
        %% Judge whether arrived
        now=[pose_x(N,k+1),pose_y(N,k+1)];
        if norm(now-goal)<0.5
            disp('Arrive Goal!!');break;
        end
        
    end
    
    color='mgbkrc'; %%%corresponding to 6 colors
    type=[2,1,0.5,0.5,2,2];%%%different line type
%     xlswrite('attmse.xlsx',attmse);
    %% Draw diagram
    figure                               % Draw the path record of formation 
    for i=1:N
        plot(pose_x(i,:),pose_y(i,:),color(1,i),'LineWidth',2);
        hold on
    end
    for i=1:N-1
        plot(pose_x(i,1),pose_y(i,1),'bp','color',color(1,i),'LineWidth',1);
        hold on
    end
    plot(pose_x(N,1),pose_y(N,1),'*','color',color(1,N),'LineWidth',1);
    hold on
    for i=1:N-1
        plot(pose_x(i,k),pose_y(i,k),'m^','color',color(1,i),'LineWidth',2);
        hold on
    end
    plot(pose_x(N,k),pose_y(N,k),'o','color',color(1,N),'LineWidth',2);
    hold on
    if size(ob_temp)~=[0 0]
        plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);hold on;
    end
    grid on;
    xlabel('x');
    ylabel('y');
    legend('Follower 1','Follower 2','Follower 3','Follower 4','Follower5','Leader');
    xlabel('x(m)');
    ylabel('y(m)');
    title('Formation Consensus');


end
