function [ safe_order] = rank_safe(safe_order1,N,phi)
% detect robot's state
% 0 means follower seperated, 1 means normal follower, 2 means leader
% 3 means attacker, 4 means detecting agent
phi
for i=1:N
    safe_order1(i)=1;
    for j=1:N
        if phi(i)>phi(j) && j~=i
            safe_order1(i)=safe_order1(i)+1;
        end
    end
end
    
safe_order=safe_order1;
end

