function [hop] =find_hop(i,n,N,A)
%find the hops between agent i and m by findpath

p_s=[];
p_t=[];
s=0;t=0;
for j=1:N
    for m=1:N
        if A(j,m)==1
            s=s+1;t=t+1;
            p_s(s)=j;
            p_t(t)=m;
        end
    end
end
G = digraph(p_s,p_t);
P = shortestpath(G,i,n); % find the shortest path between i and n in graph G
hop1= size(P);
hop=hop1(2)-1; % get the min hops
end
