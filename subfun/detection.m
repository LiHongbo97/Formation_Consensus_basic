function [ phi_label] = detection(phi,N)
% detect robot's state using SVM model
% 
phi_label=[];
for i=1:N
    if phi(i)>0.5
        phi_label(i)=1;
    else
        phi_label(i)=0;
    end
end


end

