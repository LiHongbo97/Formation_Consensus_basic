function [ feature_new ] = normalization( feature ,mi,ma,lower,upper,N)
% Normalize the delta
feature_new=[];
feature_new=lower+(upper-lower)*(feature-mi)/(ma-mi);
end

