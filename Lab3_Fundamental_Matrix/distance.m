function [ dist ] = distance( V_2d_c1_noi, V_2d_c2_noi, F_noi )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Camera 2
dist_2 = [];
for i = 1:size(V_2d_c1_noi,2)
    lm_2 = F_noi * [V_2d_c1_noi(:,i);1];
    x = V_2d_c1_noi(1,i);
    y = V_2d_c1_noi(2,i);
    dist_2(i) = abs(x*lm_2(1)+y*lm_2(2)+lm_2(3))/sqrt(lm_2(1)^2+lm_2(2)^2);
end

% Camera 1
dist_1 = [];
for i = 1:size(V_2d_c2_noi,2)
    lm = F_noi * [V_2d_c2_noi(:,i);1];
    x = V_2d_c2_noi(1,i);
    y = V_2d_c2_noi(2,i);
    dist_1(i) = abs(x*lm(1)+y*lm(2)+lm(3))/sqrt(lm(1)^2+lm(2)^2);
end

dist = [dist_1; dist_2];
end

