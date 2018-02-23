function [ Homography ] = projective_hompgraphy( origin_pos, transformed_pos )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

x1y1 = [];
for n = 1:size(origin_pos,1)
    r1 = [origin_pos(n,1), origin_pos(n,2),1, 0, 0, 0, ...
        -transformed_pos(n,1)*origin_pos(n,1), -transformed_pos(n,1)*origin_pos(n,2)];
    r2 = [0, 0, 0, origin_pos(n,1), origin_pos(n,2),1, ...
        -transformed_pos(n,2)*origin_pos(n,1), -transformed_pos(n,2)*origin_pos(n,2)];
    x1y1 = [x1y1; r1; r2];
end
%disp(x1y1);
x0y0 = [];
for n = 1:size(transformed_pos,1)
    r1 = transformed_pos(n,1);
    r2 = transformed_pos(n,2);
    x0y0 = [x0y0; r1; r2];
end

H = inv(x1y1'*x1y1)*x1y1'*x0y0;
Homography = reshape([H;1], [3,3])';

end

