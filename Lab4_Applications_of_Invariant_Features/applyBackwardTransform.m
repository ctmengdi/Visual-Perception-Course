function [img_out] = applyBackwardTransform( img, m )
%APPLYBACKWARDTRANSFORM Summary of this function goes here
%   Detailed explanation goes here

img_out = zeros(size(img));

[x,y] = meshgrid(1:size(img_out,2), 1:size(img_out,1));
x_out = x(:)';
y_out = y(:)';

coords_in = inv(m)*[x_out; y_out; ones(1,numel(x))];
x_in = round(coords_in(1,:)./coords_in(3,:));
y_in = round(coords_in(2,:)./coords_in(3,:));

%Filter out invalid coordinates
non_valid_indexes = x_in < 1 | x_in > size(img_out,2) | y_in < 1 | y_in > size(img_out,1);
y_in(non_valid_indexes) = [];
x_in(non_valid_indexes) = [];
x_out(non_valid_indexes) = [];
y_out(non_valid_indexes) = [];

%sub2ind(size(img_out), Yo, Xo)
for c = 1:size(img, 3)   
    c_out = c*ones(1, numel(y_out));
    
    img_out(sub2ind(size(img_out), y_out, x_out, c_out)) = ...
        img(sub2ind(size(  img  ), y_in, x_in, c_out));
end

end

