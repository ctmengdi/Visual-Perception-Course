% Load a image and convert to gray level
input_img = imread('chessboard06.png');
im_orig = input_img;

if size(input_img,3)>1
    input_img=rgb2gray(input_img);
end

% Derivative masks
dx = [-1 0 1; -1 0 1; -1 0 1];
dy = dx';

% Image derivatives
Ix = conv2(double(input_img), dx, 'same');
Iy = conv2(double(input_img), dy, 'same');
sigma=2;

% Generate Gaussian filter of size 9x9 and std. dev. sigma.
g = fspecial('gaussian',9, sigma);

% Smoothed squared image derivatives
Ix2 = conv2(Ix.^2, g, 'same');
Iy2 = conv2(Iy.^2, g, 'same');
Ixy = conv2(Ix.*Iy, g, 'same');

% Matrix E which contains for every point
% the value of the smaller eigenvalue of M.
window = [1, 1, 1; 1, 1, 1; 1, 1, 1];
M_Ix2 = conv2(Ix2, window, 'same');
M_Iy2 = conv2(Iy2, window, 'same');
M_Ixy = conv2(Ixy, window, 'same');
M = [];
tic
E = zeros(size(input_img));
for r = 1:size(input_img,1)
    for c = 1:size(input_img,2)
        M = [M_Ix2(r,c), M_Ixy(r,c); M_Ixy(r,c), M_Iy2(r,c)];
        E(r,c) = min(eig(M));
    end
end
toc
figure(1);
imshow(mat2gray(E));
title('Matrix E');

% Matrix R
tic
R = zeros(size(input_img));
for r = 1:size(input_img,1)
    for c = 1:size(input_img,2)
        M = [M_Ix2(r,c), M_Ixy(r,c); M_Ixy(r,c), M_Iy2(r,c)];
        R(r,c) = det(M) - 0.04*(M(1,1)+M(2,2))^2;
    end
end
toc
figure(2);
imshow(mat2gray(R));
title('Metrix R');

% Build a structure and save the 81 salient points
field_x = 'p_x';     field_y = 'p_y';
[val_in_order, idx_in_order] = sort(E(:), 'descend');
largest_81_val = val_in_order(1:81);
largest_81_idx = idx_in_order(1:81);
[value_x, value_y] = ind2sub(size(E), largest_81_idx);
features = struct(field_x, value_x, field_y, value_y);

figure(3); imshow(im_orig); hold on;
for i=1:size(features,2)
    plot(features(i).p_x, features(i).p_y, 'r+');
end

% Non-maximal suppression for E(R)
[ corners ] = non_maximal_suppression( E, 11, 81 );

% Show the detected corners in original image
figure(4); imshow(im_orig); hold on;
for i=1:size(corners,2)
    plot(corners(i).p_x, corners(i).p_y, 'r+');
end

% Show the corner in E(R) image
figure(5); imshow(mat2gray(E)); hold on;
for i=1:size(corners,2),
    plot(corners(i).p_x, corners(i).p_y, 'r+');
end

%% Find corners with subpixel accuracy
corners_sub_x = [];
corners_sub_y = [];
for i=1:length(corners.p_x)
    % Parameters computation
    s1 = E(corners.p_x(i)-1, corners.p_y(i)-1);
    s2 = E(corners.p_x(i), corners.p_y(i)-1);
    s3 = E(corners.p_x(i)+1, corners.p_y(i)-1);
    s4 = E(corners.p_x(i)-1, corners.p_y(i));
    s5 = E(corners.p_x(i), corners.p_y(i));
    s6 = E(corners.p_x(i)+1, corners.p_y(i));
    s7 = E(corners.p_x(i)-1, corners.p_y(i)+1);
    s8 = E(corners.p_x(i), corners.p_y(i)+1);
    s9 = E(corners.p_x(i)+1, corners.p_y(i)+1);
    a = (s1-2*s2+s3+s4-2*s5+s6+s7-2*s8+s9)/6; 
    b = (s1-s3-s7+s9)/4;
    c = (s1+s2+s3-2*s4-2*s5-2*s6+s7+s8+s9)/6;
    d = (-s1+s3-s4+s6-s7+s9)/6;
    e = (-s1-s2-s3+s7+s8+s9)/6;
    %F = (-s1+2*s2-s3+2*s4+5*s5+2*s6-s7+2*s8-s9)/9;
    delta_x = (b*e-2*c*d)/(4*a*c-b^2);
    delta_y = (b*d-2*a*e)/(4*a*c-b^2);
    % Constrain the subpixel accuracy in range of 3x3
    if abs(delta_x) <= 1.5 && abs(delta_y) <= 1.5
        corners_sub_x(i) = delta_x + corners.p_x(i);
        corners_sub_y(i) = delta_y + corners.p_y(i);
    else
        corners_sub_x(i) = corners.p_x(i);
        corners_sub_y(i) = corners.p_y(i);
    end
end

corners_sub = struct(field_x, corners_sub_x, field_y, corners_sub_y);
figure(6); imshow(mat2gray(E)); hold on;
for i=1:size(corners,2),
    plot(corners_sub(i).p_x, corners_sub(i).p_y, 'r+');
end










