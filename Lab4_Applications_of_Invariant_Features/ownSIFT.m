function [ image, descriptor, location] = ownSIFT( img, n )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% n: window size
image = imread(img);
I = single(image);

[location, descriptor] = vl_sift(I, 'WindowSIze' , n);
location = double(location');
descriptor = double(descriptor);
descriptor = descriptor/norm(descriptor);


end

