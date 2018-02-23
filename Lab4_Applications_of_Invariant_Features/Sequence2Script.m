close all; clear all; clc;
save_path = 'Results/Sequence2/';

im = imread('Image_base.jpg');
centerPt = size(im)/2;
base_img = im(centerPt(1)-249:centerPt(1)+250,centerPt(2)-374:centerPt(2)+375, :);
imwrite(base_img, [save_path, 'Image_00','.pgm']);

i = 1;
field = 'H';

% Sequence 2
for zoom = 1.1:0.05:1.5;
    
    image = imresize(im, zoom);
    
    center = size(image)/2;
    
    image = image(center(1)-249:center(1)+250,center(2)-374:center(2)+375, :);
    
    letters = ['a', 'b', 'c', 'd'];
    std = [0, 3, 6, 18];
    image = im2double(image);
    
    for j=1:size(std,2)
        
        noisy_image = imnoise(image, 'gaussian', 0, (std(j)/255)^2);
        noisy_image = uint8(255 * noisy_image);
        imwrite(noisy_image, [save_path,'Image_0',num2str(i), letters(j),'.pgm']);
    end
    
    
    values(i) = {[zoom 0 size(image,2)/2-size(image,2)/2*zoom;
        0 zoom size(image,1)/2-size(image,1)/2*zoom;
        0 0 1]};
    
    
    i = i + 1;
    
end

Sequence2Homographies = struct(field, values);

save('Sequence2Homographies.mat','Sequence2Homographies');

%% test
Image_00a = imread('Results/Sequence2/Image_01a.png');
Image_04a = imread('Results/Sequence2/Image_04a.png');
%p_00 = [316 290 1];
p_00 = [400 200 1];
p_04 = Sequence2Homographies(4).H * p_00';
p_04 = p_04./p_04(end);
figure;
imshow(Image_00a); impixelinfo; hold on;
plot(p_00(1), p_00(2), '.g', 'MarkerSize',30);
figure; imshow(Image_04a); impixelinfo; hold on;
plot(p_04(1), p_04(2), '.r', 'MarkerSize',30);

