close all; clear all; clc;

im = imread('Image_base.jpg');
i = 1;
field = 'H';
save_path = 'Results/Sequence3/';
centerPt = size(im)/2;
base_img = im(centerPt(1)-749:centerPt(1)+750, centerPt(2)-749:centerPt(2)+750,:);
imwrite(base_img, [save_path, 'Image_00','.pgm']);

% Sequence 3
for angle = -45:5:45;
    
    image = imrotate(im, angle);
    center = size(image)/2;

    image = image(center(1)-249:center(1)+250,center(2)-374:center(2)+375,:);
    
    letters = ['a', 'b', 'c', 'd'];
    std = [0, 3, 6, 18];
    image = im2double(image);
    
    for j=1:size(std,2)
        
        noisy_image = imnoise(image, 'gaussian', 0, (std(j)/255)^2);
        noisy_image = uint8(255 * noisy_image);
        
        if i < 10
            imwrite(noisy_image, [save_path,'Image_0',num2str(i), letters(j), '.pgm']);
        else
            imwrite(noisy_image, [save_path,'Image_',num2str(i), letters(j), '.pgm']);
        end
        
    end
    
    x0 = size(noisy_image,2)/2;
    y0 = size(noisy_image,1)/2;
    %angle = deg2rad(angle);
    
    t1 = [1,0,x0; 0,1,y0; 0,0,1];
    t2 = [1,0,-x0; 0,1,-y0; 0,0,1];
    r = [cosd(angle),-sind(angle),0; sind(angle),cosd(angle),0; 0,0,1];
    
    values(i) = {t1*r*t2};
    
%     values(i) = {[cos(angle) -sin(angle) (1-cos(angle))*x0+sin(angle)*y0;
%         sin(angle) cos(angle) -sin(angle)*x0+(1-cos(angle))*y0;
%         0 0 1]};
    
    
    i = i + 1;
    
end

Sequence3Homographies = struct(field, values);

save('Sequence3Homographies.mat','Sequence3Homographies');

%% test
% Image_00a = imread('Results/Sequence3/Image_01a.png');
% Image_04a = imread('Results/Sequence3/Image_04a.png');
% % p_00 = [316 290 1];
% p_00 = [400 200 1];
% p_04 = Sequence3Homographies(4).H * p_00';
% p_04 = p_04./p_04(end);
% 
% figure;
% imshow(Image_00a); impixelinfo; hold on;
% plot(p_00(1), p_00(2), '.g', 'MarkerSize',30);
% figure; imshow(Image_04a); impixelinfo; hold on;
% plot(p_04(1), p_04(2), '.r', 'MarkerSize',30);


