ori_img = imread('Image_base.jpg');

field = 'H';
n = 1;
save_path = 'Results/Sequence1/';

center_position = [size(ori_img,1)/2, size(ori_img,2)/2];
ROI = ori_img(center_position(1)-749:center_position(1)+750, ...
    center_position(2)-749:center_position(2)+750, :);
base_img = ROI(501:1000,376:1125,:);
imwrite(base_img, [save_path, 'Image_00','.pgm']);

origin_ROI_pos = [0,0; size(ROI,2),0; 0,size(ROI,1); size(ROI,2),size(ROI,1)];

for i = 250:10:280
    
    transformed_ROI_pos = cell(1,4);
    transformed_ROI_pos{1} = [i,0; 1500-i,0; 0,size(ROI,1); size(ROI,2),size(ROI,1)];
    transformed_ROI_pos{2} = [0,i; size(ROI,2),0; 0,1500-i; size(ROI,2),size(ROI,1)];
    transformed_ROI_pos{3} = [0,0; size(ROI,2),0; i,size(ROI,1); 1500-i,size(ROI,1)];
    transformed_ROI_pos{4} = [0,0; size(ROI,2),i; 0,size(ROI,1); size(ROI,2),1500-i];
    
    for j = 1:4
        
        transformed_ROI_pose = [];
        transformed_ROI_pose = transformed_ROI_pos{j};
        H = projective_hompgraphy( origin_ROI_pos, transformed_ROI_pose);
        
        % correct the center
        proj_center = [size(ROI,1)/2, size(ROI,2)/2];
        origin_center = inv(H) * [proj_center,1]';
        origin_center = origin_center./origin_center(3);
        origin_center = origin_center';
        
        % multiplied by translation matrix
        t1 = origin_center(1)- proj_center(1);
        t2 = origin_center(2)- proj_center(2);
        T = [1,0,t1; 0,1,t2; 0,0,1];
        H = H * T;
        
        % apply the transformation
        transformed_ROI = applyBackwardTransform( ROI, H );
        
        % crop the central part
        proj_img = transformed_ROI(501:1000,376:1125,:);
        origin_img = ROI(501:1000,376:1125,:);
        
        % save images
        letters = ['a', 'b', 'c', 'd'];
        std = [0, 3, 6, 18];
        %image = im2double(proj_img);
        image = uint8(proj_img);       
              
        for k =1:size(std,2)
            
            noisy_image = imnoise(image, 'gaussian', 0, (std(k)/255)^2);
            %noisy_image = uint8(255 * noisy_image);
            
            imwrite(noisy_image, [save_path, 'Image_0',num2str(n), letters(k),'.pgm']);
        end
        
        [r, c, ~] = size(proj_img);
        
        % coordiantes of projective image in big frame
        coordinates = [376,501,1;
            1125,501,1;
            376,1000,1;
            1125,1000,1];
        
        % coordinates of origin image in big frame
        matched_co = inv(H) * coordinates';
        matched_co(1,:) = matched_co(1,:)./matched_co(3,:);
        matched_co(2,:) = matched_co(2,:)./matched_co(3,:);
        matched_co(3,:) = [];
        coordinates(:,3) = [];
        
        matched_co = matched_co';
        
        % coordinates of projective in small frame
        proj_co = [0,0; c,0; 0,r; c,r];
        
        % coordinates of origin image in small frame
        for o = 1:size(matched_co,1)
            origin_co(o,:) = matched_co(o,:) - coordinates(1,:);
        end
        h(n) = {projective_hompgraphy( origin_co, proj_co)};
        
        
        n = n+1;
    end
end
Sequence1Homographies = struct(field, h);
save('Sequence1Homographies.mat','Sequence1Homographies');