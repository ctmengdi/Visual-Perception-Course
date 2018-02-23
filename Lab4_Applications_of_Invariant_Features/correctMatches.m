% num = match(image1, image2)
%
% This function reads two images, finds their SIFT features, and
%   displays lines connecting the matched keypoints.  A match is accepted
%   only if its distance is less than distRatio times the distance to the
%   second closest match.
% It returns the number of matches displayed.
%
% Example: match('scene.pgm','book.pgm');

function [num, corr_p3] = correctMatches(image1, image2, H)

disp(['Matching ' image1 ' and ' image2])
% Find SIFT keypoints for each image
% uncomment for default window size
% n = 2;
% window size 3x3
n = 1.5;
[img1, des1, loc1] = ownSIFT(image1, n);
[~, des2, loc2] = ownSIFT(image2, n);

% For efficiency in Matlab, it is cheaper to compute dot products between
%  unit vectors rather than Euclidean distances.  Note that the ratio of 
%  angles (acos of dot products of unit vectors) is a close approximation
%  to the ratio of Euclidean distances for small angles.
%
% distRatio: Only keep matches in which the ratio of vector angles from the
%   nearest to second nearest neighbor is less than distRatio.
distRatio = 0.6; 

% For each descriptor in the first image, select its match to second image.
% des2t = des2';                          % Precompute matrix transpose
% for i = 1 : size(des1,1)
%    dotprods = des1(i,:)* des2t;        % Computes vector of dot products
%    [vals,indx] = sort(acos(dotprods));         % Take inverse cosine and sort results
% 
%    % Check if nearest neighbor has angle less than distRatio times 2nd.
%    if (vals(1) < distRatio * vals(2))
%       match(i) = indx(1);
%    else
%       match(i) = 0;
%    end
% end
% 
% % Create a new image showing the two images side by side.
% % im3 = appendimages(im1,im2);
% % 
% % % Show a figure with lines joining the accepted matches.
% % figure('Position', [100 100 size(im3,2) size(im3,1)]);
% % colormap('gray');
% % imagesc(im3);
% % hold on;
% correct = 0;
% cols1 = size(img1,2);
% 
% for i = 1: size(des1,1)
%   if (match(i) > 0)
%       corr_matches = H*[loc1(i,2:-1:1)'; 1];
%       corr_matches = corr_matches./corr_matches(3);
% %       corr_matches
% %       loc2(match(i),2)
% %       loc2(match(i),1)
%       if abs(loc2(match(i),2)-corr_matches(1))<= 1 && ...
%           abs(loc2(match(i),1)-corr_matches(2))<= 1
%           correct = 1+correct;
%       end
%    
%   end
% end
% hold off;
% num = nnz(match);
% corr_p = 100*correct/num;
% fprintf('Found %.3f matches, %i/%i\n', corr_p, correct, num);

%% Own SIFT
matches = vl_ubcmatch(des1, des2, distRatio);
correct = 0;
for i = 1: size(matches,2)
      corr_matches = H*[loc1(i,1:2)'; 1];
      corr_matches = corr_matches./corr_matches(3);
%       corr_matches
%       loc2(match(i),2)
%       loc2(match(i),1)
      if abs(loc2(matches(2,i),1)-corr_matches(1))<= 1 && ...
          abs(loc2(matches(2,i),2)-corr_matches(2))<= 1
          correct = 1+correct;
      end
   
end

num = size(matches,2);
corr_p3 = 100*correct/num;
fprintf('Found %.3f matches, %i/%i\n', corr_p3, correct, num);






