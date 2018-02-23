function [ corners ] = non_maximal_suppression( E, distance, num_salient )
%UNTITLED Summary of this function goes here
%  E: contains for every point the value of the smaller eigenvalue of M
%  distance: distance to maximize the corners
%  num_salient: number of salient points
%  corners: a structure with 2 fields to store the x
%  coordinate (p_x) and the y coordinate (p_y) of the 
%  num_salient points with the highest the cornerness value given by E

% Sorting the values in E
[~, idx_in_order] = sort(E(:), 'descend');

window_size = distance;
half_window = floor(window_size/2);

% Padding the E avoiding the boundary issue
E_padding = padarray(E, [half_window,half_window], 'symmetric');

idx = 1;
queue = idx;
E_center = E;

% Stop when the 81 salient points are selected 
while size(queue,2) < num_salient+1
    
    if E_center(idx_in_order(idx)) ~= 0
        [row, col] = ind2sub(size(E), idx_in_order(idx));
        row = row + half_window;
        col = col + half_window;
        E_padding(row-half_window:row+half_window, col-half_window:col+half_window) = 0;
        E_center = E_padding(1+half_window:size(E_padding,1)-half_window, ...
            1+half_window:size(E_padding,2)-half_window);
        queue = [queue,idx];
        idx = idx + 1;
    else
        idx = idx + 1;
    end
end
queue(1) = [];

% Build the structure to save the corners
field_x = 'p_x';     field_y = 'p_y';
[corner_x, corner_y] = ind2sub(size(E), idx_in_order(queue(:)));
corners = struct(field_x, corner_x, field_y, corner_y);
end

