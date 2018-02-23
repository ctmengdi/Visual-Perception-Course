function draw_ep (V_2d_c1, V_2d_c2, F, c1Rc2, c1Tc2, in_mat_c1, in_mat_c2, str)

% Camera 2
figure, plot(V_2d_c2(1,:), V_2d_c2(2,:), 'b+');      % Draw the projections
title(strcat('Camera 2: ', str));
hold on
x = [-500,500];
for i = 1:size(V_2d_c1, 2)
    lm_2 = F * [V_2d_c1(:,i);1];
    y = -lm_2(1) / lm_2(2) * x - lm_2(3) / lm_2(2);  % epipolar line equation
    plot(x, y, 'g');
    
    if i == 1
        y_prev = y;
    end    
    if i == 2
        [xi,yi] = polyxpoly(x,y_prev,x,y);
        plot(xi, yi, 'ro');               % epipole: epipolar lines intersection
    end
end
e_2 = in_mat_c2 * [c1Rc2', -c1Rc2'*c1Tc2] * [0;0;0;1];
plot(e_2(1)/e_2(3), e_2(2)/e_2(3), 'kx'); % epipole: projection of cam2 origin

hold off

% Camera 1
figure, plot(V_2d_c1(1,:), V_2d_c1(2,:), 'b+');
title(strcat('Camera 1: ', str));
hold on
x = [-500,500];
for i = 1:size(V_2d_c2, 2)
    lm = F' * [V_2d_c2(:,i);1];
    y = -lm(1) / lm(2) * x - lm(3) / lm(2);
    plot(x, y, 'g');
    
    if i == 1
        y_prev = y;
    end    
    if i == 2
        [xi,yi] = polyxpoly(x,y_prev,x,y);
        plot(xi, yi, 'ro'); % epipole: epipolar lines intersection
    end
end
e = in_mat_c1 * [c1Rc2, c1Tc2] * [0;0;0;1];
plot(e(1)/e(3), e(2)/e(3), 'kx'); % epipole: projection of cam1 origin
hold off

end