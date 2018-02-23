function draw_ep_calibrated (V_2d_c1, V_2d_c2, F, str)
[u,s,v] = svd(F);
s(3,3) = 0;
F = u * s * v';

% Camera 2
figure, plot(V_2d_c2(1,:), V_2d_c2(2,:), 'b+');
title(strcat('Camera 2: ', str));
hold on
x = -500:500;
for i = 1:size(V_2d_c1, 2)
    lm_2 = F * [V_2d_c1(:,i);1];
    y = -lm_2(1) / lm_2(2) * x - lm_2(3) / lm_2(2);
    plot(x, y, 'g');

end
e_2_svd = u(:,3) / u(3,3);
plot (e_2_svd(1), e_2_svd(2), 'rx');

hold off

% Camera 1
figure, plot(V_2d_c1(1,:), V_2d_c1(2,:), 'b+');
title(strcat('Camera 1: ', str));
hold on
x = -500:500;
for i = 1:size(V_2d_c1, 2)
    lm = F' * [V_2d_c2(:,i);1];
    y = -lm(1) / lm(2) * x - lm(3) / lm(2);
    plot(x, y, 'g');

end
e_svd = v(:,3) / v(3,3);
plot (e_svd(1), e_svd(2), 'rx');

hold off

end