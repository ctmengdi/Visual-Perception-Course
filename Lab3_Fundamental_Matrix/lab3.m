%% Parameter preparation 
au1 = 100; av1 = 120; uo1 = 128; vo1 = 128;  % Camera 1

au2 = 90; av2 = 110; uo2 = 128; vo2 = 128;   % Camera 2
ax = 0.1; by = pi/4; cz = 0.2; 
tx = -1000; ty = 190; tz = 230;

%% Intrinsic matrices and Transformation 
wRc1 = [1, 0, 0; 0, 1, 0; 0, 0, 1];    
wTc1 = [0; 0; 0];
c1Kw = [wRc1, wTc1];         % Camera 1 with respect to world coordinate
in_mat_c1 = [au1, 0, uo1; 0, av1, vo1; 0, 0, 1]; % Cam1 intrinsic matrix
in_mat_c1_corre = [au1, 0, uo1, 0; 0, av1, vo1, 0; 0, 0, 1, 0];
mat_0 = [0, 0, 0];
wKc1 = c1Kw;                                     % Cam1 extrinsic matrix
ex_mat_c1_corre = [wKc1; mat_0, 1];

Rot_x = [1, 0, 0; 0, cos(ax), -sin(ax); 0, sin(ax), cos(ax)];
Rot_y = [cos(by), 0, sin(by); 0, 1, 0; -sin(by), 0, cos(by)];
Rot_z = [cos(cz), -sin(cz), 0; sin(cz), cos(cz), 0; 0, 0, 1];

c1Rc2 = Rot_x * Rot_y * Rot_z;         
c1Tc2 = [tx; ty; tz]; 
c2Kw = [c1Rc2, c1Tc2];        % Camera 2 with respect to camera 1 system
in_mat_c2 = [au2, 0, uo2; 0, av2, vo2; 0, 0, 1]; % Cam2 intrinsic matrix
in_mat_c2_corre = [au2, 0, uo2, 0; 0, av2, vo2, 0; 0, 0, 1, 0];
wKc2 = [c1Rc2', -c1Rc2'*c1Tc2];                  % Cam2 extrinsic matrix
ex_mat_c2_corre = [wKc2; mat_0, 1];

%% Fundamental matrix
c1Tc2_x = [0, -tz, ty; tz, 0, -tx; -ty, tx, 0]; % antisymmetric matrix
F_mat = inv(in_mat_c2)'*c1Rc2'*c1Tc2_x*inv(in_mat_c1);
F_mat_norm = F_mat/F_mat(end);                  % Normalized Fundamental matrix
disp('Normalized Fundamental matrix');
disp(F_mat_norm);

%% Define a set of object points
V_3d = [];                        % 3d points in world(c1) coordinate
V_3d(:,1) = [100;-400;2000;1];
V_3d(:,2) = [300;-400;3000;1];
V_3d(:,3) = [500;-400;4000;1];
V_3d(:,4) = [700;-400;2000;1];
V_3d(:,5) = [900;-400;3000;1];
V_3d(:,6) = [100;-50;4000;1];
V_3d(:,7) = [300;-50;2000;1];
V_3d(:,8) = [500;-50;3000;1];
V_3d(:,9) = [700;-50;4000;1];
V_3d(:,10) = [900;-50;2000;1];
V_3d(:,11) = [100;50;3000;1];
V_3d(:,12) = [300;50;4000;1];
V_3d(:,13) = [500;50;2000;1];
V_3d(:,14) = [700;50;3000;1];
V_3d(:,15) = [900;50;4000;1];
V_3d(:,16) = [100;400;2000;1];
V_3d(:,17) = [300;400;3000;1];
V_3d(:,18) = [500;400;4000;1];
V_3d(:,19) = [700;400;2000;1];
V_3d(:,20) = [900;400;3000;1];
pt_num = size(V_3d,2);

%% Project 3d points onto both the two image planes
V_2d_c1 = [];                % Projection of 3d points on camera 1 plane 

for n = 1:pt_num
    V_2d_c1(:,n) = in_mat_c1_corre*ex_mat_c1_corre*V_3d(:,n);
    V_2d_c1(1,n) = V_2d_c1(1,n)/V_2d_c1(3,n);
    V_2d_c1(2,n) = V_2d_c1(2,n)/V_2d_c1(3,n);
end
V_2d_c1(3,:) = [];
%disp(V_2d_c1);

V_2d_c2 = [];                % Projection of 3d points on camera 2 plane 2
for n = 1:pt_num
    V_2d_c2(:,n) = in_mat_c2_corre*ex_mat_c2_corre*V_3d(:,n);
    V_2d_c2(1,n) = V_2d_c2(1,n)/V_2d_c2(3,n);
    V_2d_c2(2,n) = V_2d_c2(2,n)/V_2d_c2(3,n);
end
V_2d_c2(3,:) = [];
%disp(V_2d_c2);

%% Draw the 2d points in two planes respectively
figure(1);
plot(V_2d_c1(1,:), V_2d_c1(2,:), 'bo');
title('3d points project on camera 1 image plane');

figure(2);
plot(V_2d_c2(1,:), V_2d_c2(2,:), 'r+');
title('3d points project on camera 2 image plane');

%% Fundamental matrix computed by least-square method
U = [];
for n = 1:pt_num
    U(n,1) = V_2d_c1(1,n)*V_2d_c2(1,n);
    U(n,2) = V_2d_c1(2,n)*V_2d_c2(1,n);
    U(n,3) = V_2d_c2(1,n);
    U(n,4) = V_2d_c1(1,n)*V_2d_c2(2,n);
    U(n,5) = V_2d_c1(2,n)*V_2d_c2(2,n);
    U(n,6) = V_2d_c2(2,n);
    U(n,7) = V_2d_c1(1,n);
    U(n,8) = V_2d_c1(2,n);
end
%disp(U);
vector_ones = repelem(1,pt_num)';
F_leastS = [-inv(U'*U)*U'*vector_ones;1];
F_leastS = reshape(F_leastS,[3,3])';
disp('Fundamental matrix from least square method');
disp(F_leastS);

%% Draw epipolar geometry(LS)
draw_ep (V_2d_c1, V_2d_c2, F_leastS, c1Rc2, c1Tc2, in_mat_c1, in_mat_c2, 'no noise(LS)');

%% Add noise to 2d points
range_noi = 2;    % range of Gaussain noise
noise1 = range_noi*randn(2,pt_num)*0.5;
noise2 = range_noi*randn(2,pt_num)*0.5;
V_2d_c1_noi = V_2d_c1 + noise1;
V_2d_c2_noi = V_2d_c2 + noise2;

%% Fundamental matrix(LS) with noisy 2d points
U_noi = [];
for n = 1:pt_num
    U_noi(n,1) = V_2d_c1_noi(1,n)*V_2d_c2_noi(1,n);
    U_noi(n,2) = V_2d_c1_noi(2,n)*V_2d_c2_noi(1,n);
    U_noi(n,3) = V_2d_c2_noi(1,n);
    U_noi(n,4) = V_2d_c1_noi(1,n)*V_2d_c2_noi(2,n);
    U_noi(n,5) = V_2d_c1_noi(2,n)*V_2d_c2_noi(2,n);
    U_noi(n,6) = V_2d_c2_noi(2,n);
    U_noi(n,7) = V_2d_c1_noi(1,n);
    U_noi(n,8) = V_2d_c1_noi(2,n);
end
%disp(U);
vector_ones = repelem(1,pt_num)';
F_leastS_noi = [-inv(U_noi'*U_noi)*U_noi'*vector_ones;1];
F_leastS_noi = reshape(F_leastS_noi,[3,3])';
disp('Fundamental matrix(LS) with noisy 2d points');
disp(F_leastS_noi);

%% Draw epipolar geometry with noise(LS)
draw_ep (V_2d_c1_noi, V_2d_c2_noi, F_leastS_noi, c1Rc2, c1Tc2, in_mat_c1, in_mat_c2, 'with noise(LS)');
draw_ep_calibrated (V_2d_c1_noi, V_2d_c2_noi, F_leastS_noi, 'with noise-estimated(LS)');

%% Fundamental matrix computed by SVD method
U_2 = [U ones(pt_num,1)];
[u,d,v] = svd(U_2);
F_svd = reshape(v(:,end),[3,3])';
F_svd = F_svd/F_svd(end);
disp('Fundamental matrix from SVD method');
disp(F_svd);

%% Draw epipolar geometry
draw_ep (V_2d_c1, V_2d_c2, F_svd, c1Rc2, c1Tc2, in_mat_c1, in_mat_c2, 'no noise(SVD)');

%% Fundamental matrix(SVD) with noisy 2d points
U_noi_2 = [U_noi ones(pt_num,1)];
[u_noi,d_noi,v_noi] = svd(U_noi_2);
F_svd_noi = reshape(v_noi(:,end),[3,3])';
F_svd_noi = F_svd_noi/F_svd_noi(end);
disp('Fundamental matrix(SVD) with noisy 2d points');
disp(F_svd_noi);

%% Draw epipolar geometry with noise(SVD)
draw_ep (V_2d_c1_noi, V_2d_c2_noi, F_svd_noi, c1Rc2, c1Tc2, in_mat_c1, in_mat_c2, 'with noise(SVD)');
draw_ep_calibrated (V_2d_c1_noi, V_2d_c2_noi, F_svd_noi, 'with noise-estimated(SVD)');

%% Compare the distance from noisy points to lines
dist_leastS = distance( V_2d_c1_noi, V_2d_c2_noi, F_leastS_noi );
dist_leastS_1_max = max(dist_leastS(1,:));
dist_leastS_2_max = max(dist_leastS(2,:));
dist_leastS_1_mean = mean(dist_leastS(1,:));
dist_leastS_2_mean = mean(dist_leastS(2,:));

dist_svd = distance( V_2d_c1_noi, V_2d_c2_noi, F_svd_noi );
dist_svd_1_max = max(dist_svd(1,:));
dist_svd_2_max = max(dist_svd(2,:));
dist_svd_1_mean = mean(dist_svd(1,:));
dist_svd_2_mean = mean(dist_svd(2,:));

%% Draw the 3D epipolar geometry for a given 3D point M
f = 200;             % Focal length is assigned 100
trans_f = [1,0,0,0;0,1,0,0;0,0,1,f;0,0,0,1];

% World coordinate
wP0 = [0, 0, 0];     % worls origin
wPx = [1000, 0, 0];  % 3 axis
wPy = [0, 1000, 0];
wPz = [0, 0, 1000];
figure;
plot3([wP0(1) wPx(1)], [wP0(2) wPx(2)], [wP0(3) wPx(3)], 'b','Displayname','world x axis');
hold on;
plot3([wP0(1) wPy(1)], [wP0(2) wPy(2)], [wP0(3) wPy(3)], 'b','Displayname','world y axis');
hold on;
plot3([wP0(1) wPz(1)], [wP0(2) wPz(2)], [wP0(3) wPz(3)], 'b','Displayname','world z axis');
hold on;

% Camera 1 origin
c1P0 = [0,0,0,1]';                 % Cam1 origin in Cam1 coordinate
wC1_ori = ex_mat_c1_corre * c1P0;  % Cam1 origin in world coordinate
wC1_ori(4) = [];
plot3(wC1_ori(1),wC1_ori(2),wC1_ori(3), 'rx','Displayname','Cam1 origin');
hold on;

% Camera 2 origin
c2P0 = [0,0,0,1]';                 % Cam2 origin in Cam2 coordinate
wC2_ori = ex_mat_c2_corre * c2P0;  % Cam1 origin in world coordinate
wC2_ori(4) = [];
plot3(wC2_ori(1),wC2_ori(2),wC2_ori(3), 'rx','Displayname','Cam2 origin');
hold on;

% 3D point 
M = [800, 600, 400];
plot3(M(1),M(2),M(3), 'kx','Displayname', '3D point');
hold on;

% Projections m and m'
m_c = inv(ex_mat_c1_corre) * [M,1]';         % M in camera 1 coordinate
m_r = [];
m_r(1) = f * m_c(1)/m_c(3);             % M in retina plane of camera 1 
m_r(2) = f * m_c(2)/m_c(3);
m_r_corre = [m_r 0 1]; 
m_w = ex_mat_c1_corre * trans_f * m_r_corre';  % m in the world coordinate
plot3(m_w(1), m_w(2), m_w(3), 'gx','Displayname', 'projection 2');
hold on;

m2_c = inv(ex_mat_c2_corre) * [M,1]';         % M in camera 2 coordinate
m2_r = [];
m2_r(1) = f * m2_c(1)/m2_c(3);           % M in retina plane of camera 2 
m2_r(2) = f * m2_c(2)/m2_c(3);
m2_r_corre = [m2_r,0,1]; 
m2_w = ex_mat_c2_corre * trans_f * m2_r_corre';  % m' in the world coordinate
plot3(m2_w(1), m2_w(2), m2_w(3), 'gx','Displayname', 'projection 2');
hold on;

% image planes
rR1_1 = [-uo1*5, vo1*5, 0, 1]'; % 4 corners in image plane of camera 1
rR2_1 = [uo1*5, vo1*5, 0, 1]';
rR3_1 = [-uo1*5, -vo1*5, 0, 1]';
rR4_1 = [uo1*5, -vo1*5, 0, 1]';
wR1_1 = ex_mat_c1_corre * trans_f * rR1_1;  % 4 corners in world coordinate
wR2_1 = ex_mat_c1_corre * trans_f * rR2_1;
wR3_1 = ex_mat_c1_corre * trans_f * rR3_1;
wR4_1 = ex_mat_c1_corre * trans_f * rR4_1;
plot3([wR1_1(1) wR2_1(1)], [wR1_1(2) wR2_1(2)], [wR1_1(3) wR2_1(3)], 'k','Displayname', 'image plane 1');
hold on;
plot3([wR2_1(1) wR4_1(1)], [wR2_1(2) wR4_1(2)], [wR2_1(3) wR4_1(3)], 'k','Displayname', 'image plane 1');
hold on;
plot3([wR3_1(1) wR4_1(1)], [wR3_1(2) wR4_1(2)], [wR3_1(3) wR4_1(3)], 'k','Displayname', 'image plane 1');
hold on;
plot3([wR3_1(1) wR1_1(1)], [wR3_1(2) wR1_1(2)], [wR3_1(3) wR1_1(3)], 'k','Displayname', 'image plane 1');
hold on;

rR1_2 = [-uo2*5, vo2*5, 0, 1]'; % 4 corners in image plane of camera 2
rR2_2 = [uo2*5, vo2*5, 0, 1]';
rR3_2 = [-uo2*5, -vo2*5, 0, 1]';
rR4_2 = [uo2*5, -vo2*5, 0, 1]';
wR1_2 = ex_mat_c2_corre * trans_f * rR1_2;  % 4 corners in world coordinate
wR2_2 = ex_mat_c2_corre * trans_f * rR2_2;
wR3_2 = ex_mat_c2_corre * trans_f * rR3_2;
wR4_2 = ex_mat_c2_corre * trans_f * rR4_2;
plot3([wR1_2(1) wR2_2(1)], [wR1_2(2) wR2_2(2)], [wR1_2(3) wR2_2(3)], 'k','Displayname', 'image plane 2');
hold on;
plot3([wR2_2(1) wR4_2(1)], [wR2_2(2) wR4_2(2)], [wR2_2(3) wR4_2(3)], 'k','Displayname', 'image plane 2');
hold on;
plot3([wR3_2(1) wR4_2(1)], [wR3_2(2) wR4_2(2)], [wR3_2(3) wR4_2(3)], 'k','Displayname', 'image plane 2');
hold on;
plot3([wR3_2(1) wR1_2(1)], [wR3_2(2) wR1_2(2)], [wR3_2(3) wR1_2(3)], 'k','Displayname', 'image plane 2');
hold on;

% Epipoles and epipolar lines
e = in_mat_c1 * [c1Rc2, c1Tc2] * [0;0;0;1]; % epipole: projection of cam1 origin
e(1) = e(1)/e(3);
e(2) = e(2)/e(3);
e(3) = [];
e_r = [e; 0; 1];
e_w = ex_mat_c1_corre * trans_f * e_r;
plot3(e_w(1),e_w(2),e_w(3), 'ro', 'Displayname', 'epipole');
hold on;

e_2 = in_mat_c2 * [c1Rc2', -c1Rc2'*c1Tc2] * [0;0;0;1]; % epipole: projection of cam1 origin
e_2(1) = e_2(1)/e_2(3);
e_2(2) = e_2(2)/e_2(3);
e_2(3) = [];
e2_r = [e_2; 0; 1];
e2_w = ex_mat_c2_corre * trans_f * e2_r;
plot3(e2_w(1),e2_w(2),e2_w(3), 'ro','Displayname', 'epipole');
hold on;

plot3([m_w(1) e_w(1)], [m_w(2) e_w(2)], [m_w(3) e_w(3)], 'g','Displayname', 'epipolar line');
hold on;

plot3([m2_w(1) e2_w(1)], [m2_w(2) e2_w(2)], [m2_w(3) e2_w(3)], 'g','Displayname', 'epipolar line');
hold on;

% Plane PI
vec1 = M - wC1_ori';
vec2 = wC2_ori' - wC1_ori';
normal = cross(vec1, vec2);

d = -M * normal'; 
[xx,yy]=ndgrid(1:800,1:800);
z = (-normal(1)*xx - normal(2)*yy - d)/normal(3);
surf(xx,yy,z,'Edgecolor', 'y', 'Facecolor', 'none','Displayname','plane PI');
hold off;

legend('show');





