%% Parameter preparation
au=557.0943; av=712.9824; u0=326.3819; v0=298.6679;
f=80 ;
Tx=100 ; Ty=0 ; Tz=1500;
Phix=0.8*pi/2; Phiy=-1.8*pi/2; Phix1=pi/5;

%% Intrinsic and extrinsic matrix
in_mat = [au, 0, u0, 0; 0, av, v0, 0; 0, 0, 1, 0]; % intrinsic matrix
Rot_x = [1, 0, 0; 0, -cos(Phix), sin(Phix); 0, sin(Phix), cos(Phix)];
Rot_y = [cos(Phiy), 0, sin(Phiy); 0, 1, 0; -sin(Phiy), 0, cos(Phiy)];
Rot_x1 = [1, 0, 0; 0, -cos(Phix1), sin(Phix1); 0, sin(Phix1), cos(Phix1)];

cRw = Rot_x * Rot_y * Rot_x1; % rotation matrix
cTw = [Tx; Ty; Tz];           % tranlation matrix
mat_0 = [0, 0, 0];
ex_mat = [cRw, cTw; mat_0, 1];% extrinsic matrix

% results
disp('Intrinsic matrix : ');
disp(in_mat);
disp('Extrinsic matrix : ');
disp(ex_mat);

%% Define the number of points
pt_num = 6;
fprintf('The number of points is %d. \n\n', pt_num);

%% Generate 3d points
pt_set_3d = rand(pt_num,3)*960-480; % range[-480:480;-480:480;-480:480]
ones_pt_num = ones(pt_num,1);
pt_set_3d = [pt_set_3d ones_pt_num];
disp('3d points : ');
disp(pt_set_3d);

%% Projection: 2d points
pt_set_2d = zeros(pt_num,3);
for n = 1:pt_num
    pt_set_2d(n,:) = in_mat * ex_mat * [pt_set_3d(n,1); pt_set_3d(n,2);
        pt_set_3d(n,3);pt_set_3d(n,4)];    
    pt_set_2d(n,1) = pt_set_2d(n,1)/pt_set_2d(n,3); % normalization
    pt_set_2d(n,2) = pt_set_2d(n,2)/pt_set_2d(n,3);
end
pt_set_2d(:,3) = [];  % delete the 3rd components
disp('2d points :');
disp(pt_set_2d);

%% Draw 2d points
pt_set_2d_pxl = round(pt_set_2d);
figure(1);   
plot(pt_set_2d_pxl(:,1),pt_set_2d_pxl(:,2),'o');
title('2d projection of 3d points');

%% Transformation matrix from Method of Hall
Q = zeros(2*pt_num,11);
B = zeros(2*pt_num,1);
for n = 1:2:(2*pt_num-1)
    Q(n,:) = [pt_set_3d((n+1)/2,1), pt_set_3d((n+1)/2,2), pt_set_3d((n+1)/2,3), ...
              1, 0, 0, 0, 0, (-pt_set_2d((n+1)/2,1)*pt_set_3d((n+1)/2,1)), ...
             (-pt_set_2d((n+1)/2,1)*pt_set_3d((n+1)/2,2)), ...
             (-pt_set_2d((n+1)/2,1)*pt_set_3d((n+1)/2,3))];
    Q(n+1,:) = [0, 0, 0, 0, pt_set_3d((n+1)/2,1), pt_set_3d((n+1)/2,2), ...
               pt_set_3d((n+1)/2,3), 1, (-pt_set_2d((n+1)/2,2)*pt_set_3d((n+1)/2,1)), ...
               (-pt_set_2d((n+1)/2,2)*pt_set_3d((n+1)/2,2)), ...
               (-pt_set_2d((n+1)/2,2)*pt_set_3d((n+1)/2,3))];
    B(n,:) = pt_set_2d((n+1)/2,1);
    B(n+1,:) = pt_set_2d((n+1)/2,2);
end
A = [inv(Q'*Q)*Q'*B;1];
A = reshape(A, [4,3])';
disp('Transformation matrix from Hall : ');
disp(A);

%% Transformation matrix from intrinsic and extrinsic matrice
trans_mat = in_mat * ex_mat;
trans_mat_uni = trans_mat/trans_mat(end);
disp('Transformation matrix from intrinsic and extrinsic matrice : ');
disp(trans_mat_uni);

%% Add noise to 2d points
range_noi = 3;    % range of Gaussain noise
noise = range_noi*randn(pt_num,2)*0.5;
pt_set_2d_noi = pt_set_2d + noise;

%% Transformation matrix from Hall(2d points with noise)
Q_n = zeros(2*pt_num,11);
B_n = zeros(2*pt_num,1);
for n = 1:2:(2*pt_num-1)
    Q_n(n,:) = [pt_set_3d((n+1)/2,1), pt_set_3d((n+1)/2,2), pt_set_3d((n+1)/2,3), 1, 0, 0, 0, 0, ...
        (-pt_set_2d_noi((n+1)/2,1)*pt_set_3d((n+1)/2,1)), (-pt_set_2d_noi((n+1)/2,1)*pt_set_3d((n+1)/2,2)), ...
        (-pt_set_2d_noi((n+1)/2,1)*pt_set_3d((n+1)/2,3))];
    Q_n(n+1,:) = [0, 0, 0, 0, pt_set_3d((n+1)/2,1), pt_set_3d((n+1)/2,2), pt_set_3d((n+1)/2,3), 1, ...
        (-pt_set_2d_noi((n+1)/2,2)*pt_set_3d((n+1)/2,1)), (-pt_set_2d_noi((n+1)/2,2)*pt_set_3d((n+1)/2,2)), ...
        (-pt_set_2d_noi((n+1)/2,2)*pt_set_3d((n+1)/2,3))];
    B_n(n,:) = pt_set_2d_noi((n+1)/2,1);
    B_n(n+1,:) = pt_set_2d_noi((n+1)/2,2);
end
A_n = [inv(Q_n'*Q_n)*Q_n'*B_n;1];
A_n = reshape(A_n, [4,3])';
disp('Transformation matrix from Hall(2d points with noise) : ');
disp(A_n);

%% 2d points from estimated transformation(Hall)
pt_set_2d_n = zeros(pt_num,3);
for n = 1:pt_num
    pt_set_2d_n(n,:) = A_n * [pt_set_3d(n,1); pt_set_3d(n,2);
        pt_set_3d(n,3);pt_set_3d(n,4)];
    pt_set_2d_n(n,1) = pt_set_2d_n(n,1)/pt_set_2d_n(n,3);
    pt_set_2d_n(n,2) = pt_set_2d_n(n,2)/pt_set_2d_n(n,3);
end
pt_set_2d_n(:,3) = [];
disp('2d points from estimated transformation(Hall) :');
disp(pt_set_2d_n);

%% Check accuracy computing the discrepancy between points (Hall)
dist = [];
for n = 1:pt_num
    dist(n) = sqrt((pt_set_2d_n(n,2)-pt_set_2d(n,2))^2 + ...
                   (pt_set_2d_n(n,1)-pt_set_2d(n,1))^2);
end
dist = mean(dist);
disp('Discrepancy between points(Hall): ');
disp(dist);

%% Conpute X matrix
Q_fauge = zeros(2*pt_num,11);
B_fauge = zeros(2*pt_num,1);
for n = 1:2:(2*pt_num-1)
    Q_fauge(n,:) = [pt_set_3d((n+1)/2,1), pt_set_3d((n+1)/2,2), pt_set_3d((n+1)/2,3), ...
        (-pt_set_2d((n+1)/2,1)*pt_set_3d((n+1)/2,1)), (-pt_set_2d((n+1)/2,1)*pt_set_3d((n+1)/2,2)), ...
        (-pt_set_2d((n+1)/2,1)*pt_set_3d((n+1)/2,3)), 0, 0, 0, 1, 0];
    Q_fauge(n+1,:) = [0, 0, 0, (-pt_set_2d((n+1)/2,2)*pt_set_3d((n+1)/2,1)), ...
        (-pt_set_2d((n+1)/2,2)*pt_set_3d((n+1)/2,2)), (-pt_set_2d((n+1)/2,2)*pt_set_3d((n+1)/2,3)), ...
        pt_set_3d((n+1)/2,1), pt_set_3d((n+1)/2,2), pt_set_3d((n+1)/2,3), 0, 1];
    B_fauge(n,:) = pt_set_2d((n+1)/2,1);
    B_fauge(n+1,:) = pt_set_2d((n+1)/2,2);
end
X = inv(Q_fauge'*Q_fauge)*Q_fauge'*B_fauge;
disp('X matrix from Faugeras');
disp(X);

%% Extract camera parameters
T1 = X(1:3);  T2 = X(4:6);  T3 = X(7:9);
T1 = T1';     T2 = T2';     T3 = T3';
C1 = X(10);   C2 = X(11);
u0_f = (T1*T2')/(norm(T2)^2);
v0_f = T2*T3'/(norm(T2)^2);
au_f = norm(cross(T1',T2'))/(norm(T2)^2);
av_f = norm(cross(T2',T3'))/(norm(T2)^2);
r1_f = norm(T2)/norm(cross(T1',T2'))*(T1-(T1*T2'/(norm(T2)^2))*T2);
r2_f = norm(T2)/norm(cross(T2',T3'))*(T3-(T2*T3'/(norm(T2)^2))*T2);
r3_f = T2/norm(T2);
tx_f = norm(T2)/norm(cross(T1',T2'))*(C1-T1*T2'/(norm(T2)^2));
ty_f = norm(T2)/norm(cross(T2',T3'))*(C2-T2*T3'/(norm(T2)^2));
tz_f = 1/norm(T2);

%% Compute transformation matrix from Faugeras
in_mat_f = [au, 0, u0, 0; 0, av, v0, 0; 0, 0, 1, 0];
ex_mat_f = [r1_f(1), r1_f(2), r1_f(3), tx_f;
            r2_f(1), r2_f(2), r2_f(3), ty_f;
            r3_f(1), r3_f(2), r3_f(3), tz_f;
            0, 0, 0, 1];
A_fauge = in_mat_f * ex_mat_f;
A_fauge = A_fauge/A_fauge(end);
disp('Transformation matrix from Faugeras : ');
disp(A_fauge);

%% Conpute X matrix (with noise)
Q_fauge_noi = zeros(2*pt_num,11);
B_fauge_noi = zeros(2*pt_num,1);
for n = 1:2:(2*pt_num-1)
    Q_fauge_noi(n,:) = [pt_set_3d((n+1)/2,1), pt_set_3d((n+1)/2,2), pt_set_3d((n+1)/2,3), ...
        (-pt_set_2d_noi((n+1)/2,1)*pt_set_3d((n+1)/2,1)), (-pt_set_2d_noi((n+1)/2,1)*pt_set_3d((n+1)/2,2)), ...
        (-pt_set_2d_noi((n+1)/2,1)*pt_set_3d((n+1)/2,3)), 0, 0, 0, 1, 0];
    Q_fauge_noi(n+1,:) = [0, 0, 0, (-pt_set_2d_noi((n+1)/2,2)*pt_set_3d((n+1)/2,1)), ...
        (-pt_set_2d_noi((n+1)/2,2)*pt_set_3d((n+1)/2,2)), (-pt_set_2d_noi((n+1)/2,2)*pt_set_3d((n+1)/2,3)), ...
        pt_set_3d((n+1)/2,1), pt_set_3d((n+1)/2,2), pt_set_3d((n+1)/2,3), 0, 1];
    B_fauge_noi(n,:) = pt_set_2d_noi((n+1)/2,1);
    B_fauge_noi(n+1,:) = pt_set_2d_noi((n+1)/2,2);
end
X_noi = inv(Q_fauge_noi'*Q_fauge_noi)*Q_fauge_noi'*B_fauge_noi;
disp('X matrix from Faugeras(with noise) : ');
disp(X_noi);

%% Extract camera parameters
T1_noi = X_noi(1:3);  T2_noi = X_noi(4:6);  T3_noi = X_noi(7:9);
T1_noi = T1_noi';     T2_noi = T2_noi';     T3_noi = T3_noi';
C1_noi = X_noi(10);   C2_noi = X_noi(11);
u0_f_noi = (T1_noi*T2_noi')/(norm(T2_noi)^2);
v0_f_noi = T2_noi*T3_noi'/(norm(T2_noi)^2);
au_f_noi = norm(cross(T1_noi',T2_noi'))/(norm(T2_noi)^2);
av_f_noi = norm(cross(T2_noi',T3_noi'))/(norm(T2_noi)^2);
r1_f_noi = norm(T2_noi)/norm(cross(T1_noi',T2_noi'))*(T1_noi-(T1_noi*T2_noi'/(norm(T2_noi)^2))*T2_noi);
r2_f_noi = norm(T2_noi)/norm(cross(T2_noi',T3_noi'))*(T3_noi-(T2_noi*T3_noi'/(norm(T2_noi)^2))*T2_noi);
r3_f_noi = T2_noi/norm(T2_noi);
tx_f_noi = norm(T2_noi)/norm(cross(T1_noi',T2_noi'))*(C1_noi-T1_noi*T2_noi'/(norm(T2_noi)^2));
ty_f_noi = norm(T2_noi)/norm(cross(T2_noi',T3_noi'))*(C2_noi-T2_noi*T3_noi'/(norm(T2_noi)^2));
tz_f_noi = 1/norm(T2_noi);

%% Compute transformation matrix from Faugeras(with noise)
in_mat_f_n = [au_f_noi, 0, u0_f_noi, 0; 0, av_f_noi, v0, 0; 0, 0, 1, 0];
ex_mat_f_n = [r1_f_noi(1), r1_f_noi(2), r1_f_noi(3), tx_f_noi;
            r2_f_noi(1), r2_f_noi(2), r2_f_noi(3), ty_f_noi;
            r3_f_noi(1), r3_f_noi(2), r3_f_noi(3), tz_f_noi;
            0, 0, 0, 1];
A_fauge_n = in_mat_f_n * ex_mat_f_n;
A_fauge_n = A_fauge_n/A_fauge_n(end);
disp('Transformation matrix from Faugeras(with noise) : ');
disp(A_fauge_n);

%% 2d points from estimated transformation(Faugeras)
pt_set_2d_f = zeros(pt_num,3);
for n = 1:pt_num
    pt_set_2d_f(n,:) = A_fauge_n * [pt_set_3d(n,1); pt_set_3d(n,2);
        pt_set_3d(n,3);pt_set_3d(n,4)];
    pt_set_2d_f(n,1) = pt_set_2d_f(n,1)/pt_set_2d_f(n,3);
    pt_set_2d_f(n,2) = pt_set_2d_f(n,2)/pt_set_2d_f(n,3);
end
pt_set_2d_f(:,3) = [];
disp('2d points from estimated transformation(Faugeras) :');
disp(pt_set_2d_f);

%% Check accuracy computing the discrepancy between points (Faugeras)
dist_f = [];
for n = 1:pt_num
    dist_f(n) = sqrt((pt_set_2d_f(n,2)-pt_set_2d(n,2))^2 + ...
                   (pt_set_2d_f(n,1)-pt_set_2d(n,1))^2);
end
dist_f = mean(dist_f);
disp('Discrepancy between points(Faugeras): ');
disp(dist_f);

%% World coordinate
wP0 = [0, 0, 0];     % origin points
wPx = [480, 0, 0];   % 3 axis
wPy = [0, 480, 0];
wPz = [0, 0, 480];
figure;
plot3([wP0(1) wPx(1)], [wP0(2) wPx(2)], [wP0(3) wPx(3)], 'b');
hold on;
plot3([wP0(1) wPy(1)], [wP0(2) wPy(2)], [wP0(3) wPy(3)], 'b');
hold on;
plot3([wP0(1) wPz(1)], [wP0(2) wPz(2)], [wP0(3) wPz(3)], 'b');
hold on;

%% Camera coordinate
cP0 = [0, 0, 0, 1]';      % points in camera coordinate
cPx = [200, 0, 0, 1]';
cPy = [0, 200, 0, 1]';
cPz = [0, 0, 200, 1]';
wC0 = ex_mat * cP0;       % points in world coordinate
wCx = ex_mat * cPx;
wCy = ex_mat * cPy;
wCz = ex_mat * cPz;
plot3([wC0(1) wCx(1)], [wC0(2) wCx(2)], [wC0(3) wCx(3)], 'g');
hold on;
plot3([wC0(1) wCy(1)], [wC0(2) wCy(2)], [wC0(3) wCy(3)], 'g');
hold on;
plot3([wC0(1) wCz(1)], [wC0(2) wCz(2)], [wC0(3) wCz(3)], 'g');
hold on;

%% Focal point
rF0 = [0, 0, 0, 1]';     % focal point in retina plane
trans_f = [1,0,0,0;0,1,0,0;0,0,1,f;0,0,0,1];
wF0 = ex_mat * trans_f * rF0;   % focal point in world coordinate
plot3(wF0(1), wF0(2), wF0(3), 'or');
hold on;

%% 3d points
scatter3(pt_set_3d(:,1), pt_set_3d(:,2), pt_set_3d(:,3), 'xb');
hold on;

%% Optical rays from 3d points 
for n = 1:pt_num
    plot3([pt_set_3d(n,1) wC0(1)], [pt_set_3d(n,2) wC0(2)], [pt_set_3d(n,3) wC0(3)],'y');
    hold on;
end

%% Image plane
rR1 = [-u0, v0, 0, 1]'; % 4 corners in image plane
rR2 = [u0, v0, 0, 1]';
rR3 = [-u0, -v0, 0, 1]';
rR4 = [u0, -v0, 0, 1]';
wR1 = ex_mat * trans_f * rR1;  % 4 corners in world coordinate
wR2 = ex_mat * trans_f * rR2;
wR3 = ex_mat * trans_f * rR3;
wR4 = ex_mat * trans_f * rR4;
plot3([wR1(1) wR2(1)], [wR1(2) wR2(2)], [wR1(3) wR2(3)], 'k');
hold on;
plot3([wR2(1) wR4(1)], [wR2(2) wR4(2)], [wR2(3) wR4(3)], 'k');
hold on;
plot3([wR3(1) wR4(1)], [wR3(2) wR4(2)], [wR3(3) wR4(3)], 'k');
hold on;
plot3([wR3(1) wR1(1)], [wR3(2) wR1(2)], [wR3(3) wR1(3)], 'k');
hold on;

%% 2d projection on retina plane
cPt_2d = [];
rPt_2d = [];
for n = 1:pt_num
    cPt_2d(n,:) = inv(ex_mat) * pt_set_3d(n,:)';% 2d points in camera coordinate
    rPt_2d(n,1) = f * cPt_2d(n,1)/cPt_2d(n,3);  % 2d points in retina plane
    rPt_2d(n,2) = f * cPt_2d(n,2)/cPt_2d(n,3);
end
zero_z = zeros(pt_num, 1);
one_num = ones(pt_num, 1);
rPt_2d = [rPt_2d zero_z one_num];
for n = 1:pt_num
    wPt_2d(n,:) = ex_mat * trans_f * rPt_2d(n,:)'; % 2d points in world coordinate
end
for n = 1:pt_num
scatter3(wPt_2d(n,1), wPt_2d(n,2), wPt_2d(n,3), '*c');
hold on;
end




