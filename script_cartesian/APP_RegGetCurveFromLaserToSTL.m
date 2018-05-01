% Test 

clc; clear; close all;
startup_rvc;

%% Load Data
% X - Relatively dense STL data (about 13k)
% Y - Relatively coarse Laser scan with curve (about 12k)
laser = load('NonDeformedPointCloud');
nonDeformedPCD = laser.PointCloudData;
clear laser;

% Curve Fitting
laser = load('NonDeformedTargetCurve.mat');
curve = laser.TargetCurveData;
clear laser;

% Downsampling non deformed data
YOrig = nonDeformedPCD(1:30:length(nonDeformedPCD), :);
YOrig = [YOrig; curve];       % Append curve data in Y

% STL 1mm non deformed without curve
Xorig = read_ply('model_1mm.ply');
X = Xorig * rotx(-pi/2) + repmat([50, 50, 16], length(Xorig), 1);

%% Transform Curve to STL data

Y = YOrig * rotz(pi/2);   % Rotate Y to match STL coordinate frame 
Y = Y - repmat(min(Y), length(Y), 1);

%% Display initial guess
close all;
figure;
draw_coordinate_system(5, eye(3), [0, 0, 0], 'rgb');
scatter3(Y(:,1), Y(:,2), Y(:,3), '.b');
axis([0, 100, 0, 100, 0, 20]);

figure;
draw_coordinate_system(5, eye(3), [0, 0, 0], 'rgb');
scatter3(X(:,1), X(:,2), X(:,3), '.r');
axis([0, 100, 0, 100, 0, 20]);


%% Deform
figure; cpd_plot_iter(X, Y);

% Registering Y to X
[T C]= deformReg(X, Y);

close all;
cpd_plot_iter(X, T.Y);
%% Getting correspondance and curve in stl data
correspondingInX = X(C,:);
% Curve Indices in Laser scan
curveIndices = (length(Y)-length(curve)+1:length(Y));	
curveSTL = correspondingInX(curveIndices,:); 	% Transformed curve in STL

figure;
draw_coordinate_system(5, eye(3), [0, 0, 0], 'rgb');
scatter3(X(:,1), X(:,2), X(:,3), '.r');
scatter3(curveSTL(:,1), curveSTL(:,2), curveSTL(:,3), '.g');
axis([0, 100, 0, 100, 0, 20]);

%% Curve Fitting Here
close all;
n = 15; 
step = 0.5;
coef_x_on_y = polyfit(curveSTL(:,2), curveSTL(:,1), 10);
tmpY = min(curveSTL(:,2)):step:max(curveSTL(:,2));
tmpX = polyval(coef_x_on_y, tmpY);
curveSTLFit = [tmpX' tmpY' zeros(length(tmpX),1)];

draw_coordinate_system(5, eye(3), [0, 0, 0], 'rgb');
scatter3(X(:,1), X(:,2), X(:,3), '.r');
scatter3(curveSTLFit(:,1), curveSTLFit(:,2), curveSTLFit(:,3), '.b');
axis([0, 100, 0, 100, 0, 20]);

%% Display the deformed mesh
figure;
draw_coordinate_system(5, eye(3), [0, 0, 0], 'rgb');
scatter3(T.Y(:,1), T.Y(:,2), T.Y(:,3), '.r');

%% Registration error
% rmse(target, estimate)
regErrorPoints = rmse(correspondingInX, T.Y);
regErrorCurve = rmse(T.Y(curveIndices,:), curveSTL);

%% save data
save('STLPCD', 'X','curveSTLFit');
   

%% Display laser scanned deformed curve
% Fit a curve to laser scan data

close all;
n = 10;
step = 0.5;
x = (min(curve(:,1)):step:max(curve(:,1)))';
coef_y = polyfit(curve(:,1),curve(:,2),n);
coef_z = polyfit(curve(:,1),curve(:,3),n);
y = polyval(coef_y,x);
z = polyval(coef_z,x);
curveLaserFit = [x y z];
clear x y z step n

draw_coordinate_system(5, eye(3), [0, 0, 0], 'rgb');
scatter3(YOrig(:,1), YOrig(:,2), YOrig(:,3), '.r');
scatter3(curveLaserFit(:,1), curveLaserFit(:,2), curveLaserFit(:,3), '.b');
axis([0, 100, 0, 100, 0, 20]);
