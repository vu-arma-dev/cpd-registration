% Date: 2014-09-30
% Author: Preetham 
% Brief: deformable registration from STL to Robot Scanned data

clc; clear; close all;
Setup_Dir_DeformableReg;
startup_rvc;

%% Load Data
% X - Relatively dense robot scanned data (about 19k)
% Y - Relatively coarse STL data with curve (about 13k)
% Deformed Robot Scanned Data
robot = load('JHU_Deformed_T5_Merged.mat');
DeformedRobotPCD = robot.POINT;
clear robot;
X = DeformedRobotPCD(1:2:length(DeformedRobotPCD), :);  % Downsampling deformed robot data (Dense enough)

% STL data with without the curve
stl = load('STLPCD');
Y = stl.X;
curveSTL = stl.curveSTLFit;
clear stl;

% Append curve to STL
Y = [Y; curveSTL];

%% Transform the STL to robot data (Close enough for proper initial guess)

Y = Y * rotz(pi/2);
Y = Y + repmat(min(X) - min(Y), length(Y), 1);


%%  Display initial guess
close all;
figure;
draw_coordinate_system(5*ones(3,1), eye(3), [0, 0, 0], 'rgb');
scatter3(Y(:,1), Y(:,2), Y(:,3), '.r');

figure;
draw_coordinate_system(5*ones(3,1), eye(3), [0, 0, 0], 'rgb');
scatter3(X(:,1), X(:,2), X(:,3), '.r');

%% Deform
figure; cpd_plot_iter(X, Y);

% Registering Y to X
[T C]= deformReg(X, Y);

close all;
cpd_plot_iter(X, T.Y);

%% Getting the curve in the deformed data
clear curveRobot;
curveRobot = T.Y(length(Y)-length(curveSTL)+1:length(Y),:);

figure; 
draw_coordinate_system(5, eye(3), [0, 0, 0], 'rgb');
scatter3(T.Y(:,1), T.Y(:,2), T.Y(:,3), '.r');
scatter3(curveRobot(:,1), curveRobot(:,2), curveRobot(:,3), '.b');

%% Read ply & plot surface normals
close all;
YNoCurve_SI = T.Y(1:length(Y)-length(curveSTL),:) ./ 1000.0;
figure; hold on;
[vert tri] = read_ply('model_1mm_old.ply');
PlotTransparentMesh(YNoCurve_SI, tri)

% Recompute the surface normals
TR = triangulation(tri, YNoCurve_SI);
TCenter = incenter(TR);
fn = faceNormal(TR);

quiver3(TCenter(:,1), TCenter(:,2), TCenter(:,3), ...
    fn(:,1), fn(:,2), fn(:,3), 'color', 'r');

%% Save as a mesh file (.mesh seth's format)
MeshSave(YNoCurve_SI, tri, fn, 'deformedRobot.mesh')

%% Registration error

correspondingInX = X(C,:);
curveIndices = (length(Y)-length(curveSTL)+1:length(Y));	
curveRobot_estimate = correspondingInX(curveIndices,:); 	% Transformed curve in STL

% rmse(target, estimate)
regErrorPoints = rmse(correspondingInX, T.Y);
regErrorCurve = rmse(curveRobot, curveRobot_estimate);
