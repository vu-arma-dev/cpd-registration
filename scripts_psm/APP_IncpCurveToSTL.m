function APP_IncpCurveToSTL(varargin)
%%  Incoporating target curve (Virtual fixture) to the A-priori STL model
%   By Long Wang, 2016/10/5
%   This is consistent with Long's JMR draft.
%   A curve is digitized in the same frame as a point cloud is collected
%   using laser scanner.
%   How to incorporating this digitized curve into an A-priori STL model is
%   the problem solved here.
%%  parse optional arguments
OnlyLoadResult = 'no';
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'only load result')
            OnlyLoadResult = propertyValue;
            %   This option will skip the computation only show result
        end
    end
end%%  Load data files
Setup_Dir_DeformableReg;
PLY_path_PSM = [getenv('UDPREGJHU'),'\PSM_Data\PLY\'];
Apriori_Name= 'STL_model_1mm_PSM_use.ply';
CurvePtName = 'digitized_curve_laser_PSM.ply';
LaserPtName = 'undeformed_model_PSM_downsampled.ply';
ptApriori = pcread([PLY_path_PSM,Apriori_Name]);
ptCurve = pcread([PLY_path_PSM,CurvePtName]);
ptLaser = pcread([PLY_path_PSM,LaserPtName]);
%%  compute the registration or load previous result
PC_path_PSM = [getenv('UDPREGJHU'),'\PSM_Data\PointCloudData\'];
if strcmp(OnlyLoadResult,'yes')
    load([PC_path_PSM,'PSMCurveIncorpApriori']);
else
    %%  Registering ptLaser to ptApriori
    LaserCurveMerge = [ptLaser.Location;ptCurve.Location];
    [T,C]= deformReg(ptApriori.Location,LaserCurveMerge,...
        'max iter',50);
    ptCurveIncp = pointCloud(T.Y(ptLaser.Count+1:end,:));
    ptLaserRegistered = pointCloud(T.Y(1:ptLaser.Count,:));
    %%  Fit the curve
    CurveIncpFitted = FitPointCloudToCurve(ptCurveIncp);
    %%  Project the curve onto the phantom surface
    %   This can be easily done by replacing the Z coord with -16 which is
    %   the current plane height.
    CurveIncpFittedProjected = [CurveIncpFitted(:,1:2),-16*ones(length(CurveIncpFitted),1)];
    %%  Save the results
    save([PC_path_PSM,'PSMCurveIncorpApriori'],'ptCurveIncp','ptLaserRegistered','CurveIncpFitted','CurveIncpFittedProjected');
end
%%  Plot the result
figure;
draw_coordinate_system(5*ones(3,1), eye(3), [0, 0, 0], 'rgb');
pcshow(ptApriori);
plot3(CurveIncpFitted(:,1),CurveIncpFitted(:,2),CurveIncpFitted(:,3),'-r','LineWidth',2);
plot3(CurveIncpFittedProjected(:,1),CurveIncpFittedProjected(:,2),CurveIncpFittedProjected(:,3),'-m','LineWidth',2);
end

