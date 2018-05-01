function Plot_Cartesian_curve_error()
%%  Plot the Virtual Fixture curve error in the Cartesian Exploration
%   There are in total three curves:
%       1) The registered curve using robot data - "curveRobot"
%       2) The registered curve using Laser data - "curveLaser"
%       3) The ground truth curve using Faro Digitizer - "DeformedTargetCurve"
%       All of these three are written in robot frame
%       Note that this is only available in Cartesian robot case becase its
%       base is really easy to be registred to Faro base.
if nargin<1
    mode = 'laser';
end
%%  Load the digitized deformed curve - ground truth
PC_folder_path = [getenv('UDPREGJHU'),'\Cartesian_Data\PointCloudData\'];
DeformedCurve_GroundTruth_Data = ...
    load([PC_folder_path,'DeformedTargetCurve']);
DeformedCurve_GT = DeformedCurve_GroundTruth_Data.TargetCurveData;
clear DeformedCurve_GroundTruth_Data;
ptDeformedCurve = pointCloud(DeformedCurve_GT);
DeformedCurveFitted = FitPointCloudToCurve(ptDeformedCurve);
%%  Load the registered curve
if strcmp(mode,'robot')
    RegisteredCurve_Data = ...
        load([PC_folder_path,'curveRobot']);
    CurveRegistered = RegisteredCurve_Data.curveRobot;
    %%  Note that the registered curve is written in robot frame
    %   To avoid frame misalignment to introduce error, we do a rigid reg
    %   first
    [Transform, Correspondence] = rigidReg(DeformedCurveFitted,CurveRegistered);
    close all;
    CurveRegistered = Transform.Y; % replace the registered curve with the rigidly transformed version
elseif strcmp(mode,'laser')
    RegisteredCurve_Data = ...
        load([PC_folder_path,'curveLaser']);
    CurveRegistered = RegisteredCurve_Data.curveLaser;
end
clear RegisteredCurve_Data;
figure;
axis equal;
hold on;
plot3(CurveRegistered(:,1),CurveRegistered(:,2),CurveRegistered(:,3),...
    '-g','LineWidth',5);
plot3(DeformedCurveFitted(:,1),DeformedCurveFitted(:,2),DeformedCurveFitted(:,3),...
    '-r','LineWidth',5);
[closestPtOnA,distances] = distance2curve(CurveRegistered,DeformedCurveFitted);
for i = 1:length(distances)
    plot3([closestPtOnA(i,1),DeformedCurveFitted(i,1)],...
        [closestPtOnA(i,2),DeformedCurveFitted(i,2)],...
        [closestPtOnA(i,3),DeformedCurveFitted(i,3)],...
        '-k','LineWidth',0.3);
end
curveErr = norm(distances)/sqrt(length(distances));
fprintf('The target curve error is: %.4f\n',curveErr);

end

