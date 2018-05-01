function Plot_deformReg_info_Cartesian(robot_or_laser,varargin)
%%  Plots of registration results for
%%  Deformable registration using Cartesian robot data
%%  (or laser baseline for Cartesian)
%   By Long Wang, 2016/10
%   This func plot:
%       1) the registered surface (deforming the A-priori) vs. the collected data
%       2) the registered curve (deforming the A-priori) vs. the ground truth
if nargin<1
    robot_or_laser = 'robot';
end
ground_truth_mode= 'frame_transform';
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'ground truth mode')
            ground_truth_mode = propertyValue;
        end
    end
end

%%  Figure 1 - Registered suface overlay on the explr data (or laser data)
figure;
hold on;
box on;
grid on;
markerSize = 25;
%   Get the explr point cloud and show it
ptExplr = ...
    Get_registered_result('Cartesian',robot_or_laser,'data type','ExplrPtCloud');
ptExplr.Color = uint8(repmat([0,0,255],ptExplr.Count,1));
pcshow(ptExplr,'MarkerSize',markerSize);
%   Get the registered point cloud and show it
ptCurrentModel = ...
    Get_registered_result('Cartesian',robot_or_laser,'data type','RegPtCloud');
ptCurrentModel.Color = uint8(repmat([255,0,0],ptCurrentModel.Count,1));
pcshow(ptCurrentModel,'MarkerSize',markerSize);
%   Get the registered curve and show it
CurrentRegCurve = ...
    Get_registered_result('Cartesian',robot_or_laser,'data type','RegCurve');
plot3(CurrentRegCurve(:,1),CurrentRegCurve(:,2),CurrentRegCurve(:,3),...
    '-g','LineWidth',5);
view(90,68);
%%  Figure 2 - Registered curve and the ground truth
%   The ground truth (GT)
GT_DigitizedCurveData = Get_registered_result('Cartesian',robot_or_laser,...
    'data type','GroundTruthDigitized');
fprintf('Digitized curve as Ground Truth fitted, fit err RMS=%0.3f\n',...
    GT_DigitizedCurveData.FittedRMS);
GT_DigitizedCurveFit = GT_DigitizedCurveData.FittedPt;
if strcmp(robot_or_laser,'robot') && strcmp(ground_truth_mode,'frame_transform')
    %   calculate distance between curveA and curveB and plot the curves with
    %   errors
    %   Note that in the case of Cartesian robot
    %   The ground truth is collected using only Faro and the base frame might
    %   have been moved.
    %   So, to avoid this effect on the error, a rigid registration is
    %   performed before plot/compute the curve errors
    T_rob2laser = ...
        Get_registered_result('Cartesian','robot',...
        'data type','frame_rob2laser');
    GT_Transpose = T_rob2laser.R'*GT_DigitizedCurveFit' - ...
        repmat(T_rob2laser.t,1,length(GT_DigitizedCurveFit));
    GT_DigitizedCurve_Final = GT_Transpose';
elseif strcmp(ground_truth_mode,'shape feature')
    [T_rigidReg,C] = ...
        rigidReg(CurrentRegCurve,GT_DigitizedCurveFit,'max iter',50);
    GT_DigitizedCurve_Final = T_rigidReg.Y;
else
    GT_DigitizedCurve_Final = GT_DigitizedCurveFit;
end
figure;
view(90,90);
axis equal;
grid on;
box on;
set(gca,'GridAlpha',0.6,'GridLineStyle','--');
hold on;
plot3(CurrentRegCurve(:,1),CurrentRegCurve(:,2),CurrentRegCurve(:,3),...
    '-g','LineWidth',5);
plot3(GT_DigitizedCurve_Final(:,1),GT_DigitizedCurve_Final(:,2),GT_DigitizedCurve_Final(:,3),...
    '-r','LineWidth',5);
%%  pick the shorter distances
%   curve_dist_mode = 1, distance2curve(curve,GT)
%   curve_dist_mode = 2, distance2curve(GT,curve)
[closestPtOnCurve,distances_mode1] = distance2curve(CurrentRegCurve,GT_DigitizedCurve_Final);
[closestPtOnGT,distances_mode2] = distance2curve(GT_DigitizedCurve_Final,CurrentRegCurve);
norm_dist_mode1 = norm(distances_mode1)/sqrt(length(distances_mode1));
norm_dist_mode2 = norm(distances_mode2)/sqrt(length(distances_mode2));
if norm_dist_mode1<norm_dist_mode2
    for i = 1:length(distances_mode1)
        plot3([closestPtOnCurve(i,1),GT_DigitizedCurve_Final(i,1)],...
            [closestPtOnCurve(i,2),GT_DigitizedCurve_Final(i,2)],...
            [closestPtOnCurve(i,3),GT_DigitizedCurve_Final(i,3)],...
            '-k','LineWidth',0.3);
    end
    CurveErrorToGroundTruth = norm_dist_mode1;
else
    for i = 1:length(distances_mode2)
        plot3([closestPtOnGT(i,1),CurrentRegCurve(i,1)],...
            [closestPtOnGT(i,2),CurrentRegCurve(i,2)],...
            [closestPtOnGT(i,3),CurrentRegCurve(i,3)],...
            '-k','LineWidth',0.3);
    end
    CurveErrorToGroundTruth = norm_dist_mode2;
end
fprintf('The target curve error w.r.t. ground truth is: %.3f\n',CurveErrorToGroundTruth);
%%  Calculate Registration Residual error
CorrespPointCloudInExplr = Get_registered_result('Cartesian',robot_or_laser,...
    'data type','CorrespPtInExplr');
RegResidualError = rmse(ptCurrentModel.Location,CorrespPointCloudInExplr.Location);
fprintf('The registration residual error is: %.3f\n',RegResidualError);
%%  Registration Target error
CorrespCurveInExplr = Get_registered_result('Cartesian',robot_or_laser,...
    'data type','CorrespCurveInExplr');
regErrorCurve = rmse(CorrespCurveInExplr, CurrentRegCurve);
fprintf('The target curve registration error is: %.3f\n',regErrorCurve);

end