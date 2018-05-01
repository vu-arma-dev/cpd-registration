function Plot_deformReg_iteration_info_PSM(iter_num,robot_or_laser,varargin)
%%  Plots of registration results for a particular iteration of
%%  Deformable registration using PSM data (or laser baseline for PSM)
%   By Long Wang, 2016/10
%   This func plot:
%       1) the registered surface (deforming the A-priori) vs. the collected data
%       2) the registered curve (deforming the A-priori) vs. the ground
%       truth
if nargin<2
    robot_or_laser = 'robot';
end
ground_truth_mode= 'rob_dig';
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
str = sprintf([robot_or_laser,' deformReg','iter = %0.0f'],iter_num);
% title(str);
hold on;
box on;
grid on;
markerSize = 25;
%   Get the explr point cloud and show it
ptExplr = Get_registered_result('PSM',robot_or_laser,...
    'data type','ExplrPtCloud','iter num',iter_num);
ptExplr.Color = uint8(repmat([0,0,255],ptExplr.Count,1));
pcshow(ptExplr,'MarkerSize',markerSize);
%   Get the registered point cloud and show it
ptCurrentModel = Get_registered_result('PSM',robot_or_laser,...
    'data type','RegPtCloud','iter num',iter_num);
ptCurrentModel.Color = uint8(repmat([255,0,0],ptCurrentModel.Count,1));
pcshow(ptCurrentModel,'MarkerSize',markerSize);
%   Get the registered curve and show it
CurrentRegCurve = Get_registered_result('PSM',robot_or_laser,...
    'data type','RegCurve','iter num',iter_num);
plot3(CurrentRegCurve(:,1),CurrentRegCurve(:,2),CurrentRegCurve(:,3),...
    '-g','LineWidth',5);
view(90,68);
%%  Figure 2 - Registered curve and the ground truth
%   The ground truth
if strcmp(robot_or_laser,'laser')||...
        strcmp(ground_truth_mode,'rigid_reg')||...
        strcmp(ground_truth_mode,'shape feature')
    GroundTruthDigitizedCurveData = Get_registered_result('PSM','laser',...
        'data type','GroundTruthDigitized','iter num',iter_num);
    fprintf('Digitized curve as Ground Truth fitted, fit err RMS=%0.2f\n',...
        GroundTruthDigitizedCurveData.FittedRMS);
    GT_DigitizedCurveFit = GroundTruthDigitizedCurveData.FittedPt;
    %   calculate distance between curveA and curveB and plot the curves with
    %   errors
    if strcmp(ground_truth_mode,'rigid_reg')
        %   The ground truth is collected using only Faro and the base frame might
        %   have been moved.
        %   So, to avoid this effect on the error, a rigid registration is
        %   performed before plot/compute the curve errors
        T_rob2laser = ...
            Get_registered_result('PSM','robot',...
            'data type','frame_rob2laser','iter num',100);
        GT_Transpose = T_rob2laser.R'*GT_DigitizedCurveFit'- ...
            repmat(T_rob2laser.t,1,length(GT_DigitizedCurveFit));
        GT_DigitizedCurve_Final = GT_Transpose';
    elseif strcmp(ground_truth_mode,'shape feature')
        %   in this mode, we use a rigid registration to capture only the
        %   shape difference
        [T_rigidReg,C] = ...
            rigidReg(CurrentRegCurve,GT_DigitizedCurveFit,'max iter',50);
        GT_DigitizedCurve_Final = T_rigidReg.Y;
    else
        GT_DigitizedCurve_Final = GT_DigitizedCurveFit;
    end
elseif strcmp(robot_or_laser,'robot') && strcmp(ground_truth_mode,'rob_dig')
    GroundTruthDigitizedCurveData = Get_registered_result('PSM','robot',...
        'data type','GroundTruthDigitized');
    fprintf('Digitized curve as Ground Truth fitted, fit err RMS=%0.3f\n',...
        GroundTruthDigitizedCurveData.FittedRMS);
    GT_DigitizedCurve_Final = GroundTruthDigitizedCurveData.FittedPt;
end
%   plot
figure;
view(0,90);
axis equal;
grid on;
box on;
set(gca,'GridAlpha',0.6,'GridLineStyle','--');
hold on;
plot3(CurrentRegCurve(:,1),CurrentRegCurve(:,2),CurrentRegCurve(:,3),...
    '-g','LineWidth',5);
plot3(GT_DigitizedCurve_Final(:,1),GT_DigitizedCurve_Final(:,2),GT_DigitizedCurve_Final(:,3),...
    '-r','LineWidth',5);
[closestPtOnA,distances] = distance2curve(GT_DigitizedCurve_Final,CurrentRegCurve);
for i = 1:length(distances)
    plot3([closestPtOnA(i,1),CurrentRegCurve(i,1)],...
        [closestPtOnA(i,2),CurrentRegCurve(i,2)],...
        [closestPtOnA(i,3),CurrentRegCurve(i,3)],...
        '-k','LineWidth',0.3);
end
CurveErrorToGroundTruth = norm(distances)/sqrt(length(distances));
fprintf('The target curve error w.r.t. ground truth is: %.3f\n',CurveErrorToGroundTruth);
%%  Calculate Registration Residual error
CorrespPointCloudInExplr = Get_registered_result('PSM',robot_or_laser,...
    'data type','CorrespPtInExplr','iter num',iter_num);
RegResidualError = rmse(ptCurrentModel.Location,CorrespPointCloudInExplr.Location);
fprintf('The registration residual error is: %.3f\n',RegResidualError);
%%  Registration Target error
CorrespCurveInExplr = Get_registered_result('PSM',robot_or_laser,...
    'data type','CorrespCurveInExplr','iter num',iter_num);
regErrorCurve = rmse(CorrespCurveInExplr, CurrentRegCurve);
fprintf('The target curve registration error is: %.3f\n',regErrorCurve);

end