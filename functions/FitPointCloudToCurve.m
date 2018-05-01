function [FittedCurve,RMS_error] = FitPointCloudToCurve(ptCloud,varargin)
%%  This func fit a point cloud to a curve in space
%   By Long Wang, 2016/10/5 (modified from Jason Pile's scripts)
%   preferably, this point cloud should have a good projection in XY plane.
%% default and parse optional argument
res_ds = 0.5;
n_order = [5,2];
mode = '[x,z]=f(y)';
fig_preview = 'off';
EvaluateError = 'on';
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'resolution')
            res_ds = propertyValue;
        elseif strcmp(propertyName,'poly order')
            n_order = propertyValue;
            %   This is [2x1], the first element is for the first function
            %   the second element is for the second function
        elseif strcmp(propertyName,'variable mode')
            mode = propertyValue;
        elseif strcmp(propertyName,'figure')
            fig_preview= propertyValue;
        elseif strcmp(propertyName,'error')
            EvaluateError= propertyValue;
        end
    end
end
%%  fit a curve for y=f(x) or x=f(y)
X = ptCloud.Location(:,1);
Y = ptCloud.Location(:,2);
Z = ptCloud.Location(:,3);
if strcmp(mode,'[x,z]=f(y)')
    coef_x_on_y = polyfit(Y,X,n_order(1));
    coef_z_on_y = polyfit(Y,Z,n_order(2));
    tmpY = min(Y):res_ds:max(Y);
    tmpX = polyval(coef_x_on_y, tmpY);
    tmpZ = polyval(coef_z_on_y, tmpY);
elseif strcmp(mode,'[y,z]=f(x)')
    coef_y_on_x = polyfit(X,Y,n_order(1));
    coef_z_on_x = polyfit(X,Z,n_order(2));
    tmpX = min(X):res_ds:max(X);
    tmpY = polyval(coef_y_on_x, tmpX);
    tmpZ = polyval(coef_z_on_x, tmpX);
end
FittedCurve= [tmpX',tmpY',tmpZ'];
if strcmp(EvaluateError,'on')
    [~,distances] = distance2curve(FittedCurve,[X,Y,Z]);
    RMS_error = norm(distances)/sqrt(length(distances));
else
    RMS_error = nan;
end
if strcmp(fig_preview,'on')
    figure;
    axis equal;
    hold on;
    draw_coordinate_system(5*ones(3,1), eye(3), [0, 0, 0], ['r','g','b']);
    ptCloud.Color = uint8(repmat([255,0,0],ptCloud.Count,1));
    pcshow(ptCloud)
    plot3(FittedCurve(:,1),FittedCurve(:,2),FittedCurve(:,3), '-b');
end
end

