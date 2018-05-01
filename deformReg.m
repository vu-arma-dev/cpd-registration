function [Transform, C] = deformReg(X,Y,varargin)
%%  Deformable registration
%   Default options
opt.method='nonrigid_lowrank'; % use nonrigid registration
% opt.method='rigid'; % use nonrigid registration
opt.beta=6;            			% the width of Gaussian kernel (smoothness)
opt.lambda=3;          			% regularization weight

opt.viz=1;              % show every iteration
opt.outliers=0;       % noise weight
opt.fgt=0;              % do not use FGT (default)
opt.normalize=1;        % normalize to unit variance and zero mean before registering (default)
opt.corresp=1;          % compute correspondence vector at the end of registration (not being estimated by default)

opt.max_it=100;         % max number of iterations
opt.tol=1e-10;          % tolerance
%%  parsing options
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'max iter')
            opt.max_it = propertyValue;
        elseif strcmp(propertyName,'tol')
            opt.tol = propertyValue;
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[Transform, C]=cpd_register(X,Y,opt);
% DeformedMesh = Transform.Y;
figure,cpd_plot_iter(X, Y); title('Before');
figure,cpd_plot_iter(X, Transform.Y);  title('After registering Y to X');
end