function [Transform, Correspondence] = rigidReg(X,Y,varargin)
%%  Rigid registration
%   Set the options
opt.method='rigid'; % use rigid registration
opt.viz=1;          % show every iteration
opt.outliers=0;     % do not assume any noise 

opt.normalize=0;    % normalize to unit variance and zero mean before registering (default)
opt.scale=0;        % estimate global scaling too (default)
opt.rot=1;          % estimate strictly rotational matrix (default)
opt.corresp=1;      % do not compute the correspondence vector at the end of registration (default)

opt.max_it=100;     % max number of iterations
opt.tol=1e-8;       % tolerance

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

% registering Y to X
[Transform, Correspondence]=cpd_register(X,Y,opt);

figure,cpd_plot_iter(X, Y); title('Before');
figure,cpd_plot_iter(X, Transform.Y);  title('After registering Y to X');