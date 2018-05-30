function DefinePlatformFiducials(FiducialName)
%%  Define the fiducials that give the transformation from robot frame to organ frame
%   Rashid Yasin, 2018/05/30
%   Select the 4 organ registration points using the PSM
%   Specific for the NRI-CSA User study Protocol
%   See documentation at https://git.lcsr.jhu.edu/nri-csa/cisst-saw-nri/wikis/user-study-experiment
%   Note that before running this function, need to init dvrk by:
%       1) add the matlab interface folder path (? I'm not sure this is
%       relevant anymore?)
%       2) run the dvrk console/teleop
%%  Input
%   MapName  -    The name to give for this map
if nargin<1
    FiducialName = input('Give fiducial name name:(enter to be default "FiducialRobot")','s');
    if isempty(FiducialName)
        FiducialName = 'FiducialRobot';
    end
end
while startsWith(FiducialName,'FiducialLocations_')
    fprintf('You cannot use the name ''FiducialLocations_#'' (that is used for organs) \n');
    FiducialName = input('Give fiducial name name:(enter to be default "FiducialRobot")','s');
    if isempty(FiducialName)
        FiducialName = 'FiducialRobot';
    end
end
% dvrk_init_continous_palp;
start_matlab_ros;
setenv('CATKIN_BASE','/home/arma/catkin_ws');
addpath([getenv('CATKIN_BASE') filesep 'src' filesep 'cisst-saw-nri' filesep 'nri-ros' filesep 'dvrk_nri_matlab']);
dvrk = nripsm('PSM2');
DefineNextPoint = 1;
SaveResult = 1;
organNumber=-1;
while organNumber>6 || organNumber <1
    clc
    fprintf('Which Organ are you registering (use numbers 1-6 not letters A-F)?\n');
    organNumber = input('Organ Number: ');
end
clc;
fprintf('Manually move the PSM to a point and \n');
fprintf('[empty] - save this point \n');
fprintf('[q] - quit without saving previous points \n');
N = 4;
FiducialPositions = zeros(N,3);
idx = 1;
while DefineNextPoint && idx<=4
    KeyInput = input('Select:','s');
    switch KeyInput
        case ''
            [p,~,~] = getRobotData(dvrk);
            FiducialPositions(idx,:) = p;
            fprintf('Fiducial point %0.0f added.\n',idx);
            idx = idx + 1;
        case 'q'
            DefineNextPoint = 0;
            SaveResult=0;
    
    end
end
if SaveResult
    %% Save the fiducial locations
    save_mat_path = ...
        [getenv('CATKIN_BASE'),filesep,...
        'src',filesep,'cpd-registration',filesep,'userstudy_data',filesep,'FiducialLocations'];
    if ~exist(save_mat_path,'dir')
        mkdir(save_mat_path);
    end
    save([save_mat_path,filesep,FiducialName],'FiducialPositions');
    fprintf('Fiducial Positions Saved\n');

    %% Calculate and write the registration result to file
    organFiducial=load([save_mat_path,filesep,...
        'FiducialLocations_',num2str(organNumber)]);
    [R,t] = rigidPointRegistration(organFiducial.FidLoc,FiducialPositions'*1000);
    qOut=rotm2quat(R);
    fileID = fopen([save_mat_path,filesep, 'organ_' num2str(organNumber) '_reg.txt'],'w');
    fprintf(fileID,'Position (mm)\n');
    fprintf(fileID,'%f %f %f\n',t);
    fprintf(fileID,'Orientation (w x y z)\n');
    fprintf(fileID,'%f %f %f %f',qOut);
    fclose(fileID);

end


end

