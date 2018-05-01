function APP_RegFrames_PSM(iter_num)
%%  Register the robot base frame to Faro ARM frame
%   By Long Wang, 2016/10/10
%   This is needed because when evaluating the error from the updated VF
%   curve to the ground truth digitization, both curve need to be written
%   in the same frame.
%%  parse optional arguments
%%  Load data files
Setup_Dir_DeformableReg;
ptModel_in_rob_frame = Get_registered_result('PSM','robot',...
    'data type','RegPtCloud','iter num',iter_num);
ptModel_in_laser_frame = Get_registered_result('PSM','laser',...
    'data type','RegPtCloud','iter num',iter_num);
%% register
PC_path_PSM = [getenv('UDPREGJHU'),'\PSM_Data\PointCloudData\'];
SaveResultsFolder = [PC_path_PSM,'RegFrames_rob2laser\'];
if ~exist(SaveResultsFolder,'dir')
    mkdir(SaveResultsFolder);
end
k = 30;
[T,C]= rigidReg(ptModel_in_laser_frame.Location,ptModel_in_rob_frame.Location,'max iter',k);
save([SaveResultsFolder,'iter_',num2str(iter_num)],...
    'T','C');
end