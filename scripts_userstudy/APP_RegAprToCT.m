function APP_RegAprToCT(phantomModelName,CPDDir)
%%  Register the Apriori model to the CT scan data
%   By Long Wang, 2018/4/4
if nargin<1
%     phantomModelName = 'A';
    phantomModelName = 10;
end
switch phantomModelName
    case 'A'
        CT_Scan_PtName = 'KidneyA_TopOnly_Reduced.stl';
    case 'B'
        CT_Scan_PtName = 'KidneyB_TopOnly_Reduced.stl';
    case 'C'
        CT_Scan_PtName = 'KidneyC_TopOnly_Reduced.stl';
    case 'D'
        CT_Scan_PtName = 'KidneyD_TopOnly_Reduced.stl';
    case 'E'
        CT_Scan_PtName = 'KidneyE_TopOnly_Reduced.stl';
    case 'F'
        CT_Scan_PtName = 'KidneyF_TopOnly_Reduced.stl';
end
% Kidney10_Top.stl to Kidney22_Top.stl
if phantomModelName>=10&& phantomModelName<=22
    CT_Scan_PtName = ['Kidney' num2str(phantomModelName) '_top.stl'];
end
%%  Load data files
% Setup_Dir_DeformableReg;
%   load the A-priori
PLY_path_PSM = [CPDDir filesep 'psm_data' filesep 'PLY' filesep];
Apriori_Name= 'kidney_and_base_2mm_aligned.ply';
ptApriori = pcread([PLY_path_PSM,Apriori_Name]);
%  rotate the apriori to match the exploration better
R1 = axang2rotm([0 0 1 pi]);
R2 = axang2rotm([1 0 0 pi/4]);
if phantomModelName=='F'
    R2=R2*axang2rotm([0 1 0 pi]);
end
A = [R2*R1,zeros(3,1); ...
     0 0 0 1];
tform = affine3d(A);
ptApriori = pctransform(ptApriori,tform);
%   load the CT-scan
STL_path_CT = [CPDDir filesep 'userstudy_data' filesep 'STL' filesep];
[F,V] = stlread([STL_path_CT,CT_Scan_PtName]);
ptCTScan = pointCloud(V);
ptCTScan = pcdownsample(ptCTScan,'gridAverage',1);
%% register
PC_path_CT = [CPDDir filesep 'userstudy_data' filesep 'PointCloudData' filesep];
SaveResultsFolder = [PC_path_CT,'RegAprToCT' filesep];
AprioriModel = ptApriori.Location;
if ~exist(SaveResultsFolder,'dir')
    mkdir(SaveResultsFolder);
end
k = 1;
[T,C]= deformReg(ptCTScan.Location,AprioriModel,'max iter',k);
save([SaveResultsFolder,'Kidney_',num2str(phantomModelName) ,'_iter_',num2str(k),'_NoOpt'],...
    'T','C','ptCTScan','ptApriori');
close all;
end

