function APP_RegAprToCT(phantomModelName)
%%  Register the Apriori model to the CT scan data
%   By Long Wang, 2018/4/4
if nargin<1
    phantomModelName = 'A';
end
switch phantomModelName
    case 'A'
        CT_Scan_PtName = 'CT_kidneyA_mesh.ply';
end
%%  Load data files
Setup_Dir_DeformableReg;
%   load the A-priori
PLY_path_PSM = [getenv('UDPREGJHU'),'\PSM_Data\PLY\'];
Apriori_Name= 'kidney_and_base_2mm_aligned.ply';
ptApriori = pcread([PLY_path_PSM,Apriori_Name]);
%  rotate the apriori to match the exploration better
R1 = axang2rotm([0 0 1 pi]);
R2 = axang2rotm([1 0 0 pi/4]);
A = [R2*R1,zeros(3,1); ...
     0 0 0 1];
tform = affine3d(A);
ptApriori = pctransform(ptApriori,tform);
%   load the CT-scan
PLY_path_CT = [getenv('UDPREGJHU'),'\UserStudy_Data\PLY\'];
ptCTScan = pcread([PLY_path_CT,CT_Scan_PtName]);
ptCTScan = pcdownsample(ptCTScan,'gridAverage',2);
%% register
PC_path_CT = [getenv('UDPREGJHU'),'\UserStudy_Data\PointCloudData\'];
SaveResultsFolder = [PC_path_CT,'RegAprToCT\'];
AprioriModel = ptApriori.Location;
if ~exist(SaveResultsFolder,'dir')
    mkdir(SaveResultsFolder);
end
k = 100;
[T,C]= deformReg(ptCTScan.Location,AprioriModel,'max iter',k);
save([SaveResultsFolder,'iter_',num2str(k)],...
    'T','C','ptCTScan','ptApriori');
close all;
end

