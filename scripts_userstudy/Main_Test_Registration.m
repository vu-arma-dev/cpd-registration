clear
close all
clc

addpath(genpath(getenv('ECLDIR')))
addpath(genpath('R:\DesignFiles\Matlab Code\TriangleRayIntersection'));

a=load('FiducialLocations_1');
a.FidLoc=a.FidLoc/1000;
b=load('FiducialLocations_2');
b.FidLoc=b.FidLoc/1000;
c=load('FiducialLocations_3');
c.FidLoc=c.FidLoc/1000;
d=load('FiducialLocations_4');
d.FidLoc=d.FidLoc/1000;
e=load('FiducialLocations_5');
e.FidLoc=e.FidLoc/1000;
f=load('FiducialLocations_6');
f.FidLoc=f.FidLoc/1000;

load('Kidney_A_Artery_Pts')
artery_h{1}=[ptOutput;ones(1,size(ptOutput,2))];
load('Kidney_C_Artery_Pts')
artery_h{2}=[ptOutput;ones(1,size(ptOutput,2))];
load('Kidney_D_Artery_Pts')
artery_h{3}=[ptOutput;ones(1,size(ptOutput,2))];

[F{1},V{1}]=stlread('Kidney_A_UserStudy.stl');
[F{2},V{2}]=stlread('Kidney_B_UserStudy.stl');
[F{3},V{3}]=stlread('Kidney_C_UserStudy.stl');
[F{4},V{4}]=stlread('Kidney_D_UserStudy.stl');
[F{5},V{5}]=stlread('Kidney_E_UserStudy.stl');
[F{6},V{6}]=stlread('Kidney_F_UserStudy.stl');


scanFids{1}=a.FidLoc;
scanFids{2}=c.FidLoc;
scanFids{3}=d.FidLoc;

organFids{1}=a.FidLoc;
organFids{2}=b.FidLoc;
organFids{3}=c.FidLoc;
organFids{4}=d.FidLoc;
organFids{5}=e.FidLoc;
organFids{6}=f.FidLoc;

colorString={'red','blue','green','black','yellow','cyan'};

%%
[n,p]=fitPlane(V{1}');
% plotVec(repmat(n,1,size(artery_h{1},2))*-.02,artery_h{1}(1:3,:));
v=V{1}; f=F{1};
P1=v(f(:,1),:);
P2=v(f(:,2),:);
P3=v(f(:,3),:);

intersect=zeros(size(P1,1),1);
xhist=zeros(size(artery_h{1},2),3);
for i=1:size(artery_h{1},2)
    orig=artery_h{1}(1:3,i)';
    dir=-n'*.01;
    [intersecti, t, bary1, bary2, xcoor] = TriangleRayIntersection(orig,dir, P1,P2,P3,'lineType','line','border','inclusive','eps',1e-8);
    intersect = intersect | intersecti;
    plotVec(dir',orig')
    hold on
    
    if find(intersecti)
        xhist(i,:)=xcoor(intersecti,:);
    end
end
xhist=xhist((any(xhist,2)),:);
trisurf(f,v(:,1),v(:,2),v(:,3), intersect*1.0,'FaceAlpha', 0.9)
plot3(xhist(:,1),xhist(:,2),xhist(:,3),'kx');

figure
plot3(xhist(:,1),xhist(:,2),xhist(:,3),'.')
%%
cpd_dir=getenv('CPDREG');
organDir=[cpd_dir filesep 'userstudy_data' filesep 'PointCloudData' filesep 'RegAprToCT'];

robPosns=[0.0138716380604 -0.0353429420436 -0.0351697119807 0.0115195207286
     	-0.0618899405864 -0.064376841522 0.0862448993117 0.0883603120564
    	-0.182638550176 -0.181607703424 -0.178479708829 -0.178706222603];

temp=load([organDir filesep 'Kidney_A_iter_100_NoOpt.mat']);
kidneyPoints{1}=temp.T.Y'/1000;
temp=load([organDir filesep 'Kidney_B_iter_100_NoOpt.mat']);
kidneyPoints{2}=temp.T.Y'/1000;
temp=load([organDir filesep 'Kidney_C_iter_100_NoOpt.mat']);
kidneyPoints{3}=temp.T.Y'/1000;
temp=load([organDir filesep 'Kidney_D_iter_100_NoOpt.mat']);
kidneyPoints{4}=temp.T.Y'/1000;
temp=load([organDir filesep 'Kidney_E_iter_100_NoOpt.mat']);
kidneyPoints{5}=temp.T.Y'/1000;
temp=load([organDir filesep 'Kidney_F_iter_100_NoOpt.mat']);
kidneyPoints{6}=temp.T.Y'/1000;
clear('temp');


for i=1
    [R,t]=rigidPointRegistration(organFids{i},robPosns);
    fidTransf=R*organFids{i}+t;
    HOrgan=transformation(R,t);

    % Load Organ and register to robot frame
    kidneyHomog=[kidneyPoints{i};ones(1,size(kidneyPoints{i},2))];
    kidneyReg=HOrgan*kidneyHomog;
%     plot3(kidneyReg(1,:),kidneyReg(2,:),kidneyReg(3,:),'.')
    
    V_h=[V{i}';ones(1,length(V{i}))];
    V_transf=HOrgan*V_h;
    patch('Faces',F{i},'Vertices',V_transf(1:3,:)','FaceColor',colorString{i},'FaceAlpha',0.2)
    hold on
    switch i
        case 1
            curve_transf=HOrgan*artery_h{1};
            plot3(curve_transf(1,:),curve_transf(2,:),curve_transf(3,:));
        case 3
            curve_transf=HOrgan*artery_h{2};
            plot3(curve_transf(1,:),curve_transf(2,:),curve_transf(3,:));
        case 4
            curve_transf=HOrgan*artery_h{3};
            plot3(curve_transf(1,:),curve_transf(2,:),curve_transf(3,:));
    end

end


plot3(robPosns(1,:),robPosns(2,:),robPosns(3,:),'g*')
plot3(fidTransf(1,:),fidTransf(2,:),fidTransf(3,:),'bo')