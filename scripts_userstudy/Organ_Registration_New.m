
% Save the fiducial locations of each of the new sphere organs
function Organ_Registration_New(plotOption)
if nargin<1
    plotOption=0;
end
FidSaveFolder = 'R:\Robots\CPD_Reg.git\userstudy_data\FiducialLocations\';
FiducialPlyFolder = 'R:\Projects\NRI\PalpationSTL\User_Study\Output\';

%   offset the raidus of probe ball
%       According to the spec of dimples, 45 degree slop counter-sink
HalfCounterSinkAngle = pi/2/2;

probeBall_R = 6.35/2; % unit in [mm]


for jj=[10:12,20:22]
    subFolder = ['Kidney' num2str(jj) '_output\'];
    
    A=pcread([FiducialPlyFolder subFolder 'FidA.ply']);
    B=pcread([FiducialPlyFolder subFolder 'FidB.ply']);
    C=pcread([FiducialPlyFolder subFolder 'FidC.ply']);
    D=pcread([FiducialPlyFolder subFolder 'FidD.ply']);
    
    ptA=double(mean(A.Location))';
    ptB=double(mean(B.Location))';
    ptC=double(mean(C.Location))';
    ptD=double(mean(D.Location))';

    FidLoc = [ptA, ptB, ptC, ptD];
    
    offset_dir=fitPlane(FidLoc);
    ba=FidLoc(:,1)-FidLoc(:,2);
    bc=FidLoc(:,3)-FidLoc(:,2);
    vertical=cross(normc(ba),normc(bc));
    flip=sign(dot(offset_dir,vertical));
    offset_dir=offset_dir*flip;
    
    if plotOption
        figure
        plot3(FidLoc(1,:),FidLoc(2,:),FidLoc(3,:),'bo')
        hold on
        plot3(FidLoc(1,1),FidLoc(2,1),FidLoc(3,1),'k*')
    end    
    FidLoc=FidLoc+offset_dir*(probeBall_R/sin(HalfCounterSinkAngle));
    
    if plotOption
        plot3(FidLoc(1,:),FidLoc(2,:),FidLoc(3,:),'rx')
        axis equal
    end
    
    
    save([FidSaveFolder, 'FiducialLocations_' num2str(jj)],'FidLoc');
end

end