% Fiducial Locations in base stl (undeformed model)
% Each fidicial is made up of 3 points
% point t is the top of the screw hole next to that fiducial
% point b is the bottom of the screw hole next to that fiducial
% point p is the fiducial point
% Aligned as
% N = [tx, ty, tz
%      bx, by, bz
%      px, py, pz];

A = [25 , 7 , 62.5
    25, 0, 62.5
    25,2.05025,76.5];

B = [-25 , 7 ,62.5
    -25, 0, 62.5
    -25,2.05025,76.5];

C = [-25 , 7 , -62.5
    -25, 0, -62.5
    -25,2.05025,-76.5];

D = [25 , 7, -62.5
     25 , 0, -62.5
     25 , 2.05025,-76.5];

stlPointList=[A;B;C;D];