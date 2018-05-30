% Perform rigid registration with no scaling
% This uses the SVD method, which has a long and varied history
% Find R and t to minimize Q = Rx + t - y
% Therefore we are finding R and t to rotate the set x to match the set y
% x and y are matrices of data size [D x N] where D is the dimension, and N
% is the number of data points
% References include Shonemann 1966, Farrell 1966, Umeyama 1991, as well as
% Kabsch, Arun and many others

% http://cnx.org/contents/1d5f91b1-dc0b-44ff-8b4d-8809313588f2@23/Molecular_Distance_Measures

function [R,t]=rigidPointRegistration(x,y)
x0=repmat(mean(x,2),1,size(x,2));
y0=repmat(mean(y,2),1,size(y,2));

xtild= x-x0;
ytild= y-y0;

cMat=xtild*ytild';

[U,~,V]=svd(cMat);

signmat=eye(size(x,1));
signmat(end,end)=det(V*U);

R=V* signmat*U';
t=y0(:,1)-R*x0(:,1);
end