% Interpolate inside a mesh
% Adapted from:
% https://blogs.mathworks.com/videos/2010/05/14/code-review-finding-the-closest-point-on-a-surface/

function P = upsamplemesh(F,V,upsampling)
P1=[V(F(:,1),1),V(F(:,1),2),V(F(:,1),3)];
P2=[V(F(:,2),1),V(F(:,2),2),V(F(:,2),3)];
P3=[V(F(:,3),1),V(F(:,3),2),V(F(:,3),3)];

V1=P2-P1;
V2=P3-P2;

Ntot=size(V,1)* (upsampling^2+3*upsampling+2)/2;
P=zeros(Ntot,3);
index=0;
for fIndex=1:size(F,1)
    p1=P1(fIndex,:);
    v1=V1(fIndex,:);
    v2=V2(fIndex,:);
    for i=0:upsampling
        for j=linspace(0,1,upsampling-1+1)
            r1=i*1/upsampling;
            r2=(1-r1)*j;
            index=index+1;
            P(index,:)=v1*r1+v2*r2+p1;
        end
    end
end

P=unique(P,'rows');

end