function [Hipz,indexList,actions,key] = randTraj(CommonPara, N)

% DSP = double support phase (sec)
% SSP = single support phase (sec)
% ST = step time = DSP+SSP (sec)
% SD = step distance (one step) (cm)
% LD = distance between convex hull of each foot (cm)
% Step = How many step in forward and lateral direction (number)
% delt = sampling time (sec)
% int = initial waiting time for walking
% endd = extra finishing time
% DSP = 0.1;
% SSP = 0.9;
% SD = 30;
% LD = 15;
% NumOfStep = 2;
% delt = 0.01;
% init = 1;
% endd = 1;

zc = CommonPara(1);
g = CommonPara(2);
DSP = CommonPara(3);
SSP = CommonPara(4);
SD = CommonPara(5);
LD = CommonPara(6);
NumOfStep = CommonPara(7);
delt = CommonPara(8);
init = CommonPara(9);
endd = CommonPara(10);

%%Crouching not necessary, taken care of later
TotalTimeSequence = 0:delt:(init+(NumOfStep+2)*DSP + (NumOfStep+1)*SSP + endd);
[r,c] = size(TotalTimeSequence);

% minZ = 240;
% maxZ = 270;
minZ = 240;
maxZ = 270;

deltZ = .5;

key = (minZ:deltZ:maxZ)';
index = randi(size(key,1),1);
indexList = index;
hipInit = key(index);

yy = hipInit;
for j = 1:NumOfStep+1
    for i = 1:SSP/delt
        index = index+sign((-1)^randi(2))*randi([-N,N],1);
        if (index <1) 
            index = 1;
        elseif index > size(key,1)
            index = size(key,1);
        end
        indexList = [indexList index]
        yy = [yy key(index)];
    end
    yy = [yy key(index)*ones(1,DSP/delt)];
    indexList = [indexList index*ones(1,DSP/delt)];
end

% n = 4;
% 
% xx = delt:delt:SSP;
% x = [delt,SSP*rand(1,n),SSP];
% x = sort(x);
% y = [hipInit,randi([minZ,maxZ],1,n),hipInit];
% yy = interp1(x,y,xx,'cubic');
% 
% while(sum(yy<minZ) || sum(yy>maxZ) || yy(1)~=hipInit ||yy(end)~=hipInit)
%     x = [delt,SSP*rand(1,n),SSP];
%     x = sort(x);
%     y = [hipInit,randi([minZ,maxZ],1,n),hipInit];
%     yy = interp1(x,y,xx,'cubic');
% end
Hipz = [ hipInit*ones(1,(init+DSP)/delt),yy,yy(end)*ones(1,endd/delt)];
Hipz = Hipz-mod(Hipz,deltZ);
actions = (Hipz-[Hipz(2:end),Hipz(end)])*1/deltZ;
