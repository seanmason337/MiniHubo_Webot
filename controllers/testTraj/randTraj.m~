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

minZ = 240;
maxZ = 270;
deltZ = 1;

key = (minZ:deltZ:maxZ)';
index = randi(size(key,1),1);
hipInit = key(index);
Hipz = zeros(1,c);
indexList = zeros(1,c);
indexList(1) = index;
Hipz(1,1) = hipInit;
for i = 2:c
    index = index+sign((-1)^randi(2))*randi([-N,N],1);
    if (index <1) 
        index = 1;
    elseif index > size(key,1)
        index = size(key,1);
    end
    indexList(i) = index;
    Hipz(1,i) = key(index);
end

actions = [(indexList(2:end)-indexList(1:end-1))+2 , 2];
