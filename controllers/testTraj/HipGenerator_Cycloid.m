function [Hipx, Hipy] = HipGenerator_Cycloid(CommonPara,CycloidTune)
% - height (l) = the height of CoM (m)
% - gravity = 9.81 m/sec^2
% DSP = double support phase (sec)
% SSP = single support phase (sec)
% ST = step time = DSP+SSP (sec)
% SD = step distance (one step) (cm)
% LD = distance between foot (cm)
% Step = How many step in forward and lateral direction (number)
% delt = sampling time (sec)
% int = initial waiting time for walking
% endd = extra finishing time

l = CommonPara(1);
g = CommonPara(2);
DSP = CommonPara(3);
SSP = CommonPara(4);
SD = CommonPara(5);
LD = CommonPara(6);
NumOfStep = CommonPara(7);
delt = CommonPara(8);
init = CommonPara(9);
endd = CommonPara(10);

% Tune = Tuning paramenters
% - HipSmoothingWeight_X = to smooth the hip motion in forward direction
% - RatioOfHipPosition_X = hip position at the center in DSP
% if ratio < 0.5, CoM is in forward from the center
% - HipAmpWeight_Y = hip amplitude weight
% if weight < 1, amplitude would be bigger. 
% HipSmoothingWeight_X = 3;
% RatioOfHipPosition_X = 0.5;
% HipAmpWeight_Y = 1;
% HTune = [HipSmoothingWeight_X RatioOfHipPosition_X HipAmpWeight_Y];

weight1 = CycloidTune(1);
ratio = CycloidTune(2);
weight2 = CycloidTune(3);

freq = 1/(DSP+SSP);
omega = 2*pi*freq;
if mod(ratio,0.1) ~= 0
    temp = delt;
else
    temp = 0;
end

%% Hip in Forward Direction Trajectory Generation
Hipx = [];
for i = 0:delt:(init+DSP+SSP*ratio)
    Hipx = [Hipx 0]; 
end
for i = delt:delt:(DSP+SSP)*NumOfStep
    Hipx = [Hipx (SD/weight1)/(2*pi)*(weight1*omega*i - (1+(l*omega^2)/g)*sin(omega*i))];
end
for i = delt:delt:(SSP*(1-ratio)+DSP+endd+temp)
    Hipx = [Hipx Hipx(end)];
end

%% Hip in Lateral Direction Trajectory Generation
Hipy = [];
T = omega/2;
A = (LD/2)/(1+(l*weight2/g)*(T)^2);
for i = 0:delt:DSP/2+init
    Hipy = [Hipy 0];
end
for i = delt:delt:(DSP+SSP)*(1+NumOfStep)
    Hipy = [Hipy A*sin(T*i)];
end
for i = delt:delt:(DSP/2+endd)
    Hipy = [Hipy Hipy(end)];
end





