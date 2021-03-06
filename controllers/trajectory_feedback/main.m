function [Hipx_Preview,Hipy_Preview] = main(Hipz,CommonPara)
%% Common Parameters for Walking Pattern Generation
% Trajectory Generation Parameters
% - height = the height of CoM (m)
% - gravity = 9.81 m/sec^2
% DSP = double support phase (sec)
% SSP = single support phase (sec)
% ST = step time = DSP+SSP (sec)
% SD = step distance (one step) (m)
% LD = distance between convex hullof each foot, but will be changed for tuning (m)
%    = (Lateral ZMP location)
% Step = How many step in forward and lateral direction (number)
% delt = sampling time (sec)
% int = initial waiting time for walking
% endd = extra finishing time

Height = CommonPara(1);
Gravity = CommonPara(2);
DSP = CommonPara(3);
SSP = CommonPara(4);
SD = CommonPara(5);
LD = CommonPara(6);
NumOfStep = CommonPara(7);
delt = CommonPara(8);
init = CommonPara(9);
endd = CommonPara(10);
stairH = CommonPara(11);
%% Tunning Parameters for FootGenerator.m (Cycloid algorithms)
% FootForwardDistance = foot forward distance for one step
% FootLateralDistance = foot lateral distance for one step
% FootUpwardHeight = maximum height of foot when swinging
% FootLateralInit = orignal position of each foot from the center
% FootHmax = Foot maximum height scale factor for stair climbing
% FootHmax(1) = maximum height of foot for the first step (scale factor),
% normally = 0.9
% FootHmax(2) = maximum height of foot after the first step (scale factor)
% normally = 2
% coordinate
FootForwardDistance = SD;
FootLateralDistance = 0;
FootUpwardHeight = 50;
FootLateralInit = 40;
FootHmax = [0.8 2];

FootPara = [FootForwardDistance FootLateralDistance FootUpwardHeight FootLateralInit FootHmax(1) FootHmax(2)];

%% Tuning Parameter for HipGenerator_Cycloid.m (Cycloid algorithms)
% Tune = Tuning paramenters
% - HipSmoothingWeight_X = to smooth the hip motion in forward direction
% - RatioOfHipPosition_X = hip position at the center in DSP
% if ratio < 0.5, CoM is in forward from the center
% - HipAmpWeight_Y = hip amplitude weight
% if weight < 1, amplitude would be bigger. 
HipSmoothingWeight_X = 3.3;
RatioOfHipPosition_X = 0.45;
HipAmpWeight_Y = .4;

CycloidTune = [HipSmoothingWeight_X RatioOfHipPosition_X HipAmpWeight_Y];

% ZMP Trajectory Generation
[Xzmp, Yzmp,TotalTimeSequence] = ZMPGenerator(CommonPara);
ZMPTrajectory = [Xzmp; Yzmp;TotalTimeSequence];

%% Foot Trajectory Generation
% Foot trajectory Generation �� �� �����ϰ� ����, Ŀ�굹�� �ְ� ����
[Footrx,Footry,Footrz,Footlx,Footly,Footlz]=FootGenerator_DSP(CommonPara,FootPara);


%% Hip Trajectory Generation (Fourier)
% [Hipx_Fourier, Hipy_Fourier] = HipGenerator_Fourier(CommonPara,FourierTune);


%% Hip Trajectory Generation (Cycloid)
[Hipx_Cycloid, Hipy_Cycloid] = HipGenerator_Cycloid(CommonPara,CycloidTune);

%% Hip Trajectory Generation (ZMP Preview)
[Hipx_Preview, Hipy_Preview] = HipGenerator_Preview(CommonPara,ZMPTrajectory);
Hipx_Bias = 10;

Hipx_Preview = Hipx_Preview+Hipx_Bias;

% figure(3)
% plot(TotalTimeSequence, [Hipx_Cycloid; Hipx_Preview; Hipy_Cycloid; Hipy_Preview; Hipz-Height;Footrx;Footry;Footrz; Footlx;Footly;Footlz;Xzmp;Yzmp])
% hold on
% grid on
% title('Walking Pattern Generation (ZMP Preview algorithm)')
% xlabel('Time (second)');
% ylabel('Distance (m)');
% legend('Hipx-Clc','Hipx','Hipy-Cyc','Hipy','Hipz','Right foot x','Right foot y','Right foot z','Left foot x','Left foot y','Left foot z','Xzmp','Yzmp');
% hold off


%% IKvector Test
% Hipz = Height*ones(1,length(Hipx_Preview));
walkingspace=[Hipx_Preview(1) Hipx_Preview(end)+100 0-LD/2-20 0+LD/2+20 0 Height+(stairH*NumOfStep)+20];
[JointAngle,RawAngle,MachineAngle] = IKvector(Hipx_Cycloid', Hipy_Preview', Hipz',Footrx', Footry', Footrz', Footlx', Footly', Footlz',walkingspace,CommonPara);
%                 Hip LHY LHR LHP LKP LAP LAR RHY RHR RHP RKP RAP RAR
% RawAngle Sign =  0   0   -   -   +   +   +   0   -   -   +   +   +
% JointAngle
dtor = pi/180;

% Crouching Trajectory
Initial_Position = [0 0 0 0 0 0 0 0 0 0 0 0 0];
init_time = 0:delt:2;
gap = MachineAngle(1,:)./length(init_time);
Crouching = [0 0 0 0 0 0 0 0 0 0 0 0 0];
for k = 1:length(init_time)
     Crouching = [Crouching; Crouching(end,:)+gap];
end

MachineAngle = [Crouching;MachineAngle];
[row,col]=size(MachineAngle);

% junk for 21 DOF
junk = zeros(row,21-col);
MachineAngle = [MachineAngle junk];

IDF = fopen('trajectory.txt','wt');
for k = 1:row
    fprintf(IDF,'%f ',MachineAngle(k,:)*dtor);
    fprintf(IDF,'\n');
end
fclose(IDF);

