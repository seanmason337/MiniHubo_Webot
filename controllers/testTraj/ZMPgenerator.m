function [Xzmp, Yzmp,TotalTimeSequence] = ZMPGenerator(CommonPara)
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

% Kine,Htune,FootVal are not using in this program

% For varialble ZMP trajectory,
% Need to change DSP and SSP?
% SD and LD are variable
% NumOfStep shoul be removed
% �������� ó�� �̴ϼ������Ƕ�, ������ ���� �����Ǵ� ��������,
% �߰��� ������ ���� ���� �ٲ𲨴�.
% NumOfStep�� 1�� �ɲ���, �� �ѽ��ܸ��� �ٸ��� ������ ����
% SD�� ���ܸ��� �ٲ𲨰�, LD�� ��������..
% �׷��ٸ� ���� ���ڱ��� �˸�, LD�ϰ� SD�ϰ� �����ǰ�,
% �׷� ������ ���� �ȴ�. ����?

TotalTimeSequence = 0:delt:(init+(NumOfStep+2)*DSP + (NumOfStep+1)*SSP + endd);
xdouble = SD/(DSP/delt); % increment in double support phase in X
ydouble = LD/(DSP/delt); % increment in double support phase in Y

Xzmp = [];
Yzmp = [];
%% Xzmp Generation
for i = 0:delt:(init+DSP+SSP)
    Xzmp = [Xzmp 0];
end

for k = 1:NumOfStep
    for i = delt:delt:DSP
        Xzmp = [Xzmp Xzmp(end)+xdouble];
    end
    for i = delt:delt:SSP
        Xzmp = [Xzmp Xzmp(end)];
    end
end

for i = delt:delt:DSP+endd
    Xzmp = [Xzmp Xzmp(end)];
end

%% Yzmp Generation
for i = 0:delt:init
    Yzmp = [Yzmp 0];
end
for i = delt:delt:DSP
    Yzmp = [Yzmp Yzmp(end)+(ydouble/2)];
end
for i = delt:delt:SSP
    Yzmp = [Yzmp Yzmp(end)];
end

for k = 1:NumOfStep
    sign = (-1)^k;
    for i = delt:delt:DSP
        Yzmp = [Yzmp Yzmp(end)+(sign)*ydouble];
    end
    for i = delt:delt:SSP
        Yzmp = [Yzmp Yzmp(end)];
    end
end

for i = delt:delt:DSP
    Yzmp = [Yzmp Yzmp(end)+(-sign)*(ydouble/2)];
end
for i = delt:delt:endd
    Yzmp = [Yzmp Yzmp(end)];
end

    
    
  
    






