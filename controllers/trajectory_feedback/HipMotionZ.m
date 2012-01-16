function [hipz] = HipMotionZ(CommonPara, HipzPara)
Height = CommonPara(1);
DSP = CommonPara(3);
SSP = CommonPara(4);
NumOfStep = CommonPara(7);
delt = CommonPara(8);
init = CommonPara(9);
endd = CommonPara(10);
stairH = CommonPara(11);

% assert
begin = HipzPara(1);
during = HipzPara(2);
ending = (SSP - (begin+during));
assert((begin+during) <= SSP, 'timing problem in the motion in Z direction')


hipz = [];

for i = 0:delt:(init+DSP+SSP)
    hipz = [hipz Height];

end

offsetz = Height;
for k = 1:NumOfStep
    for i = delt:delt:DSP
        hipz = [hipz hipz(end)];

    end

    for i = delt:delt:begin
        hipz = [hipz hipz(end)];

    end  

    for i = delt:delt:during
        hipz = [hipz offsetz+stairH*(i/(during))];

    end

    for i = delt:delt:ending
        hipz = [hipz hipz(end)];

    end

    offsetz = hipz(end);
end
for i = delt:delt:DSP+endd
    hipz = [hipz hipz(end)];

end



end
