function [Footrx, Footry, Footrz, Footlx, Footly, Footlz]=FootGenerator(CommonPara,FootPara)

% FootForwardDistance = foot forward distance for one step
% FootLateralDistance = foot lateral distance for one step
% FootUpwardHeight = maximum height of foot when swinging
% FootLateralInit = orignal position of each foot from the center
% coordinate
% FootForwardDistance = SD;
% FootLateralDistance = 0;
% FootUpwardHeight = 30;
% FootLateralInit = 38;
% FootPara = [FootForwardDistance FootLateralDistance FootUpwardHeight FootLateralInit];

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
% FFD = SD;
FFD = FootPara(1);
% FLD = 0;
FLD = FootPara(2);
% FootH = 10;
FootH = FootPara(3);
% FootVal = [FFD FLD FootH];
FootYOffset = FootPara(4);
% FootHmax = Foot maximum height scale factor for stair climbing
% FootHmax(1) = maximum height of foot for the first step (scale factor),
% normally = 0.9
% FootHmax(2) = maximum height of foot after the first step (scale factor)
% normally = 2
FootHmax = [FootPara(5) FootPara(6)];




DSP = CommonPara(3);
SSP = CommonPara(4);
SD = CommonPara(5);
LD = CommonPara(6);
NumOfStep = CommonPara(7);
delt = CommonPara(8);
init = CommonPara(9);
endd = CommonPara(10);

stairH = CommonPara(11); % mm




% additional Y direction can be added with SDL
freq = 1/(DSP/2+SSP);
omega = 2*pi*freq;

%% Right Foot
Footrx = [];
Footrz = [];

for i = 0:delt:DSP+init
    Footrx = [Footrx 0];
    Footrz = [Footrz 0];
    
end
offsetx = Footrx(end);

if NumOfStep == 1
    for i = delt:delt:(SSP+DSP/2)
            Footrx = [Footrx FFD/(2*pi)*(omega*i - sin(omega*i))];
            if i <= (SSP+DSP/2)/2
                Footrz = [Footrz FootH/(2*pi)*(2*omega*i-sin(2*omega*i))+FootHmax(1)*stairH*sin(omega/4*i) ];
                FootPeak = Footrz(end)-stairH;
            else
                Footrz = [Footrz 2*FootPeak+FootPeak/(2*pi)*(sin(2*omega*i)-2*omega*i)+stairH];
            end            
%             Footrz = [Footrz FootH*sin(omega/2*i)+stairH*i];
%             Footry = [Footry ref+FLD/(2*pi)*(omega*i - sin(omega*i))];
    end
    for i = delt:delt:(DSP/2+SSP+DSP)
            Footrx = [Footrx Footrx(end)];
            Footrz = [Footrz Footrz(end)];
%             Footry= [Footry Footry(end)];
    end
else
    for k =1:2:NumOfStep
        if (k == 1)
            for i = delt:delt:(SSP+DSP/2)
                Footrx = [Footrx offsetx+FFD/(2*pi)*(omega*i - sin(omega*i))];
%                 Footrz = [Footrz FootH*sin(omega/2*i)+stairH*sin(omega/4*i)]; 
                if i <= (SSP+DSP/2)/2
                    Footrz = [Footrz FootH/(2*pi)*(2*omega*i-sin(2*omega*i))+FootHmax(1)*stairH*sin(omega/4*i) ];
                    FootPeak = Footrz(end)-stairH;
                else
                    Footrz = [Footrz 2*FootPeak+FootPeak/(2*pi)*(sin(2*omega*i)-2*omega*i)+stairH];
                end
%                 Footrz = [Footrz FootH*sin(omega/2*i)+stairH*i];
%                 Footry = [Footry ref+offsety+FLD/(2*pi)*(omega*i - sin(omega*i))];
            end
            for i = delt:delt:(DSP/2+SSP+DSP)
                Footrx = [Footrx Footrx(end)];
                Footrz = [Footrz Footrz(end)];
%                 Footry = [Footry Footry(end)];
            end
        else
            for i = delt:delt:(SSP+DSP/2)
                Footrx = [Footrx offsetx+2*FFD/(2*pi)*(omega*i - sin(omega*i))];
%                 Footrz = [Footrz offsetz+FootH*sin(omega/2*i)+2*stairH*sin(omega/4*i)];               
                if i <= (SSP+DSP/2)/2
                    Footrz = [Footrz offsetz+FootH/(2*pi)*(2*omega*i-sin(2*omega*i))+FootHmax(2)*stairH*sin(omega/4*i) ];
                    FootPeak = Footrz(end)-k*stairH;
                else
                    Footrz = [Footrz 2*FootPeak+FootPeak/(2*pi)*(sin(2*omega*i)-2*omega*i)+k*stairH];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                end
%                 Footrz = [Footrz offsetz+FootH*sin(omega/2*i)+2*stairH*i];
%                 Footry = [Footry ref+offsety+2*FLD/(2*pi)*(omega*i - sin(omega*i))];
            end
            for i = delt:delt:(DSP/2+SSP+DSP)
                Footrx = [Footrx Footrx(end)];
                Footrz = [Footrz Footrz(end)];
%                 Footry = [Footry Footry(end)];
            end
        end
        offsetx = Footrx(end);
        offsetz = Footrz(end);
%         offsety = Footry(end);
    end
end
offsetx = Footrx(end);
offsetz = Footrz(end);
if mod(NumOfStep,2) == 0
    for i= delt:delt:(DSP/2+SSP)
        Footrx = [Footrx offsetx+FFD/(2*pi)*(omega*i - sin(omega*i))];
%         Footrz = [Footrz offsetz+FootH*sin(omega/2*i)+stairH*sin(omega/4*i)];
        if i <= (SSP+DSP/2)/2
            Footrz = [Footrz offsetz+FootH/(2*pi)*(2*omega*i-sin(2*omega*i))+FootHmax(1)*stairH*sin(omega/4*i) ];
            FootPeak = Footrz(end)-NumOfStep*stairH;
        else
            Footrz = [Footrz 2*FootPeak+FootPeak/(2*pi)*(sin(2*omega*i)-2*omega*i)+NumOfStep*stairH];
        end

%         Footrz = [Footrz offsetz+FootH*sin(omega/2*i)+stairH*i];
%         Footry = [Footry offsety+FLD/(2*pi)*(omega*i - sin(omega*i))];
    end 
    for i = delt:delt:DSP/2+endd
        Footrx = [Footrx Footrx(end)];
        Footrz = [Footrz Footrz(end)];
%         Footry = [Footry Footry(end)];
    end
else
    for i= delt:delt:endd
        Footrx = [Footrx Footrx(end)];
        Footrz = [Footrz Footrz(end)];
%         Footry = [Footry Footry(end)];
    end
end
% Footry = -(LD*10)/2*ones(1,length(Footrx));
Footry = -FootYOffset*ones(1,length(Footrx));

%% Left Foot
Footlx = [];
Footlz = [];
% Footly = [];
for i = 0:delt:DSP+SSP+DSP+init
    Footlx = [Footlx 0];
    Footlz = [Footlz 0];
end
offsetx = Footlx(end);
offsetz = Footlz(end);
if NumOfStep ~= 1   
    for k =1:2:NumOfStep-1
        if (k ~= NumOfStep)
            for i = delt:delt:(SSP+DSP/2)
                Footlx = [Footlx offsetx+2*FFD/(2*pi)*(omega*i - sin(omega*i))];
%                 Footlz = [Footlz offsetz+FootH*sin(omega/2*i)+2*stairH*sin(omega/4*i)];
                
                if i <= (SSP+DSP/2)/2
                    Footlz = [Footlz offsetz+FootH/(2*pi)*(2*omega*i-sin(2*omega*i))+FootHmax(2)*stairH*sin(omega/4*i) ];
                    FootPeak = Footlz(end)-(k+1)*stairH;
                else
                    Footlz = [Footlz 2*FootPeak+FootPeak/(2*pi)*(sin(2*omega*i)-2*omega*i)+(k+1)*stairH];
                end
%                 Footlz = [Footlz offsetz+FootH*sin(omega/2*i)+2*stairH*i];
%                 Footly = [Footly offsety+2*FLD/(2*pi)*(omega*i - sin(omega*i))];
            end
            for i = delt:delt:(DSP/2+SSP+DSP)
                Footlx = [Footlx Footlx(end)];
                Footlz = [Footlz Footlz(end)];
%                 Footly = [Footly Footly(end)];
            end
        else
            for i = delt:delt:(SSP+DSP/2)
                Footlx = [Footlx offsetx+FFD/(2*pi)*(omega*i - sin(omega*i))];
%                 Footlz = [Footlz offsetz+FootH*sin(omega/2*i)+2*stairH*sin(omega/4*i)];
                
                if i <= (SSP+DSP/2)/2
                    Footlz = [Footlz offsetz+FootH/(2*pi)*(2*omega*i-sin(2*omega*i))+FootHmax(1)*stairH*sin(omega/4*i) ];
                    FootPeak = Footlz(end)-stairH;
                else
                    Footlz = [Footlz 2*FootPeak+FootPeak/(2*pi)*(sin(2*omega*i)-2*omega*i)+stairH];
                end
%                 Footlz = [Footlz offsetz+FootH*sin(omega/2*i)+2*stairH*i];
%                 Footly = [Footly offsety+FLD/(2*pi)*(omega*i - sin(omega*i))];
            end
            for i = delt:delt:(DSP/2+SSP+DSP)
                Footlx = [Footlx Footlx(end)];
                Footlz = [Footlz Footlz(end)];
            end            
        end
        offsetx = Footlx(end);
        offsetz = Footlz(end);
    end
end
if mod(NumOfStep,2) == 0
    for i= delt:delt:(endd)
        Footlx = [Footlx Footlx(end)];
        Footlz = [Footlz Footlz(end)];
    end
else
    for i= delt:delt:(DSP/2+SSP)
        Footlx = [Footlx offsetx+FFD/(2*pi)*(omega*i - sin(omega*i))];
%         Footlz = [Footlz offsetz+FootH*sin(omega/2*i)+stairH*sin(omega/4*i)];
        if i <= (DSP/2+SSP)/2
            Footlz = [Footlz offsetz+FootH/(2*pi)*(2*omega*i-sin(2*omega*i))+FootHmax(1)*stairH*sin(omega/4*i) ];
            FootPeak = Footlz(end)-(NumOfStep)*stairH;
        else
            Footlz = [Footlz 2*FootPeak+FootPeak/(2*pi)*(sin(2*omega*i)-2*omega*i)+(NumOfStep)*stairH];
        end
%         Footlz = [Footlz offsetz+FootH*sin(omega/2*i)+stairH*i];
    end 
    for i = delt:delt:(DSP/2+endd)
        Footlx = [Footlx Footlx(end)];
        Footlz = [Footlz Footlz(end)];
    end
end
% Footly = (LD*10)/2*ones(1,length(Footlx));
Footly = FootYOffset*ones(1,length(Footlx));
