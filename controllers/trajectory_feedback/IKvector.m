function [JointAngle,RawAngle,MachineAngle]=IKvector(xh,yh,zh,xrf,yrf,zrf,xlf,ylf,zlf,walkingspace,CommonPara)

% radian to degree
rtod = 180/pi;
% degree to radian
dtor = pi/180;

DSP = CommonPara(3);
SSP = CommonPara(4);
SD = CommonPara(5);
LD = CommonPara(6);
NumOfStep = CommonPara(7);
delt = CommonPara(8);
init = CommonPara(9);
endd = CommonPara(10);
stairH = CommonPara(11);
%% Assumption
% foot always are parallel to the ground
% upper body always is perpendicular to the ground
% no rotation (yaw)


%% Global Coordinate
% positive X = forward
% positive Y = left direction
% positive Z = up ward

%% Kinematic Parameters
% It must be specified more
% hd = (hip lateral distance) hip link / 2
% hh = (hip height) height of the servo placed at the hip
% tl = (thigh link) distance from thigh yaw joint to thigh roll & pitch joints
% ul = upper limb
% ll = lower limb
% gl = (ground limb) distance from bottom to ankle joints
hd = 38;
hh = 37.6;
tl = 31;
% each joint is alined.
% in mini-hubo, knee joint is behind of thigh and ankle joint
% so ul = sqrt(19.5^2 + 87^2) = ll
ul = sqrt(19.5^2 + 87^2);
ll = ul;
gl = 42;
HtoHipYaw = hh+tl+ul+ll+gl;
HtoThigh = ul+ll+gl;
%% Joint Positions
% Hp = hip position respect to the global coordinate
% Rfp = right foot position respect to the global coord
% Lfp = left foot position respect to the global coord
% Hp, Rfp, Lfp are given
Hp = [xh yh zh];
Rfp = [xrf yrf zrf];
Lfp = [xlf ylf zlf];

% Rhyp = right hip yaw joint position
% Rhp = right hip roll&pitch joint position
% Rkp = right knee joint position (unknown)
% Rap = right ankle joint position
xrhyp = xh;
yrhyp = yh-hd;
zrhyp = zh-hh;
Rhyp = [xrhyp yrhyp zrhyp];

xrhp = xh;
yrhp = yrhyp;
zrhp = zrhyp-tl;
Rhp = [xrhp yrhp zrhp];

xrap = xrf;
yrap = yrf;
zrap = zrf+gl;
Rap = [xrap yrap zrap];

% Lhyp = left hip yaw joint position
% Lhp = left hip roll&pitch joint position
% Lkp = left knee joint position (unknown)
% Lap = left ankle joint position
xlhyp = xh;
ylhyp = yh+hd;
zlhyp = zh-hh;
Lhyp = [xlhyp ylhyp zlhyp];

xlhp = xh;
ylhp = ylhyp;
zlhp = zlhyp-tl;
Lhp = [xlhp ylhp zlhp];

xlap = xlf;
ylap = ylf;
zlap = zlf+gl;
Lap = [xlap ylap zlap];
%% Mini Hubo Real Joint Directions and Offsets
% 조인트 박혀있는 방향하고 이니셜 포지션과의 관계로 움직이는 방향결정
%                 Hip LHY LHR LHP LKP LAP LAR RHY RHR RHP RKP RAP RAR
% RawAngle Sign =  0   0   -   -   +   +   +   0   -   -   +   +   +
M_Direction = [ 1   1  -1  -1  -1  -1   1   1   1   1   1   1  -1];
M_Offset = [0  0   0   0 pi/4  0   0   0   0   0 -pi/4 0   0];
%        Hip LHY LHR LHP LKP LAP LAR RHY RHR RHP RKP RAP RAR
M_bias = [0   0   0   0   0   0   0   0   0   0   0   0   0];
% 요 밑에있는것은 plot을 위한것 메틀랩에서..
%       Hip LHY LHR LHP LKP LAP LAR RHY RHR RHP RKP RAP RAR
sign = [ 1   1   1   1   -1   1   1   1   1  -1   1  -1   1];
offset = [0  0   0   0   0  90  90   0   0   0   0  90  90];
%% Vector Inverse Kinematics
[row,col]=size(Hp);

JointAngle = [];
RawAngle = [];
MachineAngle = [];
Rkp = [];
Lkp = [];
tic
for k = 1:row
    
%     Hp(k,:)
%     Rhp(k,:)
%     Rap(k,:)
%     Lhp(k,:)
%     Lap(k,:)
    [RHP,RAP,RKP] = HAKPitch(Rhp(k,:),Rap(k,:),ul,ll,rtod);
%     disp('Right done')
    [LHP,LAP,LKP] = HAKPitch(Lhp(k,:),Lap(k,:),ul,ll,rtod);
%     disp('left done')
    [RHR,RAR] = HipAnkleRoll(Rhp(k,:),Rap(k,:),rtod);
    [LHR,LAR] = HipAnkleRoll(Lhp(k,:),Lap(k,:),rtod);
    Hip = 0;
    LHY = 0;
    RHY = 0;
    raw = [Hip LHY LHR LHP LKP LAP LAR RHY RHR RHP RKP RAP RAR];
    % RawAngle = 실질적으로 서보가 움직여야될 각도 (절대각도)
    % JointAngle = offset과 방향 고려한 각도 (simulation을 위한것)
    % MachineAngle = real joint angle each servo moves amount
    RawAngle = [RawAngle; raw];
    JointAngle = [JointAngle; raw.*sign+offset];
    MachineAngle = [MachineAngle; raw.*M_Direction+M_Offset+M_bias];
    [Rkpx,Rkpy,Rkpz,Lkpx,Lkpy,Lkpz]=KneePosition(ll,JointAngle(k,:),dtor);
    Rkp = [Rkp;[Rap(k,1)+Rkpx Rap(k,2)+Rkpy Rap(k,3)+Rkpz]];
    Lkp = [Lkp;[Lap(k,1)+Lkpx Lap(k,2)+Lkpy Lap(k,3)+Lkpz]];
    assert(sqrt((Rhp(k,1)-Rkp(k,1))^2+(Rhp(k,2)-Rkp(k,2))^2+(Rhp(k,3)-Rkp(k,3))^2) < ll+0.1,'Right Knee Wrong');
    assert(sqrt((Lhp(k,1)-Lkp(k,1))^2+(Lhp(k,2)-Lkp(k,2))^2+(Lhp(k,3)-Lkp(k,3))^2) < ll+0.1,'Left Knee Wrong');
end
computation = toc
% %% Plot Animation
% h=figure(10);
% axis(walkingspace);
% view([0 0])
% xlabel('x')
% ylabel('y')
% zlabel('height')
% grid on
% Xani = [Rfp(1,1);Rap(1,1);Rkp(1,1);Rhp(1,1);Rhyp(1,1);Hp(1,1);Lhyp(1,1);Lhp(1,1);Lkp(1,1);Lap(1,1);Lfp(1,1)];
% Yani = [Rfp(1,2);Rap(1,2);Rkp(1,2);Rhp(1,2);Rhyp(1,2);Hp(1,2);Lhyp(1,2);Lhp(1,2);Lkp(1,2);Lap(1,2);Lfp(1,2)];
% Zani = [Rfp(1,3);Rap(1,3);Rkp(1,3);Rhp(1,3);Rhyp(1,3);Hp(1,3);Lhyp(1,3);Lhp(1,3);Lkp(1,3);Lap(1,3);Lfp(1,3)];
% h1 = line(Xani,Yani,Zani);
% 
% StariXani = [0;SD/2];
% StariYani = [0;0];
% StariZani = [0; 0];
% for i = 1:NumOfStep
%     StariXani = [StariXani; StariXani(end);i*SD+SD/2];
%     StariYani = [StariYani; 0;0];
%     StariZani = [StariZani; i*stairH; i*stairH];
% end
% h2 = line(StariXani,StariYani,StariZani,'color','k','linewidth', 4);
%     
% for k = 1:row
%     Xani = [Rfp(k,1);Rap(k,1);Rkp(k,1);Rhp(k,1);Rhyp(k,1);Hp(k,1);Lhyp(k,1);Lhp(k,1);Lkp(k,1);Lap(k,1);Lfp(k,1)];
%     Yani = [Rfp(k,2);Rap(k,2);Rkp(k,2);Rhp(k,2);Rhyp(k,2);Hp(k,2);Lhyp(k,2);Lhp(k,2);Lkp(k,2);Lap(k,2);Lfp(k,2)];
%     Zani = [Rfp(k,3);Rap(k,3);Rkp(k,3);Rhp(k,3);Rhyp(k,3);Hp(k,3);Lhyp(k,3);Lhp(k,3);Lkp(k,3);Lap(k,3);Lfp(k,3)];
% %     set(h1,'XData',Xani);
% %     set(h1,'YData',Yani);
% %     set(h1,'ZData',Zani);
%     h1 = line(Xani,Yani,Zani);
%     drawnow;
%     pause(.001)
% end


end

%% Knee Position Calculation for Simulation in MATLAB
function [Rx,Ry,Rz,Lx,Ly,Lz]=KneePosition(ll,JointAngle,dtor)

% Ry = -ll*cos(JointAngle(1,13)*dtor);
Rx = ll*cos(JointAngle(1,12)*dtor);
Ry = -ll*sin(JointAngle(1,12)*dtor)*cos(JointAngle(1,13)*dtor);
Rz = sqrt(ll^2-(Rx^2+Ry^2));

% Ly = -ll*cos(JointAngle(1,7)*dtor);
Lx = -ll*cos(JointAngle(1,6)*dtor);
Ly = -ll*sin(JointAngle(1,6)*dtor)*cos(JointAngle(1,7)*dtor);
Lz = sqrt(ll^2-(Lx^2+Ly^2));
% [Rx Ry Rz;
%     Lx Ly Lz];
% sqrt(Rx^2+Ry^2+Rz^2);
% sqrt(Lx^2+Ly^2+Lz^2);

end

%% All Roll Motion Calculation
function [HR, AR] = HipAnkleRoll(hp,ap,rtod)
temp = (hp-ap).^2;
vec = sqrt(temp(1,2)+temp(1,3));
AR = 90-acos( (hp(1,2)-ap(1,2))/vec )*rtod;
HR = -AR;
end

%% All Pitch Motion Calculation
function [HP,AP,KP] = HAKPitch(hp,ap,uplimb,lowlimb,rtod)
temp = (hp-ap).^2;
vec = sqrt(temp(1,1)+temp(1,2)+temp(1,3));
assert(vec < uplimb+lowlimb,'unreachable')    
theta = acos( (hp(1,1)-ap(1,1))/vec)*rtod;
HP = -(theta+ acos((uplimb^2+vec^2-lowlimb^2)/(2*uplimb*vec))*rtod-90);
AP = 90- theta + acos((lowlimb^2+vec^2-uplimb^2)/(2*lowlimb*vec))*rtod;
KP = 180-acos((uplimb^2+lowlimb^2-vec^2)/(2*uplimb*lowlimb))*rtod;
end

%% The Second Law of Cosine
function [deg] = cos2(a,b,c,rtod)
% a, b = two lines between the angle to be calculated
% c = the inclined line
deg = acos((a^2+b^2-c^2)/(2*a*b))*rtod;
end
