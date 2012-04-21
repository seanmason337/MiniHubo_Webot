function [CoMf,CoMl]=HipGenerator_Preview(CommonPara,ZMPTrajectory)

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


% State-Space representation
A = [1 delt delt^2/2;
    0 1 delt;
    0 0 1];
B = [delt^3/6;
    delt^2/2;
    delt];
C = [1 0 -zc/g];


% function [Xzmp, Yzmp,TotalTimeSequence] = ZMPGenerator(DSP,SSP,SD,LD,NumOfStep,delt,init,endd)
% For preview control, int value should be larger than 4
% sampling time = 0.005 but hip motion depends on the sampling time.
ydf = ZMPTrajectory(1,:);
ydl = ZMPTrajectory(2,:);
x = ZMPTrajectory(3,:);
% [ydf,ydl,x]=ZMPGenerator(CommonPara);

% ydf = forward ZMP
% ydl = lateral ZMP
% x = TotalTimeSequence

% optimal gain refered to preview H2 theory 
A1 = [1 C*A;zeros(3,1) A];
B1 = [C*B;B];
C1 = [1 0 0 0];

Qe = [1];
Qx = zeros(3,3);
%Qx = ones(3,3);

Q = [Qe zeros(1,3);zeros(3,1) Qx];
R = 1e-6;

% P = solution of DARE, G = optimal gain
[P,L,G] = dare(A1,B1,Q,R);
%[K,X,E] = lqr(A1,B1,Q,R)
%[X1,L1,G1] = dare(A,B,Qx,R)



% Optimal gain from DARE
Ke = G(1,1);
Kx = G(1,2:end);

%% Preview Controller Design
% Preview size
% N = [x1 x2 x3];
N = 0:delt:1;


% Calculation of Preview gain
% recursively computed
Ac = A1-B1*G;
Gp = zeros(1,length(N)); % Preview gain matrix
X1 = zeros(4,length(N)); % Preview state

Gp(1) = -Ke;
X1(:,1) = -Ac'*P*[1;0;0;0];

for i = 2:length(N)
    Gp(i) = inv(R+ B1'*P*B1)*B1'*X1(:,i-1);
    X1(:,i) = Ac'*X1(:,i-1);
end

xkf = zeros(3,length(x)); % state
xkl = zeros(3,length(x));
ukf = zeros(1,length(x)); % Control input
ukl = zeros(1,length(x));

ykf = zeros(1,length(x)); % forward direction
ykl = zeros(1,length(x)); % lateral direction
CoMf = zeros(1,length(x));
CoMl = zeros(1,length(x));
ef = 0; % summation of error
el = 0;
for k = 1:length(x)
    
    if (k+length(N))<length(x) 
        
        CoMf(k) = xkf(1,k);
        CoMl(k) = xkl(1,k);
        
        ykf(k) = C*xkf(:,k); % C = output matrix, xk = 
        ykl(k) = C*xkl(:,k);
        
        ef = ef + ykf(k) - ydf(k);
        el = el + ykl(k) - ydl(k);
        prevf = 0;
        prevl = 0;
        
        for i = 1:length(N)
            if (k+length(N))<length(x) 
                prevf = prevf + 1*Gp(i)*ydf(k+i);
                prevl = prevl + 1*Gp(i)*ydl(k+i);
            end        
        end
        
    else
        ykf(k) = ykf(k-1);  % there is no enough preview point
        ykl(k) = ykl(k-1);
        CoMf(k) = xkf(1,k-1);
        CoMl(k) = xkl(1,k-1);
    end
    
    ukf(k) = -Ke*ef - Kx*xkf(:,k) - prevf;
    ukl(k) = -Ke*el - Kx*xkl(:,k) - prevl;
    xkf(:,k+1) = A*xkf(:,k)+B*ukf(k);
    xkl(:,k+1) = A*xkl(:,k)+B*ukl(k);
end

% figure(1)
% plot(N,Gp,'linewidth',3);
% Title('Preview Gain, Gp, vs. Time');
% xlabel('Time (second)');
% ylabel('Magnitude of Gp');
% legend('Gp');
% grid on

% 
% figure(2)
% plot(x,ydf,'b','linewidth',3);grid on
% hold on
% plot(x,ykf,'r-.','linewidth',4)
% hold on
% plot(x,CoMf,'g--','linewidth',3);
% Title('Desired ZMP, ZMP via Preview, and CoM via Preview');
% xlabel('Time (second)');
% ylabel('Distance in Forward Direction (m)');
% legend('ZMP ref.','ZMP Preview','CoM Preview');
% hold off

% figure(3)
% plot(x,ydl,'b','linewidth',3);grid on
% hold on
% plot(x,ykl,'r-.','linewidth',4)
% hold on
% plot(x,CoMl,'g--','linewidth',3);
% Title('Desired ZMP, ZMP via Preview, and CoM via Preview');
% xlabel('Time (second)');
% ylabel('Distance in Lateral Direction (m)');
% legend('ZMP ref.','ZMP Preview','CoM Preview');
% hold off

% figure(5)
% plot(x,ydl,'linewidth',3);
% Title('ZMP in Lateral');
% xlabel('Time (second)');
% ylabel('Step Distance');
% ylim([-8 8])
% legend('ZMP in Y');
% grid on
% hold off

% xl = [];
% yl = [];
% zl = [];
% 
% xr = [];
% yr = [];
% zr = [];
% 
% % right foot x direction
% for t=0:.005:(1.1-0.005)
%     
%     xr = [xr, 0];
%     yr = [yr, 0];
%     zr = [zr, 0];
% end
% for t=0:.005:2
%     [a, b, c] = foot(t, 2, 0.9, 7.5, 30, 10, 0, 0, 0 );
%     xr = [xr, a];
%     yr = [yr, b];
%     zr = [zr, c];
% end
% for t=0:.005:(6-0.005)
%     [a, b, c] = foot(t, 2, 0.9, 15, 30, 10, 0, 7.5, 0 );
%     xr = [xr, a];
%     yr = [yr, b];
%     zr = [zr, c];
% end
% for t=0:.005:(2-0.005)
%     
%     xr = [xr, xr(end)];
%     yr = [yr, yr(end)];
%     zr = [zr, zr(end)];
% end
% 
% % left foot x direction
% for t=0:.005:(2.1-0.005)
%     
%     xl = [xl, 0];
%     yl = [yl, 0];
%     zl = [zl, 0];
% end
% for t=0:.005:6
%     [a, b, c] = foot(t, 2, 0.9, 15, 30, 10, 0, 0, 0 );
%     xl = [xl, a];
%     yl = [yl, b];
%     zl = [zl, c];
% end
% for t=0:.005:(1-0.005)
%     [a, b, c] = foot(t, 2, 0.9, 7.5, 30, 10, 0, 45, 0 );
%     xl = [xl, a];
%     yl = [yl, b];
%     zl = [zl, c];
% end
% for t=0:.005:(2-0.005)
%     
%     xl = [xl, xl(end)];
%     yl = [yl, yl(end)];
%     zl = [zl, zl(end)];
% end
% 
% figure(4)
% plot(x,ydf,'linewidth',1);
% hold on
% plot(x, [xr;zr;xl;zl;CoMf;CoMl]);
% Title('ZMP in Forward');
% xlabel('Time (second)');
% ylabel('Step Distance');
% ylim([-8 60])
% legend('ZMP in X','R foot in x','R foot in z','L foot in x','L foot in z','CoM in x','CoM in y');
% grid on
% hold off



