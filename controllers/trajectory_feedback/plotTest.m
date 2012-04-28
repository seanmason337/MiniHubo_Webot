% w = 4;
% h = 2;
% tx = 2;
% ty = 1; 
% theta = 40;
% coor = [-w/2 -h/2 0 ;
%         w/2 -h/2 0 ;
%         w/2 h/2 0 ;
%         -w/2 h/2 0 ;
%         -w/2 -h/2 0 ]'
% plot(coor(1,:)',coor(2,:)');
% hold on
% coor = [coor; 1 1 1 1 1]
% h = hZ(30,2,2,0)
% coor = hZ(theta,tx,ty,0)*coor;
% plot(coor(1,:)',coor(2,:)');
% hold off
n = 4;
for i = 1:1000
    hold on
xx = delt:delt:SSP;
x = [delt,SSP*rand(1,n),SSP];
x = sort(x);
y = [0,randi(FootUpwardHeight,1,n),0];
yy = interp1(x,y,xx,'v5cubic');

while(sum(sign(yy) == -1) || sum(yy>FootUpwardHeight) || yy(1)~=0 ||yy(end)~=0)
    x = [delt,SSP*rand(1,n),SSP];
    x = sort(x);
    y = [0,randi(FootUpwardHeight,1,n),0];
    yy = interp1(x,y,xx,'v5cubic');
   
end
 plot(xx,yy);
    hold off
end