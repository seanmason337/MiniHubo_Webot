function forceData = testTraj()
    close all
    clear all
    clc
    % 
   desktop;
   keyboard;

TIME_STEP=100;

%% Initialize Sensors
gps = wb_robot_get_device('zero');
wb_gps_enable(gps, TIME_STEP);

gyro = wb_robot_get_device('imugyro');
wb_gyro_enable(gyro, TIME_STEP);

lTouch = wb_robot_get_device('LFoot');
wb_touch_sensor_enable(lTouch, TIME_STEP)

rTouch = wb_robot_get_device('RFoot');
wb_touch_sensor_enable(rTouch, TIME_STEP)

lFootC= wb_robot_get_device('LFootC');
wb_compass_enable(lFootC,TIME_STEP)

rFootC= wb_robot_get_device('RFootC');
wb_compass_enable(rFootC,TIME_STEP)
%wb_compass_enable('RFoot',TIME_STEP)

robot_node = wb_supervisor_node_get_from_def('MiniHubo');
trans_field = wb_supervisor_node_get_field(robot_node, 'translation');
rot_field = wb_supervisor_node_get_field(robot_node, 'rotation');

    wb_supervisor_field_set_sf_vec3f(trans_field, [0 0.3565 0])
    wb_supervisor_field_set_sf_rotation(rot_field, [1 0 0 0]) %[.240158 .355747 .903199 .000136024]
    
crouch_time = length(0:TIME_STEP/1000:2)+2;




Height = 220;
Gravity = 9810;
% DSP = 0.2;
% SSP = 1.3;
DSP = 0.5;
SSP = 1.8;
SD = 95;
%LD = 78.5; % Lateral ZMP = LD/2
LD=65;
%LD=89
NumOfStep = 20;
delt = 0.1;
init = 1;
endd = 2;
stairH = 0;
%% Q learn Parameters
gamma = .3;
alpha = .7;
w1 = 1;
w2 = 0;

CommonPara = [Height Gravity DSP SSP SD LD NumOfStep delt init endd stairH];
jointNames  = {'HY'; 'LHY'; 'LHR'; 'LHP'; 'LKP'; 'LAP'; 'LAR'; 'RHY';...
        'RHR'; 'RHP'; 'RKP'; 'RAP'; 'RAR'; 'LSP'; 'LSR'; 'LSY'; 'LEP'; 'RSP'; 'RSR'; 'RSY'; 'REP'};

minZ = 240;
maxZ = 270;
deltZ = .9;

TotalTimeSequence = 0:delt:(init+(NumOfStep+2)*DSP + (NumOfStep+1)*SSP + endd);
[rows,cols] = size(TotalTimeSequence)

N = 1;
neighbors =1;

Q = zeros(((maxZ-minZ)/deltZ-1)*(neighbors*2+1)+2*(((neighbors*2+1)-1)/2+1), cols);
paths = 2;

%path = [1	1	2	1	1	1	1	1	1	1	2	1	2	3	3	2	3	3	3	3	4	4	5	5	4	4	5	5	6	7	6	7	6	6	7	8	9	9	9	10	11	10	9	8	7	8	8	8	9	8	7	6	7	6	6	5	5	6	5	5	4	5	6	5	6	6	6	5	6	7	7	8	7	6	7	8	8	8	7	6	7	6	5	5	6	7	7	6	7	8	8	9	8	7	7	7	7	7	7	7	8	7	8	7	8	7	6	5	4	3	4	3	2	1	2	3	3	2	1	2	3	2	3	4	5	4	5	4	5	5	5	6	7	6	5	4 0 0];
%9800
path = [15,15,14,13,13,12,11,10,10,9,8,7,6,5,4,3,3,3,2,1,1,1,2,1,1,2,1,1,2,2,1,1,1,1,1,2,2,1,1,1,1,1,1,1,1,1,1,1,2,3,3,2,3,3,3,4,3,3,3,4,5,5,4,4,5,4,4,3,4,3,2,3,3,2,1,1,2,2,1,1,2,1,1,1,1,1,2,1,1,1,2,3,3,3,2,2,1,1,1,1,2,1,1,1,1,1,2,2,1,1,2,2,1,1,1,1,1,1,1,1,1,1,1,2,1,2,3,4,3,2,3,2,1,1,1,1,1,1,1,1,1,2,3,2,2,1,1,1,1,1,1,1,1,2,1,1,2,2,1,1,2,2,3,2,1,2,2,3,3,2,1,2,1,2,1,1,1,2,3,4,3,4,4,4,5,4,5,5,6,6,6,6,6,6,5,5,5,5,5,6,5,4,3,3,3,2,1,1,2,1,1,1,1,1,2,2,1,1,1,1,1,1,1,1,1,2,1,1,2,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,4,3,2,2,3,2,2,1,1,2,3,3,3,3,2,1,1,1,2,2,2,1,1,1,1,1,1,1,1,1,1,2,3,2,3,4,3,2,1,1,1,1,2,1,2,2,2,1,2,2,3,2,1,1,1,1,1,1,2,2,1,1,1,1,2,2,1,1,2,2,3,3,3,2,1,1,2,3,3,3,3,3,3,3,2,2,1,1,1,2,3,3,2,1,1,1,2,1,2,1,1,1,1,2,1,1,2,3,4,4,3,3,2,1,1,1,1,1,2,2,1,2,3,3,4,4,3,3,2,1,2,2,1,1,1,1,1,1,1,1,1,2,3,3,2,3,4,5,6,5,4,3,3,2,2,1,2,2,1,1,2,3,3,2,2,3,3,2,2,2,3,2,1,1,2,3,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,2,2,3,2,3,3,2,2,3,2,1,1,1,1,1,1,1,1,1,1,1,2,2,3,4,5,4,5,4,4,5,5,5,4,5,5,5,5,6,7,8,8,9,8,7,6,5,4,5,5,4,5,4,5,6,6,5,5,6,6,7,6,7,8,9,10,10,11,11,11,12,13,13,13,14,15];
paths = [path;path(1).*ones(1,size(path,2))];
paths = paths+minZ;
paths(3,:) = [262  262	263	262	261	262	261	261	261	261	261	260	261	262	261	262	261	262	263	263	262	263	263	262	262	261	262	263	263	263	263	263	262	263	262	263	263	262	263	263	263	262	262	262	262	263	262	262	261	262	262	262	262	261	262	262	262	261	260	261	260	260	260	259	259	260	259	259	259	259	259	260	260	261	262	262	262	262	263	264	265	265	266	266	266	267	266	266	266	266	265	265	266	265	264	265	264	264	264	265	266	266	267	266	266	265	264	264	263	263	263	263	262	261	261	260	261	262	263	262	262	263	264	265	266	265	265	264	264	264	264	265	266	265	266	266	266	265	266	267	267	267	266	266	267	268	267	268	267	267	267	266	265	264	263	263	262	261	260	261	260	259	260	260	261	262	263	264	263	263	263	264	265	264	264	263	263	263	264	265	265	264	263	263	264	263	264	264	263	264	263	264	265	266	267	267	267	266	265	265	265	265	265	265	265	266	267	267	266	267	268	267	268	268	268	267	268	269	269	270	270	270	269	269	269	269	268	268	268	267	267	267	266	267	267	266	267	267	268	267	266	266	265	264	263	262	263	263	264	264	264	264	264	263	262	262	263	263	262	262	262	261	260	260	260	259	259	260	260	260	261	260	260	259	259	259	260	259	259	259	259	258	257	257	258	259	258	258	257	258	259	259	258	259	259	258	257	257	256	256	255	256	257	256	256	255	254	254	255	255	254	255	254	255	256	256	257	256	256	256	257	258	258	257	257	258	258	257	257	258	257	258	258	258	257	258	257	257	258	257	258	259	259	260	260	260	259	259	258	259	258	257	257	257	258	258	259	260	259	259	259	260	260	260	261	260	260	259	260	261	260	261	260	261	260	259	260	260	261	260	261	261	260	260	259	259	258	258	257	257	256	256	255	256	257	257	256	257	256	256	256	256	257	258	257	256	256	256	257	257	256	256	257	256	256	257	257	257	257	256	256	255	255	254	254	255	255	254	254	254	253	253	254	253	253	253	252	252	252	251	250	250	249	250	249	250	251	250	249	250	249	249	250	249	248	248	248	249	250	250	249	249	248	249	250	251	250	251	250	250	250	249	248	247	246	247	248	248	247	247	247	246	245	246	247	246	246	245	246	246	245	244	243	242	242	242	241	240	240	240	240	241	240	241	240	240	241	241	241	240	240	241	240	240	241	241	242	242	242];
while N <size(paths,1)+1
  % Insert Tuning Parameter Here
    CommonPara = [Height Gravity DSP SSP SD LD NumOfStep delt init endd stairH];
    Hipz = paths(N,:);
    [Hipx_Preview,Hipy_Preview] = main(Hipz,CommonPara);
    wb_robot_step(TIME_STEP);
    jointNames  = {'HY'; 'LHY'; 'LHR'; 'LHP'; 'LKP'; 'LAP'; 'LAR'; 'RHY';...
            'RHR'; 'RHP'; 'RKP'; 'RAP'; 'RAR'; 'LSP'; 'LSR'; 'LSY'; 'LEP'; 'RSP'; 'RSR'; 'RSY'; 'REP'};

    joints = zeros(1,21);

    % Initialize joints
    for i = 1:21
        joints(i) = wb_robot_get_device(jointNames{i});
        wb_servo_enable_motor_force_feedback(joints(i), TIME_STEP);
    end

    Lsp = 0.0;
    Rsp = 0.0;
    Hy = 0.0;
    Lhp = 0.0;
    Lkp = 0.0;
    Lap = 0.0;
    Rhp = 0.0; 
    Rkp = 0.0;
    Rap = 0.0;
    Lsy = 0.0;
    Rsy = 0.0;
    Lep = 0.0;
    Rep = 0.0;


    %% Execute Trajectory

    [forceData,gpsData,footPos,footOr,steplist] = commandServos('trajectory.txt',joints,TIME_STEP);
    
    % Eliminate data for crouching period
	forceDataSum(N) = sum(sum(forceData(:,crouch_time:end)));
    
    resetRobot(0,0,jointNames,TIME_STEP);
    N = N+1;
end
   desktop;
   keyboard;

function Q = qlearn(indexList,actions,forceDataSum,zmpData,Q)
    zmpMax = 60;
    zmpData = min(zmpData,zmpMax);
    gamma = .5;
    alpha = .7;
    w1 = 1;
    w2 = 0;
    for i = 1:length(indexList)
        penalty1 = forceDataSum(i);
        penalty2 = zmpData(i);
        if  i ==length(indexList)
            S = (indexList(i)-1)*3-1;
            Q(S+(actions(i)),i) = Q(S+(actions(i)),i) + w2*penalty2+w1*penalty1;
        elseif indexList(i) == 1
            Q(-1+(actions(i)),i) = Q(-1+(actions(i)),i) + alpha*(w2*penalty2+w1*penalty1+gamma*max(Q(1:2,i+1))-Q(-1+(actions(i)),i));
        else
            S = (indexList(i)-1)*3-1;
            Q(S+(actions(i)),i) = Q(S+(actions(i)),i) + alpha*(w2*penalty2+w1*penalty1+gamma*max(Q(S+1:S+3,i+1))-Q(S+(actions(i)),i));
        end
    end
end


function [forceData,gpsData,footPos,footOr,steplist] = commandServos(file,joints,TIME_STEP)
    
    gps = wb_robot_get_device('zero');
    lTouch = wb_robot_get_device('LFoot');
    rTouch = wb_robot_get_device('RFoot');
    
    
    lFoot = wb_supervisor_node_get_from_def('LFoot');
    rFoot = wb_supervisor_node_get_from_def('RFoot');
    lForce = zeros(1,1);
    rForce = zeros(1,1);
    gpsData = zeros(3,1);
    footPos =zeros(6,1);
    footOr = zeros(2,1);
    signs = [-1 -1 -1 1 -1 -1 1 -1 -1 -1 1 1 -1 ];
    forceData = zeros(1,13);
    fid = fopen(file,'r');
    jointNames  = {'HY'; 'LHY'; 'LHR'; 'LHP'; 'LKP'; 'LAP'; 'LAR'; 'RHY';...
        'RHR'; 'RHP'; 'RKP'; 'RAP'; 'RAR'; 'LSP'; 'LSR'; 'LSY'; 'LEP'; 'RSP'; 'RSR'; 'RSY'; 'REP'};

    t = 0;
    step = 1;
    steplist = 1;
    state = 1;  %State of step
    while 1
        traj = textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f',1);
        hip = traj{1};
        Lhy = traj{2};
        Lhr = traj{3};
        Lhp = traj{4};
        Lnp = traj{5};
        Lap = traj{6};
        Lar = traj{7};
        Rhy = traj{8};
        Rhr = traj{9};
        Rhp = traj{10};
        Rnp = traj{11};
        Rap = traj{12};
        Rar = traj{13};
        Lsp = traj{14};
        Lsr = traj{15};
        Lsy = traj{16};
        Le = traj{17};
        Rsp = traj{18};
        Rsr = traj{19};
        Rsy = traj{20};
        Re = traj{21};
        
        nextPos = [hip, Lhy, Lhr, Lhp, Lnp, Lap, Lar, Rhy, Rhr, Rhp, Rnp, Rap, Rar, Lsp, Lsr, Lsy, Le, Rsp, Rsr, Rsy, Re];
        
        if numel(nextPos) == 0
            footPos;
            footOr;
            break
        end
        for i=1:13
            wb_servo_set_position(wb_robot_get_device(jointNames{i}),nextPos(i)*signs(i));
        end
        
        wb_robot_step(TIME_STEP);

        for i=1:13
        	forceData(i,step) = abs(wb_servo_get_motor_force_feedback(joints(i)));
        end
        gpsData(:,step) = wb_gps_get_values(gps)'.*1000; 
        lForce(1,step) = wb_touch_sensor_get_value(lTouch);
        yFl = lForce(1,step);
        rForce(1,step) = wb_touch_sensor_get_value(rTouch);
        yFr = rForce(1,step);
        
        if step == 1
            footPos(1:3,step) = wb_supervisor_node_get_position(lFoot)*1000; 
            footPos(4:6,step) = wb_supervisor_node_get_position(rFoot)*1000; 
            footOr(1:2,step) = getHeading();
            
        end
        switch state
            case 1  
                if yFl >0 
                    footPos(1:3,step) = wb_supervisor_node_get_position(lFoot)*1000;
                    footOr(1:2,step) = getHeading();
                    steplist = [steplist,step];
                end
                state = 2;
            case 2    
                if yFl <1
                    state = 3;
                end
            case 3
                if yFr >0
                    footPos(4:6,step) = wb_supervisor_node_get_position(rFoot)*1000;
                    footOr(1:2,step) = getHeading();
                    steplist = [steplist,step];
                end
                state = 4;
            case 4
                if yFr < 1
                    state = 1;
                end
            otherwise
        end
                
        t = t+ TIME_STEP / 1000.0;
        step = step+1;
    end
end

function out = resetRobot(x,z, jointNames,TIME_STEP)
      for i=1:13
        endPositions(i) = wb_servo_get_position(wb_robot_get_device(jointNames{i}));
    end  
    for j = 1:500
        
        for i=1:13
                wb_servo_set_position(wb_robot_get_device(jointNames{i}),endPositions(i)-j*endPositions(i)/500);
        end    
        wb_robot_step(TIME_STEP);
    end
    wb_supervisor_simulation_physics_reset();    
    wb_supervisor_field_set_sf_vec3f(trans_field, [x 0.3565 z]) %354263
    wb_supervisor_field_set_sf_rotation(rot_field, [1 0 0 0])
    wb_supervisor_simulation_physics_reset();
    wb_robot_step(TIME_STEP);
end
function [Ldeg,Rdeg] = getHeading()
    lFootC= wb_robot_get_device('LFootC');
    rFootC= wb_robot_get_device('RFootC');
    LComp = wb_compass_get_values(lFootC);
    Ldeg=atan2(LComp(1),LComp(3))*180/pi;
    RComp = wb_compass_get_values(rFootC);
    Rdeg = atan2(RComp(1),RComp(3))*180/pi;
end
function [footPosFilt,footOrFilt] = footFilter(footPos,footOr)
    index = logical(footPos(1,:)|footPos(2,:)|footPos(3,:)|footPos(4,:)|footPos(5,:)|footPos(6,:));
    a = footPos(1,:);
    b = footPos(2,:);
    c = footPos(3,:);
    d = footPos(4,:);
    e = footPos(5,:);
    f = footPos(6,:);

    footPosFilt = [a(index);
                   b(index);
                   c(index);
                   d(index);
                   e(index);
                   f(index)];
   a = footOr(1,:);
   b = footOr(2,:);      
   
   footOrFilt = [a(index);
                 b(index)];
           
   tol =2;
   i = 1;
   while i ~= size(footPosFilt,2)
       if i+1 <= size(footPosFilt,2)
            if ((footPosFilt(3,i) > (footPosFilt(3,i+1) - tol)) && (footPosFilt(3,i) < (footPosFilt(3,i+1) + tol))) ||  ((footPosFilt(6,i) > (footPosFilt(6,i+1) - tol)) && (footPosFilt(6,i) < (footPosFilt(6,i+1) + tol)))
                if i+2 < size(footPosFilt,2)
                    footPosFilt = [footPosFilt(:,1:i),footPosFilt(:,i+2:end)];
                    footOrFilt = [footOrFilt(:,1:i),footOrFilt(:,i+2:end)];
                else
                    footPosFilt = footPosFilt(:,1:i);
                    footOrFilt = footOrFilt(:,1:i);
                end
            else
                i = i+1;
            end
       end
   end
  
            
end

function coor = footCoor(theta,tx,ty)
    % Foot Dimensions
    l = 110;    %length
    w = 64;     %width
    of = .3;     %center offset in the backwards direction (%/100)
    
    coor = [-of*l       -w/2    0 ; %Coordinates of corner points
            (1-of)*l    -w/2    0 ;
            (1-of)*l    w/2     0 ;
            -of*l       w/2     0 ;
            -of*l       -w/2    0 ]';
    coor = [coor; 1 1 1 1 1];       %Add an additional matrix to multiply with the homogenous transfromation (4x4)
    coor = hZ(theta,tx,ty,0)*coor;
    coor = coor(1:2,:);
end

function out = footPlot(footPos,footOr)
    for i = 1:size(footPos,2)
        if rem(i,2) == 1
            coor = footCoor(footOr(1,i),footPos(3,i),footPos(1,i));
            plot(coor(1,:)',coor(2,:)');
        else
            coor = footCoor(footOr(2,i),footPos(6,i),footPos(4,i));
            plot(coor(1,:)',coor(2,:)');
        end
    end
end
        
        
end