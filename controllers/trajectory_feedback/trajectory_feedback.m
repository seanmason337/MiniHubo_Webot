function out = trajectory_feedback()
    close all
    clear all
    clc
    % 
   %desktop;
   %keyboard;

TIME_STEP=10;

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
DSP = 0.2;
SSP = 1.3;
SD = 92;
%LD = 78.5; % Lateral ZMP = LD/2
LD=66
%LD=89
NumOfStep = 6;
delt = 0.01;
init = 1;
endd = 2;
stairH = 0;
CommonPara = [Height Gravity DSP SSP SD LD NumOfStep delt init endd stairH];
jointNames  = {'HY'; 'LHY'; 'LHR'; 'LHP'; 'LKP'; 'LAP'; 'LAR'; 'RHY';...
        'RHR'; 'RHP'; 'RKP'; 'RAP'; 'RAR'; 'LSP'; 'LSR'; 'LSY'; 'LEP'; 'RSP'; 'RSR'; 'RSY'; 'REP'};

min = 200;
max = 300;
deltZ = .1;

TotalTimeSequence = 0:delt:(init+(NumOfStep+2)*DSP + (NumOfStep+1)*SSP + endd);
[r,c] = size(TotalTimeSequence)

Q = zeros(length(min:deltZ:max), c);

N = 0;
while N <20
% Insert Tuning Parameter Here
    CommonPara = [Height Gravity DSP SSP SD LD NumOfStep delt init endd stairH];
    [Hipz,indexList,key] = randTraj(CommonPara,4);
    Hipz = (220)*ones(1,c);
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

    [forceData,gpsData,footPos,footOr] = commandServos('trajectory.txt',joints,TIME_STEP);
	forceData = forceData(:,crouch_time:end);
    gpsData = gpsData(:,crouch_time:end);
    footPos = footPos(:,crouch_time:end);
    figure(1)
    plot(gpsData(3,:)',gpsData(1,:)',Hipx_Preview',Hipy_Preview')
    hold on
    %Left Foot
    scatter(footPos(3,:)',footPos(1,:)')
    %Right Foot
    scatter(footPos(6,:)',footPos(4,:)')
    drawnow()
    hold off
    %% Update Q
    for(i = 1:c)
        index = indexList(i);
        Q(index,i) = sum(forceData(:,i));
    end

    resetRobot(0,0,jointNames,TIME_STEP);

    N=N+1;
end


function [forceData, gpsData,footPos,footOr] = commandServos(file,joints,TIME_STEP)
    
    gps = wb_robot_get_device('zero');
    lTouch = wb_robot_get_device('LFoot');
    rTouch = wb_robot_get_device('RFoot');
    
    
    lFoot = wb_supervisor_node_get_from_def('LFoot');
    rFoot = wb_supervisor_node_get_from_def('RFoot');
    lForce = zeros(3,1);
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
        lForce(:,step) = wb_touch_sensor_get_values(lTouch);
        yFl = lForce(2,step);
        rForce(:,step) = wb_touch_sensor_get_values(rTouch);
        yFr = rForce(2,step);
        
        if step == 1
            footPos(1:3,step) = wb_supervisor_node_get_position(lFoot)*1000; 
            footPos(4:6,step) = wb_supervisor_node_get_position(rFoot)*1000; 
            footOr(1:2,step) = getHeading();
            
        end
        if yFl >140 
            footPos(1:3,step) = wb_supervisor_node_get_position(lFoot)*1000;
            footOr(1:2,step) = getHeading();
        end
        if yFr >140 
            footPos(1:3,step) = wb_supervisor_node_get_position(rFoot)*1000;
            footOr(1:2,step) = getHeading();
        end         
        
        %updateQ(forceData(:,step),gpsData(:,step), step)
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


end