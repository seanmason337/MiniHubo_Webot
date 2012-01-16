function out = HMC_Controller
    close all
    clear all
    clc
    % 
%     desktop;
%     keyboard;

TIME_STEP=32;
startOrientation = 0;
orientation = startOrientation;

%% Initialize Sensors
gps = wb_robot_get_device('zero');
wb_gps_enable(gps, TIME_STEP);

gyro = wb_robot_get_device('imugyro');
wb_gyro_enable(gyro, TIME_STEP);

robot_node = wb_supervisor_node_get_from_def('MiniHubo');
trans_field = wb_supervisor_node_get_field(robot_node, 'translation');




N = 0;
while N <3
    N
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
    [forceData,gpsData] = commandServos('trajectory.txt',joints,TIME_STEP);
    t = 1:size(forceData,2);

%     figure(1)
%     plot(t,forceData)
%     legend(jointNames{1},jointNames{2},jointNames{3},jointNames{4},jointNames{5},jointNames{6},jointNames{7},jointNames{8},jointNames{9},jointNames{10},jointNames{11},jointNames{12},jointNames{13});
% 
%     figure(2)
%     %plot(t,gpsData)
%     %legend('X','Y','Z');
%     plot(gpsData(1,:),gpsData(3,:));
%     legend('COM X-Y');
    %wb_supervisor_simulation_revert() 
    wb_supervisor_field_set_sf_vec3f(trans_field, [0,.354263s,0])
    wb_supervisor_simulation_physics_reset()
    N=N+1;
end


function [forceData, gpsData] = commandServos(file,joints,TIME_STEP)
    gps = wb_robot_get_device('zero');
    gpsData = zeros(3,1);
    signs = [-1 -1 -1 1 -1 -1 1 -1 -1 -1 1 1 -1 ];
    hip = 1;
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
        	forceData(i,step) = wb_servo_get_motor_force_feedback(joints(i));
        end
        gpsData(:,step) = wb_gps_get_values(gps)'.*1000;
    
        %updateQ(forceData(:,step),gpsData(:,step), step)
        t = t+ TIME_STEP / 1000.0;
        step = step+1;
    end
end

end