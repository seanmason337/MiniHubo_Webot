clc; clear;
tests = {}
for i = 700:700:9800
    tests(i/700) =  {num2str(i)}
end

hold on;
for i = 1: length(tests)
    Q_name = strcat('Qmat_',tests(i),'_g5_a7_1_0.mat');
    load(cell2mat(Q_name))

    % Q = randi(20,7,20)


    Q = Q(1:91,1:end);
    maxQ = max(max(Q))
    Qsc = Q/maxQ;
    Qsc = [ones(1,size(Qsc,2)) ;Qsc;ones(1,size(Qsc,2))]
    %    gamma = .5;
    %     alpha = .7;
    %     w1 = 0;
    %     w2 = 1;




    [stateActions totalSteps] = size(Qsc);
    states = stateActions/3;
    path = zeros(states,totalSteps);
    sum1 = zeros(states,1);

    for i = 1:states
        step = 1;
        state = i;
        SA = (state-1)*3;
        while step <totalSteps-1
            path(i,step) = state;
            [minValue,minIndex] = min(Qsc(state*3-2:state*3,step));

            action = minIndex;
            SA = (state-1)*3+action;
            sum1(i) = sum1(i)+Qsc(SA,step);


            state = state+(action-2);
            step = step+1;
        end
    end
    [minPathVal minPathIndex] = min(sum1);
    path = path(minPathIndex,1:end);
    plot(path');
end