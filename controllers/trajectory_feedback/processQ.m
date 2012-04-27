clc; clear; close all;
tests = {}
x = 1000
for i = 1000:x:9000
    tests(i/x) =  {num2str(i)};
end
colormap('winter')
for i = 1: length(tests)
    Q_name = strcat('Qmat_',tests(i),'_g3_a7_1_0_steps20.mat');
    load(cell2mat(Q_name));

    Q = Q(1:91,1:end);
    maxQ = max(max(Q))
    Qsc = Q/maxQ;
    Qsc = [ones(1,size(Qsc,2)) ;Qsc;ones(1,size(Qsc,2))];

    [stateActions totalSteps] = size(Qsc);
    states = stateActions/3;
    path = zeros(states,totalSteps);
    sum1 = zeros(states,1);

    for j = 1:states
        step = 1;
        state = j;
        SA = (state-1)*3;
        while step <totalSteps-1
            path(j,step) = state;
            [minValue,minIndex] = min(Qsc(state*3-2:state*3,step));

            action = minIndex;
            SA = (state-1)*3+action;
            sum1(j) = sum1(j)+Qsc(SA,step);


            state = state+(action-2);
            step = step+1;
        end
    end
    [minPathVal minPathIndex] = min(sum1);
    learnedPaths(i,:) = path(minPathIndex,1:end);
end
n=length(tests);
for k=1:n
    colors(k,:) = [0 1-k/n 0];
end
   
for p=1:n
    figure(p)
    plot(learnedPaths(p,:)','Color',colors(p,:),'LineWidth',2);
    
end

shg