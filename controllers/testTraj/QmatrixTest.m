%% Matrix test
dt = .5;
t = 0:dt:4;

%Define abount of states. (up, same, down)
states = 3;

%del should be a multiple of max-min
min  = 1;
max = 11;
deltZ = 2;

Qcols = length(t);
Qrows = ((max-min)/deltZ-1)*states+2*((states-1)/2+1);

%% Example
% 1s2   1s2 1s2 
% 1s3   1s3 1s3   
% 
% 2s1   2s1 2s1
% 2s2   2s2 2s2
% 2s3   2s3 2s3
% 
% 3s1   3s1 3s1
% 3s2   3s2 3s2

Q = zeros(Qrows,Qcols)

key = (min:deltZ:max)';
index = randi(size(key,1),1);
hipInit = key(index);
Hipz = zeros(1,Qcols);
indexList = zeros(1,Qcols);
indexList(1) = index;
Hipz(1,1) = hipInit;
N = 1;
for i = 2:Qcols
    index = index+sign((-1)^randi(2))*randi([-N,N],1);
    if (index <1) 
        index = 1;
    elseif index > size(key,1)
        index = size(key,1);
    end
    indexList(i) = index;
end
indexList
actions = [(indexList(2:end)-indexList(1:end-1))+2 , 2]

for i = 1:length(indexList)
    if indexList(i) == min
        Q(-1+(actions(i)),i) = 1;
    else
        Q(((indexList(i)-1)*3)-1+(actions(i)),i) = 1;
    end
end
Q
