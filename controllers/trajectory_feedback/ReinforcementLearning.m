%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q learning of single agent move in N rooms 
% Matlab Code companion of 
% Q Learning by Example, by Kardi Teknomo 
% (http://people.revoledu.com/kardi/)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
function Hipz =ReinforcementLearning(q,N)
clc;
format short
format compact
% Two input: R and gamma
% immediate reward matrix; 
% row and column = states; -Inf = no door between room
if N < 100
   % random initial state
   y=randperm(size(R,1));
   state=y(1);
   
   % select any action from this state
   x=find(R(state,:)>=0);        % find possible action of this state
   if size(x,1)>0,
      x1=RandomPermutation(x);   % randomize the possible action
      x1=x1(1);                  % select an action 
   end
   qMax=max(q,[],2);
   q(state,x1)= R(state,x1)+gamma*qMax(x1);   % get max of all actions 
   state=x1;
   
end 
%normalize q
g=max(max(q));
if g>0, 
   q=100*q/g;
end
  