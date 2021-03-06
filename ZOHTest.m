
function  [y, time] = ZOHTest(A,B,C,D,r,initialConditions)
% perform ZOH sampling and simulation on the input statespace equation
  p = eig(A);
  t = 1/10000;
  count = 1/t;
  Adt = diag(exp(p*t));
  Bdt = zeros(size(B));
  for x = 1:size(B,1)
      
    Bdt(x,:) = (exp(p(x)*t)-1)/p(x).* B(x,:);
  
  end
   
  [ y, time ] = Simulate(Adt, Bdt, C, D, r, initialConditions,count);
  close all
  time = repmat(time*t,3,1);
  y = cell2mat(y');
  figure
  hold all
  
  for x = 1: size(time,1)
    plot(time(1,:),y(x,:));
  end
  
end
  
  
  % A = [-1   250 0 0 0 0 0; ...
%      -250 -1  0 0 0 0 0;...
%       0    0 -1 190 0 0 0 ; ...
%       0    0 -190 -1 0 0 0;...
%       0    0  0    0 -1 150 0;...
%       0    0  0    0 -150 -1 0;...
%       0    0  0    0   0   0 -300];
% B = [21.1;...
%     7.63;...
%     7.47;...
%     2.17;...
%     6.45;...
%     1.52;...
%     25.0];
% C = [7.63 21.1 2.17 7.47 1.52 6.45 25.0];
% initialConditions = [-1.94 -1.65 -.78 .45 -.72 .32 -.87]';
% D = 0;
% r = 0;
% 
% sys_mine = ss(A,B,C,D)
% %Zero Order Hold
% 
% % T = [(1-1j)/2 (1+1j)/2; (1+1j)/2 (1-1j)/2];
% T = [(1-1j)/2 (1+1j)/2 0 0 0 0 0; ...
%      (1+1j)/2 (1-1j)/2  0 0 0 0 0;...
%       0    0 (1-1j)/2 (1+1j)/2 0 0 0 ; ...
%       0    0 (1+1j)/2 (1-1j)/2 0 0 0;...
%       0    0  0    0 (1-1j)/2 (1+1j)/2 0;...
%       0    0  0    0 (1+1j)/2 (1-1j)/2 0;...
%       0    0  0    0   0   0 1];
%   A2 = T*A*inv(T);
%   temp = diag(A2)