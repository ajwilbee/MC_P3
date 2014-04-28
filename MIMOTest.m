function [K] = MIMOTest(A,B,C,D,U,desiredPoles,initialConditions)
% creates a MIMO controler for the given system and then tests it through
% simultion
% test data from class notes
% A = [1 -1 2; 0 2 1; 0 0 3];
% B = [1 -1; -1 1/2; 2 1];
% C = [1 2 -1; -1 1 0];
% D = [.01 0; 0 .02];
% 
% desiredPoles = [-5 -2+2*1j -2-2*1j];

% actual data

% A = [-1 250 0 0 0 0 0; -250 -1 0 0 0 0 0 ;0 0 -1 190 0 0 0 ; ...
%     0 0 -190 -1 0 0 0; 0 0 0 0 -1 150 0; 0 0 0 0 -150 -1 0;...
%     0 0 0 0 0 0 -300];
% B = [21.1 10.2;7.63 12.7;7.47 4.21;2.17 8.32;6.45 .75;1.52 5.13;25.0 21.3];
% C = [7.63 21.1 2.17 7.47 1.52 6.45 25.0];
% D = [0 0];
% U = [0 0]';
% desiredPoles = [ -300, -31-150*1j, -31+150*1j,  -39-190*1j, -39+190*1j, -51-250*1j, -51+250*1j];
% initialConditions = [-1.94 -1.65 -.78 .45 -.72 .32 -.87]';

u = cell(length(desiredPoles),1);
s = cell(length(desiredPoles),1);
v = cell(length(desiredPoles),1);
zeta = cell(length(desiredPoles),1);

for x = 1:length(desiredPoles)
    temp = [(eye(size(A))*desiredPoles(x)-A) B];
    [u{x}, s{x}, v{x}] = svd(temp);
    r = length(v{x}) - length(u{x});
    n = length(u{x});
    zeta{x} = sum(v{x}(:,n+1:end),2);
end

psy = cell2mat(zeta');

psyn = psy(1:n,:);
psyr = psy(n+1:end,:);

K = real(psyr*inv(psyn));

check = eig(A-B*K);


% sim('MiMo_Sim');
% y1 = y;
% t1 = t;
% A = NewA;
% [y2,t2] = sim('MiMo_Sim');
% y2 = y;
% t2 = t;





% close all
% figure
% hold all
%% plot the simulations
% subplot(2,2,1),plot(t1,y1),title('Open Loop Simulink Simulation'),ylabel('Response'),xlabel('Time(s)');
% subplot(2,2,2), plot(t2,y2),title('Closed Loop Simulink Simulation'),ylabel('Response'),xlabel('Time(s)');
% plot( timeol, Yol),title('MIMO ZOH Simulation'),ylabel('Response'),xlabel('Time(s)');
% plot( timecl, Ycl);
% legend('Open Loop ZOH Simulation','Closed Loop ZOH Simulation');
% does the B C D matrix change?
% Yol = Simulate(A, B, C, D, r, initialConditions);
% Ycl = Simulate(ClosedLoopA, B, ClosedLoopC, D, r, initialConditions);


% title('Open Loop Simulink Simulation'),ylabel('Response'),xlabel('Time(s)')
% subplot(2,2,2), plot(t2,y2),title('Closed Loop Simulink Simulation'),ylabel('Response'),xlabel('Time(s)');
% subplot(2,2,3),plot( timeol, Yol),title('Open Loop ZOH Simulation'),ylabel('Response'),xlabel('Time(s)');
% subplot(2,2,4), plot( timecl, Ycl),title('Closed Loop ZOH Simulation'),ylabel('Response'),xlabel('Time(s)');

