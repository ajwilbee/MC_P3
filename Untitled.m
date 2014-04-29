load('ee661_proj3_ss_model.mat')

IC = zeros(8,1);
U = [0,0,0]';
%% part 1 design K
%design K using projet 1
DP = [-65+5900i, -65-5900i -3+180i, -3-180i ...
            -4+130i,   -4-130i,  -5+50i,  -5-50i];
DPK = 1.005*DP;
K = MIMOTest(ee661_proj3_ss_model.A,ee661_proj3_ss_model.B, ...
         ee661_proj3_ss_model.C, ee661_proj3_ss_model.D, ...
         U, DPK,IC);
     
%% part 2 design L
% NewA = A-B*K;
% [Yol, timeol] = ZOHTest(A, B, C, D, U, initialConditions);
% [Ycl, timecl] = ZOHTest(NewA, B, C, D, U, initialConditions);
% design L using transposes and project 1


DPL = .995*DP;

LTrans = MIMOTest(ee661_proj3_ss_model.A',ee661_proj3_ss_model.C', ...
         ee661_proj3_ss_model.B, ee661_proj3_ss_model.D, ...
         U, DPK,IC);
%% part three varify the stand alone controller     
L = LTrans';
A = ee661_proj3_ss_model.A;
B = ee661_proj3_ss_model.B;
C = ee661_proj3_ss_model.C;
D = ee661_proj3_ss_model.D;

AStandAlone = A-B*K-L*C+L*D*K;
eigCheckOC = eig(AStandAlone);
test = sum(real(eigCheckOC < 0));

if test == length(eigCheckOC)
   disp('Observer controller is independantly stable');
else
    disp('Observer controller is independantly unstable');
end
%%Part 5 Closed loop model verify the pole locations with error
%assuming that the model is identical to the plant
Acl = [A -B*K; L*C A-B*K-L*C];
Bcl = [B;B];
Ccl = [C -D*K];
Dcl = D;

eigCheckCL = eig(Acl);
test = sum(real(eigCheckCL < 0));
if test == length(eigCheckCL)
   disp('System is stable');
else
    disp('System is unstable');
end

poles = zeros(length(DPL),1);
for x = 1:length(DPL)
    poles(x)=eigCheckCL(2*x-1) ; 
end
designError = DP'-poles;

%% part 4 FRF Char loci

%complete closed-loop
CLssModelOC= ss(Acl, Bcl, Ccl, Dcl);

[ char_loci_ol,w,FRFOL] = FRF_CharLoci( ee661_proj3_ss_model );
[ char_loci_cl ,w,FRFCL] = FRF_CharLoci( CLssModelOC );

%% Part 7 plot the magnitudes of char loci against
figure
hold on
magnitude_OL = 20*log10(abs(FRFOL));
  scatter(w,magnitude_OL(1,1,:),'b.','o' )
  scatter(w,magnitude_OL(2,2,:),'r.','o')
  scatter(w,magnitude_OL(3,3,:),'g.','o')
  title('Open Loop Magnitude Plot');
  legend('Mag lambda1 OL', 'Mag lambda2 OL ','Mag lambda3 OL')
  xlabel('frequency');
  ylabel('magnitude (db)');
figure
hold on
magnitude_CL = 20*log10(abs(FRFCL));
  scatter(w,magnitude_CL(1,1,:),'b','*' )
  scatter(w,magnitude_CL(2,2,:),'r','*')
  scatter(w,magnitude_CL(3,3,:),'g','*')
  title('Closed Loop Magnitude Plot');
  legend('Mag lambda1 CL','Mag lambda2 CL','Mag lambda3 CL');
  xlabel('frequency');
  ylabel('magnitude (db)');
 % both together
  figure
hold on
magnitude_OL = 20*log10(abs(FRFOL));
  scatter(w,magnitude_OL(1,1,:),'b','o' )
  scatter(w,magnitude_OL(2,2,:),'r','o')
  scatter(w,magnitude_OL(3,3,:),'g','o')
magnitude_CL = 20*log10(abs(FRFCL));
  scatter(w,magnitude_CL(1,1,:),'y','*' )
  scatter(w,magnitude_CL(2,2,:),'m','*')
  scatter(w,magnitude_CL(3,3,:),'c','*')
  title('Closed Loop Magnitude Plot');
  legend('Mag lambda1 CL','Mag lambda2 CL','Mag lambda3 CL','Mag lambda1 CL','Mag lambda2 CL','Mag lambda3 CL');
  xlabel('frequency');
  ylabel('magnitude (db)');

%% part 6 matlab gram function for Grammians
ControlGram = gram(ee661_proj3_ss_model,'c');
if(rank(ControlGram) == size(ControlGram,1))
    disp('controlable');
else
    disp('not Entirely Controlable');
end

ObserveGram = gram(ee661_proj3_ss_model,'o');
if(rank(ObserveGram) == size(ObserveGram,1))
    disp('Observable');
else
    disp('not Entirely Observable');
end
     
%% part 8
[placeK,precK,messageK] = place(A,B,DPK); 
[placeL,precL,messageL] = place(A',C',DPL);
placeL = placeL';
% no semicolon so it will display
Design_Knorm = norm(K)
Place_Knorm = norm(placeK)
Design_Lnorm = norm(L)
Place_Lnorm = norm(placeL)
% check the stand alone controller
AmStandAlone = A-B*placeK-placeL*C+placeL*D*placeK;
eigmCheckOC = eig(AmStandAlone);
test = sum(real(eigmCheckOC < 0));

if test == length(eigmCheckOC)
   disp('Observer controller is independantly STABLE using "place" K and L');
else
    disp('Observer controller is independantly UNSTABLE using "place" K and L');
end

% NewA = A-B*K;
% [Yol, timeol] = ZOHTest(A, B, C, D, U, initialConditions);
% [Ycl, timecl] = ZOHTest(NewA, B, C, D, U, initialConditions);