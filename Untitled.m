load('ee661_proj3_ss_model.mat')

IC = zeros(8,1);
U = [0,0,0]';
%design K using projet 1
DPK = 1.005*[-65+5900i, -65-5900i -3+180i, -3-180i ...
            -4+130i,   -4-130i,  -5+50i,  -5-50i];
K = MIMOTest(ee661_proj3_ss_model.A,ee661_proj3_ss_model.B, ...
         ee661_proj3_ss_model.C, ee661_proj3_ss_model.D, ...
         U, DPK,IC);
% NewA = A-B*K;
% [Yol, timeol] = ZOHTest(A, B, C, D, U, initialConditions);
% [Ycl, timecl] = ZOHTest(NewA, B, C, D, U, initialConditions);
% design L using transposes and project 1


DPL = .995*[-65+5900i, -65-5900i -3+180i, -3-180i ...
            -4+130i,   -4-130i,  -5+50i,  -5-50i];

LTrans = MIMOTest(ee661_proj3_ss_model.A',ee661_proj3_ss_model.C', ...
         ee661_proj3_ss_model.B, ee661_proj3_ss_model.D, ...
         U, DPK,IC);
     
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
CLssModelOC= ss(Acl, Bcl, Ccl, Dcl);
[ char_loci ] = FRF_CharLoci( CLssModelOC );

poles = zeros(length(DPL),1);
for x = 1:length(DPL)
    poles(x)=eigCheckCL(2*x-1) ; 
end
designError = DPL'/.95-poles;
     
% NewA = A-B*K;
% [Yol, timeol] = ZOHTest(A, B, C, D, U, initialConditions);
% [Ycl, timecl] = ZOHTest(NewA, B, C, D, U, initialConditions);