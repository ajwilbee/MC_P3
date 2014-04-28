function [ char_loci ] = FRF_CharLoci( inputss )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

load('cbeam_measured_frf_data_20k_pts.mat');
% Ap = cbeam_40_3x2.a;
% Ac = inputss.a;
% 
% Bp = cbeam_40_3x2.b;
% Bc = inputss.b; 
% 
% Cp = cbeam_40_3x2.c;
% Cc = inputss.c;
% 
% Dp = cbeam_40_3x2.d;
% Dc = inputss.d;
% 
%   Atilda =  [Ac                Bc*Cp;...
%              zeros(size(Ac))   Ap];
%   Btilda = [Bc*Dp;...
%             Bp];
%   Ctilda = [Cc Dc*Cp];
%   Dtilda = [Dc*Dp];
  
  
  w = cbeam_frf_20k_pts.freqs;
  
  G = freqresp(inputss,w);
  H = freqresp(inputss,w);% controller frf
  compOL_frf = zeros(size(H,1),size(G,2),numel(w));
  char_loci = zeros(size(H,1),numel(w));

% multiply H*G ateach frequency in w
  for k = 1:numel(w)
     compOL_frf(:,:,k) = H(:,:,k)*G(:,:,k); 
     char_loci(:,k) = eig(compOL_frf(:,:,k)); 
     
  end
  % for each eigen value plot magnitude in db vs phase in degrees
  figure(1)
  hold on
  magnitude_CL = 20*log10(abs(char_loci));%y
  phase_CL = angle(char_loci);%x
  scatter(phase_CL(1,:)*180/pi,magnitude_CL(1,:),'b.' )
  scatter(phase_CL(2,:)*180/pi,magnitude_CL(2,:),'r.' )
  scatter(phase_CL(3,:)*180/pi,magnitude_CL(3,:),'g.' )
  title('Characteristic Loci');
  legend('first eigen value', 'second eigen value','third eigen value');
  xlabel('angle (deg)');
  ylabel('magnitude (db)');
  ngrid
%   axis([-1 1 -1 1])
end

