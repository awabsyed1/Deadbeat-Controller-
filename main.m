Wn = 10 
p = 70
zeta = 0.2
k  = 30
s = tf('s')

%Effects of changes of loop gain on the bandwidth of the system
G = k*Wn^2 / ((s^2 + 2*zeta*Wn*s+Wn^2)*(s+p)) %bandwidth (G,-3)
G_CL = feedback(G,1) %Closed loop
band_wdith = bandwidth (G_CL,-3) 
%Loop gain does not affect the bandwidth of the open loop 
%Loop gain does affect the bandwdith of the CL (Increas, increases)

%Loop gain does not affect the rise time of the open Loop 
%Loop gain does affect the rise time of CL (Increasing gain, decreases)
step(G_CL)

%Increasing zeta decreases the bandwidth of Open Loop 
%Increase zeta decreaes the bandwdith of CL
%Incraese zeta increases increases ries time 

%Increasing the Gain, moves closed loop poles away 
%Complex poles on right - unstable 

%Higher Loop gain for Linear system

%% Week 5 
%--------------------Controlled Plant Transfer Function----------------%
Ts = 0.06
s = tf('s')
z = tf('z',Ts)

num = [0.04 0.04] %Numerator of the Plant
den = [1 0.2 0.04] %Denominator of the Plant
Gs = tf(num,den) %Plant Transfer Function
CL_Gs = feedback(Gs,1) %Closed Loop Plant Transfer Function
%----------------------Lead Compensator---------------------------------%
Gc = zpk ([-5.88],[-5.88],142) % Lead Compensator
G_overall = Gs * Gc %Plant with Lead Compensator
CL = feedback(G_overall,1)
band_width = bandwidth(CL)%Bandwdidth of the Plant with Lead Compensator
Dz = c2d(G_overall,Ts,'zoh') %Digital Controller 
Dz1 = feedback(Dz,1) %Closed Loop Digital Controller

%% Deadbeat
Ts = 1 %Sampling Time same as before
%---------------------Plant Transfer Function----------------------------%
z = tf('z',Ts)
s = tf('s')
%s = (z-1)/Ts %Eulers Approx 
Gs = (0.04*(s+1))/((s^2)+(0.2*s)+0.04) %Plant Transfer Function

%----------------------Deadbeat Digital Controller------------------------%
deadbeat =((z^2)-(1.8*z)+0.84)/((0.04*z^2)-(0.04*z))
Gz = d2d(Gs,Ts,'zoh')
Mz = 1/z;
D = Mz/(Gz*(1-Mz))
D = minreal(D);
figure (2), step(D/(1+D*Gz))
%---------------------Plot------------------------------------------------%s
step(feedback((Gs*deadbeat),1));
grid on;
title('System Response with a deadbeat digital controller');

%%
num=[0.04 0.04];
den=[1 0.2 0.04];
sys = tf(num,den)
Gpd = c2d (sys,1,'matched')
% Discrete-time transfer function 
Dznum=[1];
Dzden=[1 -1];
D = tf (Dznum, Dzden, -1)

Dz=D*1/Gpd
sysold=Dz*Gpd
syscld = connect (sysold, [1 -1])

step(syscld)

%%
Gp_ol = tf ([0.04 0.04] ,[1 0.2 0.04]) ;
G_open_info = stepinfo ( Gp_ol );
D = feedback(Gp_ol,1)
figure (1)
step ( Gp_ol )
Ts = 0.2;
Gp_z = c2d(Gp_ol ,Ts ,'zoh ');
G_z_info = stepinfo ( Gp_z );
figure (2)
% step ( Gp_z );
D_z = tf ([1 -1.959 0.9608] ,[0.008629 -0.007 0 -0.0086 0.007]) ;
step (D_z)

%%
%Solution given
%   D = zpk([-0.81],[-8.1],115.757)
%  k = c2d(D,Ts)

% Num = [2.12 2.12] %new gain 
% Gs1 = tf(Num,den) %New Plant Transfer Function 

% Lecture version Compensated System
% num_C = [0.92 0.92]
% den_C = [1 0.2 0.04]
% Gs_comp = tf(num_C,den_C)
% CL_comp = feedback(Gs_comp,1)
% disc = c2d(Gs_comp,Ts,'zoh')
% step (Gs_comp); hold on
% step (disc); hold off

% Ts = 0.5
% Gd1 = c2d(Gs,Ts,'ZOH')