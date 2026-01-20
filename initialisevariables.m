clear
clc
clear all

simulinkModelName="rcams.slx";

m = 120000; %Aircraft total mass (kg)
%NOTE: We will define Ib and invib later

cbar = 6.6; %Mean Aerodynamic Chord (m)
lt = 24.8;  %Distance by AC of tail and body (m)
S = 260;    %Wing planform area (m^2)
St = 64;    %Tail planform area (m~2)
Xcg = 0.23*cbar;%x position of CoG in Fm (m)
Ycg = 0;        %y position of CoG in Fm (m)
Zcg = 0.10*cbar;%z position of CoG in Fm (m)
Xac = 0.12*cbar;%x position of aerodynamic center in Fm (m) 
Yac = 0;        %y position of aerodynamic center in Em (m)
Zac = 0;        %z position of aerodynamic center in Fm (m)
%Engine constants
Xapt1 = 0;      %x position of engine 1 force in Em (m)
Yapt1 = -7.94;  %y position of engine 1 force in Em (m)
Zapt1 = -1.9;   %z position of engine 1 force in Em (m)

Xapt2 = 0;      %x position of engine 1 force in Em (m)
Yapt2 = 7.94;  %y position of engine 1 force in Em (m)
Zapt2 = -1.9;  
%other constants

rho = 1.225;
g = 9.81;
depsda = 0.25;
alpha_L0 = -11.5*pi/180;
n = 5.5;
a3 = -768.5;
a2 = 609.2;
a1 = -155.2;
a0 = 15.212;
alpha_switch = 14.5* (pi/180);

Ib=m*[40.07 0 -2.0923;
      0 64 0;
      -2.0923 0 99.92];

temp=load("XU_data");
out.simU= temp.simU;

U=simU.signals.values';
t=simU.time;
simUin.time=t;
simUin.signals.values=U;
 
lat0=deg2rad(Convert)


