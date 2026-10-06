%%1st assignment modelling
%%NAME: Nmesoma Osakwe, NEPTUN: F8F29H
%%DC Motor Modelling
syms s
%DC MOTOR PARAMTERS
V_an = 24; %V, Nominal Voltage
R_a = 37.5; %Ohm arm. res.
L_a = 0.00137; %H, arm. ind
kfi = 0.0225; %Vs/rad, torque const.
I_anl = 0.0168 %A , no load current
B_m = (kfi^2*I_anl)/(V_an-R_a*I_anl); %Nms, viscous fr. coeff
J_m = 1.16*10^-7; %kgm2, inertia

%Load parameters
T_nl= 0; %Nm no-load T.
%T_1 = 5*10^-3; %Nm, between n-1 and nominal T
T_n = 4.76*10^-3; %Nm nominal T
%T_2 = 50*10^-3; %Nm, between nominal and stall T
T_s= 14.4*10^-3  %Nm, stall T

%Transfer function for the DC motor
Numerator = [kfi];
Denominator = [L_a*J_m* + (L_a*B_m + J_m*R_a) + R_a*B_m+kfi^2];

%TF HARD CODED
num = [kfi];
denom = [(J_m*L_a) (B_m*L_a+J_m*R_a) (kfi^2+B_m*R_a)];
SYS1 = tf (num, denom)

%TF using feedback comnand
Num1 = [1];
eDen = [L_a R_a];
mDen = [J_m B_m];
TFe = tf (Num1, eDen)
TFm = tf(Num1, mDen)
kfi1 = tf(kfi)
W1 = TFe * kfi1 * TFm;
W2 = kfi1;
SYS2 = feedback (W1, W2)

%TF from Simulink model, using Model Linearizer (ss - state-space)
ss = linsys1;
SYS3 = tf(ss);
SYS3


%residue command - partial fraction expression
[r, p, k] = residue(num, denom)

%residue command - partial 24v step input
num_step = V_an * num;
denom_step = [denom, 0];
[r1, p1, k1] = residue(num_step, denom_step)
