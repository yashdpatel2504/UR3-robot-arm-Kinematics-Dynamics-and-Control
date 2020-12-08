function [dyn]=dparameters()

dyn = struct('m',[],'Il',[],'Im',[],'r_cl',[]);


dyn(1).r_cl = [0;-0.00968000000000000;0.0132400000000000];
dyn(2).r_cl = [-0.128000000000000;0;0.0872100000000000];
dyn(3).r_cl = [-0.118120000000000;0;0.00989000000000000];
dyn(4).r_cl = [0;0.00616000000000000;0.00998000000000000];
dyn(5).r_cl = [0;-0.00587000000000000;0.0134100000000000];
dyn(6).r_cl = [0;0;-0.0156100000000000];
%dyn.r_cl motor = 75.00

%7 dyn � o motor
dyn(1).m = 0.571750000000000;
dyn(2).m = 1.36263000000000;
dyn(3).m = 1.05204000000000;
dyn(4).m = 0.321950000000000;
dyn(5).m = 0.338520000000000;
dyn(6).m = 0.161130000000000;
dyn(7).m = 95.43*10^-3;



dyn(1).Il=[0.0011 -0.0000 -0.0000; -0.0000 0.0009 0.0073; -0.0000 0.0073 0.0009];

dyn(2).Il=[0.0021 -0.0000 0.0003; -0.0000 0.0153 0.0000; 0.0003 0.0000 0.0150];

dyn(3).Il=[0.0012 -0.0000 -0.0004; -0.0000 0.0085 0.0000; -0.0004 0.0000 0.0083];

dyn(4).Il=[0.2938 -0.0000 -0.0000; -0.0000 0.2610 -0.0199; -0.0000 -0.0199 0.2318]*10^-3;

dyn(5).Il=[0.3462 -0.0000 0; -0.0000 0.3127 0.0267; 0 0.0267 0.2449]*10^-3;

dyn(6).Il=[0.5678 0.00 0.00; 0.00 0.5678 0.00; 0.00 0.00 0.8652]*10^-4;

% [1349552.78 -0.47 -0.15; -0.47 1209670.33 -44535.19; -0.15 -44535.19 1111850.22]
% [3137126.35	-11.91 -70704.80; -11.91 22261208.50 -8.55; -70704.80 -8.55 21518612.94]
% [732976.75 -2.85 264552.44;	-2.85 6627555.93 -0.34; 264552.44 -0.34 6428989.57]
% [299370.29 -0.02 0.04; -0.02 304168.50 -427.65; 0.04 -427.65 176908.53]
% [299370.29 -0.02 0.04; -0.02 304168.50 -427.65; 0.04 -427.65 176908.53]
% [28687.08 0.00 0.00; 0.00 28687.08 0.00; 0.00 0.00 52139.64]

% [1464330.62 -0.50	-0.21;-0.50 1303254.61 0.00;-0.21 0.00 1133043.77];
% [12296705.60 -47.13 -19898058.54; -47.13 74340221.71 7.72; -19898058.54 7.72 64438046.91];
% [1957307.32 -2.44 -2550860.04; -2.44 14326075.42 -0.52; -2550860.04 -0.52 12903178.48];
% [305681.33 -0.02 0.03; -0.02 310450.42 0.00; 0.03 0.00 176937.64];
% [305681.33 -0.02 -0.03; -0.02 310450.42 0.00; -0.03 0.00 176937.64];
% [44660.19 0.00 0.00; 0.00 44660.19 0.00; 0.00 0.00 52139.64];


dyn(1).Im= [1.68983000000000e-06];
dyn(2).Im= [1.68983000000000e-06];
dyn(3).Im= [1.68983000000000e-06];
dyn(4).Im= [1.68983000000000e-06];
dyn(5).Im= [1.68983000000000e-06];
dyn(6).Im =[1.68983000000000e-06];

