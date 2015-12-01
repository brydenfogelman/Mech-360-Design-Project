%% Constants
g = 9.81;

%% Design Specs
grossWieght = 22700; % N
approachSpeed = 35; % m/s
descentSpeed = 5; % m/s
mass = grossWieght/g;
% Material Properties
Sut = 310; % MPa
Sy = 276; % MPa
density = 2700; % kg/m³
E = 68.9*10^9; % GPa

%% Design Criterion
L = 1.5; % m
nfs = 1.8; % Factor of Safety

%% Setup
alwStress = Sy * 10^6
R = grossWieght / 2; % N, reaction force due to the weight of the plane

%% Impact
% Assuming that the impact speed is the descent speed and the approach
% speed remains the same after touching down.
kineticEnergy = @(mass, speed) 0.5 * mass * speed^2;

% To compute the impact stresses due to the impact, the descent velocity
% was split into a different coordinate system where either could be
% analyzed seperate of the other.

impactEnergy = kineticEnergy(mass,descentSpeed) / 2 % spilt between both landing gears

%% Design Iteration for Impact
% Hollow shaft with changing cross section

% Paramters
do_min = 0.31*2; % m, smallest outer diameter
do_max = 0.49*2; % m, largest outer diameter
t = 0.13; % m, thickness of the landing gear
phi = 8 * pi / 180; % rad, Angle between the landing gear and the vertical axis

% Functions for the changing cross section area
do =    @(x) do_min + (do_max-do_min)*x/L;
di =    @(x) do_min - 2*t + (do_max-do_min)*x/L;
A =     @(x) pi / 4 * ((do(x)).^2 - (di(x)).^2); % areaNonUniformXSectionHollowShaft
I =     @(x) pi / 64 * ((do(x)).^4 - (di(x)).^4); % MoINonUniformXSectionHollowShaft
fun1 =  @(x) 1 ./ A(x); 
fun2 =  @(x) x.^2 ./ I(x); 

% Numerical Integration
int1 = integral(fun1,0,L)
int2 = integral(fun2,0,L)

% Equivalent force due to impact
P = ( 2 * E * impactEnergy / (cos(phi)^2 * int1 + sin(phi)^2 * int2) )^0.5;

% Vector that contains all the diameters
dx = 1000;
d = linspace(do_min,do_max,dx);
% Computing force due to the weight of the landing gear
V = sum( A(d) * L / dx);
Fg = V * density * g

% Total Force
F = R + Fg + P
% Components of the total force
F_u = F * cos(phi); % axial
F_v = F * sin(phi); % bending

% Stress at different points along the shaft
stress = F_u ./ A(d) + F_v * L * d/2 ./ I(d);

% Finding the max Stress
maxStress = max(stress)

% Computing the safety factor
n = alwStress / maxStress

%% Impact Stress Plots

% Figure(1)

%% Design Iteration for Buckling
% Assume one end free and the other fixed


