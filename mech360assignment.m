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

alwStress = Sy / nfs * 10^6

R = grossWieght / 2; % N, reaction force due to the weight of the plane
% phi = pi/4; % Angle between the landing gear and the vertical axis
phi = 8 * pi / 180;

% Converting to a different coordinate system (u=y' and v=x')
v_u = descentSpeed * cos(phi);
v_v = descentSpeed * sin(phi);

R_u = R * cos(phi);
R_v = R * sin(phi);

%% Impact
% Assuming that the impact speed is the descent speed and the approach
% speed remains the same after touching down.
kineticEnergy = @(mass, speed) 0.5 * mass * speed^2;

% To compute the impact stresses due to the impact, the descent velocity
% was split into a different coordinate system where either could be
% analyzed seperate of the other.
impactEnergy = kineticEnergy(mass,descentSpeed) / 2 % spilt between both landing gears

%% Buckling
% Assume one end pinned and the other fixed

%% Formulas
% Area = @(do, di) pi / 4 * ( do.^2 - di.^2 );
Volume = @(A, L) L * A;
% massMomentInertia = @(do, di) pi / 64 * do.^4 - pi / 64 * di.^4;

% Stress

axial = @(F,A) F ./ A;
bending = @(M, I, d) M .* d/2 ./ I;

%% First Iteration
% % Exploring a simple uniform hollow circular shaft
% 
% % Shaft Geometry
% % Assume hollow cylindrical shaft
% di = 0; % m
% do = 0.01; % m
% 
% % Let d be a vector to plot
% d = linspace(0,0.5,100);
% t = d ./ 10; % m, thickness
% 
% d(1)=d(2);
% 
% A = Area(d,0);
% V = Volume(A,L); % m^2, volume
% I = massMomentInertia(d,0); % m^4, MoI
% 
% % Stresses From Impact
% % energySigmaAxial = sqrt( 2 * axialImpactEnergy * E / V );
% % energySigmaBending = sqrt( 6 * bendingImpactEnergy * E * (do/2)^2 / L / I );
% 
% % Stresses From Reaction Force
% % forceSigmaAxial = R_u ./ A;
% % forceSigmaBending = R_v * L * (d./2) ./ I;
% 
% % totalStress = ( energySigmaAxial + energySigmaBending + forceSigmaAxial + forceSigmaBending ) / 10^6 % MPa
% 
% P = ( 6 * E .* I .* A * impactEnergy ./ ( 3 * I * L + L^3 * A ) ) .^ 0.5;
% 
% stress = P ./ A + P * L .* d / 2 ./ I;
% 
% stress(10)/10^6
% 
% figure(1)
% plot(d,P)
% 
% figure(2)
% plot(d,stress)

%% First Iteration
% Hollow shaft with changing cross section

% Paramters

do_min = 0.31*2; % m
do_max = 0.49*2;
t = 0.13;

% Function for the changing cross section area
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
dx = 100;
d = linspace(do_min,do_max,dx);
% Computing force due to the weight of the landing gear
V = sum( A(d) * L / dx);
Fg = V * density * g

% Total Force
F = R + Fg + P
F_u = F * cos(phi)
F_v = F * sin(phi)

% Stress at different points along the shaft
stress = axial(F_u, A(d)) + bending(F_v, I(d), d);

% Finding the max Stress
maxStress = max(stress)

% Computing the safety factor
n = alwStress / maxStress













%% Iteration Setup




