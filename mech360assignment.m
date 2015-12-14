%% Design Specs And Constants
g = 9.81;
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
alwStress = Sy * 10^6;
R = grossWieght / 2; % N, reaction force due to the weight of the plane

%% Impact
% Assuming that the impact speed is the descent speed and the approach
% speed remains the same after touching down.
kineticEnergy = @(mass, speed) 0.5 * mass * speed^2;

impactEnergy = kineticEnergy(mass,descentSpeed) / 2; 
% spilt between both landing gears

%% First Design Iteration
% Solid uniform cross section shaft

% Parameters
d_cyl = 0.33*2; % m, smallest outer diameter % m, largest outer diameter
phi = 5 * pi / 180; % rad, Angle between the landing gear and the vertical axis

% Calculating the constant A and I of the landing gear
A_const = pi / 4 * d_cyl^2; I_const = pi / 64 * d_cyl^4;

P1 = ( 6 * impactEnergy * E * A_const * I_const / ( 3*L*I_const*cos(phi)^2 + L^3*A_const*sin(phi)^2) )^0.5;

% Volume and Mass
V_const = A_const*L;
mass1 = V_const * density
Fg = mass * g;

% Total Force
F1 = R + Fg + P1;
% Components of the total force
F_u = F1 * cos(phi); % axial
F_v = F1 * sin(phi); % bending

% Stress at end of landing gear
stress_cyl = F_u / A_const + F_v * L * d_cyl/2 / I_const

% Computing the safety factor
n = alwStress / stress_cyl

%% Second Design Iteration
% Hollow circular shaft with changing cross section

% Parameters
do_min = 0.25*2; % m, smallest outer diameter
do_max = 0.35*2; % m, largest outer diameter
t =      0.085; % m, thickness of the landing gear
phi =    5 * pi / 180; % rad, Angle between the landing gear and the vertical axis

% Functions for the changing cross section area
do =    @(x) do_min + (do_max-do_min)*x/L;
di =    @(x) do_min - 2*t + (do_max-do_min)*x/L;
A =     @(x) pi / 4 * ((do(x)).^2 - (di(x)).^2);
I =     @(x) pi / 64 * ((do(x)).^4 - (di(x)).^4);
fun1 =  @(x) 1 ./ A(x); 
fun2 =  @(x) x.^2 ./ I(x); 

% Numerical Integration
int_a = integral(fun1,0,L);
int_b = integral(fun2,0,L);

% Equivalent force due to impact
P = ( 2 * E * impactEnergy / (cos(phi)^2 * int_a + sin(phi)^2 * int_b) )^0.5;

% Vectorizing the Length and Diameter
dx = 1000;
x = linspace(0,L,dx);
d = linspace(do_min,do_max,dx);

% Minimum I for buckling
Imin = min(I(x));

% Computing force due to the weight of the landing gear
V = sum( A(x) * L / dx);
mass = V * density
Fg = mass * g;

% Total Force
F = R + Fg + P;
% Components of the total force
F_u = F * cos(phi); % axial
F_v = F * sin(phi); % bending

% Stress at different points along the shaft
stress = F_u ./ A(x) + F_v .* x .* d/2 ./ I(x);

% Finding the max Stress
maxStress = max(stress)

% Computing the safety factor
n = alwStress / maxStress

% Plots
figure(1)
plot(x,stress,'r')
xlabel('Location on Shaft (m)')
ylabel('Stress (Pa)')
title('Varying Stress over Length of Landing Gear')


%% Third Design Iteration
% Hollow ellipse with changing cross section

% Parameters
a_min = 0.2*2; % m, smallest diameter in x direction
a_max = 0.21*2; % m, largest diameter in x direction
b_min = 0.28*2; % m, smallest diameter in y direction
b_max = 0.5*2; % m, largest diamter in y direction
t =     0.07; % m, thickness of the landing gear
phi =   5 * pi / 180; % rad, Angle between the landing gear and the vertical axis

% Functions for the changing cross section area
ao =    @(x) a_min + (a_max - a_min) * x/L;
ai =    @(x) a_min - 2*t + (a_max - a_min) * x/L;
bo =    @(x) b_min + (b_max - b_min) * x/L;
bi =    @(x) b_min - 2*t + (b_max - b_min) * x/L;
A =     @(x) pi / 4 * (ao(x).*bo(x) - ai(x).*bi(x));
I =     @(x) pi / 64 * (ao(x).*bo(x).^3 - ai(x).*bi(x).^3);
fun1 =  @(x) 1 ./ A(x); 
fun2 =  @(x) x.^2 ./ I(x); 

% Numerical Integration
int_a = integral(fun1,0,L);
int_b = integral(fun2,0,L);

% Equivalent force due to impact
P = ( 2 * E * impactEnergy / (cos(phi)^2 * int_a + sin(phi)^2 * int_b) )^0.5;

% Vectorizing Length and Diameter
dx = 1000;
x = linspace(0,L,dx);
d = linspace(b_min,b_max,dx);

% Minimum I for buckling
I_min = min(I(0));

% Computing force due to the weight of the landing gear
V = sum( A(x) * L / dx);
mass = V * density
Fg = mass * g;

% Total Force
F = R + Fg + P;

% Components of the total force
F_u = F * cos(phi); % axial
F_v = F * sin(phi); % bending

% Stress at different points along the shaft
stress = F_u ./ A(x) + F_v .* x .* d/2 ./ I(x);

% Finding the max Stress
maxStress = max(stress)

% Computing the safety factor
n = alwStress / maxStress

% Plots
figure(2)
plot(x,stress,'g')
xlabel('Location on Shaft (m)')
ylabel('Stress (Pa)')
title('Varying Stress over Length of Landing Gear')

% Code to plot ellipse
% figure(3) % Ellipse plot
% loc = linspace(0,2*pi);
% % Ellipse at the very end
% % outer ellipse
% x1=a_max*cos(loc);
% y1=b_max*sin(loc);
% % inner ellipse
% x2=(a_max-2*t)*cos(loc);
% y2=(b_max-2*t)*sin(loc);
% plot(x1,y1,'r',x2,y2,'r')
% axis([-b_max b_max -b_max b_max])