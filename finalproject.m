%	Example 1.3-1 Paper Airplane Flight Path
%	Copyright 2005 by Robert Stengel
%	August 23, 2005

	global CL CD S m g rho	
	S		=	0.017;			% Reference Area, m^2
	AR		=	0.86;			% Wing Aspect Ratio
	e		=	0.9;			% Oswald Efficiency Factor;
	m		=	0.003;			% Mass, kg
	g		=	9.8;			% Gravitational acceleration, m/s^2
	rho		=	1.225;			% Air density at Sea Level, kg/m^3	
	CLa		=	3.141592 * AR/(1 + sqrt(1 + (AR / 2)^2));
							% Lift-Coefficient Slope, per rad
	CDo		=	0.02;			% Zero-Lift Drag Coefficient
	epsilon	=	1 / (3.141592 * e * AR);% Induced Drag Factor	
	CL		=	sqrt(CDo / epsilon);	% CL for Maximum Lift/Drag Ratio
	CD		=	CDo + epsilon * CL^2;	% Corresponding CD
	LDmax	=	CL / CD;			% Maximum Lift/Drag Ratio
	Gam		=	-atan(1 / LDmax);	% Corresponding Flight Path Angle, rad
	V		=	sqrt(2 * m * g /(rho * S * (CL * cos(Gam) - CD * sin(Gam))));
							% Corresponding Velocity, m/s
	Alpha	=	CL / CLa;			% Corresponding Angle of Attack, rad

%	a) Equilibrium Glide at Maximum Lift/Drag Ratio
	H		=	2;			% Initial Height, m
	R		=	0;			% Initial Range, m
	to		=	0;			% Initial Time, sec
	tf		=	6;			% Final Time, sec
	tspan	=	[to tf];
	xo		=	[V;Gam;H;R];
	[ta,xa]	=	ode23('EqMotion',tspan,xo);

%	b) Oscillating Glide due to Zero Initial Flight Path Angle
	xo		=	[V;0;H;R];
	[tb,xb]	=	ode23('EqMotion',tspan,xo);

%	c) Effect of Increased Initial Velocity
	xo		=	[1.5*V;0;H;R];
	[tc,xc]	=	ode23('EqMotion',tspan,xo);

%	d) Effect of Further Increase in Initial Velocity
	xo		=	[3 * V;0;H;R];
	[td,xd]	=	ode23('EqMotion',tspan,xo);

	figure
	plot(xa(:,4),xa(:,3),xb(:,4),xb(:,3),xc(:,4),xc(:,3),xd(:,4),xd(:,3))
	xlabel('Range, m'), ylabel('Height, m'), grid

	figure
	subplot(2,2,1)
	plot(ta,xa(:,1),tb,xb(:,1),tc,xc(:,1),td,xd(:,1))
	xlabel('Time, s'), ylabel('Velocity, m/s'), grid
	subplot(2,2,2)
	plot(ta,xa(:,2),tb,xb(:,2),tc,xc(:,2),td,xd(:,2))
	xlabel('Time, s'), ylabel('Flight Path Angle, rad'), grid
	subplot(2,2,3)
	plot(ta,xa(:,3),tb,xb(:,3),tc,xc(:,3),td,xd(:,3))
	xlabel('Time, s'), ylabel('Altitude, m'), grid
	subplot(2,2,4)
	plot(ta,xa(:,4),tb,xb(:,4),tc,xc(:,4),td,xd(:,4))
	xlabel('Time, s'), ylabel('Range, m'), grid

rad = @(d)d * pi / 180;
% Initial parameters
initVelocity = [3.55 - 2, 3.55, 3.55 + 7.5];      % velocities in m/s
initAngles = [rad(-0.18 - 0.5), rad(-0.18), rad(-0.18 + 0.4)];       % angles in radians

% Store results
results = cell(3, 2);      

% Loop for varying initial velocities
for i = 1:3
    xo = [initVelocity(i); Gam; H; R];      % initial states
    [t, x] = ode23(@(t, x) EqMotion(t, x), tspan, xo);
    results{i, 1} = [t, x];
end

% Loop for varying initial flight path angles
for j = 1:3
    xo = [V; initAngles(j); H; R];      % initial states
    [t, x] = ode23(@(t, x) EqMotion(t, x), tspan, xo);
    results{j, 2} = [t, x];
end

% Plotting the results
figure;
subplot(2,1,1);      % velocity variations
hold on;
for k = 1:3
    plot(results{k, 1}(:,2), results{k, 1}(:,3), 'DisplayName', sprintf('V = %.2f m/s', initVelocity(k)));
end
title('Height vs. Range for Different Initial Velocities');
xlabel('Range(m)');
ylabel('Height(m)');
legend show;
hold off;

subplot(2,1,2);      % Angle variations
hold on;
for k = 1:3
    plot(results{k, 2}(:,2), results{k, 2}(:,3), 'DisplayName', sprintf('Gamma = %.2f rad', initAngles(k)));
end
title('Height vs. Range for Different Flight Path Angles');
xlabel('Range(m)');
ylabel('Height(m)');
legend show;
hold off;

% Define all parameters
m = 0.003;
S = 0.017;
rho = 1.225;
g = 9.8;
CL = sqrt(CDo / epsilon);
CD = CDo + epsilon * CL^2;
H = 2;          
R = 0;          
to = 0;         
tf = 6;         
tspan = linspace(to, tf, 100);      % more points give smoother plots

% Define ranges for velocity and flight path angle
minVel = 3.55 - 2;
maxVel = 3.55 + 7.5;
minAngle = deg2rad(-0.18 - 0.5);
maxAngle = deg2rad(-0.18 + 0.4);

% Preallocation for efficiency
trajectories = zeros(length(tspan), 4, 100);

% Run 100 simulations with random initial conditions
figure;
hold on;
for i = 1:100
    % Random initial conditions 
    initVelocity = minVel + (maxVel - minVel) * rand(1);
    initAngle = minAngle + (maxAngle - minAngle) * rand(1);
    
    % Define initial state vector
    V = sqrt(2 * m * g / (rho * S * (CL * cos(initAngle) - CD * sin(initAngle))));
    xo = [V; initAngle; H; R];
    33;
    
    % Solve the equations of motion
    [t, x] = ode23(@(t, x) EqMotion(t, x), tspan, xo);
    
    % Store results
    trajectories(:,:,i) = interp1(t, x, tspan);      
    
    % Plot
    plot(x(:,4), x(:,3), 'Color', [0.7 0.7 0.7]);      
end
xlabel('Range(m)');
ylabel('Height(m)');
title('Flight Trajectories with Random Initial Conditions');
grid on;
hold off;

% Assuming trajectories are stored in a 100x4xlength(tspan) array
Time = repmat(tspan', [1, 100]);      % Time repeated for each simulation
Range = squeeze(trajectories(:,4,:));      % Extracting range data
Height = squeeze(trajectories(:,3,:));      % Extracting height data

% Flatten arrays to fit polynomials
TimeFlat = Time(:);
RangeFlat = Range(:);
HeightFlat = Height(:);

% Fit polynomials to the data
% Choose degree of polynomial according to data
degRange = 3; 
degHeight = 3; 
pRange = polyfit(TimeFlat, RangeFlat, degRange);
pHeight = polyfit(TimeFlat, HeightFlat, degHeight);

% Generate fit data for plotting
fitRange = polyval(pRange, tspan);
fitHeight = polyval(pHeight, tspan);

% Plot the fit curves
figure;
subplot(2,1,1);
plot(tspan, fitRange, 'r-', 'LineWidth', 3);
title('Fit Polynomial for Range');
xlabel('Time(s)');
ylabel('Range(m)');

subplot(2,1,2);
plot(tspan, fitHeight, 'b-', 'LineWidth', 3);
title('Fitted Polynomial for Height');
xlabel('Time(s)');
ylabel('Height(m)');

% Compute derivatives of the polynomial coefficients
pRangeDer = polyder(pRange); 
pHeightDer = polyder(pHeight);

% Evaluate the derivatives over tspan
rangeRate = polyval(pRangeDer, tspan);
heightRate = polyval(pHeightDer, tspan);

% Plot the derivatives
figure;
subplot(2,1,1);
plot(tspan, rangeRate, 'r-', 'LineWidth', 3);
title('Rate of Change of Range');
xlabel('Time(s)');
ylabel('dRange/dTime (m/s)');
grid on;

subplot(2,1,2);
plot(tspan, heightRate, 'b-', 'LineWidth', 3);
title('Rate of Change of Height');
xlabel('Time(s)');
ylabel('dHeight/dTime (m/s)');
grid on;