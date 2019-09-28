clear all
close all

% Set variables for trajectory
radius = 230;
points = 200;
depth = 30;

% Initialize xy plane circle paths
th = linspace(0, 2*pi, 5*points);
x = radius * cos(th);
y = radius * sin(th);

% Plot data in xy plane and format
figure(1)
sectionA = {'k', 'r', 'g', 'b', 'm'};

hold on
plot(x(1:points), y(1:points), sectionA{1})

for ii = 1:4
    plot(x(ii*points+1:(ii+1)*points), y(ii*points+1:(ii+1)*points), sectionA{ii+1})
end

hold off
axis equal

% Initialize z parabolic trajectory and plot
t = linspace(-points/2, points/2, points);
z = (depth/(points/2)^2)*t.^2 - depth;

figure(2)
plot(t,z)

% Visualize in 3 dimensions
figure(3)
sectionB = {'--k', '--r', '--g', '--b', '--m'};

hold on
plot3(x(1:points), y(1:points), zeros(1,200), sectionA{1})
plot3(x(1:points), y(1:points), z, sectionB{1})

for ii = 1:4
  plot3(x(ii*points+1:(ii+1)*points), y(ii*points+1:(ii+1)*points), zeros(1,200), sectionA{ii+1})
  plot3(x(ii*points+1:(ii+1)*points), y(ii*points+1:(ii+1)*points), z, sectionB{ii+1})
end

hold off
view([55, 25])
axis equal
grid on
