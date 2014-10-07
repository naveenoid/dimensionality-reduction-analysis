% TESTNDPERIODIC Script to test if the periodic basis functions extracted
% by IBE can be used to fit points chosen by the createInteractiveSpline
% procedure.

global rcps;

%% Initial values
dimensions = 4;
dt         = 0.001; %0.001
tau        = 1;
n_rfs      = 25; %20
IDs        = [1 2 3 4];
anchors    = [0 0 0 0];
As         = [1 1 1 1];


%% Initialize some variables for plotting
T   = zeros(round(tau/dt+1),3, dimensions);
P   = zeros(size(T,1),2, dimensions);
Z   = zeros(size(T,1),2, dimensions);
Y   = zeros(size(T,1),3, dimensions);
PSI = zeros(size(T,1),n_rfs, dimensions);
W   = zeros(size(T,1),n_rfs, dimensions);

%% Create dimensions
for dim = 1:dimensions
    rcp('init',IDs(dim),n_rfs, ['RCP_Dim' num2str(IDs(dim))]);
    rcp('reset_state',IDs(dim));
    rcp('set_baseline',IDs(dim),anchors(dim));
    rcp('set_amplitude',IDs(dim),As(dim));
end

%% generate a test trajectory
for dim = 1:dimensions
    sampleCount = tau/dt+1;
    [t, y] = CreateInteractiveSpline(sampleCount, [0 tau], [-10 10], [0 anchors(dim); tau anchors(dim)], [0 0]);
    tDemo(:, dim) = t - t(1);
    yDemo(:, dim) = y;
    T(:,1, dim) = yDemo(:, dim);
    T(:,2, dim) = gradient(yDemo(:, dim), dt);
    T(:,3, dim) = gradient(gradient(yDemo(:, dim), dt), dt);
end

%% use the inbuilt function to fit the rcp
for dim = 1:dimensions
    rcp('Batch_Fit',IDs(dim),tau,dt, yDemo(:, dim));
end

%% Test the fit
% create predicted trajectory
for dim = 1:dimensions
    rcp('reset_state',IDs(dim));
    rcp('set_baseline',IDs(dim), anchors(dim));
    rcp('set_amplitude',IDs(dim), As(dim));
end

for i=0:tau/dt
    for dim = 1:dimensions
        [y,yd,ydd]=rcp('run',IDs(dim),tau,dt);
        P(i+1,:, dim)   = [mod(rcps(IDs(dim)).p,2*pi) rcps(IDs(dim)).pd];
        Z(i+1,:, dim)   = [rcps(IDs(dim)).z rcps(IDs(dim)).zd];
        Y(i+1,:, dim)   = [y yd ydd];
        PSI(i+1,:, dim) = rcps(IDs(dim)).psi';
        W(i+1,:, dim) = rcps(IDs(dim)).w;
    end
end

%% Plotting
time = (0:dt:tau)';

figPos = figure;
figStates = figure;

for dim = 1:dimensions
    % plot position, velocity, acceleration vs. target
    figure(figPos);
    
    subplot(dimensions, 3, 3*(dim-1)+1);
    plot(time,[Y(:,1, dim) T(:,1, dim)]);
    legend({'Output', 'Teacher'}, 'Location', 'NorthWest');
    title('y');
    aa=axis;
    axis([min(time) max(time) aa(3:4)]);

    subplot(dimensions, 3, 3*(dim-1)+2);
    plot(time,[Y(:,2, dim) T(:,2, dim)]);
    title('yd');
    aa=axis;
    axis([min(time) max(time) aa(3:4)]);

    subplot(dimensions, 3, 3*(dim-1)+3);
    plot(time,[Y(:,3, dim) T(:,3, dim)]);
    title('ydd');
    aa=axis;
    axis([min(time) max(time) aa(3:4)]);

    % plot internal states
    figure(figStates);
    
    subplot(dimensions, 5, 5*(dim-1)+1);
    plot(time,Z(:,1, dim));
    title('z');
    aa=axis;
    axis([min(time) max(time) aa(3:4)]);

    subplot(dimensions, 5, 5*(dim-1)+2);
    plot(time,PSI(:, :, dim));
    title('Weighting Kernels');
    aa=axis;
    axis([min(time) max(time) aa(3:4)]);

    subplot(dimensions, 5, 5*(dim-1)+3);
    plot(time,P(:,1, dim));
    title('p');
    aa=axis;
    axis([min(time) max(time) aa(3:4)]);

    subplot(dimensions, 5, 5*(dim-1)+4);
    plot(time,P(:,2, dim));
    title('pd');
    aa=axis;
    axis([min(time) max(time) aa(3:4)]);

    subplot(dimensions, 5, 5*(dim-1)+5);
    plot(W(end,:, dim));
    title('Weights');
    xlabel(sprintf('tau=%f',tau));

    drawnow;
end
