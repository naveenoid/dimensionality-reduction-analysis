% TESTNDDISCRETE Script to test if the discrete DMP basis functions extracted
% by IBE can be used to fit points chosen by the createInteractiveSpline
% procedure.

global dcps;

%% Initial values
dimensions = 4;
dt         = 0.001; %0.001
goals      = [0 0 0 0];
tau        = 1;
n_rfs      = 20; %5
IDs        = [1 2 3 4];

%% Initialize some variables for plotting
Z=zeros(floor(tau/dt+1),2, dimensions);
X=Z;
V=Z;
T=zeros(floor(tau/dt+1),3, dimensions);
Y=T;
PSI=zeros(floor(tau/dt+1),n_rfs, dimensions);
W=PSI;

%% Create dimensions
for dim = 1:dimensions
    dcp('clear',IDs(dim));
    dcp('init',IDs(dim),n_rfs, ['DCP_Dim' num2str(IDs(dim))], 0);
end

%% generate a test trajectory
for dim = 1:dimensions
    sampleCount = tau/dt+1;
    [t, y] = CreateInteractiveSpline(sampleCount, [0 tau], [-10 10], [], [0 0]);
    %[t, y] = CreateInteractivePCHIP(sampleCount, [0 tau], [-10 10], [], []);
    %t = linspace(0, 1, sampleCount);
    %y = 2 + t.^3 + t.^4; %1-exp(-1*t).*cos(1*t);
    tDemo(:, dim) = t - t(1);
    yDemo(:, dim) = y - y(1);
    goals(dim) = yDemo(end, dim);
    T(:,1, dim) = yDemo(:, dim);
    T(:,2, dim) = gradient(yDemo(:, dim), dt);
    T(:,3, dim) = gradient(gradient(yDemo(:, dim), dt), dt);
end

%% use the inbuilt function to fit the dcp
for dim = 1:dimensions
    dcp('Batch_Fit',IDs(dim),tau,dt, yDemo(:, dim));
end

%% Test the fit
for dim = 1:dimensions
    dcp('reset_state',IDs(dim));
    dcp('set_goal',IDs(dim),goals(dim),1);
end

for i=0:tau/dt
    for dim = 1:dimensions
        [y,yd,ydd]=dcp('run',IDs(dim),tau,dt);

        Z(i+1,:, dim)   = [dcps(IDs(dim)).z dcps(IDs(dim)).zd];
        Y(i+1,:, dim)   = [y yd ydd];
        X(i+1,:, dim)   = [dcps(IDs(dim)).x dcps(IDs(dim)).xd];
        V(i+1,:, dim)   = [dcps(IDs(dim)).v dcps(IDs(dim)).vd];
        PSI(i+1,:, dim) = dcps(IDs(dim)).psi';
        W(i+1,:, dim)   = dcps(IDs(dim)).w';
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
    plot(time,V(:,1, dim));
    title('v');
    aa=axis;
    axis([min(time) max(time) aa(3:4)]);

    subplot(dimensions, 5, 5*(dim-1)+4);
    plot(time,X(:,1, dim));
    title('x');
    aa=axis;
    axis([min(time) max(time) aa(3:4)]);

    subplot(dimensions, 5, 5*(dim-1)+5);
    plot(W(end,:, dim));
    title('Weights');
    xlabel(sprintf('tau=%f',tau));

    drawnow;
end
