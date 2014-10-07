function [ ResponseProfiles ] = IBE( NBasis, Timestep, tau )
%IBE Summary of this function goes here
%   Detailed explanation goes here

global rcps;

%% Initial values
dt         = Timestep;
%tau        = tau;
periods    = 3;
n_rfs      = NBasis;
ID         = 1;
g          = 0;
a          = 1;

%% Convergence check: y_conv and g_conv values
y_conv = 0.001;
g_conv = 0.001;

%% Create rdmp
rcp('init', ID, n_rfs, 'rcp_test_w_i');
rcp('reset_state', ID);
rcp('set_baseline', ID, g);
rcp('set_amplitude', ID, a);

%% Test the effect of w_i, and extract responses
figure, hold on;
cmap = colormap(hot);

for i = 1 : n_rfs + 2
    rcp('reset_state', ID);
    rcps(ID).w = zeros(n_rfs, 1);
    
    if(i <= n_rfs)
        rcps(ID).w(i) = 1;
    elseif(i == n_rfs + 1)
        rcps(ID).y = y_conv;
    else
        rcp('set_baseline', ID, g_conv);
    end
    
    y = [];
    for t = 0 : dt : periods * tau
        [y_t, yd_t, ydd_t] = rcp('run', ID, tau, dt);
        y = [y; y_t];
    end
    plot(0 : dt: periods * tau, y, 'Color', cmap(i * floor(64 / (n_rfs + 2)), :), 'LineWidth', 3);
    
    if(i <= n_rfs)
        response(i, :) = y(end - tau / dt + 1 : end) ./ max(y);
    elseif(i == n_rfs + 1)
        response(i, :) = y(1 : tau / dt) ./ y_conv;
    else
        response(i, :) = y(1 : tau / dt) ./ g_conv;
    end
end

legend(num2str((1 : n_rfs + 2)'), 'Location', 'NorthEastOutside');
title('Solutions');

%% 
figure, plot(response');
legend(num2str((1 : n_rfs + 2)'), 'Location', 'NorthEastOutside');
title('Normalized Responses');

%% 
ResponseProfiles = response;

end