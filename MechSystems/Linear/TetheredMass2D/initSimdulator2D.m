function [opt,fName] = initSimdulator(void)

% Init Simdulator

opt.n = 4;  % no. of inputs

ringDistance = 1; 
k =0.1.*ones(1,2); % soft spring in center mass
b = 1.*ones(1,2);
m = 1;
ang0 = (0:(opt.n-1)).*(pi/(opt.n/2)); %uniformly distributed at n*pi/2 
mount = ringDistance.*[cos(ang0); sin(ang0)];

ki = 1.*ones(1,opt.n);
tau = 1.*ones(1,opt.n);

disp('Initialising plant settings');

A11 = [zeros(2,2)  eye(2) ; -(1/m)*diag(k)  -(1/m)*diag(b)];
A12 = [zeros(2, opt.n); ki.*cos(ang0); ki.*sin(ang0)]; 
A21 = zeros(opt.n);
A22 = -diag(tau);

opt.A = [A11 A12 ; A21 A22];
opt.B = [zeros(opt.n) ; diag(1./tau)];
opt.C = [eye(4) zeros(opt.n)]; %position and velocity as output
%opt.C = [zeros(2,8)]; %position as output
opt.D = [zeros(size(opt.C,1),size(opt.B,2))];


%opt.controlFunc = @(t,y)[0.5;0.5;zeros(size(opt.B,2)-2,1)];

%yIni = zeros(size(opt.A,1),1);

fName = './data/simdulator2D.mat';

end

