% MAINNAVEEN Script to compute IBE for the basis function sizes defined by
% the variable n_basisRange. The computed bases are stored in a file

n_basisRange = [5 10 15 20 25];%[20 25 30];%[5,10,15,20,15,30,40,50];%,75,100,150,200,250];

for td = 1%:0.5:2.5%2.5
    for n_basis = n_basisRange
        figure;

        clearvars -except n_basis n_basisRange td
        %% Parameters
       % n_basis = 10;
        tau = td;
        dt = 0.001;
        u = @(t) sin(2 * pi * t) + cos(3 * 2 * pi * t + 1);

        %% Process
        responses = IBE(n_basis, dt,tau);
        Tester(responses, u, tau);

        PsiStar = responses;
        tOut = linspace(0,tau,1/dt);

        %% Save
        if(tau>1)
            save(sprintf('./Data/DMPPaper/DMPBasis/psiStar_N%d_td%d.mat',n_basis,floor(100*tau)), 'PsiStar','tOut');
        else
            save(sprintf('./Data/DMPPaper/DMPBasis/psiStar_N%d.mat',n_basis,floor(100*tau)), 'PsiStar','tOut');
        end
    end
end