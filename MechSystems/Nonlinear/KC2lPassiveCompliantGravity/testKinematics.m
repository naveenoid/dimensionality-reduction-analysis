function [rhoOut, phiOut] = testKinematics(rho, phi, opt)

    p = correctReachability(rho*[cos(phi);sin(phi)]',opt);
    a = iKin(p,'down',opt);
   
    pOut = fkin(a(1),a(2),opt);
    fprintf('Point : (%1.2f,%1.2f)\n',pOut(3),pOut(4));
    
    phiOut = atan2(pOut(4),pOut(3));
    rhoOut = (pOut(3).^2 + pOut(4).^2).^0.5;
    fprintf('Rho : %3.2f, Phi : %2.2f\n',rhoOut,phiOut);
end
