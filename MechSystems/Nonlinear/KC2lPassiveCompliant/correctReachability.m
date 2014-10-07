function [pOut] = correctReachability(p, opt)
    pOut = p;
    
    %Rho = 0.999999*opt.phys.lengthLink1  + 0.999999*opt.phys.lengthLink2;
    Rho = 0.999999999*(opt.phys.lengthLink1  + opt.phys.lengthLink2);
    normP = (p(:,1).^2 + p(:,2).^2).^0.5;
    maxPts  = find(normP>Rho);
    
 %   if(normP > Rho)
    Phi = atan2(p(maxPts,2),p(maxPts,1));
    pOut(maxPts,1) = Rho*cos(Phi);
    pOut(maxPts,2) = Rho*sin(Phi);
    
%     fprintf('In:\n');
%     disp(p);
%     
%     fprintf('Out:\n');
%     disp(pOut);
   % fprintf('(%2.2f,%2.2f) Pts corrected (%2.2f,%2.2f)', p(1),p(2),pOut(1),pOut(2));
end