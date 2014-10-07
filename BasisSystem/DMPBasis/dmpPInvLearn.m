function [ What, DMPopt ] = dmpPInvLearn( trainingTSpan, PsiStar, inputu,smoothening, DMPopt )
%DMPPINVLEARN Summary of this function goes here
%   Detailed explanation goes here

    DMPopt.psiStar = PsiStar;
    response = dmpDataSys(trainingTSpan, DMPopt);
    
     resp = response(1 : end - 2, :);
     What = inputu(trainingTSpan')' * resp' * inv(resp * resp');
     What = [What, zeros(size(What,1),2)];
    
    %What = inputu(trainingTSpan')'*pinv(dmpDataSys(trainingTSpan,DMPopt));
    DMPopt.What = What;
    
   % What = DMPparamExtractor(DMPopt);
   % What(:,1:DMPopt.N) = What(:,1:DMPopt.N)./DMPopt.rescale;
end

