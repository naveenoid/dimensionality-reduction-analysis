function [redSys , redOrder ] = modred_BT(inputSys,threshold,col)
%     ABmat = [inputSys.A inputSys.B];
    
%     [U,S,V] = svd(ABmat);    
%     sEle = diag(S);    
%     cumsEle = cumsum(sEle)./sum(sEle);
%     redOrder = sum(cumsEle<threshold);
%     
%     plot(cumsEle,'bo-'); hold on;
%     plot(1:size(cumsEle),threshold*ones(size(cumsEle)),'r');
%     redSysComposite = U(1:redOrder,1:redOrder)*S(1:redOrder,:)*V';
%     redSys=inputSys;
%     redSys.A = redSysComposite(:,1:redOrder);
%     redSys.B = redSysComposite(:,redOrder+1:end);
    
    
    inpSys = ss(inputSys.A,inputSys.B,inputSys.C,inputSys.D);
    hank = hsvd(inpSys);
    normHank = cumsum(hank)./sum(hank);
    
    
    if (exist('col','var') ~= 1)
        col = 'b';
    end
    
    plot(normHank,[col,'o-']); hold on;
    plot(1:size(normHank),threshold*ones(size(normHank)),'k');     
    
    redOrder = sum(normHank<threshold);
    
    opt = balredOptions('StateElimMethod','Truncate');
        
    redSysSS = balred(ss(inpSys), redOrder,opt);
    
    redSys.A = redSysSS.a;
    redSys.B = redSysSS.b;
    redSys.C = redSysSS.c;
    redSys.D = redSysSS.d;
   
end