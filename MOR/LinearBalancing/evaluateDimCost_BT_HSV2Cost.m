function [score, redOrder , redSys, hank, normHank ] = evaluateDimCost_BT_HSV2Cost(WHat,mechOpt,basisOpt,threshold,redOrder,figNum,col,algoChoice)
%function [score, redOrder, redSys, hsv, normHsv, Wc3, Wo3 ] = evaluateDimCost_NBT( WHat,tspan,mechSystem, mechOpt, basisOpt, threshold, cm,redOrder )

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
    
    %reordering DStar
    %Dprime maps the old set of inputs - i.e. the actuators to the new
    %inputs i.e. the basis functions
   inputSys = mechOpt;
   numBasis = basisOpt.order;
    
	if size(WHat,2) == 1 %% &&   mod(size(WHat,1),2) == 0   
		%Its organised as a column, presumably for optimisation, so 		reshape
		 %DPrime = (reshape(WHat,[numBasis+1,size(inputSys.B,2)]))';
         DPrime = (reshape(WHat,[basisOpt.order+1,size(mechOpt.B,2)]))';
	else
		DPrime = WHat;
	end

    inpSys = ss(inputSys.A,inputSys.B*DPrime,inputSys.C,inputSys.D*DPrime);
    hank = hsvd(inpSys);
%      
%     fprintf('modB');
%     disp(inputSys.B*DPrime);

    
    if (sum(isnan(hank))==0 && sum(isinf(hank))==0 && size(hank,1)>0)
        normHank = cumsum(hank)./sum(hank);
    
    
        if (exist('col','var') ~= 1 || length(col) == 0)
            col = 'b';
 %          plot(normHank,[col,'o-']); hold on;
 %   plot(1:size(normHank),threshold*ones(size(normHank)),'k');     
        end
% 
% 	if(~isempty(figNum))
% 		%figureNum = 5;	
% 		if(figNum ~=0)
%             figure(figNum);
%             clf;
%                 plot(normHank,[col,'o-']); hold on;
%                 plot(1:size(normHank),threshold*ones(size(normHank)),'k');     
% 	
%         end
%     end

		%disp(redOrder);
	 if (length(redOrder) == 0)
	        redOrder = sum(normHank<threshold);
         %   redOrder = round(interp1(newHank,1:size(newHank),threshold));
		%disp(redOrder);
	end
   % vectTemp = normHank ./ threshold;

	%normHank
	%hank

        %checking for multiple states with HSV 1
        unitHSVPos = find(normHank>=1);
        if(isempty(unitHSVPos) == 1)
            newHank = normHank;            
        else
            newHank = normHank(1:unitHSVPos(1));
        end
        
        if(sum(isnan(newHank)) <1)
            %score = interp1(newHank,1:size(newHank),threshold);%redOrder + vectTemp(redOrder);
            
            redOrder = round(interp1(newHank,1:size(newHank),threshold));
            score =100*(1-newHank(2));
%             if( score < 1e-4) 
%                 score = 0; 
%             end
        else
            score = length(normHank);
        end    

%change back
	%redOrder = 2;



	if(exist('OCTAVE_VERSION')~=0) 
% Octave
		if(algoChoice == 1)
			redSysSS = btamodred(inpSys, redOrder);
		else
			redSysSS = spamodred(inpSys, redOrder);
		end
		%redSysSS = btamodred(inpSys, redOrder);
		%redSysSS = inpSys;
		%isnan(hank)
		%isinf(hank)
		%hank

	else
	%Matlab
	        opt = balredOptions('StateElimMethod','Truncate');

        	redSysSS = balred(inpSys, redOrder,opt);
	end

        redSys.A = redSysSS.a;
        redSys.B = redSysSS.b;
        redSys.C = redSysSS.c;
        redSys.D = redSysSS.d;
    else
        
        dispText = sprintf('Cannot Reduce, Hankel Values have %d NaN and %d Inf out of total %d',sum(isnan(hank)),sum(isinf(hank)),numel(hank));
        disp(dispText);
        
        redOrder = numel(hank);
        score = redOrder;
        redSys = inputSys;
        redSys.B = inputSys.B*DPrime;
    end
