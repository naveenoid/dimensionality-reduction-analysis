function [score, redOrder , redSys ] = evaluateDimCost_BTHistogramPlot(DStar,numBasis,inputSys,threshold,redOrder,figNum,col,algoChoice)
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
   
	if(size(DStar,2) == 1) 
		%Its organised as a column, presumably for optimisation, so 		reshape
		 DPrime = (reshape(DStar,[numBasis+1,size(inputSys.B,2)]))';
	else
		DPrime = DStar;
	end

    inpSys = ss(inputSys.A,inputSys.B*DPrime,inputSys.C,inputSys.D);
    hank = hsvd(inpSys);
     

    
    if (sum(isnan(hank))==0 && sum(isinf(hank))==0 && size(hank,1)>0)
        normHank = cumsum(hank)./sum(hank);
    
    
        if (exist('col','var') ~= 1 || length(col) == 0)
            col = 'b';
 %          plot(normHank,[col,'o-']); hold on;
 %   plot(1:size(normHank),threshold*ones(size(normHank)),'k');     
        end

	if(~isempty(figNum))
		%figureNum = 5;	
		if(figNum ~=0)
            figure(figNum);
            clf;
                bar(normHank,0.5); hold on;
                plot(normHank,[col,'-o']); hold on;
                %plot(1:size(normHank),threshold*ones(size(normHank)),'k');     
                a = get(gca);
		if(exist('OCTAVE_VERSION')~=0) 
			line(a.xlim',[threshold,threshold]','Color','k','LineWidth',2);	                	
		else
			line(a.XLim',[threshold,threshold]','Color','k','LineWidth',2);
		end
        end
    end

		%disp(redOrder);
	 if (length(redOrder) == 0)
	        redOrder = sum(normHank<threshold);
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
            score = interp1(newHank,1:size(newHank),threshold);%redOrder + vectTemp(redOrder);
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
