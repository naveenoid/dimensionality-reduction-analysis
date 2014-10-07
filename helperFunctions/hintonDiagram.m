function hintonDiagram(w,faceCol, max_m,min_m)
% Display a matrix W where the square's area represents W(i,j)
% and red = negative, green = positive
%
%  Examples
%
%    W = randn(4,5); hintonDiagram(W)

% Based on hintonw function from Mathworks neural net toolbox

if nargin < 1,error('Not enough input arguments.');end
if nargin < 3, max_m = max(max(abs(w))); end
if nargin < 4, min_m = max_m / 100; end

%if nargin == 4, col = faceCol; else col = 'k';end

if(exist('faceCol','var')==0)
    col = 'k';
else
    col = faceCol;
end

if max_m == min_m, max_m = 1; min_m = 0; end
% DEFINE BOX EDGES
xn1 = [-1 -1 +1]*0.5;
xn2 = [+1 +1 -1]*0.5;
yn1 = [+1 -1 -1]*0.5;
yn2 = [-1 +1 +1]*0.5;

t = 0:0.1:2*pi;

% DEFINE POSITIVE BOX
xn = 0.5*cos(t);
yn = 0.5*sin(t);

% DEFINE POSITIVE BOX
xp = [xn 0.5*cos(t)];
yp = [yn 0.5*sin(t)];

[S,R] = size(w);

cla reset
hold on
set(gca,'xlim',[0 R]+0.5);
set(gca,'ylim',[0 S]+0.5);
set(gca,'xlimmode','manual');
set(gca,'ylimmode','manual');
xticks = get(gca,'xtick');
%set(gca,'xtick',xticks(find(xticks == floor(xticks))))

set(gca,'xtick', 1:size(w,2))%xticks(find(xticks == floor(xticks))))
yticks = get(gca,'ytick');
set(gca,'ytick',yticks(find(yticks == floor(yticks))))
set(gca,'ydir','reverse');
if get(0,'screendepth') > 1
  %set(gca,'color',[1 1 1]*.5);
  %set(gcf,'color',[1 1 1]*.3);
end

for i=1:S
  for j=1:R
    m = sqrt((abs(w(i,j))-min_m)/max_m);
    m = min(m,max_m)*0.95;
    if real(m)
      if w(i,j) >= 0
         h = fill(xn*m+j,yn*m+i,[0.7 0.7 0.7]); hold on;
         set(h,'edgecolor',col,'LineWidth',3.0);
%         plot(xn1*m+j,yn1*m+i,'w',xn2*m+j,yn2*m+i,'k')
   %     plot(m+j,m+i,'w',m+j,m+i,'k')
        %plot(m+j,m+i,'w','LineWidth',2.0); hold on;
        
        plot(m+j,m+i,'Color',col,'LineWidth',2.0)
      elseif w(i,j) < 0
        h = fill(xn*m+j,yn*m+i,[1 1 1]);hold on;
        set(h,'edgecolor',col,'LineWidth',3.0);
       % fill(xn*m+j,yn*m+i,col);hold on;
%         plot(xn1*m+j,yn1*m+i,'k',xn2*m+j,yn2*m+i,'w');
         
          %plot(m+j,m+i,'k',m+j,m+i,'w');
         plot(m+j,m+i,'Color',col,'LineWidth',2.0); hold on; 
         %plot(m+j,m+i,'w','LineWidth',2.0);
         %fprintf('faceCol :');  disp(col);
      end
    end
  end
end

set(gca,'FontSize',14);

plot([0 R R 0 0]+0.5,[0 0 S S 0]+0.5,'w');
%xlabel('Input');
%ylabel('Neuron');
grid on
a = axis;
axis([a(1)-0.15,a(2)+0.15,a(3),a(4)]);
