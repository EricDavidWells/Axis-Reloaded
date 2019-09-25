function gaitSim(varargin)
if nargin == 0
   radius = 5;
   legs = 5;
else
radius = varargin{1};
legs = varargin{2};
end

f = figure;
ax = axes(f);
lims = radius+(0.2*radius);


% Initialize circle paths
th = 0:pi/150:2*pi;
xs = radius * cos(th);
ys = radius * sin(th);

plot(ax,xs,ys,'k');
hold(ax,'on');
[ax.XLim,ax.YLim] = deal([-lims,lims]);
pLength = floor(length(th)/legs);
paths = [];
pathH = [];
pointH = [];
pathStep=[];
for p=1:legs
    legPath = [xs(pLength*(p-1)+1:(p)*pLength)',ys(pLength*(p-1)+1:(p)*pLength)'];
    % off paths so that 1 is always off - the /4 is hardcoded for 5 legs
%     paths(p,:,:) = circshift(legPath,pLength*(p-1)/4);
paths(p,:,:) = legPath;
    pathStep(p) = floor(pLength*(p-1)/4 + 1);
    pathH(p) = plot(ax,legPath(:,1),legPath(:,2),'g');
    pointH(p) = plot(ax,paths(p,1,1),paths(p,1,2),'bo','markerSize',12);
end

while 1

   for p=1:legs
       if pathStep(p) > pLength
        pathStep(p) = (-pLength/4 + 1);
        set(pointH(p),'visible','off');
        set(pathH(p),'color','r');
       elseif pathStep(p)<=pLength && pathStep(p) >=1
       set(pathH(p),'color','g');
       set(pointH(p), 'XData',paths(p,pathStep(p),1),'YData',paths(p,pathStep(p),2),'visible','on');
       end
       pathStep(p) = pathStep(p)+1;
   end
   drawnow;
   
          pause(0.1);
end
end