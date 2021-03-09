% downloaded from 
%https://www.mathworks.com/matlabcentral/fileexchange/29104-get-points-for-a-line-between-2-points
function [xx,yy]=straightLine(start,ending,pts)
        m=(ending(2)-start(2))/(ending(1)-start(1)); %gradient 
        if abs(m)==Inf %vertical line
            xx(1:pts)=start(1);
            yy(1:pts)=linspace(start(2),ending(2),pts);
        elseif abs(m)==0 %horizontal line
            xx(1:pts)=linspace(start(1),ending(1),pts);
            yy(1:pts)=start(2);
        else %if (ending(1)-start(1))~=0
            xx=linspace(start(1),ending(1),pts);
            yy=m*(xx-start(1))+start(2);
        end
end
