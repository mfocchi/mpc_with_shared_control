function h = plotOrientation(location,  theta, scaling)
    
     Rz = [cos(theta)    -sin(theta) 
           sin(theta)   cos(theta)];
     location= location(:);
     
     %get X axis
     vec = Rz(:,1)*scaling;
     tip = location + vec;
     line([location(1) tip(1)], [location(2) tip(2)], 'linewidth',5);
     %quiver(location(1), location(2), vec(1), vec(2),'k', 'linewidth',1);
     
end