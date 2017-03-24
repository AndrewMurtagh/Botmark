%{
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% 
%   Copyright (C) 2017 Trinity Robotics Group.
%
%
%   FILENAME:   occupancygridmapping.m 
%
%
%   DATE: 20/02/2017
%
%
%   DESCRIPTION: program to compute a path by probabilistic roadmap method.
%
%
%   AUTHOR: Andrew Murtagh, 
%           Trinity Robotics Group, Trinity College Dublin.
%
%
%   NOTES:  -map representation is a b/w image.
%           -uses A* to search graph.
%
%
%   VERSION: v1
%
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%}

close all; clear; clc;



minX = -35; 
maxX = 82;
minY = -60; 
maxY = 77;
grid_size = 1500;
alpha = .2;
prob_prior = 0;
prob_occ = 2;
prob_free = -2;
grid_inc_size = 0.1;


grid = zeros(grid_size, grid_size);
mapx = minX:grid_inc_size:(minX+(grid_inc_size*grid_size)-grid_inc_size);
mapy = minY:grid_inc_size:(minY+(grid_inc_size*grid_size)-grid_inc_size);


data = importdata('../Data/visualise_ogm.csv');
disp('Finished reading data');

data = data(3:end,:);
maxdist = 5.6; 
angles = (-120 : 360/1024 : 120) *pi/180; 
max_axis=[-3 maxdist*1.1 -maxdist*1.1 maxdist*1.1];


figure;
for p=1:size(data,1) 
    disp(strcat(['Processing line ', num2str(p), ' of ', num2str(size(data,1))]));
    
    if(p==1000), break; end
    ranges = data(p, 2:end);
    
    
    x = data(p,2);
    y = data(p,3);
    theta = data(p,4);
    angle_min = angles(1);
    angle_max = angles(683);
    angle_inc = (360/1024)*(pi/180);
    range_min = 0;
    range_max = maxdist;
    
    beta = angle_inc;
    
    
    min_angle_range = theta + angle_min;
    max_angle_range = theta + angle_max;
    
    
    % for each cell
    for i = 1:grid_size 
        for j = 1:grid_size 
            
            
         angle2 = atan2(mapy(i) - y, mapx(j) - x);
         
            
            %if within perceptual range
            if (angle2 > min_angle_range && angle2 < max_angle_range)
                
                
                r = sqrt((x - mapx(j))^2 + (y - mapy(i))^2);
                phi = angle2 - theta;
                
                
                k = abs(round((phi - angle_min)/angle_inc));
                %for matlab only
                if (k == 0)
                    k = 1;
                end
                
                if (ranges(k) > range_max || ranges(k) < range_min)
                    %don't update
                elseif (r > range_max || r < range_min)
                    %don't update                    
                elseif (r > min(range_max, ranges(k) + alpha/2) || abs(phi - k*angle_inc - angle_min) > beta/2)
                    %don't update                
                elseif (ranges(k) < range_max && abs(r - ranges(k)) < alpha/2)
                   grid(i,j) = grid(i,j) + prob_occ - prob_prior;
                elseif (r <= ranges(k))
                   grid(i,j) = grid(i,j) + prob_free - prob_prior;
                end
                
            end
            %in perception range
            
            
            
        end
    end
    
end




final_grid = zeros(grid_size, grid_size);

%convert from log odds
for m = 1:grid_size
    for n = 1:grid_size
        final_grid(m,n) = 1 - (1/(1 + exp(grid(m,n))));
    end
end

final_grid = flipdim(final_grid,1);
imagesc(mapx, mapy, final_grid);
xlabel('x');
ylabel('y');

%{
writeToFile = false;
if writeToFile
    dlmwrite('mapFromMATLAB.txt',final_grid,'delimiter',' ','precision',3)
end
%}




