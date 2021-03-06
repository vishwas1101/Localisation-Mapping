%replace these variable with values here and comment the above line
F = [10; 10; 30];
point = [12.8; 8.3];

%[newPoint, jacPF_f, jacPF_p] = localToGlobal(F, point);
[newPoint, jacPF_f, jacPF_p] = globalToLocal(F, point);

disp(newPoint);
%disp(jacPF_f);
%disp(jacPF_p);


%rFrame is the robot frame
%scanPoint is the range/bearing mesurement


%SLAM Operations

%Direct observation model i.e. to transform a point by scanning it into the
%robot frame and take a measurement
function [scanPoint, jacScanPoint_rFrame, jacScanPoint_point] = directObserve(rFrame, point)

[newPoint, jacPF_rFrame, jacPF_point] = globalToLocal(rFrame, point);

[scanPoint, jacScanPoint_newPoint] = scan(newPoint);

jacScanPoint_rFrame = jacScanPoint_newPoint * jacPF_rFrame;
jacScanPoint_point = jacScanPoint_newPoint * jacPF_point;

end 

%the inverse observation model i.e to tranformed a already scanned point to
%the world frame with the known measurement
function [newPoint, jacPoint_rFrame, jacPoint_scanPoint] = inverseObserve(rFrame, scanPoint)

[point, jacPR_ScanPoint] = inverseObserve(scanPoint);

[newPoint, jacPoint_rFrame, jacNewPoint_point] = localToGlobal(rFrame, point);

jacPoint_scanPoint = jacNewPoint_point * jacPR_ScanPoint;

end

%F is the new reference frame the point is being transformed to
%point is a point in the global frame 
%newPoint is the point in frame F
%jacPF_f is the jacobian wrt frame F
%jacPF_point is the jacobian wrt the point 
%scanPoint is the point scanned by the sensor
%jacScanPoint_point is the jacobian wrt the point 
%jacPoint)scanPoint is the jacobian wrt the scanned point


%Four basic functions i.e they are geomatric functions that will be used to
%perform 2D SLAM. They are the functions that are used to change the frame
%from body frame to world frame or vice versa and scanning a 2D point or
%projecting back the 2D point to the space

%transforming a point in a global frame in a local frame
function [newPoint, jacPF_f, jacPF_point] = globalToLocal(F, point)

refx = F(1);
refy = F(2);
refPoint = [refx; refy];

x = point(1);
y = point(2);
 
angle = F(3);

%transformation matrix...
R = [cos(angle) -sin(angle); sin(angle) cos(angle)];

%new point with the different reference frame...
newPoint = R' * (point - refPoint);

%jacobians
jacPF_point = [...
    [cos(angle), sin(angle)]
    [-sin(angle), cos(angle)]];

jacPF_f = [...
    [-cos(angle), -sin(angle), cos(angle)*(y - refy) - sin(angle)*(x - refx)]
    [sin(angle), -cos(angle), -cos(angle)*(x - refx) - sin(angle)*(y - refy)]];

end

%transforming a point in a local frame in a global frame
function [newPoint, jacPW_f, jacPW_point] = localToGlobal(F,point)

refx = F(1);
refy = F(2);
refPoint = [refx; refy];

x = point(1);
y = point(2);

angle = F(3);

R = [cos(angle) -sin(angle); sin(angle) cos(angle)];

newPoint = R * point + repmat(refPoint, 1, size(point, 2));

jacPW_f = [...
    [1, 0, -y*cos(angle) - x*sin(angle)]
    [0, 1, x*cos(angle) - y*sin(angle)]];

jacPW_point = [...
    [cos(angle), -sin(angle)]
    [sin(angle), cos(angle)]];

end 

%reading 2D point from sensor
function [scanPoint, jacScanPoint_point] = scan(point)

x = point(1);
y = point(2);

scanPoint = [sqrt(x^2 + y^2) ; atan2(y,x)];

jacScanPoint_point = [...
    [x/sqrt(x^2 + y^2), y/sqrt(x^2 +y^2)]
    [-y/(x^2 * ((y^2)/(x^2) + 1)), 1/(x^2 * ((y^2)/(x^2) + 1))]];

end 


%project back the scanned point
function [point, jacPoint_ScanPoint] = project(scanPoint)

x = scanPoint(1) * cos(scanPoint(2));
y = scanPoint(1) * sin(scanPoint(2));

point  = [x; y];

jacPoint_ScanPoint = [...
    [cos(scanPoint(2)), -scanPoint(1)*sin(scanPoint(2))]
    [sin(scanPoint(2)), -scanPoint(1)*cos(scanPoint(2))]];

end 


