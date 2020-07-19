%replace these variable with values here and comment the above line
F = [10; 10; 30];
point = [12.8; 8.3];

%[newPoint, jacPF_f, jacPF_p] = localToGlobal(F, point);
[newPoint, jacPF_f, jacPF_p] = globalToLocal(F, point);

disp(newPoint);
%disp(jacPF_f);
%disp(jacPF_p);


%F is the new reference frame the point is being transformed to
%p is the point in the global frame 
%new point is the point in frame F
%jacPF_f is the jacobian wrt frame F
%jacPF_point is the jacobian wrt the point 


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


