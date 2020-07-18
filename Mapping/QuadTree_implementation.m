%HELP TAKEN FROM THE BOOK OF "WHEELED MOBILE ROBOTICS" 

%GENERATING RANDOM ENVIRONMENT WITH OBTACLE - START
noObs = 10; %enter the number of obstacles 
minDimObs = [0.1; 0.1]; %enter min dimension of obstacle (xmin, ymin)
maxDimObs = [2; 2]; %enter max dimension of obstacle (xmax,ymax)
envDim = [0,16,0,12]; % enter the environement dimension(xmin,xmax,ymin,ymax) 

%THE OBSTACLES IN THIS PROGRAM ARE ASSUMED TO BE RECTANGULER
%so they will have 4 vertices, so 8 points x1, y1, x2, y2, x3, y3, x4, y4
obstacle = zeros(8,noObs);

%getting random obtacles map along with its vertices
for i = 1:noObs
    
    pos = [envDim(1); envDim(3)] + [envDim(2)-envDim(1); envDim(4)-envDim(3)].*rand(2,1);
    %disp(rand(2,1));
    %disp(pos);
    dim = minDimObs/2 + (maxDimObs-minDimObs)/2*rand();
    %disp(dim);
    theta = 2*pi*rand();
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)]; %rotation matrix
    v = repmat(pos, 1, 4) + R*([-1, 1, 1, -1; -1, -1, 1, 1].*repmat(dim, 1, 4)); 
    obstacle(:,i) = reshape(v, [], 1);
    
end

%disp(obstacle);

%IMPROVEMENT: THE OBSTACLE MIGHT OVERLAP (ALTHOUGH IT DOESNT CAUSE A
%PROBLEM) IT RENDERS THE EXECUTION LESS EFFECIENT.

%a region quadtree with a depth of n may be used to 
%represent an image consisting of 2^n × 2^n pixels.
%where each pixel value is obstacle or not (0 or 1)

%GENERATING RANDOM ENVIRONMENT WITH OBSTACLE - END

%now to create quadtree of depth level 4
tree = quadTree(obstacle, envDim, 4);

%now to generate quadtree of certain depth,
%around randomly placed obstacles
%in an environment

function tree = quadTree(obstacle, envDim, level)    

    minDimSq = [(envDim(2)-envDim(1)); (envDim(4)-envDim(3))]/(2^level);
    %disp(minDimSq);
    
    %ROOT NODE 
    tree(1).leaf = true;
    tree(1).free = false;
    tree(1).bounds = envDim;
    tree(1).centre = [(envDim(2)-envDim(1))/2; (envDim(4)-envDim(3))/2];
    
    node = 1;
    next = 2;
    
    while node < next
        occupied = isOccupied(tree(node).bounds, obstacle); %to check if its occupied
        d = [(tree(node).bounds(2)-tree(node).bounds(1)), (tree(node).bounds(4)-tree(node).bounds(3))]/2;
        %disp(d);
        
        if occupied && d(1) > minDimSq(1)/2  %split cells into four
            tree(node).leaf = false;
            tree(node).free = false;
            
            b = tree(node).bounds;
            
            bs = [b(1), b(1)+d(1), b(3), b(3)+d(2);...
                  b(1)+d(1), b(2), b(3), b(3)+d(2);...
                  b(1), b(1)+d(1), b(3)+d(2), b(4);...
                  b(1)+d(1), b(2), b(3)+d(2), b(4)];
              
            for i = 1:4
                tree(next).leaf = true;
                tree(next).free = false;
                tree(next).bounds = bs(i,:);
                tree(next).center = [mean(bs(i,1:2)); mean(bs(i,3:4))];
                next = next+1;    
            end
        elseif ~occupied
            tree(node).free = true;
        end
        node = node+1;
    end 
    
    %create visibility graph
    a = zeros(2, length(tree)*4);  
    leafs = zeros(1, length(tree));
    for i = 1:length(tree)
        a(:, i*4 - 3:i*4) = tree(i).bounds([1, 2, 2, 1;3, 3, 4, 4]);
        leafs(i) = tree(i).leaf;
    end 
    
    offset = [-1, 1, 1, -1; -1, -1, 1, 1]/2.*repmat(minDimSq, 1, 4);
    
    for i = 1:length(tree)
        tree(i).neighbours = [];
        if tree(i).leaf
            b = tree(i).bounds([1, 2, 2, 1; 3, 3, 4, 4]) + offset;
            c = reshape(inpolygon(a(1,:), a(2,:), b(1,:), b(2,:)), 4, []);
            tree(i).neighbours = setdiff(find(any(c).*leafs), i);
        end 
    end 
end 
        
  
function occupied = isOccupied(bounds, obstacle)
    occupied = false;
    pb = bounds([1, 2, 2, 1, 1; 3, 3, 4, 4, 3]);
    for j = 1:size(obstacle, 2)
        pa = reshape(obstacle(:,j), 2, []);
        N = size(pa, 2);
        ina = inpolygon(pa(1,:), pa(2,:), pb(1, :), pb(2, :));
        inb = inpolygon(pb(1,:), pb(2,:), pa(1, :), pa(2, :));
        if any(ina) || any(inb) % are any vertices in the obstacle or cell
            occupied = true;
            break;
        else %check if there are any intersecting edges 
            for k = 1:size(pb, 2)-1 %loop through the boundary edges
                for i = 1:N % loop through the obstacle edges
                    a1 = [pa(:, i); 1];
                    a2 = [pa(:,mod(i, N)+1); 1];
                    b1 = [pb(:,k); 1];
                    b2 = [pb(:,k+1); 1];
                    pc = cross(cross(a1, a2), cross(b1, b2));%intersection
                    if abs(pc(3)) > eps
                        pc = pc/pc(3);
                        da = a2-a1;
                        ca = pc-a1;
                        ea = (ca.'*da)/(da.'*da);
                        db = b2-b1;
                        cb = pc-b1;
                        eb = (cb.'*db)/(db.'*db);
                        if eb>eps && eb<1 && ea>eps && ea<1
                            occupied = true;
                            break;
                        end 
                    end 
                end 
                if occupied
                    break;
                end 
            end 
        end 
        if occupied
            break;
        end 
    end 
end 
        
                        
                        
                    

