function W = wrench(U,m,u)
     %P{i}:接触点在质心坐标系下的坐标
    P{1} = [cos(U(1)) sin(U(1)) U(2)]';           %
    P{2} = [cos(U(3)) sin(U(3)) U(4)]';          
    P{3} = [cos(U(5)) sin(U(5)) U(6)]';           %
    P{4} = [cos(U(7)) sin(U(7)) U(8)]';
    P{5} = [cos(U(9)) sin(U(9)) U(10)]';
    P{6} = [cos(U(11)) sin(U(11)) U(12)]';
    P{7} = [cos(U(13)) sin(U(13)) U(14)]';
    P{8} = [cos(U(15)) sin(U(15)) U(16)]';
    %TP{i}：物体坐标系到接触点坐标系旋转矩阵
    TP{1} = [-sin(U(1)) 0 -cos(U(1));cos(U(1)) 0 -sin(U(1));0 -1 0];       %
    TP{2} = [-sin(U(3)) 0 -cos(U(3));cos(U(3)) 0 -sin(U(3));0 -1 0];      
    TP{3} = [-sin(U(5)) 0 -cos(U(5));cos(U(5)) 0 -sin(U(5));0 -1 0];       %
    TP{4} = [-sin(U(7)) 0 -cos(U(7));cos(U(7)) 0 -sin(U(7));0 -1 0]; 
    TP{5} = [-sin(U(9)) 0 -cos(U(9));cos(U(9)) 0 -sin(U(9));0 -1 0]; 
    TP{6} = [-sin(U(11)) 0 -cos(U(11));cos(U(11)) 0 -sin(U(11));0 -1 0]; 
    TP{7} = [-sin(U(13)) 0 -cos(U(13));cos(U(13)) 0 -sin(U(13));0 -1 0]; 
    TP{8} = [-sin(U(15)) 0 -cos(U(15));cos(U(15)) 0 -sin(U(15));0 -1 0]; 
    
    d = {m};
    %d{j}：接触力在接触点坐标系下的表示

    for j = 1:m
        d{j} = [u*cos(2*j*pi/m) u*sin(2*j*pi/m) 1]';                           
    end
    dP = ones(3,m*3);
    W = ones(6,m*3);
    for i = 1:8
        for j = 1:m
            dP(:,(i-1)*m+j) = TP{i}*d{j};                                          %接触力在质心坐标系下的表示
            W(:,(i-1)*m+j) = [dP(:,(i-1)*m+j) ; cross(P{i},dP(:,(i-1)*m+j))];    %接触力螺旋在质心坐标系下的表示
        end
    end
    
end
    