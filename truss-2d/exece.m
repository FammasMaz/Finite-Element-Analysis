function exece(data, uxy)
  % Description:-
  %  - Plots the truss obtained in previous function
  %  - Plots the deflected version
  % Input:-
  %  - Takes file data.m which provides the truss data
  %  - uxy obtained in previous function
  % Ouput:-
  %  - Plots of both original and deflected truss
run(data);
nElems = size(eProp,1);
nNodes = size(nXY,1);

adjac  = zeros(nNodes); % adjacenecy matrix

for e = 1:nElems
    i = eProp(e,1); j = eProp(e,2); % i, j nodes for e element
    adjac(i,j) = 1; adjac(j,i) = 1;
end

dXY = nXY + 10*uxy; % Coordinates of the deformed matrix


% Plotting both the matrices
gplot(adjac, nXY, "r--");
hold on;
gplot(adjac, dXY, "b");
endfunction
