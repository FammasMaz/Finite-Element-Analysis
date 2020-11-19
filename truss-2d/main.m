function [uxy, strain, stress, axialload] = main(data)
% Description:-
%  - Solves the plane truss problems
%  - Uses the data provided in data.m
% Input:-
%  - Takes file data.m which provides the truss data
% Ouput:-
%  - Nodal Displacement
%  - Element Strains
%  - Element Stresses
%  - Axial Force in Each Element

% Initializes dataset in memory
run(data);

% Caclucates necessary parameters
nElems = size(eProp,1);
nNodes = size(nXY,1);
motion_axes = 2; % Axes where element can deform in
DOF = motion_axes * nNodes; % Degree of Freedom of Body

% Initialization of required empty matrices
k = zeros(2); % Stiffness matrix of Element
K = zeros(DOF); % Stiffness matrix of System
u = F = zeros(DOF,1); % Global Displacement and Force vectors
axialload = stress = strain = l = zeros(nElems,1); % Output Matrices

% Looping of all elements  for calculate stiffness matrices
for e=1:nElems
  % Defines the eProp variables for element e
  i = eProp(e, 1);
  j = eProp(e, 2);
  m = eProp(e, 3);

  % Coordinates of the first node
  xi = nXY(i, 1);
  yi = nXY(i, 2);
  % Coordinates of the second node
  xj = nXY(j, 1);
  yj = nXY(j, 2);

  % Length of the vector, i.e. the length of the element e
  l(e) = sqrt((xj-xi)^2 + (yj-yi)^2);

  % Necessary Rotation Matrix element calculations
  c = (xj - xi)/l(e);
  s = (yj - yi)/l(e);

  % Defining Material props for each element
  E = mater(m,1);
  A = mater(m,2);

  % Stiffness Matrix for each element
  k = A*E/l(e) * [
  c*c s*c
  c*s s*s
  ];

  % Assembly Matrix by putting k into K
  n = 2*i - 1;
  p = 2*j - 1;
  K(n:n+1, n:n+1) = K(n:n+1, n:n+1) + k;
  K(n:n+1, p:p+1) = K(n:n+1, p:p+1) - k;
  K(p:p+1, n:n+1) = K(p:p+1, n:n+1) - k;
  K(p:p+1, p:p+1) = K(p:p+1, p:p+1) + k;
endfor

% Loads in Global Coordinate system

for n = 1:size(loads, 1)
  F(2*loads(n,1)-1) = loads(n, 2); % X-Coordinate
    F(2*loads(n,1)) = loads(n, 3); % Y-Coordinate
endfor

% finding row to thin-down matrix, considering boundary conditions
% and removing matrix singularity

bcrem = zeros(DOF, 1);

for n = 1:size(bc, 1)
  bcnode = bc(n,1);
  bcrem(2*bc(n,1)-1:2*bc(n,1)) = bc(n,2:3);

endfor

rmK = K(~bcrem, ~bcrem); % Removing the corresponding rows and columns of bcrem
newF = F(~bcrem); % Removing the corresponding rows of bcrem


% Matrix Solution to [K]{u} = {F}
U = rmK\newF;

% Rentering the previosuly removed nodal data
j = 1;

for i = 1:size(u, 1)
  if bcrem(i) == 1
    u(i) = 0; % 0 strain because of fixed support
  else
    u(i) = U(j); % value from U as no fixed support
    j = j+1;
  endif
endfor

for e=1:nElems
  % Defines the eProp variables for element e
  i = eProp(e, 1);
  j = eProp(e, 2);
  m = eProp(e, 3);

  % Coordinates of the first node
  xi = nXY(i, 1);
  yi = nXY(i, 2);
  % Coordinates of the second node
  xj = nXY(j, 1);
  yj = nXY(j, 2);

  % Length of the vector, i.e. the length of the element e
  l(e) = sqrt((xj-xi)^2 + (yj-yi)^2);

  % Necessary Rotation Matrix element calculations
  c = (xj - xi)/l(e);
  s = (yj - yi)/l(e);

  % Strain, Stress and Axial Load calculations from theoretical formulae
  strain(e) = (((c * u(j*2-1) + s * u(j*2))) - ((c * u(i*2-1) + s * u(i*2)))) / l(e);
  stress(e) = mater(m, 1) * strain(e);
  axialload(e) = mater(m, 2) * stress(e);

endfor

% Recreating the strain matrix containing x and y displacements in a single row

for i=1:DOF/2
   uxy(i,1) = u(2*i-1);
   uxy(i,2) = u(2*i);
endfor

endfunction
