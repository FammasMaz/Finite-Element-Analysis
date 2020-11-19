function [u_spat,u,U, strain, stress, K, axForce] = main(data)
% Description:-
%  - Solves the plane truss problems, automatically judging whether the problem
% is 3D or 2D
%  - Uses the data provided in data.m
% Input:-
%  - Takes file data.m which provides the truss data
% Ouput:-
%  - Nodal Displacement
%  - Element Strains
%  - Element Stresses
%  - Axial Force in Each Element

% Initializes dataset from inout file into memory
run(data);

% Caclucates necessary parameters

elem_num = size(elem_prop,1); % Total number of elements in the truss
nodes_num = size(nodalCord,1); % Total number of nodes in containing the truss

if size(nodalCord, 2) == 2
  motion_axes = 2; # 2D truss
elseif size(nodalCord, 2) == 3
  motion_axes = 3; # 3D truss
else
  fprintf("Please Check Your Nodal Coordinate Matrix")
  return;
endif

DOF = motion_axes * nodes_num; % Calculated Degrees of Freedom of the System


% The code uses the standard [K]{u} = {F} solution method after finding the K
% and u. Through matrix multipication laws, K must have a dimension of (DOFxDOF)
% while u must have (DOFx1), resultantly, the force should have the dimension of
% (DOFx1)

% To make things easier, we are initializing the empty matrices to be filled by
% incoming loops.

% Initialization of required empty matrices
ke = zeros(motion_axes); % Stiffness matrix of each element
K = zeros(DOF); % Stiffness matrix of System
u = F = zeros(DOF,1); % Global Displacement and Force vectors
l = axForce = stress = strain = zeros(elem_num,1); % Output Matrices

% Looping of all elements  for calculate stiffness matrices
for i=1:elem_num
  % Defines the nodal variables for element e
  node_a = elem_prop(i, 1);
  node_b = elem_prop(i, 2);
  struct_prop = elem_prop(i, 3); % Material properties row tag

  % Coordinates of the first node
  x_node_a = nodalCord(node_a, 1)
  y_node_a = nodalCord(node_a, 2)
  if motion_axes == 3
    z_node_a = nodalCord(node_a, 3)
  endif
  % Coordinates of the second node
  x_node_b = nodalCord(node_b, 1)
  y_node_b = nodalCord(node_b, 2)
  if motion_axes == 3
    z_node_b = nodalCord(node_b, 3)
  endif

  if motion_axes ==3
    % Length of the vector, i.e. the length of the element e
    l(i) = sqrt(((x_node_b-x_node_a)^2) + ((y_node_b-y_node_a)^2) + ((z_node_b-z_node_a)^2));
  else
    l(i) = sqrt((x_node_b-x_node_a)^2 + (y_node_b-y_node_a)^2);
  endif

  % Necessary Rotation Matrix element calculations
  c = (x_node_b - x_node_a)/l(i); % cos(theta_x)
  s = (y_node_b - y_node_a)/l(i); % cos(theta_y)
  if motion_axes == 3
    v = (z_node_b - z_node_a)/l(i); % cos(theta_z)
  endif

  % Defining Material props for each element
  E = mat_prop(struct_prop,1); % Mod of Elasticity
  A = mat_prop(struct_prop,2); % Cross-Sectional Area of the Member

  if motion_axes == 3

    % Stiffness Matrix for each element
    ke = A*E/l(i) * [
      c*c c*s c*v
      s*c s*s s*v
      v*c v*s v*v
    ];
  else
    % Stiffness Matrix for each element
    ke = A*E/l(i) * [
      c*c c*s
      s*c s*s
    ];
  endif

  % Assembly Matrix by putting k into K
    n = motion_axes*node_a - (motion_axes-1);
    p = motion_axes*node_b - (motion_axes-1);
    K(n:n+(motion_axes-1), n:n+(motion_axes-1)) = K(n:n+(motion_axes-1), n:n+(motion_axes-1)) + ke;
    K(n:n+(motion_axes-1), p:p+(motion_axes-1)) = K(n:n+(motion_axes-1), p:p+(motion_axes-1)) - ke;
    K(p:p+(motion_axes-1), n:n+(motion_axes-1)) = K(p:p+(motion_axes-1), n:n+(motion_axes-1)) - ke;
    K(p:p+(motion_axes-1), p:p+(motion_axes-1)) = K(p:p+(motion_axes-1), p:p+(motion_axes-1)) + ke;
endfor

% Loads Calculated in Global Coordinate system

for i = 1:size(loadings, 1)
  F(motion_axes*loadings(i,1)-(motion_axes-1)) = loadings(i, 2); % X-Coordinate
  F(motion_axes*loadings(i,1)-(motion_axes-2)) = loadings(i, 3); % Y-Coordinate
  if motion_axes ==3
    F(motion_axes*loadings(i,1)) = loadings(i, 4) % Z-Coordinate
  endif
endfor

% finding row to thin-down matrix, considering boundary conditions
% and removing matrix singularity

bcrem = zeros(DOF, 1); % Temporary matrix to find nodes with boundary condition

for n = 1:size(bound_con, 1)
  bcnode = bound_con(n,1);
  bcrem((motion_axes*bound_con(n,1)-(motion_axes-1)):motion_axes*bound_con(n,1)) = bound_con(n,2:(motion_axes+1));

endfor

rmK = K(~bcrem, ~bcrem); % Removing the corresponding rows and columns of bcrem
newF = F(~bcrem); % Removing the corresponding rows of bcrem


% Matrix Solution to [K]{u} = {F}
U = rmK\newF;

% Rentering the previosuly removed nodal data
x = 1;

for i = 1:size(u, 1)
  if bcrem(i) == 1
    u(i) = 0; % 0 strain because of fixed support
  else
    u(i) = U(x); % value from U as no fixed support
    x = x+1;
  endif
endfor

for i=1:elem_num
  % Defines the elem_prop variables for element e
  node_a = elem_prop(i, 1);
  node_b = elem_prop(i, 2);
  struct_prop = elem_prop(i, 3);

  % Coordinates of the first node
  x_node_a = nodalCord(node_a, 1);
  y_node_a = nodalCord(node_a, 2);
  if motion_axes == 3
    z_node_a = nodalCord(node_a, 3);
  endif
  % Coordinates of the second node
  x_node_b = nodalCord(node_b, 1);
  y_node_b = nodalCord(node_b, 2);
  if motion_axes == 3
    z_node_b = nodalCord(node_b, 3);
  endif

  if motion_axes ==3
    % Length of the vector, i.e. the length of the element e
    l(i) = sqrt((x_node_b-x_node_a)^2 + (y_node_b-y_node_a)^2 + (z_node_b-z_node_a)^2);
  else
    l(i) = sqrt((x_node_b-x_node_a)^2 + (y_node_b-y_node_a)^2);
  endif

  % Necessary Rotation Matrix element calculations
  c = (x_node_b - x_node_a)/l(i);
  s = (y_node_b - y_node_a)/l(i);
  if motion_axes == 3
    v = (z_node_b - z_node_a)/l(i);
  endif
  if motion_axes == 3
    strain(i) = ((c * u(node_b*3-2)) + (s * u(node_b*3-1)) + (v*u(node_b*3)) - (c * u(node_a*3-2)) - (s * u(node_a*3-1)) - (v*u(node_a*3))) / l(i);
  else
    strain(i) = ((c * u(node_b*2-1)) + (s * u(node_b*2)) - (c * u(node_a*2-1)) - (s * u(node_a*2))) / l(i);
  endif
  % Strain, Stress and Axial Load calculations from theoretical formulae
  stress(i) = mat_prop(struct_prop, 1) * strain(i);
  axForce(i) = mat_prop(struct_prop, 2) * stress(i);

endfor

% Recreating the strain matrix containing x and y displacements in a single row

for i=1:DOF/3
   u_spat(i,1) = u(motion_axes*i-(motion_axes-1));
   u_spat(i,2) = u(motion_axes*i-(motion_axes-2));
   if motion_axes == 3
     u_spat(i,3) = u(3*i)
   endif
endfor

endfunction
