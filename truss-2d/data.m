%This file provides the dataset required in the main.m file


% The coordinates of nodes
% [x, y, z] positions, row represents node number
nXYZ = [
  0 60 0 % z free
  0 0 24
  0 0 -24
  96 30 0
];

% [node_i, node_j,  material_prop], row represents truss element number
eProp = [
  1 4 1
  1 2 0
  1 3 0
  2 3 0
  2 4 0
  3 4 0
];


% Boundary conditions on any given trusses
% [node_id, x_axis_condition, y_axis_condition] where 1 = fixed; 0 = free
bc = [
1 1 1 1
2 1 1 1
3 1 1 1
];

% Material Properties and Area of the Cross_sec area of the elements
% [mod_of_elasticity, cross_area], where each row is a different material
mater = [
29e6 0.5
11e6 1
];

% Specifies any loads on any nodes
% [node_id, Fx, Fy]
loads = [
4 0 -1500 0
];
