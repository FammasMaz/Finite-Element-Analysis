%This file provides the dataset required in the main.m file


% The coordinates of nodes
% [x, y] positions, row represents node number
nXY = [
  0 0
  6 0
  3 3
  0 3
  6 3
  0 6
  6 6
  3 6
  3 9
  0 9
  6 9
  3 12
  0 12
  6 12
  3 15
];

% [node_i, node_j, material_prop], row represents truss element number
eProp = [
1 2 1
1 3 1
2 3 1
2 5 1
1 4 1
3 4 1
3 5 1
5 8 1
4 8 1
5 7 1
6 4 1
6 8 1
8 7 1
6 9 1
7 9 1
6 10 1
7 11 1
9 10 1
9 11 1
10 12 1
11 12 1
10 13 1
11 14 1
12 13 1
12 14 1
14 15 1
13 15 1
];


% Boundary conditions on any given trusses
% [node_id, x_axis_condition, y_axis_condition] where 1 = fixed; 0 = free
bc = [
1 1 1
2 0 1
];

% Material Properties and Area of the Cross_sec area of the elements
% [mod_of_elasticity, cross_area], where each row is a different material
mater = [
200e9 6.25e-3
];

% Specifies any loads on any nodes
% [node_id, Fx, Fy]
loads = [
15 500000 0
];
