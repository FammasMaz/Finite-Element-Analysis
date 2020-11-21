%This file provides the dataset required in the main.m file


% The coordinates of nodes
% [x, y, z] positions, row represents node number
nodalCord = [
5 0 0
0 0 0
0 -3 0
5 -3 0
0 -1.5 0.9144
5 -1.5 0.9144
];

% [node_i, node_j,  material_prop], row represents truss element number
elem_prop = [
2 1 1
1 5 1
5 2 1
2 3 1
3 5 1
5 6 1
6 3 1
3 4 1
4 6 1
6 1 1
1 4 1
1 3 1
];


% Boundary conditions on any given trusses
% [node_id, x_axis_condition, y_axis_condition] where 1 = fixed; 0 = free
bound_con = [
1 0 1 0
2 1 0 1
5 1 0 0
4 1 0 1
];

% Material Properties and Area of the Cross_sec area of the elements
% [mod_of_elasticity, cross_area], where each row is a different material
mat_prop = [
200e9 0.02581
];

% Specifies any loads on any nodes
% [node_id, Fx, Fy]
loadings = [
6 2000 -3000 -4000
];
