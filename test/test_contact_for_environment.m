%% For testing hand functions for starting and ik

close all;
clear;
clc;
run(fullfile('..', 'tools', 'resolve_paths.m'))

% Load the environment and the box (both initial and final poses)
% run('book_on_shelf_no_target.m')
% run('book_on_table.m')
run('book_on_shelf_no_other_books.m')
% run('book_on_table_cluttered_no_target.m')
% run('book_on_box_corner_no_target.m')

axis([-10 10 -15 15 -15 15]); % Change the axis and view
view(50, 30);
legend off;

% Get object position as row
Co = box_object.T(1:3,4).';

% Get contacts with the environment and plot
[Cp, Cn] = get_contacts(environment, box_object, box_object.T);
% plot_contacts(Cp,Cn);
