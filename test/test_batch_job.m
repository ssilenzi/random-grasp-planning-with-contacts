%% TO RUN THE TREE EXPANSION AS A BATCH JOB %%

clc; clear all;

% Script to be run
script_name = 'test_contact_based_grasp_rrt';

% Running the script as a batch job
job_script = batch(script_name);

% Loading the job variables
load(job_script);