function Setup_Dir_DeformableReg()
fprintf('Setting up all the directories ..')
RVCDIR = getenv('RVCDIR'); % http://petercorke.com/Robotics_Toolbox.html
CPD2DIR = getenv('CPD2DIR'); % This is from Seth
ARMADIR = getenv('ARMA_CL');
setenv('CPDREG',pwd);
CPDREGDIR= getenv('CPDREG');
restoredefaultpath;
addpath(genpath(RVCDIR),...
    genpath(CPDREGDIR), ...
    genpath(CPD2DIR),...
    genpath(ARMADIR));
fprintf('..[ok]\n');
end