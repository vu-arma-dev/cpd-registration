function Setup_Dir_DeformableReg()
fprintf('Setting up all the directories ..')
RVCDIR = getenv('RVCDIR'); % http://petercorke.com/Robotics_Toolbox.html
CPD2DIR = getenv('CPD2DIR'); % This is from Seth
UDPREGJHUDIR = getenv('UDPREGJHU'); % UdpRegistration JHU folder
UDPREGVUDIR = getenv('UDPREGVU'); % UdpRegistration VU folder
restoredefaultpath;
addpath(genpath(RVCDIR),...
    genpath(UDPREGJHUDIR),...
    genpath(UDPREGVUDIR),...
    genpath(CPD2DIR));
fprintf('..[ok]\n');
end