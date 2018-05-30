function start_matlab_ros()
%% ros shutdown
try
    rosshutdown();
catch
end

%% ros Init
try
    rosinit();
catch
    warning('Using existing ros node');
end

end