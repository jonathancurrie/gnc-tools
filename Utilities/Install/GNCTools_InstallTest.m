function GNCTools_InstallTest
% GNCTools Installation Test File

%Set default ok
ok = 1;

fprintf('\nChecking GNCTools Installation:\n');

%% TEST 1 - Check Main Paths
fprintf('Checking Paths...                ');
paths = {'Control','Utilities'};
len = length(paths);
for i = 1:len
    pass = exist(paths{i},'dir');
    if(~pass)
        fprintf('\nFailed Path Check on %s\n',paths{i});
        ok = 0;
        return
    end
end
fprintf('Ok\n');