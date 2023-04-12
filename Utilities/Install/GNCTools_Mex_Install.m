function GNCTools_Mex_Install()
%% Mex Install File

%%
%mex -setup
clc
% fprintf(2,'Please Specify Visual Studio 2019 as your Compiler...\n\n');
% mex -setup

%%
%Ensure you are in the GNCTools directory!
fprintf('\n------------------------------------------------\n');
fprintf('GNCTools MEX FILE INSTALL\n\n');


% Files to Compile
files{1} = {'mxCPID','Control/PID','cpid.c'};
files{2} = {'mxPID','Control/PID',{'pid.cpp','cpid.c'}};
files{3} = {'vc19MexCheck','Utilities/Install'};

% Additional Source
postSrc = ' Common/Source/mexHelpers.cpp';
postInc = ' -I.';

% Compile 1 by 1
for i = 1:length(files)
    mexName = files{i}{1};
    srcLoc = files{i}{2};
    addSrc = [];
    if (length(files{i}) > 2)
        addSrc = files{i}{3};
        if (~iscell(addSrc))
            addSrc = {addSrc};
        end
    end
    % Check for Source directory
    oldSrcLoc = srcLoc;
    if (exist([cd filesep srcLoc filesep 'Source'],'dir'))
        srcLoc = [srcLoc filesep 'Source']; %#ok<AGROW>
    end

    try    
        fprintf('Compiling "%s.cpp"... ',mexName);
        clear(mexName);
        pre = sprintf('mex %s/%s.cpp',srcLoc,mexName);
        for j = 1:length(addSrc)
            pre = sprintf('%s %s/%s',pre,srcLoc,addSrc{j});
        end
        eval([pre postSrc postInc]);
        fprintf('Done!\n\n');
        movefile([mexName '.' mexext],oldSrcLoc,'f') 
    catch ME
        error('gnctools:mex','Error Compiling GNCTools MEX File!\n%s',ME.message);
    end
end

fprintf('------------------------------------------------\n');

