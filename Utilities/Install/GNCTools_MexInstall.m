function GNCTools_MexInstall()
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
files{1} = {{'mxPID','mxCPID'},'Control/PID','cpid.c','CPID'};
files{2} = {'mxPID','Control/PID',{'pid.cpp','cpid.c'}};
files{3} = {{'mxFilter','mxCFilter'},'Control/Filter','cfilter.c','CFILTER'};
files{4} = {'mxFilter','Control/Filter',{'filter.cpp','cfilter.c'}};
files{5} = {{'mxSOSFilter','mxCSOSFilter'},'Control/Filter','csosFilter.c','CSOSFILTER'};
files{6} = {'mxSOSFilter','Control/Filter',{'sosFilter.cpp','csosFilter.c'}};
files{end+1} = {'vc19MexCheck','Utilities/Install'};

% Additional Source
postSrc = ' Common/Source/mexHelpers.cpp';
postInc = ' -I.';

% Compile 1 by 1
for i = 1:length(files)
    mexName = files{i}{1};
    if (iscell(mexName))
        mexInName = mexName{1};
        mexOutName = mexName{2};
    else
        mexInName = mexName;
        mexOutName = mexName;
    end
    srcLoc = files{i}{2};
    addSrc = [];
    pp = [];
    if (length(files{i}) > 2)
        addSrc = files{i}{3};
        if (~iscell(addSrc))
            addSrc = {addSrc};
        end
    end
    if (length(files{i}) > 3)
        pp = files{i}{4};
        if (~iscell(pp))
            pp = {pp};
        end
    end
    % Check for Source directory
    oldSrcLoc = srcLoc;
    if (exist([cd filesep srcLoc filesep 'Source'],'dir'))
        srcLoc = [srcLoc filesep 'Source']; %#ok<AGROW>
    end
    % Build preprocessor defines
    ppDef = [];
    for j = 1:length(pp)
        ppDef = sprintf(' %s -D%s',ppDef,pp{j});
    end
    try    
        fprintf('Compiling "%s.cpp"... ',mexOutName);
        clear(mexInName);
        clear(mexOutName);
        pre = sprintf('mex %s/%s.cpp',srcLoc,mexInName);
        for j = 1:length(addSrc)
            pre = sprintf('%s %s/%s',pre,srcLoc,addSrc{j});
        end
        eval([pre postSrc postInc ppDef]);
        fprintf('Done!\n\n');
        movefile([mexInName '.' mexext],[oldSrcLoc filesep mexOutName '.' mexext],'f') 
    catch ME
        error('gnctools:mex','Error Compiling GNCTools MEX File!\n%s',ME.message);
    end
end

fprintf('------------------------------------------------\n');

