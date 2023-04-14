function GNCTools_Install(savePath,runTests,openBrowser)
% GNCTools Toolbox Installation File
%
%  GNCTools_Install(savePath, runTests, openBrowser)
%
%   savePath: Save the paths added by GNCTools to the MATLAB path 
%   runTests: Run the post-installation tests 
%   openBrowser: Whether to open the GNCTools Wiki after installation 
%
% All arguments are optional and if not supplied, the user will be prompted
% to enter their selection in the MATLAB Command Window. True is the
% default option for each argument.
%
% You MUST be in the current directory of this file!
%
%   Copyright (C) 2023 Jonathan Currie (Control Engineering)
%   https://controlengineering.co.nz/Wikis/GNC/

% Handle missing input args
if (nargin < 3), openBrowser = []; end
if (nargin < 2), runTests = []; end
if (nargin < 1), savePath = []; end

cpath = cd;
try
    cd('Utilities/gnctools');
catch %#ok<CTCH>
    error('You don''t appear to be in the GNCTools directory');
end
%Get current versions    
localVer = gnctoolsver();

fprintf('\n------------------------------------------------\n')
fprintf(['  INSTALLING GNCTools ver ' sprintf('%1.2f',localVer) '\n'])

%Perform pre-req check
cd(cpath);
if(~preReqChecks(cpath))
    return;
end

%Uninstall previous versions of GNCTools
fprintf('\n- Checking for previous versions of GNCTools...\n');
no = GNCTools_Uninstall('GNCTools_Install.m',0);
if(no < 1)
    fprintf('Could not find a previous installation of GNCTools\n');
else
    fprintf('Successfully uninstalled previous version(s) of GNCTools\n');
end

%Add toolbox path to MATLAB
fprintf('\n- Adding GNCTools Paths to MATLAB Search Path...');
genp = genpath(cd);
genp = regexp(genp,';','split');
%Folders to exclude from adding to Matlab path
i = 1;
rInd{:,:,i} = strfind(genp,'Diagrams'); i = i + 1;
rInd{:,:,i} = strfind(genp,'Documentation'); i = i + 1;
rInd{:,:,i} = strfind(genp,'Source'); i = i + 1;
rInd{:,:,i} = strfind(genp,'Tests'); i = i + 1;
rInd{:,:,i} = strfind(genp,['Utilities' filesep 'Install']); i = i + 1;
rInd{:,:,i} = strfind(genp,'slprj'); i = i + 1;
rInd{:,:,i} = strfind(genp,'.vscode'); i = i + 1;
rInd{:,:,i} = strfind(genp,'.git'); 

ind = NaN(length(rInd{1}),1);
%Track indices of paths to remove from list
for i = 1:length(rInd{1})
    for j = 1:size(rInd,3)
        if(any(rInd{j}{i}))
            ind(i) = 1;
        end
    end
end

%Remove paths from above and add to matlab path
genp(ind == 1) = [];
addpath(genp{:});
rehash
fprintf('Done\n\n');
if (isempty(savePath))
    in = input('- Would You Like To Save the Path Changes? (Recommended) (y/n): ','s');
else
    in = bool2yn(savePath);
end
if(strcmpi(in,'y'))
    try
        savepath;
    catch %#ok<CTCH>
        warning('gnctools:install',['It appears you do not have administrator rights on your computer to save the Matlab path. '...
                                'In order to run GNCTools you will need to install it each time you wish to use it. To fix '...
                                'this please contact your system administrator to obtain administrator rights.']);
    end
end

%Post Install Test if requested
if (isempty(runTests))
    in = input('\n- Would You Like To Run Post Installation Tests? (Recommended) (y/n): ','s');
else
    in = bool2yn(runTests);
end
if(strcmpi(in,'y'))
    try
        cd('Utilities/Install');
        GNCTools_InstallTest;
    catch ME
        cd(cpath);
        rethrow(ME);
    end
    cd(cpath);
end

%Launch Examples page
if (isempty(openBrowser) || (openBrowser == true))
    web('https://www.controlengineering.co.nz/Wikis/GNC/pmwiki.php/Main/HomePage','-browser');
end

%Finished
fprintf('\n\nGNCTools Installation Complete!\n');
disp('------------------------------------------------')



function no = GNCTools_Uninstall(token,del)
no = 0;
%Check nargin in, default don't delete and gnctools mode
if(nargin < 2 || isempty(del))
    del = 0;
end

%Check if we have anything to remove
paths = which(token,'-all');
len = length(paths);
%If mode is gnctools, should always be at least 1 if we are in correct directory
if(~len)
    error('Expected to find "%s" in the current directory - please ensure you are in the GNCTools directory');        
%If mode is gnctools, and there is one entry    
elseif(len == 1)
    %if len == 1, either we are in the correct folder with nothing to remove, or we are in the
    %wrong folder and there are files to remove, check CD
    if(any(strfind(paths{1},cd)))
        no = 0;
        return;
    else
        error('Expected to find "%s" in the current directory - please ensure you are in the GNCTools directory');
    end    
else %old ones to remove
    %Remove each folder found, and all subdirs under
    for n = 2:len
        %Absolute path to remove
        removeP = paths{n};
        %Search backwards for first file separator (we don't want the filename)
        for j = length(removeP):-1:1
            if(removeP(j) == filesep)
                break;
            end
        end
        removeP = removeP(1:max(j-1,1));        

        %Everything is lowercase to aid matching
        lrpath = lower(removeP);
        opath = regexp(lower(path),';','split');

        %Find & Remove Matching Paths
        no = 0;
        for i = 1:length(opath)
            %If we find it in the current path string, remove it
            fnd = strfind(opath{i},lrpath);        
            if(~isempty(fnd))  
                rmpath(opath{i});
                no = no + 1;
            end
        end

        %If delete is specified, also delete the directory
        if(del)
            stat = recycle; recycle('on'); %turn on recycling
            rmdir(removeP,'s'); %not sure if we dont have permissions here
            recycle(stat); %restore to original
        end
    end    
end


function OK = preReqChecks(cpath)
%Search for each required prereq
% Note we no longer search the registry, simply check if we can load a mex
% file which requires each runtime

fprintf('\n- Checking MATLAB version and operating system...\n');
mver = ver('MATLAB');

switch(mexext)
    case 'mexw32'
        error('GNCTools is compiled only for 64bit systems - sorry!');
    case 'mexw64'
        fprintf('MATLAB %s 64bit (Windows x64) detected\n',mver.Release);
    otherwise
        error('GNCTools is compiled only for Windows systems - sorry!');
end

% Only 64bit
arch = 'x64';

fprintf('\n- Checking for the required pre-requisites...\n');
missing = false;
cd('Utilities/Install');
havVC = GNCTools_PreReqCheck(cpath);
cd(cpath);
%See if missing anything
if(~havVC)
    missing = true;
end

%Print Missing PreReqs
if(~havVC)
    fprintf(2,'Cannot find the Microsoft VC++ 2019 %s Redistributable!\n',arch); 
else
    fprintf('Found the Microsoft VC++ 2019 %s Redistributable\n',arch); 
end 

%Install Instructions for each Package
if(missing)
    fprintf(2,'\nYou are missing one or more pre-requisites. Please read the instructions below carefully to install them:\n\n');
    
    if(~havVC)
        fprintf(2,' Microsoft VC++ 2019:\n');
        switch(arch)
            case 'x64'
                fprintf(2,'- Download from: https://aka.ms/vs/17/release/vc_redist.x64.exe\n');
            case 'x86'
                fprintf(2,'- Download from: https://aka.ms/vs/17/release/vc_redist.x86.exe\n');
        end
        fprintf(2,['NOTE: If you have already downloaded and installed VC++ 2019 (and restarted MATLAB) - it may be that you are missing the Universal C Runtime (Universal CRT).\nThis is automatically installed '...
                    'with Windows Updates - but if you don''t have those turned on, you can download it from here:\nhttps://www.microsoft.com/en-us/download/details.aspx?id=48234\n\n']);
    end

    fprintf(2,'\nOnce you have downloaded AND installed all the above packages, you MUST restart MATLAB.\n\nIf this message appears again after installing the above packages, try restarting your computer.\n\n\n');
    
    OK = false;
else
    OK = true;
end


function havVC = GNCTools_PreReqCheck(cpath) %#ok<INUSD>
havVC = true;
try
    a = vc19MexCheck; %#ok<NASGU>
catch
    havVC = false;
end


function in = bool2yn(val)
if (isempty(val) || val == true)
    in = 'y';
else
    in = 'n';
end
