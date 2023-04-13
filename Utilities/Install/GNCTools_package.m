function GNCTools_package
% GNCTOOLS_PACKAGE  Prepares toolbox for release

% Update the contents file
UpdateContentsFile('GNCTools',gnctoolsver,'Utilities/gnctools');

function UpdateContentsFile(name,tbxver,contentsFile)
%Update contents file description line

if(~isempty(strfind(contentsFile,'Contents.m')))
    p = contentsFile;
elseif(contentsFile(end) == filesep)
    p = [contentsFile 'Contents.m'];
else
    p = [contentsFile filesep 'Contents.m'];
end
s = fileread(p);
%Find initial date
idx = strfind(s,'(C)');
idxe = strfind(s(idx:end),'-');
stDate = strtrim(s(idx+3:idx+idxe-2));
mver = ver('matlab');
%Regenerate Contents File
fid = fopen(p,'w');
if(fid)
    try
        fprintf(fid,'%% %s\n',name);
        fprintf(fid,'%% Version %.2f %s %s\n',tbxver,mver.Release,datestr(now,1));
        fprintf(fid,'%% Copyright (C) %s-%s Jonathan Currie (Control Engineering)\n',stDate,datestr(now,10));
        fprintf(fid,'%% License: https://github.com/jonathancurrie/gnc-tools\n');
    catch ME
        fclose(fid);
        rethrow(ME);
    end
end
fclose(fid);