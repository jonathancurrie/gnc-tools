function ok = GNCTools_InstallTest
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

%% TEST 2 - Check PID Results
fprintf('Checking PID Results...          ');

% PID
tol = 1e-14;
pidData = load('TestResults/pidData.mat');
pidData = pidData.pidData;
cstatus = mxCPID('Init',pidData.params);
cppstatus = mxPID('Init',pidData.params);
if (cstatus ~= 0 || cppstatus ~= 0)
    fprintf('\nFailed to Init PID Controller\n');
    ok = 0;
    return
end
n = length(pidData.r);
for i = 1:n
    cu = mxCPID('Update',pidData.r(i),pidData.y(i));
    cppu = mxPID('Update',pidData.r(i),pidData.y(i));
    err = abs(cu - pidData.cu(i));
    if (err > tol)
        fprintf('\nFailed on CPID Update %d, abs error %g\n',i,err);
        ok = 0;
        return
    end
    err = abs(cppu - pidData.cppu(i));
    if (err > tol)
        fprintf('\nFailed on PID Update %d, abs error %g\n',i,err);
        ok = 0;
        return
    end
end
fprintf('Ok\n');

%% TEST 3 - Check Filter Results
fprintf('Checking Filter Results...       ');

% PID
tol = 1e-14;
filterData = load('TestResults/filterData.mat');
filterData = filterData.filterData;
cstatus = mxCFilter('Init',filterData.num, filterData.den);
cppstatus = mxFilter('Init',filterData.num, filterData.den);
if (cstatus ~= 0 || cppstatus ~= 0)
    fprintf('\nFailed to Init Filter\n');
    ok = 0;
    return
end
n = length(filterData.u);
for i = 1:n
    cy = mxCFilter('Update',filterData.u(i));
    cppy = mxFilter('Update',filterData.u(i));
    err = abs(cy - filterData.cy(i));
    if (err > tol)
        fprintf('\nFailed on CFilter Update %d, abs error %g\n',i,err);
        ok = 0;
        return
    end
    err = abs(cppy - filterData.cppy(i));
    if (err > tol)
        fprintf('\nFailed on Filter Update %d, abs error %g\n',i,err);
        ok = 0;
        return
    end
end
fprintf('Ok\n');

%% TEST 4 - Check SOS Filter Results
fprintf('Checking SOS Filter Results...   ');

% PID
tol = 1e-14;
filterData = load('TestResults/sosFilterData.mat');
filterData = filterData.filterData;
cstatus = mxCSOSFilter('Init',filterData.coeffs);
cppstatus = mxSOSFilter('Init',filterData.coeffs);
if (cstatus ~= 0 || cppstatus ~= 0)
    fprintf('\nFailed to Init SOS Filter\n');
    ok = 0;
    return
end
n = length(filterData.u);
for i = 1:n
    cy = mxCSOSFilter('Update',filterData.u(i));
    cppy = mxSOSFilter('Update',filterData.u(i));
    err = abs(cy - filterData.cy(i));
    if (err > tol)
        fprintf('\nFailed on CSOSFilter Update %d, abs error %g\n',i,err);
        ok = 0;
        return
    end
    err = abs(cppy - filterData.cppy(i));
    if (err > tol)
        fprintf('\nFailed on sosFilter Update %d, abs error %g\n',i,err);
        ok = 0;
        return
    end
end
fprintf('Ok\n');
