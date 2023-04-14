function output = GNCTools_RunUnitTests
% Run all GNCTools Unit Tests

clc
k = 1;

% Control
testFolders{k} = 'Control/PID/Tests'; k = k + 1;

% Navigation

% Guidance

% Signal Processing



% Now Run
output = runtests(testFolders,'OutputDetail',3);