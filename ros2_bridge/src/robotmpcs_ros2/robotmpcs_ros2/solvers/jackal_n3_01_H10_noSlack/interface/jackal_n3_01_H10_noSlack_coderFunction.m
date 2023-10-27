% jackal_n3_01_H10_noSlack : A fast customized optimization solver.
% 
% Copyright (C) 2013-2023 EMBOTECH AG [info@embotech.com]. All rights reserved.
% 
% 
% This program is distributed in the hope that it will be useful.
% EMBOTECH makes NO WARRANTIES with respect to the use of the software 
% without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
% PARTICULAR PURPOSE. 
% 
% EMBOTECH shall not have any liability for any damage arising from the use
% of the software.
% 
% This Agreement shall exclusively be governed by and interpreted in 
% accordance with the laws of Switzerland, excluding its principles
% of conflict of laws. The Courts of Zurich-City shall have exclusive 
% jurisdiction in case of any dispute.
% 
% [OUTPUTS] = jackal_n3_01_H10_noSlack(INPUTS) solves an optimization problem where:
% Inputs:
% - x0 - matrix of size [100x1]
% - xinit - matrix of size [8x1]
% - all_parameters - matrix of size [270x1]
% Outputs:
% - x01 - column vector of length 10
% - x02 - column vector of length 10
% - x03 - column vector of length 10
% - x04 - column vector of length 10
% - x05 - column vector of length 10
% - x06 - column vector of length 10
% - x07 - column vector of length 10
% - x08 - column vector of length 10
% - x09 - column vector of length 10
% - x10 - column vector of length 10
function [x01, x02, x03, x04, x05, x06, x07, x08, x09, x10] = jackal_n3_01_H10_noSlack(x0, xinit, all_parameters)
    
    [output, ~, ~] = jackal_n3_01_H10_noSlackBuildable.forcesCall(x0, xinit, all_parameters);
    x01 = output.x01;
    x02 = output.x02;
    x03 = output.x03;
    x04 = output.x04;
    x05 = output.x05;
    x06 = output.x06;
    x07 = output.x07;
    x08 = output.x08;
    x09 = output.x09;
    x10 = output.x10;
end
