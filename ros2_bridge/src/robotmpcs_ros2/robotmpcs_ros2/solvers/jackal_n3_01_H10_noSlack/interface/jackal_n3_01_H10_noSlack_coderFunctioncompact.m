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
% - outputs - column vector of length 100
function [outputs] = jackal_n3_01_H10_noSlack(x0, xinit, all_parameters)
    
    [output, ~, ~] = jackal_n3_01_H10_noSlackBuildable.forcesCall(x0, xinit, all_parameters);
    outputs = coder.nullcopy(zeros(100,1));
    outputs(1:10) = output.x01;
    outputs(11:20) = output.x02;
    outputs(21:30) = output.x03;
    outputs(31:40) = output.x04;
    outputs(41:50) = output.x05;
    outputs(51:60) = output.x06;
    outputs(61:70) = output.x07;
    outputs(71:80) = output.x08;
    outputs(81:90) = output.x09;
    outputs(91:100) = output.x10;
end
