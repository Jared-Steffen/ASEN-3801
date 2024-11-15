function [t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData(filename)
%   LOADASPENDATA | The purpose of this function is to organize/categorize
%   the input data given in the input file to individualized object
%   position/attitude vectors
%   Inputs: A .csv file containing one column for each category with
%   subsequent rows below each indicating values for an incrementing snapshot in time
%   Outputs: Multiple vectors/matrices formatted such that the physical
%   values for each snapshot in time are per columm and each component of
%   position/angle values is per row. Ex. Time has only one component,
%   incrementing through every snapshot in time, therefore its size is:
%   1 x length(time snapshots)
    
    data = readmatrix(filename); % store the excel-like file data in a matrix

    FR = 100; % Frame Rate Frequency [Hz]
    t_vec = data(:, 1)'./FR; % Seconds = Frames/Hz [Seconds]
    
    av_pos_inert = [data(:, 12)'; data(:, 13)'; data(:, 14)'] * 10^-3; % Data formating into new vector + unit converison [mm -> m]
    av_att = [data(:, 9)'; data(:, 10)'; data(:, 11)']; % Data formating into new vector

    tar_pos_inert = [data(:, 6)'; data(:, 7)'; data(:, 8)'] * 10^-3; % Data formating into new vector + unit converison [mm -> m]
    tar_att = [data(:, 3)'; data(:, 4)'; data(:, 5)']; % Data formating into new vector

    % Pass the formatted data into a conversion function that substitutes
    % NaN values to 0 as well as the measured angles into more
    % lecture-conventional angles that will be applicable to formulas. Then
    % return said values.
    [av_pos_inert, av_att, tar_pos_inert, tar_att] = ConvertASPENData(av_pos_inert, av_att, tar_pos_inert, tar_att);
end

