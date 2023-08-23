function plot_nonzero_values(varargin)
% PLOT_NONZERO_VECTORS Plot non-zero elements of one or more vectors as a line chart.
% Usage: plot_nonzero_vectors(vector1, vector2, ..., vectorN)
%        plot_nonzero_vectors(vectors)
%
% This function takes in one or more column vectors as input, and plots the
% non-zero elements of each vector as a line chart. The non-zero elements are
% plotted in the same order as they appear in the original vector, and the
% chart is drawn with a white background and thick lines.
%
% Example:
% >> v1 = [1; 0; 3; 0; 5; 0; 0];
% >> v2 = [0; 2; 0; 4; 0; 6; 0];
% >> v3 = [1; 2; 3; 4; 5; 6; 7];
% >> plot_nonzero_vectors(v1, v2, v3);
%
% Input:
% - vectors: One or more column vectors, each containing numeric values.
%
% Output: None.

% Check input arguments
if nargin == 1 && iscell(varargin{1})
    vectors = varargin{1};
elseif nargin >= 1
    vectors = varargin;
else
    error('Error: At least one input vector must be specified.');
end

% Set up the figure
figure;
hold on;
set(gcf, 'Color', 'w');

% Plot each vector
for i = 1:length(vectors)
    vector = vectors{i};
    plot(find(vector), vector(vector~=0), 'LineWidth', 2, 'DisplayName', ['Vector ', num2str(i)]);
end

% Add a legend
legend('show', 'Location', 'best');

% Set the axis labels
xlabel('Number of robot postures');
ylabel('Error value (mm)');

% % Set the title
% if length(vectors) == 1
%     title('Non-zero elements of Vector');
% else
%     title('Non-zero elements of Vectors');
% end

% Release the figure
hold off;

end