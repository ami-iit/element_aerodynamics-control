function [] = plotLine(X, labelX)

% verify the pattern
plot(X,zeros(size(X)),'ob')
grid on
xlabel(labelX)

tolX = 1;
tolY = 10;
xlim([min(min(X))-tolX,max(max(X))+tolX]);
ylim([-tolY,tolY]);

end