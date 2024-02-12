function [] = plotXYGrid(X, Y, labelX, labelY)

% verify the pattern
scatter(X(:),Y(:),'b')
grid on
xlabel(labelX)
ylabel(labelY)

tol = 1;
xlim([min(min(X))-tol,max(max(X))+tol]);
ylim([min(min(Y))-tol,max(max(Y))+tol]);

end