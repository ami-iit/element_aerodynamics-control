import scipy as sp
import pathlib
import numpy as np
import matplotlib.pyplot as plt
from sklearn import preprocessing, svm
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression, Lasso, LassoCV



# define functions
def X_matrix_def(alpha):
    X_matrix = np.hstack([#np.ones((len(alpha),1)), 
                          np.cos(np.deg2rad(alpha)), 
                          np.power(np.sin(np.deg2rad(alpha)),2), 
                          np.power(np.sin(np.deg2rad(alpha)),3), 
                          np.power(np.cos(np.deg2rad(alpha)),3)
                        ])
    return X_matrix



# Load .mat file
matFilePath = pathlib.Path(__file__).parents[1] / "src" / "dataset.mat"
mat_data = sp.io.loadmat(matFilePath)

# Access variables
cfdLinkNames = mat_data['cfdLinkNames']
pitchAngles  = mat_data['pitchAngles_full']
yawAngles    = mat_data['yawAngles_full']
linkAoAs     = mat_data['linkAoAs_matrix']
linkCdAs     = mat_data['linkCdAs_matrix']
linkClAs     = mat_data['linkClAs_matrix']
linkCsAs     = mat_data['linkCsAs_matrix']

# link
linkIndex = 0

alpha_train = linkAoAs[:, linkIndex][:, np.newaxis]

X_train = X_matrix_def(alpha_train)
y_train = linkCdAs[:, linkIndex]

# Linear Regression
lin_regr = LinearRegression()
lin_regr.fit(X_train, y_train)
w_lin = lin_regr.coef_

# Lasso Regression
lasso_regr = LassoCV()
lasso_regr.fit(X_train, y_train)
w_lasso = lasso_regr.coef_

# test data
alpha_test = np.linspace(0,180,1801)[:, np.newaxis]
X_test = X_matrix_def(alpha_test)
y_lin = lin_regr.predict(X_test)
y_lasso = lasso_regr.predict(X_test)

print(lin_regr.score(X_train, y_train))

################################# PLOTS ######################################
fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(alpha_train, y_train, s=16, label='CFD data', facecolors='none', edgecolors='tab:blue', alpha=0.5)
ax.plot(alpha_test, y_lin, 'k-', linewidth=3, label='Linear Regression')
#ax.plot(alpha_test, y_lasso, 'k--', linewidth=3, label='LassoCV Regression')

ax.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
ax.set_xlabel(r'$\alpha$ [deg]')
ax.xaxis.label.set_fontsize(12)
ax.set_xlim([0,180])

ax.yaxis.set_label_coords(-0.1, 0.5)  # Adjust the y-axis label position
ax.set_ylabel(r'$C_D A$')
ax.yaxis.label.set_fontsize(12)
ax.yaxis.set_tick_params()

ax.set_title(str(cfdLinkNames[linkIndex][0][0]))
ax.grid()
ax.legend()
plt.show(block=False)

# For displaying all the plots
plt.show()