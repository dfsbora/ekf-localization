import math
from math import sqrt

import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
from matplotlib.patches import Ellipse

import numpy as np
from numpy.random import randn


import time
start_time = time.time()


############ PLOT FUNCTIONS ##############

#Simple line plot of yData against xData
def plotData(ax, xData, yData, param_dict):
	ax.plot(xData, yData, **param_dict)

#Simple scatter plot of yData against xData
def scatterData(ax, xData, yData, param_dict):
	ax.scatter(xData, yData, **param_dict)

#Plot up to three state values against xData and the standard deviation area if specified
def plotStateTime(ax, xData, sigmaValue=0, sigmaRegion=0, estimate=None, prediction=None, measurement=None):
	"""
	Create a plot of the estimate, prediction and/or measurement against xData. Fill the standard deviation area if specified.

	Parameters
	----------
	ax : matplotlib.axes.Axes
		The axes object to plot into.

	xData : 
		The data for axis X

	sigmaValue : float
		The variance of variable

	sigmaRegion : int
		The number of standard deviations desired area to be filled in the graph. 

	Returns
	-------
	min : float
		The minimum value in the dataset

	max : float
		The maximum value in the dataset
	"""

	#Set parameters for plot
	sigmaName = "%s sigma" % sigmaRegion
	paramSigmaRegion = {'label':sigmaName}
	paramEstimate = {'label':'estimate'}
	paramPrediction = {'marker':'.', 'label':'prediction'}
	paramMeasurement = {'marker':'2', 'label':'measurement'}

	max = float('-inf')
	min = float('inf') 

	#Fill area according to the chosen sigmaRegion (0,1,2,3 standard deviations)
	if sigmaRegion != 0:
		upperCurve = estimate + sigmaValue*sigmaRegion
		lowerCurve = estimate - sigmaValue*sigmaRegion
		ax.fill_between(xData, upperCurve, lowerCurve,  alpha=0.25, **paramSigmaRegion) #color='red'
		max = upperCurve.max()
		min = lowerCurve.min()
	#Plot estimate data
	if estimate is not None:
		plotData(ax, xData, estimate, paramEstimate)
		if estimate.max() > max:
			max = estimate.max()
		if estimate.min() < min:
			min = estimate.min()
	#Plot prediction data
	if prediction is not None:
		scatterData(ax, xData, prediction, paramPrediction)
		if prediction.max() > max:
			max = prediction.max()
		if prediction.min() < min:
			min = prediction.min()
	#Plot measurement data
	if measurement is not None:
		scatterData(ax, xData, measurement, paramMeasurement)
		if measurement.max() > max:
			max = measurement.max()
		if measurement.min() < min:
			min = measurement.min()

	#Return the min and max values that appeared in data
	return min, max

#Plot the variance against xData
def plotVarianceTime(ax, xData, variance=None):
	if variance is not None:
		plotData(ax, xData, variance, {'label':'variance'})

#Adjust graph appearance
def setProperties(ax, param_dict, xLeft, xRight, yMax=None, yMin=None, legend=True):
	"""
	Adjust graph overall appearance 

	Parameters
	----------
	ax : matplotlib.axes.Axes
		The axes object to plot into.

	xLeft : 
		The left limit for X axis

	xRight : 
		The right limit for X axis

	yMax : 
		The upper limit for Y axis

	yMin : 
		The bottom limit for Y axis

	"""

	ax.set(**param_dict)
	ax.set_xlim(xLeft, xRight)
	ax.set_ylim(yMin, yMax)
	ax.grid( which='major', axis='both')
	if legend:
		ax.legend(loc='best', fontsize='x-small',numpoints=1,scatterpoints=1,frameon=True,fancybox=True,shadow=True,framealpha=1.,labelspacing=0.2)
		#{'xx-small', 'x-small', 'small', 'medium', 'large', 'x-large', 'xx-large'} #TODO 

#MAIN PLOT FUNCTION 
#Plot the results of the filter (states value and/or variance) in up to 4 subplots.
def plotFilterResults(nRows = 1, nCols = 1, sigmaRegion=0, variance=None, index1=None, index2=None, estimate=None, prediction=None, measurement=None, xData=None,
title1='Filter result', title2='Filter result', title3='Variance', title4='Variance', ylabel1='State value',ylabel2='State value',xlabel='Tempo'):
	"""
	Plot the results of the filter (states value and/or variance) in up to 4 subfigures

	Parameters
	----------
	nRows : int
		Number of rows to create subplot grid

	nCols : int
		Number of cols to create subplot grid

	sigmaRegion : int
		The number of standard deviations desired area to be filled in the graph. 

	variance : list of matrix
		List with covariance matrix

	index1 : int
		Index for the variance of variable 1 in the variance matrix 

	index2 : int
		Index for the variance of variable 2 in the variance matrix 

	title1 : string
		Title of first subplot

	ylabel1 : string
		Label of y axis in upper subplots (state value plots)

	xlabel : string
		Label of x axis in all subplots

	"""

	#Create figure and initalize parameters and data
	fig = plt.figure()
	margin = 2
	if xData is None: #Adjust xData # TODO corrigir
		xData = np.arange(len(estimate))

	#Parameters for each subplot
	paramAx1 = {'xlabel':xlabel,'ylabel':ylabel1,'title':title1} 
	paramAx2 = {'xlabel':xlabel,'ylabel':ylabel2,'title':title2}
	paramAx3 = {'xlabel':xlabel,'ylabel':'Variance','title':title3}
	paramAx4 = {'xlabel':xlabel,'ylabel':'Variance','title':title4}

	#First subplot 
	ax1 = fig.add_subplot(nRows, nCols, 1)
	yMin,yMax = plotStateTime(ax1,xData, sigmaValue=variance[:,index1,index1], sigmaRegion=sigmaRegion, estimate=estimate[:,index1], prediction=prediction, measurement=measurement)
	setProperties(ax1,paramAx1,xLeft=xData[0]-margin,xRight=xData[-1]+margin,yMin=yMin-margin,yMax=yMax+margin)

	#Verify need of more subplots
	if nRows*nCols>1:
		#Second subplot
		ax2 = fig.add_subplot(nRows, nCols, 2)
		#Verify if the second subplot should be a variance plot or a state plot based on number of columns
		if nCols == 1:
			plotVarianceTime(ax2, xData, variance=variance[:,index1,index1])
			xLeft, xRight = ax1.get_xlim()
			setProperties(ax2,paramAx3, xLeft=xLeft, xRight=xRight, legend=False)
		else:
			yMin,yMax = plotStateTime(ax2,xData, sigmaValue=variance[:,index2,index2], sigmaRegion=sigmaRegion, estimate=estimate[:,index2], prediction=prediction, measurement=measurement)
			setProperties(ax2,paramAx2, xLeft=xData[0]-margin,xRight=xData[-1]+margin,yMin=yMin-margin,yMax=yMax+margin)

		#Verify need of more subplots
		if nRows*nCols>2:
			#Third and fourth subplots
			ax3 = fig.add_subplot(nRows, nCols, 3)
			ax4 = fig.add_subplot(nRows, nCols, 4)	

			plotVarianceTime(ax3, xData, variance=variance[:,index1,index1])
			xLeft, xRight = ax1.get_xlim()
			setProperties(ax3, paramAx3, xLeft=xLeft, xRight=xRight,legend=False)

			plotVarianceTime(ax4, xData, variance=variance[:,index2,index2])
			xLeft, xRight = ax2.get_xlim()
			setProperties(ax4, paramAx4, xLeft=xLeft, xRight=xRight,legend=False)

	#TODO apagar
	#plt.tight_layout()
	#plt.savefig('plot.png')
	plt.show()


#Plot covariance ellipses: centers and contours
def plotCovarianceXY(estimate, variance, xIndex, yIndex, sigmaRegion=1, xlabel='', ylabel=''):
	#fig = plt.figure()
	#ax1 = fig.add_subplot(nRows, nCols, 1)
	margin=5
	#fig, ax = plt.subplots()

	paramScatter = {'marker':'.', 'label':'Posicao'}
	xData = estimate[:,xIndex]
	yData = estimate[:,yIndex]
	scatterData(ax,xData, yData, paramScatter)

	#Plot ellipses for covariance
	plotCovarianceEllipse(ax, estimate, variance, xIndex, yIndex, sigmaRegion)

	#Set parameters for appearance adjustment
	xLeft=xData.min()-margin
	xRight=xData.max()+margin
	yMax=yData.max()+margin
	yMin=yData.min()-margin
	paramAx = {'xlabel':xlabel,'ylabel':ylabel,'title':'Covariancia'}
	setProperties(ax, paramAx, xLeft=xLeft, xRight=xRight, yMax=yMax, yMin=yMin, legend=True)

	plt.show()

#Plot ellipses calculating eigenvector and eigenvalues 
def plotCovarianceEllipse0(ax, estimate, variance, xIndex, yIndex, sigmaRegion=1):
	#Loop over estimate and variance to plot all covariance ellipses

	p = 0.95 # TODO corrigir
  	s = -2 * np.log(1-p)

	for center, P in zip(estimate, variance):
		x,y = center

		a = P[xIndex][xIndex]
		b = P[xIndex][yIndex]
		c = P[yIndex][xIndex]
		d = P[yIndex][yIndex]

		aux = np.sqrt((a-d) * (a-d) + 4*b*c)
		eigVec = [[-(aux-a+d)/(2*c), (aux+a-d)/(2*c)],  [1,1]]
	 	eigVal = [np.sqrt(s*(a+d-aux)/2),   np.sqrt(s*(a+d-aux)/2)] 
	 
	 	norm1 = np.hypot(eigVec[0][0],1)
	  	norm2 = np.hypot(eigVec[0][1],1)
	  	eigVec[0][0] = eigVec[0][0] / norm1
	  	eigVec[1][0] = eigVec[1][0] / norm1
	  	eigVec[0][1] = eigVec[0][1] / norm2
	  	eigVec[1][1] = eigVec[1][1] / norm2
	 
	 	if eigVal[0] > eigVal[1]:
	 		nEigVal = 0
	 	else:
	 		nEigVal = 1

	 	x1 = x + eigVec[0][nEigVal] * sqrt(eigVal[nEigVal])
	 	y1 = y + eigVec[1][nEigVal] * sqrt(eigVal[nEigVal])
	 	x2 = x + eigVec[0][1-nEigVal] * sqrt(eigVal[1-nEigVal])
	 	y2 = y + eigVec[1][1-nEigVal] * sqrt(eigVal[1-nEigVal])

	 	ell = Ellipse(xy=(x,y), width=np.hypot(x1-x,y1-y), height=np.hypot(x2-x,y2-y),angle=np.arctan2(y1-y,x1-x)) 

		#Draw ellipse 
		ax.add_artist(ell)
		ell.set_clip_box(ax.bbox)
		ell.set_alpha(np.random.rand())
		ell.set_facecolor(np.random.rand(3))
		

def plotEllipse0(ax, state, variance, xIndex=0, yIndex=1, sigmaRegion=1):
	#Loop over estimate and variance to plot all covariance ellipses
	#ax = plt.gca()

	p = 0.95 # TODO corrigir
  	s = -2 * np.log(1-p)

  	x,y = state
  	P = variance

	a = P[xIndex][xIndex]
	b = P[xIndex][yIndex]
	c = P[yIndex][xIndex]
	d = P[yIndex][yIndex]

	aux = np.sqrt((a-d) * (a-d) + 4*b*c)
	eigVec = [[-(aux-a+d)/(2*c), (aux+a-d)/(2*c)],  [1,1]]
 	eigVal = [np.sqrt(s*(a+d-aux)/2),   np.sqrt(s*(a+d-aux)/2)] 
 
 	norm1 = np.hypot(eigVec[0][0],1)
  	norm2 = np.hypot(eigVec[0][1],1)
  	eigVec[0][0] = eigVec[0][0] / norm1
  	eigVec[1][0] = eigVec[1][0] / norm1
  	eigVec[0][1] = eigVec[0][1] / norm2
  	eigVec[1][1] = eigVec[1][1] / norm2
 
 	if eigVal[0] > eigVal[1]:
 		nEigVal = 0
 	else:
 		nEigVal = 1

 	x1 = x + eigVec[0][nEigVal] * sqrt(eigVal[nEigVal])
 	y1 = y + eigVec[1][nEigVal] * sqrt(eigVal[nEigVal])
 	x2 = x + eigVec[0][1-nEigVal] * sqrt(eigVal[1-nEigVal])
 	y2 = y + eigVec[1][1-nEigVal] * sqrt(eigVal[1-nEigVal])

 	ell = Ellipse(xy=(x,y), width=np.hypot(x1-x,y1-y), height=np.hypot(x2-x,y2-y),angle=np.arctan2(y1-y,x1-x)) 

	#Draw ellipse 
	ax.add_artist(ell)
	ell.set_clip_box(ax.bbox)
	ell.set_alpha(np.random.rand())
	ell.set_facecolor(np.random.rand(3))


#Plot ellipses using transforms
def plotEllipse(ax, state, variance, xIndex=0, yIndex=1, sigmaRegion=6, facecolor='g', alpha=0.8):
	#Loop over estimate and variance to plot all covariance ellipses

	x,y = state

	P = variance
	#Auxiliary and Pearson parameters 
	varx = P[xIndex][xIndex]
	vary = P[yIndex][yIndex]
	cov = P[xIndex][yIndex]
	p = cov/(varx*vary)

	#Computation of ellipse diameter
	#TODO valor negativo
	width = 2 * np.sqrt(1+p)
	if 1-p > 0: #TODO Verificar se essa operacao esta correta
		height = 2 * np.sqrt(1-p)
	else:
		height = 2 * np.sqrt(p-1)
	angle = 45

	#Create ellipse
	ell = Ellipse(xy=(0,0), width=width, height=height,angle=angle) 

	#Scale ellipse
	wScale = 2*varx*sigmaRegion
	hScale = 2*vary*sigmaRegion
	transf = transforms.Affine2D().scale(wScale, hScale).translate(x,y)
	ell.set_transform(transf + ax.transData)

	#Draw ellipse 
	ax.add_artist(ell)
	ell.set_clip_box(ax.bbox)
	ell.set_alpha(alpha)
	ell.set_facecolor(facecolor)


#Plot ellipses using transforms
def plotCovarianceEllipse(ax, estimate, variance, xIndex, yIndex, sigmaRegion=1):
	#Loop over estimate and variance to plot all covariance ellipses

	for center, P in zip(estimate, variance):
		x,y = center

		#Auxiliary and Pearson parameters 
		varx = P[xIndex][xIndex]
		vary = P[yIndex][yIndex]
		cov = P[xIndex][yIndex]
		p = cov/(varx*vary)

		#Computation of ellipse diameter
		width = 2 * np.sqrt(1+p)
		if 1-p > 0: #TODO Verificar se essa operacao esta correta
			height = 2 * np.sqrt(1-p)
		else:
			height = 2 * np.sqrt(p-1)
		angle = 45

		#Create ellipse
		ell = Ellipse(xy=(0,0), width=width, height=height,angle=angle) 

		#Scale ellipse
		wScale = 2*varx*sigmaRegion
		hScale = 2*vary*sigmaRegion
		transf = transforms.Affine2D().scale(wScale, hScale).translate(x,y)
		ell.set_transform(transf + ax.transData)

		#Draw ellipse 
		ax.add_artist(ell)
		ell.set_clip_box(ax.bbox)
		ell.set_alpha(np.random.rand())
		ell.set_facecolor(np.random.rand(3))


#Plot ellipses using transforms
def plot_ellipse(ax, state, variance, xIndex=0, yIndex=1, sigmaRegion=3, facecolor='g', alpha=0.8):
	#Loop over estimate and variance to plot all covariance ellipses
	
	x,y = state

	angle, width, height = covariance_ellipse(variance, deviations=sigmaRegion)
	
	angle = np.degrees(angle)
	width = width * 2.
	height = height * 2.

	ell = Ellipse(xy=(x,y), width=width, height=height, angle=angle) 

	ax.add_artist(ell)
	ell.set_clip_box(ax.bbox)
	ell.set_alpha(alpha)
	ell.set_facecolor(facecolor)




def covariance_ellipse(P, deviations=1):
	"""
	Returns a tuple defining the ellipse representing the 2 dimensional
	covariance matrix P.
	Parameters
	----------
	P : nd.array shape (2,2)
	   covariance matrix
	deviations : int (optional, default = 1)
	   # of standard deviations. Default is 1.
	Returns (angle_radians, width_radius, height_radius)
	"""

	U, s, _ = np.linalg.svd(P)
	orientation = math.atan2(U[1, 0], U[0, 0])
	width = deviations * math.sqrt(s[0])
	height = deviations * math.sqrt(s[1])

	if height > width:
		raise ValueError('width must be greater than height')

	return orientation, width, height
