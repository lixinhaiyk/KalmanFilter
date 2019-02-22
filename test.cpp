/*
*   Kalmand Filter Monodimensional Example
*
*   Implementation code of the example showed here:
*
*     http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies
*
*   It simulate a noisy voltage reading from a constant source.
*
*   by Fabio Carbone, 23/12/2016
*   www.fabiocarbone.org
*/


#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctime>
#include "KalmanFilter.h"

using namespace std;
int const dim = 5;

int main(int argc, char const *argv[])
{
	clock_t  start,finish;
	while (1) {
		start = clock();
		double t1 = cv::getTickCount();
		/* Set Matrix and Vector for Kalman Filter: */
		MatrixXf A(dim, dim); A << 1,1,1,1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
		MatrixXf H(dim, dim); H << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
		MatrixXf Q(dim, dim); Q << 0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
		MatrixXf R(dim, dim); R << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
		VectorXf X0(dim); X0 << 0,0,0,0,0;
		MatrixXf P0(dim, dim); P0 << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;

		/* Create The Filter */
		KalmanFilter filter1(5, 0);

		/* Initialize the Filter*/
		filter1.setFixed(A, H, Q, R);
		filter1.setInitial(X0, P0);

		/* Create measure vector, and store measure value */
		VectorXf Z(dim);
		cout << Z << endl;
		float mesaure[10] = { 0.39, 0.50, 0.48, 0.29, 0.25, 0.32, 0.34, 0.48, 0.41, 0.45 };

		/* This loop simulate the measure/prediction process */
		for (int i = 0; i < 10; ++i)
		{

			filter1.predict(); //Predict phase
			Z << mesaure[i], mesaure[i], mesaure[i],mesaure[i],mesaure[i];
			filter1.correct(Z); //Correction phase
			cout << Z << endl;
			cout << "X" << i << ": " << filter1.X << endl;
		}
		finish = clock();
		cout<<"opencv using time"<< (cv::getTickCount()-t1)/cv::getTickFrequency()<<endl;
		cout << "using time" << (finish - start)*1000/ CLOCKS_PER_SEC << endl;
	}
	return 0;
}
