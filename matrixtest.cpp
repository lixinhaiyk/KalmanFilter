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
		//MatrixXf A(dim, dim) = A.Identity(dim, dim); 
		//MatrixXf H(dim, dim); 
		//MatrixXf Q(dim, dim); 
		//MatrixXf R(dim, dim); 
		//VectorXf X0(dim); 
		//MatrixXf P0(dim, dim); 

		MatrixXf A = A.Identity(dim, dim);
		MatrixXf H = H.Identity(dim, dim);
		MatrixXf Q = Q.Identity(dim, dim);
		MatrixXf R = R.Identity(dim, dim);
		VectorXf X0(5); X0 << 0, 0, 1, 1, 1;
		MatrixXf P0 = P0.Identity(dim,dim);
		//MatrixXf H(dim, dim); 
		//MatrixXf Q(dim, dim); 
		//MatrixXf R(dim, dim); 
		//VectorXf X0(dim); 
		//MatrixXf P0(dim, dim); 


		//for (int i = 0; i < dim; i++) {
		//	for (int j = 0; j < dim; j++) {
		//		A(i, j) = 1;
		//		H(i, j) = 1;
		//		Q(i, j) = 1;
		//		R(i, j) = 1;
		//		P0(i, j) = 1;
		//	}
		//	X0(i) = 1;
		//}

		//cout << MatrixXd::Random(5,1).array().abs() << endl;

		/* Create The Filter */
		KalmanFilter filter1(5, 0);

		/* Initialize the Filter*/
		filter1.setFixed(A, H, Q, R);
		filter1.setInitial(X0, P0);

		/* Create measure vector, and store measure value */
		VectorXf Z(dim);
		//cout << Z << endl;
		float mesaure[10] = { 0.39, 0.50, 0.48, 0.29, 0.25, 0.32, 0.34, 0.48, 0.41, 0.45 };

		/* This loop simulate the measure/prediction process */
		
		for (int i = 0; i < 10; ++i)
		{
			filter1.predict(); //Predict phase
			//Z << mesaure[i], mesaure[i], mesaure[i],mesaure[i],mesaure[i];
			vector<int>random_input{};
			Z << VectorXf::Random(5).array().abs();
			filter1.correct(Z); //Correction phase
			//cout << Z << endl;
			cout << "X" << i << ": " << filter1.X << endl;
		}
		finish = clock();
		//cout<<"opencv using time"<< (cv::getTickCount()-t1)/cv::getTickFrequency()<<endl;
		//cout << "using time" << (finish - start)*1000/ CLOCKS_PER_SEC << endl;
	}
	return 0;
}
