/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */



/*

IMPORTANT: This file should serve as a starting point to develop the user
code for the OCP solver. The code below is for illustration purposes. Most
likely you will not get good results if you execute this code without any
modification(s).

Please read the examples in order to understand how to write user code how
to run the OCP solver. You can find more info on the website:
www.acadotoolkit.org

*/

#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <stdio.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */
#define NUM_STEPS	10
		

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;


double calc_dist(double x1, double y1, double z1, double x2, double y2, double z2);
void init_goal(float x, float y, float z);
void init_start0(float x, float y, float z, float psi, float v, float phi, float gamma);
void optimize(int max);
void checkIntersection();
void plot();
void plant_model();
double getAcc_norm();
double getPhiDot_norm();
double getGammaDot_norm();
void init_weights(double wx, double wy, double wz, double wxf, double wyf, double wzf, double wa, double wphi, double wgamma);


double obsx[13] = {10, 45, 50 ,70 ,110, 15 ,40 ,60, 100 ,120 ,80 ,100 ,0};
double obsy[13] = {55, 50,  10, 70, 45, 100, 110, 105,  95,  80,  120, 0, 20}; 
double A[13] =    {5, 5 ,  10 , 5 , 5 , 12 ,5  ,7 ,  5 ,  5 ,  9,    5,  5}; 
double B[13] =    {5 , 5  , 10 ,  5,  5 , 12, 5,  7,   6,   7,   9,    5,  5}; 
double C[13] =    {35,  35,  65,  85 , 25 , 50 , 55 , 35 , 50 ,  65,   75,   40,  45};

double obsz[13] = { 0 };
int inter[13] = { 0 };
double tf = 14.0;

FILE *fp;
real_t te;
/* A template for testing of the solver. */
int main( )
{

	int    i, iter;
	acado_timer t;

	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	
	float X = 5.0, 
	      Y = 130.0,
	      Z = 30.0, 
	      PSI = 0.2 * M_PI,
	      V = 12.0,
          PHI = 0.0, 
	      GAMMA = 0.0;

	float XG = 120.0, 
	      YG = 40.0,
	      ZG = 10.0;	
	
	init_start0(X, Y, Z, PSI, V, PHI, GAMMA); 
	init_goal(XG, YG, ZG);			           
	init_weights(1.0, 1.0, 1.0, 1000.0, 1000.0, 1000.0, 100.0, 10000.0, 10000.0); // x y z x[-1] y[-1] z[-1] a phidot gammadot
	
	for (i = 0; i < (N + 1); ++i)  
	{
		acadoVariables.x[ i*NX + 0 ] = X;
		acadoVariables.x[ i*NX + 1 ] = Y;
		acadoVariables.x[ i*NX + 2 ] = Z;
		acadoVariables.x[ i*NX + 3 ] = PSI;
		acadoVariables.x[ i*NX + 4 ] = V;
		acadoVariables.x[ i*NX + 5 ] = PHI;
		acadoVariables.x[ i*NX + 6 ] = GAMMA;
	}
	for (i = 0; i <  N; ++i) 
	{	
		acadoVariables.u[ i*NU + 0 ] = 0.0001;
		acadoVariables.u[ i*NU + 1 ] = 0.0001;
		acadoVariables.u[ i*NU + 2 ] = 0.0001;
	}	
	plant_model();
	
	acado_preparationStep();
	
	acado_tic( &t );	
	optimize(NUM_STEPS);
	te = acado_toc( &t );
	

	printf("Iteration= %d \n", NUM_STEPS);
	printf("Norm of phidot= %f \n",getPhiDot_norm());
	printf("Norm of acc= %f \n",getAcc_norm());
	printf("Norm of gammadot= %f \n",getGammaDot_norm());
	printf("Time=   %.3g secs\n\n", (te));
	printf("Final position cost= %f \n", pow(acadoVariables.x[ N*NX + 0 ] - XG, 2) + pow(acadoVariables.x[ N*NX + 1] - YG, 2) + pow(acadoVariables.x[ N*NX + 2 ] - ZG, 2));	
	
	fp = fopen("details.txt","w");
	fprintf(fp, "time :   %.3g secs\n\n", (te));
	fprintf(fp, "Iteration= %d", NUM_STEPS);
	fprintf(fp, "Planning Time= %f \n", tf);
	fprintf(fp, "Norm of acc= %f \n",getAcc_norm());
	fprintf(fp, "Norm of phidot= %f \n",getPhiDot_norm());
	fprintf(fp, "Norm of gammadot= %f \n",getGammaDot_norm());
	fprintf(fp, "Final position cost= %f \n", pow(acadoVariables.x[ N*NX + 0 ] - XG, 2) + pow(acadoVariables.x[ N*NX + 1] - YG, 2) + pow(acadoVariables.x[ N*NX + 2 ] - ZG, 2));	
	fprintf(fp,"Start= %f %f %f %f \n", acadoVariables.x0[ 0 ], acadoVariables.x0[ 1 ], acadoVariables.x0[ 2 ], acadoVariables.x0[3]);	
	fprintf(fp,"Goal= %f %f %f \n", acadoVariables.yN[ 0 ], acadoVariables.yN[ 1 ], acadoVariables.yN[ 2 ]);
	fclose(fp);

	//plot();	
	checkIntersection();
	
    return 0;
}
void optimize(int max)
{
	int iter;

	for(iter = 0; iter < max; ++iter)
		{			
			acado_feedbackStep( );
			acado_preparationStep();
			//printf("XYZ %f %f %f \n", acadoVariables.x[(N)*NX + 0], acadoVariables.x[(N)*NX + 1], acadoVariables.x[(N)*NX + 2]);			
			//printf("Real-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );
		}
	
}
void init_goal(float x, float y, float z)
{

	for (int i = 0; i < N; ++i)  
	{
		acadoVariables.y[ i*NY + 0 ] = x;
		acadoVariables.y[ i*NY + 1 ] = y;
		acadoVariables.y[ i*NY + 2 ] = z;		
		acadoVariables.y[ i*NY + 3 ] = 0.0;
		acadoVariables.y[ i*NY + 4 ] = 0.0;
		acadoVariables.y[ i*NY + 5 ] = 0.0;	
	}
	acadoVariables.yN[ 0 ] = x;
	acadoVariables.yN[ 1 ] = y;
	acadoVariables.yN[ 2 ] = z;	
}
void init_start0(float x, float y, float z, float psi, float v, float phi, float gamma)
{
	acadoVariables.x0[ 0 ] = x;
	acadoVariables.x0[ 1 ] = y;
	acadoVariables.x0[ 2 ] = z;
	acadoVariables.x0[ 3 ] = psi;
	acadoVariables.x0[ 4 ] = v;
	acadoVariables.x0[ 5 ] = phi;
	acadoVariables.x0[ 6 ] = gamma;
	
}
void plot()
{
	fp = fopen("data.txt","w");
	for(int i = 0; i < N+1; i++) 
	fprintf(fp,"%f %f %f \n", acadoVariables.x[ i*NX + 0 ], acadoVariables.x[ i*NX + 1 ], acadoVariables.x[ i*NX + 2 ]);
	fclose(fp);

	fp = fopen("rpy.txt","w");
	for(int i = 0; i < N+1; i++) 
	fprintf(fp,"%f %f %f \n", acadoVariables.x[ i*NX + 5 ], acadoVariables.x[ i*NX + 6 ], acadoVariables.x[ i*NX + 3 ]);
	fclose(fp);

	fp = fopen("psi.txt","w");
	for(int i = 0; i < N+1; i++) 
	fprintf(fp,"%d %f \n", i, acadoVariables.x[ i*NX + 3 ]);
	fclose(fp);
	
	fp = fopen("datags.txt","w");
	fprintf(fp,"%f %f %f \n", acadoVariables.x[ 0 ], acadoVariables.x[ 1 ], acadoVariables.x[ 2 ]);	
	fprintf(fp,"%f %f %f \n", acadoVariables.yN[ 0 ], acadoVariables.yN[ 1 ], acadoVariables.yN[ 2 ]);
	fclose(fp);

	fp = fopen("v.txt","w");
	for(int i = 0; i < N+1; i++) 
	fprintf(fp,"%d %f \n", i, acadoVariables.x[ i*NX + 4 ]);
	fclose(fp);

	fp = fopen("phi.txt","w");
	for(int i = 0; i < N+1; i++) 
	fprintf(fp,"%d %f \n", i, acadoVariables.x[ i*NX + 5 ]);
	fclose(fp);

	fp = fopen("gamma.txt","w");
	for(int i = 0; i < N+1; i++) 
	fprintf(fp,"%d %f \n", i, acadoVariables.x[ i*NX + 6 ]);
	fclose(fp);

	fp = fopen("vdot.txt","w");
	for(int i = 0; i < N; i++) 
	fprintf(fp,"%d %f \n", i, acadoVariables.u[ i*NU + 0 ]);
	fclose(fp);

	fp = fopen("phidot.txt","w");
	for(int i = 0; i < N; i++) 
	fprintf(fp,"%d %f \n", i, acadoVariables.u[ i*NU + 1 ]);
	fclose(fp);

	fp = fopen("gammadot.txt","w");
	for(int i = 0; i < N; i++) 
	fprintf(fp,"%d %f \n", i, acadoVariables.u[ i*NU + 2 ]);
	fclose(fp);

}
void init_weights(double wx, double wy, double wz, double wxf, double wyf, double wzf, double wa, double wphi, double wgamma)
{
	  for (int i = 0; i < N; i++)
	  {
			acadoVariables.W[NY*NY*i + (NY+1)*0] = wx;
    		acadoVariables.W[NY*NY*i + (NY+1)*1] = wy;
    		acadoVariables.W[NY*NY*i + (NY+1)*2] = wz;
    		acadoVariables.W[NY*NY*i + (NY+1)*3] = wa;
    		acadoVariables.W[NY*NY*i + (NY+1)*4] = wphi;
			acadoVariables.W[NY*NY*i + (NY+1)*5] = wgamma;
	  }

	  acadoVariables.WN[(NYN+1)*0] = wxf;
	  acadoVariables.WN[(NYN+1)*1] = wyf;
	  acadoVariables.WN[(NYN+1)*2] = wzf;
}
void checkIntersection()
{
	int i, j;
	
	for(j = 0; j < 13; j++)
	{
		for(i = 0; i < N+1; i++)
		{
			double x = acadoVariables.x[NX*i + 0];
			double y = acadoVariables.x[NX*i + 1];
			double z = acadoVariables.x[NX*i + 2];
			double temp = pow(x-obsx[j],2)/pow(A[j],2) + pow(y-obsy[j],2)/pow(B[j],2) + pow(z-obsz[j],2)/pow(C[j],2);
			if(temp <= 1.0)
			{
				inter[j] = 1;
				break;
			} 
		}
		printf("intersection = %d %d \n",j, inter[j]);
	}
}
void plant_model()
{
	double x = acadoVariables.x0[ 0 ];
	double y = acadoVariables.x0[ 1 ];
	double z = acadoVariables.x0[ 2 ];
	double psi = acadoVariables.x0[ 3 ];
	double v = acadoVariables.x0[ 4 ];
	double phi = acadoVariables.x0[ 5 ];
	double gamma = acadoVariables.x0[ 6 ];
	double dt = tf/N;
	double g = 10.0;
	acadoVariables.x[ 0 ] = x;
	acadoVariables.x[ 1 ] = y;
	acadoVariables.x[ 2 ] = z;
	acadoVariables.x[ 3 ] = psi;
	acadoVariables.x[ 4 ] = v;
	acadoVariables.x[ 5 ] = phi;
	acadoVariables.x[ 6 ] = gamma;

	
	for (int i = 1; i < (N + 1); ++i)  
	{
		double a = acadoVariables.u[(i-1)*NU + 0];
		double phidot = acadoVariables.u[(i-1)*NU + 1];
		double gammadot = acadoVariables.u[(i-1)*NU + 2];
		
 
		acadoVariables.x[ i*NX + 0 ] = x + (v * cos(gamma) * cos(psi)) * dt;
		acadoVariables.x[ i*NX + 1 ] = y + (v * cos(gamma) * sin(psi)) * dt;
		acadoVariables.x[ i*NX + 2 ] = z - (v * sin(gamma)) * dt;
		acadoVariables.x[ i*NX + 3 ] = psi + ((g/v) * tan(phi)) * dt;
		acadoVariables.x[ i*NX + 4 ] = v + a * dt;
		acadoVariables.x[ i*NX + 5 ] = phi + phidot * dt;
		acadoVariables.x[ i*NX + 6 ] = gamma + gammadot * dt;
		

		if ( acadoVariables.x[ i*NX + 4 ] > 16.0)
			acadoVariables.x[ i*NX + 4 ] = 16.0;
		if ( acadoVariables.x[ i*NX + 4 ] < 12.0)
			acadoVariables.x[ i*NX + 4 ] = 12.0;

		if ( acadoVariables.x[ i*NX + 5 ] > (40*M_PI)/180.0)
			acadoVariables.x[ i*NX + 5 ] = (40*M_PI)/180.0;
		if ( acadoVariables.x[ i*NX + 5 ] < -(40*M_PI)/180.0)
			acadoVariables.x[ i*NX + 5 ] = -(40*M_PI)/180.0;

		if ( acadoVariables.x[ i*NX + 6 ] > (40*M_PI)/180.0)
			acadoVariables.x[ i*NX + 6 ] = (40*M_PI)/180.0;
		if ( acadoVariables.x[ i*NX + 6 ] < -(40*M_PI)/180.0)
			acadoVariables.x[ i*NX + 6 ] = -(40*M_PI)/180.0;

		x = acadoVariables.x[ 0+i*NX ];
		y = acadoVariables.x[ 1+i*NX ];
		z = acadoVariables.x[ 2+i*NX ];
		psi = acadoVariables.x[ 3+i*NX ];
		v = acadoVariables.x[ 4+i*NX ];
		phi = acadoVariables.x[ 5+i*NX ];
		gamma = acadoVariables.x[ 6+i*NX ];
		
	}
}
double getAcc_norm()
{
	double acc = 0.0;
	for(int i = 0; i < N; i++)
	{
		acc = acc + pow(acadoVariables.u[i*NU + 0], 2);
	}
	return sqrt(acc);
}
double getPhiDot_norm()
{
	double acc = 0.0;
	for(int i = 0; i < N; i++)
	{
		acc = acc + pow(acadoVariables.u[i*NU + 1], 2);
	}
	return sqrt(acc);
}
double getGammaDot_norm()
{
	double acc = 0.0;
	for(int i = 0; i < N; i++)
	{
		acc = acc + pow(acadoVariables.u[i*NU + 2], 2);
	}
	return sqrt(acc);
}

