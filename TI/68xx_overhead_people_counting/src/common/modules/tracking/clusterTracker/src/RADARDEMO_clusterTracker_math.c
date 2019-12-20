/*!
 *   \file   RADARDEMO_clusterTracker_math.c
 *
 *   \brief   Math functions for RADARDEMO_clusterTracker.
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#include "modules/tracking/clusterTracker/api/RADARDEMO_clusterTracker.h"
//#include "modules/tracking/common/api/RADARDEMO_tracking_commonDef.h"
#include "RADARDEMO_clusterTracker_priv.h"
#include <math.h>
#include <string.h>
//#include <stdio.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif


void RADARDEMO_tracker_matrixMultiply(uint16_t m1, uint16_t m2, uint16_t m3, float *A, float *B, float *C)
{
	//C(m1*m3) = A(m1*m2)*B(m2*m3)
	uint16_t i,j, k;
	for (i = 0; i < m1; i++)
	{
		for (j = 0; j < m3; j++)
		{			
			C[i*m3 + j] = 0;
			for (k = 0; k < m2; k++)
				C[i*m3+j] += A[i*m2+k] * B[k*m3 + j];
		}
	}
}

void RADARDEMO_tracker_matrixConjugateMultiply(uint16_t m1, uint16_t m2, uint16_t m3, float *A, float *B, float *C)
{
	//C(m1*m3) = A(m1*m2)*B(m2*m3)
	uint16_t i,j, k;
	for (i = 0; i < m1; i++)
	{
		for (j = 0; j < m3; j++)
		{			
			C[i*m3 + j] = 0;
			for (k = 0; k < m2; k++)
				C[i*m3+j] += A[i*m2+k] * B[k + j*m2];
		}
	}
}

void RADARDEMO_tracker_matrixAdd(uint16_t dim, float *A, float *B, float *C)
{
	//C = A + B
	uint16_t i,j;
	for (i = 0; i < dim; i++)
	{
		for (j = 0; j < dim; j++)
		{			
			C[i*dim+j] = A[i*dim+j] + B[i*dim + j];
		}
	}
}

void RADARDEMO_tracker_matrixSub(uint16_t dim, float *A, float *B, float *C)
{
	//C = A + B
	uint16_t i,j;
	for (i = 0; i < dim; i++)
	{
		for (j = 0; j < dim; j++)
		{			
			C[i*dim+j] = A[i*dim+j] - B[i*dim + j];
		}
	}
}


void RADARDEMO_tracker_computeH(float *S, float *H)
{
    // S = [posx posy velx vely] 
    // H = [range azimuth doppler] 
    float posx = S[0]; 
	float posy = S[1]; 
	float velx = S[2]; 
	float vely = S[3];
	// calc range 
	H[0] = (float)(sqrt(posx*posx + posy*posy)); 

    //calc azimuth
	if (posx == 0)
        H[1] = RADARDEMO_CLUSTERTRACKER_PI/2;
    else if (posx > 0)
        H[1] = (float)(atan(posy/posx));
    else
        H[1] = (float)(atan(posy/posx)) + RADARDEMO_CLUSTERTRACKER_PI;
    
    //calc doppler;
    H[2] = (posx*velx+posy*vely)/H[0];
}


void RADARDEMO_clusterTracker_distCalc(RADARDEMO_trackerInputInternal_dataType *measureInfo, RADARDEMO_trackerInternal_dataType *tracker, float *dist)
{
	    dist[0] = measureInfo->range * measureInfo->range + 
		+ tracker->H_s_apriori_hat[0] * tracker->H_s_apriori_hat[0] 
		- 2 * measureInfo->range * tracker->H_s_apriori_hat[0] * (float)(cos(tracker->H_s_apriori_hat[1] - measureInfo->azimuth));

}

float RADARDEMO_clusterTracker_associateThresholdCalc(float presetTH, RADARDEMO_trackerInternal_dataType *tracker)
{
	float threshold;
	threshold = presetTH + 4*tracker->diagonal2;
	return(threshold);
}

void RADARDEMO_clusterTracker_combineMeasure(RADARDEMO_trackerInputInternal_dataType *inputInfo, int16_t *associatedList, uint16_t numAssoc, RADARDEMO_trackerInputInternal_dataType *combinedInput)
{
	float        range = 0; 
	float        azimuth = 0;                       /**< cluster center location in angle .*/
	float        doppler = 0;                       /**< averge doppler of the cluster.*/
	float        xSize = 0;                         /**< max distance of point from cluster center in x direction.*/
	float        ySize = 0;                         /**< max distance of point from cluster center in y direction.*/
	float        rangeVar = 0;                      /**< variance of the range estimation */
	float        angleVar = 0;                      /**< variance of the angle estimation */
	float        dopplerVar = 0;                    /**< variance of the doppler estimation */
	uint16_t     i, size, totSize, mid;
	float        totSizeInv;
	totSize  = 0;
	for (i = 0; i < numAssoc; i++)
	{
		mid = associatedList[i];
		size = inputInfo[mid].numPoints;
		totSize += size;
		range += inputInfo[mid].range * size;
		azimuth += inputInfo[mid].azimuth * size;
		doppler += inputInfo[mid].doppler * size;
		rangeVar += inputInfo[mid].rangeVar * size;
		angleVar += inputInfo[mid].angleVar * size;
		dopplerVar += inputInfo[mid].dopplerVar * size;
		if (xSize < inputInfo[mid].xSize)
			xSize = inputInfo[mid].xSize;
		if (ySize < inputInfo[mid].ySize)
			ySize = inputInfo[mid].ySize;
	}
	totSizeInv = (float)(1.0 / (float)(totSize));
	combinedInput->range = range * totSizeInv;	
	combinedInput->azimuth = azimuth * totSizeInv;	
	combinedInput->doppler = doppler * totSizeInv;	
	combinedInput->rangeVar = rangeVar * totSizeInv;	
	combinedInput->angleVar = angleVar * totSizeInv;	
	combinedInput->dopplerVar = dopplerVar * totSizeInv;	
	combinedInput->xSize = xSize;	
	combinedInput->ySize = ySize;	
	combinedInput->numPoints = totSize;

}

float RADARDEMO_clusterTracker_IIRFilter(float yn, float xn, float forgetFactor)
{
	float ynext;
	ynext = (float)(yn * (1.0 - forgetFactor) + xn * forgetFactor);
	return(ynext);
}

void RADARDEMO_clusterTracker_computeCartesian(float *H, float *S)
{
    // H = [range azimuth doppler] 
    // S = [posx posy velx vely] 

    S[0] = H[0]*(float)(cos(H[1]));
    S[1] = H[0]*(float)(sin(H[1]));
    S[2] = H[2]*(float)(cos(H[1]));
    S[3] = H[2]*(float)(sin(H[1]));	
}

void RADARDEMO_clusterTracker_computeJacobian(float *S, float *J)
{
    // S = [posx posy velx vely] 
    // J is with size [3x4]

    float r2 = (S[0]*S[0] + S[1]*S[1]);
	float r = (float)(sqrt(r2));
    J[0] = S[0]/r; //J[0][0]
    J[1] = S[1]/r; //J[0][1]
	J[2] = 0;
	J[3] = 0;

    J[4] = -S[1]/r2; //J[1][0]
    J[5] = S[0]/r2;  //J[1][1]
	J[6] = 0;
	J[7] = 0;
    
    J[8] = (float)((S[1]*(S[2]*S[1] - S[0]*S[3]))/r/r2); // J[2][0]
    J[9] = (float)((S[0]*(S[3]*S[0] - S[2]*S[1]))/r/r2);// J[2][1]
    J[10] = (float)(S[0]/r); //J[2][2]
    J[11] = (float)(S[1]/r); //J[2][3]

}


void RADARDEMO_cluster_matInv3(float *A, float *Ainv)
{
	uint16_t i, j, k, dim;
	//Ac and Ainv are both 3 by 3 Matrix
	float Ac[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	float Acinv[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

	RADARDEMO_tracker_cholesky3(A, Ac);

	//Acinv(1,1) = 1/Ac(1, 1); 
    //Acinv(2,2) = 1/Ac(2, 2); 
    //Acinv(3,3) = 1/Ac(3, 3); 
	dim = 3;
	for (i = 0; i < dim; i++)
		Acinv[i*dim+i] = (float)(1.0/Ac[i*dim+i]);

	//Acinv(2,1) = -Ac(2,1)*Acinv(1,1)*Acinv(2,2);
	Acinv[3] = -Ac[3]*Acinv[0]*Acinv[4];

    //Acinv(3,2) = -Ac(3,2)*Acinv(2,2)*Acinv(3,3);
    Acinv[7] = -Ac[7]*Acinv[4]*Acinv[8];

	//Acinv(3,1) = (Ac(2,1)*Ac(3,2)-Ac(2,2)*Ac(3,1))*Acinv(1,1)*Acinv(2,2)*Acinv(3,3);
	Acinv[6] = (Ac[3]*Ac[7] - Ac[4]*Ac[6])*Acinv[0] * Acinv[4] * Acinv[8];

	//Ainv = Acinv' * Acinv;
	for (i = 0; i < dim; i++)
	{
		for (j = 0; j < dim; j++)
		{			
			Ainv[i*dim + j] = 0;
			for (k = 0; k < dim; k++)
				Ainv[i*dim+j] += Acinv[k*dim+i] * Acinv[k*dim + j];
		}
	}
		
}


void RADARDEMO_clusterTracker_kalmanUpdate(RADARDEMO_trackerInternal_dataType *tracker, 
	                                       RADARDEMO_trackerInputInternal_dataType  *combinedInput, 
										   float *F, 
										   float *Q, 
										   float *R,
										   float *scratchPad)
{
	uint16_t i, j;
	//float J[12];
	float H[3];
	float *J, *temp1, *temp2, *temp3, *invMatrix, *K; 
	//float temp1[16], temp2[16], temp3[9], invMatrix[9], K[12];
	float avg;

	temp1 = &scratchPad[0];
	temp2 = &scratchPad[16];
	temp3 = &scratchPad[32];
    invMatrix = &scratchPad[48];
	K = &scratchPad[64];
	J = &scratchPad[80];


	//obj.P_apriori(:,:,tid) = obj.F * obj.P(:,:,tid) * obj.F' + obj.Q;
	RADARDEMO_tracker_matrixMultiply(4, 4, 4, F, tracker->P, temp1);
	RADARDEMO_tracker_matrixConjugateMultiply(4, 4, 4, temp1, F, tracker->P_apriori);
	RADARDEMO_tracker_matrixAdd(4, tracker->P_apriori, Q, tracker->P_apriori);

	//%Enforce symmetry constraint on P_apriori
	//obj.P_apriori(:,:,tid) = 1/2 * (obj.P_apriori(:,:,tid)+obj.P_apriori(:,:,tid)');
	for (i = 0; i < 4; i++)
	{
		for (j = i; j < 4; j++)
		{
			avg = tracker->P_apriori[i*4 + j] + tracker->P_apriori[j*4 + i]; 
			avg = (float)(avg * 0.5);
			tracker->P_apriori[i*4 + j] = avg;
			tracker->P_apriori[j*4 + i] = avg;
		}
	}

	//%Compute the Jacobian (can be over-written)
	//J = computeJacobian(obj.S_apriori_hat(:,tid));
	RADARDEMO_clusterTracker_computeJacobian(tracker->S_apriori_hat, J);

	//% Measurement update
	//% Kalman gain using matrix inverse via Cholesky decomposition
	//K = obj.P_apriori(:,:,tid) * J' * matinv(J * obj.P_apriori(:,:,tid) * J' + R);
	RADARDEMO_tracker_matrixMultiply(3, 4, 4, J, tracker->P_apriori, temp2); // 3*4: J * P_apriori
	RADARDEMO_tracker_matrixConjugateMultiply(3, 4, 3, temp2, J, temp3);     // 3*3: J * P_apriori * J'
	RADARDEMO_tracker_matrixAdd(3, temp3, R, temp3);                         // 3*3:J * P_apriori * J' + R
	RADARDEMO_cluster_matInv3(temp3, invMatrix);                             // inv(J * P_apriori * J' + R)
	RADARDEMO_tracker_matrixConjugateMultiply(4, 4, 3, tracker->P_apriori, J, temp2); // 4*3: P_apriori * J'
	RADARDEMO_tracker_matrixMultiply(4, 3, 3, temp2, invMatrix, K);                    // 4*3: K = P_apriori * J' * invMat

	//obj.P(:,:,tid) = obj.P_apriori(:,:,tid) - K * J * obj.P_apriori(:,:,tid);
	RADARDEMO_tracker_matrixMultiply(4, 3, 4, K, J, temp1);                          // 4*4: K*J
	RADARDEMO_tracker_matrixMultiply(4, 4, 4, temp1, tracker->P_apriori, temp2);     // 4*4: K*J*P_priori
	RADARDEMO_tracker_matrixSub(4, tracker->P_apriori, temp2, tracker->P);           // 4*4: P_priori - K*J*P_priori


	//obj.S_hat(:,tid) = obj.S_apriori_hat(:,tid) + K * (um - obj.H_s_apriori_hat(:,tid));
	H[0] = combinedInput->range - tracker->H_s_apriori_hat[0];
	H[1] = combinedInput->azimuth - tracker->H_s_apriori_hat[1];
	H[2] = combinedInput->doppler - tracker->H_s_apriori_hat[2];
	RADARDEMO_tracker_matrixMultiply(4, 3, 1, K, H, temp1);                   // 4*1: K*(um - H_s_apriori_hat);
	for (i = 0; i < 4; i++)
		tracker->S_hat[i] = tracker->S_apriori_hat[i] + temp1[i];             // 4*1: S_hat = S_priori + K*(um - H_s_apriori_hat);


}

void RADARDEMO_clusterTracker_kalmanUpdateWithNoMeasure(RADARDEMO_trackerInternal_dataType *tracker, float *F, float *Q)
{
	uint16_t i, j, index;
	uint16_t dim = 4;
	float temp1[16];

	//obj.P_apriori(:,:,tid) = obj.F * obj.P(:,:,tid) * obj.F' + obj.Q;
	RADARDEMO_tracker_matrixMultiply(4, 4, 4, F, tracker->P, temp1);
	RADARDEMO_tracker_matrixConjugateMultiply(4, 4, 4, temp1, F, tracker->P_apriori);
	RADARDEMO_tracker_matrixAdd(4, tracker->P_apriori, Q, tracker->P_apriori);

	//obj.P(:,:,tid) = obj.P_apriori(:,:,tid);
	//obj.S_hat(:,tid) = obj.S_apriori_hat(:,tid);
	for (i = 0; i < dim; i++)
	{
		tracker->S_hat[i] = tracker->S_apriori_hat[i];
		for (j = 0; j < dim; j++)
		{
			index = i * dim + j;
			tracker->P[index] = tracker->P_apriori[index];
		}
	}

}


void RADARDEMO_tracker_cholesky3(float *A, float *G)
{
	float v[3] = {0, 0, 0};
	float temp;
    uint16_t i, j, k;
	uint16_t dim = 3;

    for (j = 0; j < dim; j++)
    {
		//v(j:n,1) = A(j:n,j);
		for (i = j; i < dim; i++)
		    v[i] = A[i * dim + j];
        
		for (k = 0; k < j; k++)
        {
			//v(j:n,1) = v(j:n,1) - G(j,k)*G(j:n,k);
			for (i = j; i < dim; i++)
				v[i] = v[i] - G[j*dim + k] * G[i*dim + k];
		}
    
        //G(j:n,j) = v(j:n,1)/sqrt(v(j));
		temp = 1.0/sqrt(v[j]);
		for (i = j; i < dim; i++)
			G[i*dim + j] = v[i]*temp;
	}

}
