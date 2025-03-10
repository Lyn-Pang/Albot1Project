/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Kalman_filter.h
 * Author: Lyn Pang
 *
 * Created on November 1, 2016, 2:29 PM
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/* 1 Dimension */
typedef struct 
{
    float x;  /* state */
    float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
    float q;  /* process(predict) noise convariance */
    float r;  /* measure noise convariance */
    float p;  /* estimated error convariance */
    float gain;
} kalman1_state;

/* 2 Dimension */
typedef struct 
{
    float x[2];     /* state: [0]-angle [1]-diffrence of angle, 2x1 */
    float A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
    float H[2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
    float q[2];     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
    float r;        /* measure noise convariance */
    float p[2][2];  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
    float gain[2];  /* 2x1 */
} kalman2_state;                   

void kalman1_init(kalman1_state *state, float init_x, float init_p);
float kalman1_filter(kalman1_state *state, float z_measure);
void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2]);
float kalman2_filter(kalman2_state *state, float z_measure);


#endif /* KALMAN_FILTER_H */

