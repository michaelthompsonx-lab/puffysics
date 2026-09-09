// nvcc -O3 -arch=native -DB3_TWIST_APPROX=1 bench_math.cu -o /tmp/bench-math
#include "puffysics.cuh"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#define CUDA(c) do { cudaError_t e=(c); if(e!=cudaSuccess) { fprintf(stderr,"%s: %s\n",#c,cudaGetErrorString(e)); exit(1); } } while(0)
__global__ void evaluate(const B3Quat* q, float* out, int n, int iters) {
    int i=blockIdx.x*blockDim.x+threadIdx.x;
    if(i>=n) return;
    B3Quat p=q[i]; float sum=0;
    for(int j=0;j<iters;++j) {
        float a=b3_twist(p);
        sum = j == 0 ? a : sum+a;
        // Dependent input prevents hoisting, retains different per-thread angles.
        p.v.z=fmaf(a,0.00001f,p.v.z);
    }
    out[i]=sum;
}
int main() {
    int n=1<<20;
    std::vector<B3Quat> q(n);
    std::vector<float> out(n);
    unsigned rng=123;
    for(int i=0;i<n;++i) {
        rng=1664525*rng+1013904223;
        double a=((double)rng/4294967296.0*2-1)*3.141592653589793;
        float scale=ldexpf(1.0f,(i%81)-40);
        q[i]=b3_q(0,0,scale*sinf(a),scale*cosf(a));
    }
    float special[]={0.0f,-0.0f,1.0f,-1.0f,INFINITY,-INFINITY,NAN};
    for(int i=0;i<49;++i) q[i]=b3_q(0,0,special[i%7],special[i/7]);
    B3Quat* dq; float* dout;
    CUDA(cudaMalloc(&dq,n*sizeof(B3Quat))); CUDA(cudaMalloc(&dout,n*sizeof(float)));
    CUDA(cudaMemcpy(dq,q.data(),n*sizeof(B3Quat),cudaMemcpyHostToDevice));
    evaluate<<<(n+255)/256,256>>>(dq,dout,n,1);
    CUDA(cudaGetLastError());
    CUDA(cudaMemcpy(out.data(),dout,n*sizeof(float),cudaMemcpyDeviceToHost));
    double worst=0, sum2=0; int finite=0;
    for(int i=0;i<n;++i) {
        double y=q[i].v.z, x=q[i].s;
        if(x<0) { x=-x;y=-y; }
        double ref=2*atan2(y,x), err=fabs(out[i]-ref);
        if(isnan(ref)) {
            if(!isnan(out[i])) {fprintf(stderr,"NaN handling failure %d\n",i);return 2;}
            continue;
        }
        if(ref == 0 && (out[i] != 0 || signbit(out[i]) != signbit(ref))) {
            fprintf(stderr,"signed zero failure %d\n",i);return 2;
        }
        double tolerance = B3_TWIST_APPROX == 4 ? 2e-4 : 1e-5;
        if(!isfinite(out[i]) || err>tolerance) {fprintf(stderr,"accuracy failure %d %g %g\n",i,out[i],ref);return 2;}
        worst=std::max(worst,err); sum2+=err*err; ++finite;
    }
    cudaEvent_t start,end; CUDA(cudaEventCreate(&start)); CUDA(cudaEventCreate(&end));
    std::vector<float> times;
    for(int trial=0;trial<6;++trial) {
        CUDA(cudaEventRecord(start)); evaluate<<<(n+255)/256,256>>>(dq,dout,n,64);
        CUDA(cudaGetLastError()); CUDA(cudaEventRecord(end)); CUDA(cudaEventSynchronize(end));
        float ms; CUDA(cudaEventElapsedTime(&ms,start,end)); if(trial) times.push_back(ms);
    }
    std::sort(times.begin(),times.end());
    printf("mode=%d max_angle_error=%.9g rms=%.9g median_ms=%.6f min_ms=%.6f max_ms=%.6f\n",B3_TWIST_APPROX,worst,sqrt(sum2/finite),times[2],times.front(),times.back());
    CUDA(cudaFree(dq)); CUDA(cudaFree(dout)); CUDA(cudaEventDestroy(start)); CUDA(cudaEventDestroy(end));
}
