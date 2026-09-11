#ifndef NBODY_BATCH_CUH
#define NBODY_BATCH_CUH
// Fixed-stride, isolated CUDA worlds. No RL framework dependency.
// Every operation uses stream; acceleration/step enqueue work without host waits.
// Initial/reset uploads invalidate acceleration. External state edits must call
// invalidate(). A step includes initial force only if the cache is invalid.
#include "nbody.cuh"
#ifdef __CUDACC__
#include <stddef.h>
struct NbodyBatchState {
    float *x, *y, *z, *mass, *vx, *vy, *vz, *ax, *ay, *az;
    int worlds, bodies;
};
enum NbodyBatchBackend { NBATCH_AUTO, NBATCH_WARP, NBATCH_CTA128, NBATCH_CTA256 };
struct NbodyBatchTune {
    NbodyBatchBackend backend;
    float step_us[3]; // warp, CTA128, CTA256; NaN for an ineligible candidate
    size_t temporary_bytes; // one state backup, released before returning
};
static __device__ NbodyPoint nb_point(NbodyBatchState s, int i) {
    return {s.x[i], s.y[i], s.z[i], s.mass[i]};
}
static __device__ void nb_accel_store(NbodyBatchState s, int i, NbodyVec a) {
    s.ax[i]=a.x; s.ay[i]=a.y; s.az[i]=a.z;
}
// One warp per world. All 32 lanes participate in shuffles, including padding.
template<bool STEP>
static __global__ void nb_warp(NbodyBatchState s, NbodyConfig cfg, float dt) {
    int tid=blockIdx.x*blockDim.x+threadIdx.x, env=tid/32, lane=tid%32;
    if (env>=s.worlds) return;
    int i=env*s.bodies+lane; bool active=lane<s.bodies;
    NbodyPoint p={}; NbodyVec v={},a={};
    if(active) {
        p=nb_point(s,i);
        if(STEP) {
            v={s.vx[i]+0.5f*dt*s.ax[i],s.vy[i]+0.5f*dt*s.ay[i],s.vz[i]+0.5f*dt*s.az[i]};
            p.x+=dt*v.x; p.y+=dt*v.y; p.z+=dt*v.z;
        }
    }
    for(int j=0;j<s.bodies;++j) {
        NbodyPoint q={__shfl_sync(0xffffffffu,p.x,j),__shfl_sync(0xffffffffu,p.y,j),
            __shfl_sync(0xffffffffu,p.z,j),__shfl_sync(0xffffffffu,p.mass,j)};
        if(active && lane!=j) nbody_add_source(p,q,cfg.gravity,cfg.softening*cfg.softening,&a.x,&a.y,&a.z);
    }
    if(active) {
        nb_accel_store(s,i,a);
        if(STEP) {
            s.x[i]=p.x;s.y[i]=p.y;s.z[i]=p.z;
            s.vx[i]=v.x+0.5f*dt*a.x;s.vy[i]=v.y+0.5f*dt*a.y;s.vz[i]=v.z+0.5f*dt*a.z;
        }
    }
}
// Source tiles cannot cross a world boundary. grid.y selects a world; grid.x
// partitions its target bodies. For small worlds this is one CTA/world.
template<int BLOCK>
static __global__ void nb_cta(NbodyBatchState s,NbodyConfig cfg) {
    int env=blockIdx.y, local=blockIdx.x*BLOCK+threadIdx.x, base=env*s.bodies;
    NbodyPoint p={};NbodyVec a={};
    if(local<s.bodies) p=nb_point(s,base+local);
    __shared__ NbodyPoint tile[BLOCK];
    for(int first=0;first<s.bodies;first+=BLOCK) {
        int count=min(BLOCK,s.bodies-first),j=first+threadIdx.x;
        if(j<s.bodies) tile[threadIdx.x]=nb_point(s,base+j);
        __syncthreads();
        if(local<s.bodies) for(int k=0;k<count;++k) {
            if(local!=first+k) nbody_add_source(p,tile[k],cfg.gravity,cfg.softening*cfg.softening,&a.x,&a.y,&a.z);
        }
        __syncthreads();
    }
    if(local<s.bodies) nb_accel_store(s,base+local,a);
}
// For <=BLOCK bodies the entire Verlet step fits in one independent CTA.
template<int BLOCK>
static __global__ void nb_cta_step(NbodyBatchState s,NbodyConfig cfg,float dt) {
    int local=threadIdx.x,i=blockIdx.x*s.bodies+local;
    __shared__ NbodyPoint p[BLOCK];NbodyVec v={},a={};
    if(local<s.bodies) {
        v={s.vx[i]+0.5f*dt*s.ax[i],s.vy[i]+0.5f*dt*s.ay[i],s.vz[i]+0.5f*dt*s.az[i]};
        p[local]=nb_point(s,i);p[local].x+=dt*v.x;p[local].y+=dt*v.y;p[local].z+=dt*v.z;
    }
    __syncthreads();
    if(local<s.bodies) {
        NbodyPoint t=p[local];
        for(int j=0;j<s.bodies;++j) if(local!=j)
            nbody_add_source(t,p[j],cfg.gravity,cfg.softening*cfg.softening,&a.x,&a.y,&a.z);
        s.x[i]=t.x;s.y[i]=t.y;s.z[i]=t.z;
        s.vx[i]=v.x+0.5f*dt*a.x;s.vy[i]=v.y+0.5f*dt*a.y;s.vz[i]=v.z+0.5f*dt*a.z;
        nb_accel_store(s,i,a);
    }
}
static __global__ void nb_kick(NbodyBatchState s,float dt,bool drift) {
    int i=blockIdx.x*blockDim.x+threadIdx.x;
    if(i>=s.worlds*s.bodies) return;
    s.vx[i]+=0.5f*dt*s.ax[i];s.vy[i]+=0.5f*dt*s.ay[i];s.vz[i]+=0.5f*dt*s.az[i];
    if(drift) {s.x[i]+=dt*s.vx[i];s.y[i]+=dt*s.vy[i];s.z[i]+=dt*s.vz[i];}
}
struct NbodyBatch {
    NbodyBatchState s={};float *storage=nullptr;cudaStream_t stream=nullptr;
    bool own_stream=false,valid=false;NbodyConfig cached={};
    NbodyBatchBackend backend=NBATCH_AUTO;size_t persistent_bytes=0;
    NbodyBatch()=default;NbodyBatch(const NbodyBatch&)=delete;NbodyBatch& operator=(const NbodyBatch&)=delete;
    ~NbodyBatch() { if(storage) {cudaStreamSynchronize(stream);cudaFree(storage);} if(own_stream) cudaStreamDestroy(stream); }
    void invalidate() {valid=false;}
    static bool cfg_ok(NbodyConfig c) {return isfinite(c.gravity)&&isfinite(c.softening)&&nbody_cfg_ok(c);}
    cudaError_t init(int worlds,int bodies,NbodyBatchBackend which=NBATCH_AUTO,cudaStream_t external=nullptr) {
        if(storage || worlds<1 || worlds>4096 || bodies<1 || bodies>4096
                || which<NBATCH_AUTO || which>NBATCH_CTA256 || (which==NBATCH_WARP && bodies>32)) return cudaErrorInvalidValue;
        s.worlds=worlds;s.bodies=bodies;backend=which;
        if(external) stream=external;
        else {cudaError_t e=cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking);if(e!=cudaSuccess)return e;own_stream=true;}
        size_t total=size_t(worlds)*bodies;persistent_bytes=total*10*sizeof(float);
        cudaError_t e=cudaMalloc(&storage,persistent_bytes);if(e!=cudaSuccess)return e;
        s.x=storage;s.y=storage+total;s.z=storage+2*total;s.mass=storage+3*total;
        s.vx=storage+4*total;s.vy=storage+5*total;s.vz=storage+6*total;
        s.ax=storage+7*total;s.ay=storage+8*total;s.az=storage+9*total;return cudaSuccess;
    }
    // Host data has the same 10-plane SoA layout; synchronous convenience I/O.
    cudaError_t upload(const float *host) {
        if(!storage||!host)return cudaErrorInvalidValue;invalidate();
        cudaError_t e=cudaMemcpyAsync(storage,host,persistent_bytes,cudaMemcpyHostToDevice,stream);
        return e==cudaSuccess?cudaStreamSynchronize(stream):e;
    }
    cudaError_t download(float *host) {
        if(!storage||!host)return cudaErrorInvalidValue;
        cudaError_t e=cudaMemcpyAsync(host,storage,persistent_bytes,cudaMemcpyDeviceToHost,stream);
        return e==cudaSuccess?cudaStreamSynchronize(stream):e;
    }
    NbodyBatchBackend selected() const {
        return backend==NBATCH_AUTO?(s.bodies<=32?NBATCH_WARP:
            s.bodies>128&&s.bodies<=256?NBATCH_CTA256:NBATCH_CTA128):backend;
    }
    cudaError_t acceleration(NbodyConfig cfg) {
        if(!storage||!cfg_ok(cfg))return cudaErrorInvalidValue;
        auto b=selected();
        if(b==NBATCH_WARP) nb_warp<false><<<(s.worlds+3)/4,128,0,stream>>>(s,cfg,0);
        else if(b==NBATCH_CTA256) nb_cta<256><<<dim3((s.bodies+255)/256,s.worlds),256,0,stream>>>(s,cfg);
        else nb_cta<128><<<dim3((s.bodies+127)/128,s.worlds),128,0,stream>>>(s,cfg);
        cudaError_t e=cudaGetLastError();if(e==cudaSuccess){valid=true;cached=cfg;}return e;
    }
    cudaError_t step(NbodyConfig cfg,float dt) {
        if(!storage||!cfg_ok(cfg)||!isfinite(dt))return cudaErrorInvalidValue;
        if(dt==0)return cudaSuccess;
        if(!valid||cached.gravity!=cfg.gravity||cached.softening!=cfg.softening) {
            cudaError_t e=acceleration(cfg);if(e!=cudaSuccess)return e;
        }
        auto b=selected();
        if(b==NBATCH_WARP)nb_warp<true><<<(s.worlds+3)/4,128,0,stream>>>(s,cfg,dt);
        else if(b==NBATCH_CTA128 && s.bodies<=128)nb_cta_step<128><<<s.worlds,128,0,stream>>>(s,cfg,dt);
        else if(b==NBATCH_CTA256 && s.bodies<=256)nb_cta_step<256><<<s.worlds,256,0,stream>>>(s,cfg,dt);
        else {
            nb_kick<<<(s.worlds*s.bodies+255)/256,256,0,stream>>>(s,dt,true);
            cudaError_t e=cudaGetLastError();if(e!=cudaSuccess)return e;
            e=acceleration(cfg);if(e!=cudaSuccess)return e;
            nb_kick<<<(s.worlds*s.bodies+255)/256,256,0,stream>>>(s,dt,false);
        }
        return cudaGetLastError();
    }
    // Explicit startup tuning on this GPU and batch shape. Measures full steps,
    // restores all particle state and force-cache metadata, then selects the
    // fastest direct backend. Not allowed inside stream capture.
    cudaError_t tune(NbodyConfig cfg,float dt,NbodyBatchTune *result,int repeats=7) {
        if(!storage||!result||!cfg_ok(cfg)||!isfinite(dt)||dt<=0||repeats<1||repeats>15)
            return cudaErrorInvalidValue;
        float *backup=nullptr;cudaEvent_t start=nullptr,end=nullptr;
        NbodyBatchBackend original=backend,best=selected();
        bool original_valid=valid,saved=false;NbodyConfig original_cfg=cached;
        NbodyBatchTune measured={best,{NAN,NAN,NAN},persistent_bytes};
        float fastest=INFINITY;
        auto run=[&]() -> cudaError_t {
#define NBT_TRY(x) do{cudaError_t e=(x);if(e!=cudaSuccess)return e;}while(0)
            NBT_TRY(cudaMalloc(&backup,persistent_bytes));
            NBT_TRY(cudaMemcpyAsync(backup,storage,persistent_bytes,cudaMemcpyDeviceToDevice,stream));
            NBT_TRY(cudaStreamSynchronize(stream));saved=true;
            NBT_TRY(cudaEventCreate(&start));NBT_TRY(cudaEventCreate(&end));
            for(int candidate=NBATCH_WARP;candidate<=NBATCH_CTA256;++candidate) {
                if(candidate==NBATCH_WARP&&s.bodies>32)continue;
                backend=static_cast<NbodyBatchBackend>(candidate);valid=false;
                NBT_TRY(cudaMemcpyAsync(storage,backup,persistent_bytes,cudaMemcpyDeviceToDevice,stream));
                NBT_TRY(acceleration(cfg));NBT_TRY(step(cfg,dt));NBT_TRY(step(cfg,dt));
                NBT_TRY(cudaStreamSynchronize(stream));float samples[15];
                for(int k=0;k<repeats;++k) {
                    NBT_TRY(cudaEventRecord(start,stream));NBT_TRY(step(cfg,dt));
                    NBT_TRY(cudaEventRecord(end,stream));NBT_TRY(cudaEventSynchronize(end));
                    NBT_TRY(cudaEventElapsedTime(samples+k,start,end));
                }
                for(int k=1;k<repeats;++k){float v=samples[k];int j=k;while(j>0&&samples[j-1]>v){samples[j]=samples[j-1];--j;}samples[j]=v;}
                measured.step_us[candidate-1]=samples[repeats/2]*1000;
                if(samples[repeats/2]<fastest){fastest=samples[repeats/2];best=backend;}
            }
            return cudaSuccess;
#undef NBT_TRY
        };
        cudaError_t status=run();
        if(saved) {
            cudaError_t e=cudaMemcpyAsync(storage,backup,persistent_bytes,cudaMemcpyDeviceToDevice,stream);
            if(e==cudaSuccess)e=cudaStreamSynchronize(stream);
            if(status==cudaSuccess)status=e;
        }
        if(start)cudaEventDestroy(start);if(end)cudaEventDestroy(end);if(backup)cudaFree(backup);
        valid=original_valid;cached=original_cfg;backend=status==cudaSuccess?best:original;
        if(status==cudaSuccess){measured.backend=best;*result=measured;}return status;
    }
};
#endif
#endif
