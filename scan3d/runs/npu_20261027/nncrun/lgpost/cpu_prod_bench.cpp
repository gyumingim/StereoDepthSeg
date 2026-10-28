// lgpost 2 (nncrun lgpipe.h, double log per element) vs product post (no N^2 log). PC x86 1 thread.
#include <cstdio>
#include <cmath>
#include <vector>
#include <chrono>
#include <algorithm>
#include <cstdint>
static inline double logsig(double z) { return z >= 0 ? -std::log1p(std::exp(-z)) : z - std::log1p(std::exp(z)); }
static double now(){return std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now().time_since_epoch()).count();}
const int N=512, R=2*N+2;
std::vector<int> post2(const float*o){ std::vector<double> sc(N*N),rmax(N),l1v(N); std::vector<int> m0(N),m1(N),mm;
 for(int j=0;j<N;j++) l1v[j]=logsig(o[j*R+2*N+1]);
 for(int i=0;i<N;i++){double l0=logsig(o[i*R+2*N]); for(int j=0;j<N;j++){float pp=o[i*R+j]*o[j*R+N+i]; sc[i*N+j]=std::log(std::max((double)pp,1e-30))+l0+l1v[j];}}
 for(int i=0;i<N;i++){int bj=0; for(int j=1;j<N;j++) if(sc[i*N+j]>sc[i*N+bj]) bj=j; m0[i]=bj; rmax[i]=sc[i*N+bj];}
 for(int j=0;j<N;j++){int b=0; for(int i=1;i<N;i++) if(sc[i*N+j]>sc[b*N+j]) b=i; m1[j]=b;}
 for(int i=0;i<N;i++) if(m1[m0[i]]==i && std::exp(rmax[i])>0.1){mm.push_back(i);mm.push_back(m0[i]);} return mm;}
std::vector<int> postp(const float*o){ static std::vector<float> P(N*N); std::vector<float> s0(N),s1(N),cb(N,-1.f); std::vector<int> m0(N),m1(N,0),mm;
 for(int i=0;i<N;i++){s0[i]=1.f/(1.f+std::exp(-o[i*R+2*N])); s1[i]=1.f/(1.f+std::exp(-o[i*R+2*N+1]));}
 for(int i=0;i<N;i++){const float*a=o+i*R; float best=-1.f; int bj=0;
   for(int j=0;j<N;j++){float p=a[j]*o[j*R+N+i]; P[i*N+j]=p; float q=p*s1[j]; if(q>best){best=q;bj=j;}}
   m0[i]=bj;}
 for(int i=0;i<N;i++){float s=s0[i]; const float*p=&P[i*N]; for(int j=0;j<N;j++){float q=p[j]*s; if(q>cb[j]){cb[j]=q;m1[j]=i;}}}
 for(int i=0;i<N;i++){int j=m0[i]; if(m1[j]==i){double r=std::log(std::max((double)(P[i*N+j]*s1[j]),1e-30))+std::log((double)s0[i]); if(r>std::log(0.1)){mm.push_back(i);mm.push_back(j);}}} return mm;}
int main(){ std::vector<float> o(13*N*R); FILE*f=fopen("/tmp/lgp/out13.bin","rb"); fread(o.data(),4,o.size(),f); fclose(f);
 int same=0; for(int k=0;k<13;k++) same+= post2(&o[k*N*R])==postp(&o[k*N*R]); printf("same %d/13\n",same);
 for(int rep=0;rep<3;rep++){ std::vector<double> t2,tp; for(int it=0;it<10;it++) for(int k=0;k<13;k++){double a=now(); auto x=post2(&o[k*N*R]); double b=now(); auto y=postp(&o[k*N*R]); double c=now(); t2.push_back(b-a); tp.push_back(c-b);} 
  std::sort(t2.begin(),t2.end()); std::sort(tp.begin(),tp.end()); printf("rep %d: post2 p50 %.0f us, prod p50 %.0f us\n",rep,t2[t2.size()/2],tp[tp.size()/2]); } }
