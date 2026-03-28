#!/usr/bin/env python3
"""Spring-mass-damper simulation."""
import math
def simulate(mass=1.0,k=10.0,c=0.5,x0=1.0,v0=0,dt=0.01,steps=1000):
    x,v=x0,v0;history=[]
    for _ in range(steps):
        history.append((x,v));a=(-k*x-c*v)/mass;v+=a*dt;x+=v*dt
    return history
def analytical(mass,k,c,x0,t):
    omega0=math.sqrt(k/mass);zeta=c/(2*math.sqrt(k*mass))
    if zeta<1:
        wd=omega0*math.sqrt(1-zeta*zeta)
        return x0*math.exp(-zeta*omega0*t)*(math.cos(wd*t)+zeta/math.sqrt(1-zeta*zeta)*math.sin(wd*t))
    return x0*math.exp(-omega0*t)
if __name__=="__main__":
    h=simulate(steps=2000)
    print(f"Spring sim: x0={h[0][0]:.2f}, x_final={h[-1][0]:.4f}")
    assert abs(h[-1][0])<0.01;print("Spring sim OK (damped to ~0)")
