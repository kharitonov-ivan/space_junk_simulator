# Runge Kutta 

Based on http://physics.bu.edu/py502/lectures3/cmotion.pdf


```
call accel(x0,y0,vx0,vy0,t0,ax,ay)
kx1=dt2*ax
ky1=dt2*ay
lx1=dt2*vx0
ly1=dt2*vy0

call accel(x0+lx1,y0+ly1,vx0+kx1,vy0+ky1,th,ax,ay)
kx2=dt2*ax; ky2=dt2*ay
lx2=dt2*(vx0+kx1)
ly2=dt2*(vy0+ky1)

call accel(x0+lx2,y0+ly2,vx0+kx2,vy0+ky2,th,ax,ay)
kx3=dt*ax
ky3=dt*ay
lx3=dt*(vx0+kx2)
ly3=dt*(vy0+ky2)

call accel(x0+lx3,y0+ly3,vx0+kx3,vy0+ky3,t1,ax,ay)
kx4=dt2*ax
ky4=dt2*ay
lx4=dt2*(vx0+kx3)
ly4=dt2*(vy0+ky3)

x1=x0+(lx1+2.d0*lx2+lx3+lx4)/3.d0
y1=y0+(ly1+2.d0*ly2+ly3+ly4)/3.d0
vx1=vx0+(kx1+2.d0*kx2+kx3+kx4)/3.d0
vy1=vy0+(ky1+2.d0*ky2+ky3+ky4)/3.d0
```

Accelerations calculations
```
r=sqrt(x**2+y**2)
v2=vx**2+vy**2
v1=sqrt(v2)
!*** evaluates the acceleration due to gravitation
r3=1.d0/r**3
ax=-gm*x*r3
ay=-gm*y*r3
!*** evaluates the acceleration due to air drag
if (v1 > 1.d-12) then
ad=dragc*airdens(r)*v2
ax=ax-ad*vx/v1
ay=ay-ad*vy/v1
end if
!*** evaluates the acceleration due to rocket motor thrust
if (t < tbrake .and. v1 > 1.d-12) then
ax=ax-arocket*vx/v1
ay=ay-arocket*vy/v1
end if
```