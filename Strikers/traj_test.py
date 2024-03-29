# Refer: https://github.com/novice1011/trajectory-planning
import numpy as np
from matplotlib import pyplot as plt


# function for linear interpolation
def line(point0, v0, t0, t1, step=1):
    # Generate a series of timestep
    t = np.arange(t0, t1+step,step)#makit one column
    # Calculate velocity
    v = v0
    #time shift
    Ti = t0
    #equation
    s = point0 + v*(t-Ti)
    v = np.ones(t.size)*v
    a = np.zeros(t.size)
    return (t,s,v,a)

# function for linear interpolation
def line2(point0, point1, t0, t1, step=1):
    # Generate a series of timestep
    t = np.arange(t0, t1+step,step)#makit one column
    # Calculate velocity
    v = (point1-point0)/(t1-t0)
    #time shift
    Ti = t0
    #equation
    s = point0 + v*(t-Ti)
    v = np.ones(t.size)*v
    a = np.zeros(t.size)
    return (t,s,v,a)

def parab(p0, v0, v1, t0, t1, step=1):
    # Generate a series of timestep
    t = np.arange(t0, t1+step,step)
    #calculate acceleration
    a = (v1-v0)/(t1-t0)
    #time shift
    Ti=t0
    # equation
    s = p0  +v0*(t-Ti) +0.5*a*(t-Ti)**2
    v = v0 + a*(t-Ti)
    a = np.ones(t.size)*a
    return (t,s,v,a)


def lspb(via,dur,tb):
    #1. It must start and end at the first and last waypoint respectively with zero velocity
    #2. Note that during the linear phase acceleration is zero, velocity is constant and position is linear in time
    # if acc.min < 0 :
    #     print('acc must bigger than 0')
    #     return 0
    if ((via.size-1) != dur.size):
        print('duration must equal to number of segment which is via-1')
        return 0
    if (via.size <2):
        print('minimum of via is 2')
        return 0
    if (via.size != (tb.size)):
        print('acc must equal to number of via')
        return 0
    
    #=====CALCULATE-VELOCITY-EACH-SEGMENT=====
    v_seg=np.zeros(dur.size)
    for i in range(0,len(via)-1):
        v_seg[i]=(via[i+1]-via[i])/dur[i]

    #=====CALCULATE-ACCELERATION-EACH-VIA=====
    a_via=np.zeros(via.size)
    a_via[0]=(v_seg[0]-0)/tb[0]
    for i in range(1,len(via)-1):
        a_via[i]=(v_seg[i]-v_seg[i-1])/tb[i]
    a_via[-1]=(0-v_seg[-1])/tb[-1]

    #=====CALCULATE-TIMING-EACH-VIA=====
    T_via=np.zeros(via.size)
    T_via[0]=0.5*tb[0]
    for i in range(1,len(via)-1):
        T_via[i]=T_via[i-1]+dur[i-1]
    T_via[-1]=T_via[-2]+dur[-1]

    #=====GENERATING-CHART/GRAPH/FIGURE=====
    # q(t) = q_i + v_{i-1}(t-T_i) + \frac{1}{2}a(t-T_i+\frac{t_i^b}{2})^2  #parabolic phase
    # q(t) = q_i + v_i*(t-T_i)                 #linear phase
    #parabolic
    t,s,v,a = parab(via[0], 0, v_seg[0], T_via[0]-0.5*tb[0], T_via[0]+0.5*tb[0], step=1)
    time    = t
    pos     = s
    speed   = v
    accel   = a
    
    for i in range(1,len(via)-1):
        # linear
        t,s,v,a = line(pos[-1],v_seg[i-1],T_via[i-1]+0.5*tb[i],T_via[i]-0.5*tb[i+1],0.01)
        time    = np.concatenate((time,t))
        pos     = np.concatenate((pos,s))
        speed   = np.concatenate((speed,v))
        accel   = np.concatenate((accel,a))

        #parabolic
        t,s,v,a = parab(pos[-1], v_seg[i-1], v_seg[i], T_via[i]-0.5*tb[i+1], T_via[i]+0.5*tb[i+1], 0.01)
        time    = np.concatenate((time,t))
        pos     = np.concatenate((pos,s))
        speed   = np.concatenate((speed,v))
        accel   = np.concatenate((accel,a))

    # linear
    t,s,v,a = line(pos[-1],v_seg[-1],T_via[-2]+0.5*tb[-2],T_via[-1]-0.5*tb[-1],0.01)
    time    = np.concatenate((time,t))
    pos     = np.concatenate((pos,s))
    speed   = np.concatenate((speed,v))
    accel   = np.concatenate((accel,a))

    #parabolic
    t,s,v,a = parab(pos[-1], v_seg[-1], 0, T_via[-1]-0.5*tb[-1],  T_via[-1]+0.5*tb[-1], 0.01)
    time    = np.concatenate((time,t))
    pos     = np.concatenate((pos,s))
    speed   = np.concatenate((speed,v))
    accel   = np.concatenate((accel,a))

    print('v seg = ',v_seg,
    '\na via = ',a_via,
    '\nT via = ',T_via,
    '\ntime = ',time,
    '\npos = ',pos)

    return (v_seg,a_via,T_via,time,pos,speed,accel)


via = np.array([15,40,-10,15])
dur = np.array([20,20,20])
tb = np.array([1,1,1,1])*5
res = lspb(via,dur,tb)

plt.plot(res[2],via,'*',res[3],res[4])
plt.show()