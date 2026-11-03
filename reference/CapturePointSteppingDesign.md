# real-time balance: the design (2026-9-24)

design a controller whose target is recomputed every cycle, and which puts the upper body back upright.

## the numbers that decide everything

measured from the URDF and the recorded standing pose (`stand_target.yaml`):

| quantity | value | where it comes from |
|---|---|---|
| total mass $m$ | **69.0 kg** | sum of the 65 link masses |
| CoM above the ankle axis $h$ | **0.761 m** | whole-body CoM at the recorded pose, minus the ankle |
| $\omega=\sqrt{g/h}$ | **3.58 rad/s** | falling time constant $1/\omega$ = **0.278 s** |
| toppling stiffness $mgh$ | **514 N·m/rad** | what the ankles have to beat to stand by themselves |
| $mg$ | 677 N | CoP 1 cm forward = **6.8 N·m** of ankle pitch |
| ankle to toe | 0.15 m | `ankle_toe_*_fixed_joint` origin |
| ankle effort limit | 55 N·m | URDF `ankle_*_joint` limit = **8.1 cm** of CoP per foot |
| ankle joint range (pitch) | −70 … +30° | the pose sits at −28…−33°, so +60° / −40° of command headroom |
| ankle stiffness now | 2 × kp 15 = **30 N·m/rad** | `leg_kp_55/56/65/66` in the ros2_control xacro |
| stance width | 0.278 m | ±0.139 m, the lateral CoP range if load can be shifted |



## 1. linearized inverted-pendulum model

The robot can be approximated as a linearized inverted pendulum around the ankle axis.
Let

* $m$ be the total robot mass,
* $h$ be the vertical distance from the ankle axis to the whole-body CoM,
* $\theta$ be the small body pitch angle,
* $g$ be gravitational acceleration.

for a small pitch angle $\theta$, the horizontal displacement of the CoM from the ankle is approximately
$$
x_{\mathrm{CoM}} \approx h\theta
$$

gravity produces a torque about the ankle:

$$
\tau_g = m g x_{\mathrm{CoM}}
$$

therefore,
$$
\tau_g \approx mgh\theta.
$$
for the passive inverted pendulum, the equation of motion is
$$
I\ddot{\theta} = mgh\theta
$$
where $I$ is the rotational inertia of the robot about the ankle axis.

for a point-mass approximation,
$$
I=I_{\mathrm{CoM}}+mh^2 \approx mh^2
$$
substituting this into the equation of motion gives
$$
mh^2\ddot{\theta}=mgh\theta, \\
h\ddot{\theta}=g\theta, \\
\ddot{\theta}=\frac{g}{h}\theta
$$
since $\ddot{\theta}$ and $\theta$ have a linear relationship, we can define the constant quantity as $\omega^2$

$$
\omega^2=\frac{g}{h} \qquad \omega=\sqrt{\frac{g}{h}} \\ 
\boxed{\ddot{\theta}=\omega^2\theta}
$$
observe this equation. because the equation is a linear homogeneous differential equation with constant coefficients, we look for exponential solutions of the form $e^{\lambda t}$

$$
\theta(t)=e^{\lambda t}\\
\dot{\theta(t)}=\lambda e^{\lambda t}\\
\ddot{\theta(t)}=\lambda^2 e^{\lambda t}\\
$$
substituting $\ddot{\theta}=\omega^2\theta$ into the above yields:

$$
\lambda^2 e^{\lambda t}=\omega^2 e^{\lambda t}\\
\lambda^2=\omega^2\\
\lambda=\pm \omega
$$

substituting $\lambda=\pm \omega$ into $\theta(t)=e^{\lambda t}$, so the two basic solutions:

$$
\theta_1(t)=e^{+\omega t}\qquad
\theta_2(t)=e^{-\omega t}\\
\theta(t)=A e^{\omega t}+B e^{-\omega t}\\
\dot\theta(t)=A \omega e^{\omega t}+B \omega e^{-\omega t}
$$



$$
\text{at } t=0:\qquad e^0=1\\
\theta(0)=\theta_0=A+B\\
\dot\theta(0)=\dot\theta_0=A \omega -B \omega\\
\begin{cases}
A+B=\theta_0\\
A-B=\frac{\dot\theta}{\omega}
\end{cases}
\\ 
\begin{cases}
A=\frac{1}{2}(\theta_0+\frac{\dot\theta_0}{\omega})\\
B=\frac{1}{2}(\theta_0-\frac{\dot\theta_0}{\omega})
\end{cases}\\
\theta(t)=\frac{1}{2}(\theta_0+\frac{\dot\theta_0}{\omega}) e^{\omega t}
         +\frac{1}{2}(\theta_0-\frac{\dot\theta_0}{\omega}) e^{-\omega t} \\
\theta(t)=A e^{\omega t}+B e^{-\omega t}
$$


This is the unstable, linearized inverted-pendulum equation. Its solutions contain an exponentially growing term

$$
\theta(t)
= \underbrace{\frac{1}{2}\left(\theta_0+\frac{\dot\theta_0}{\omega}\right) e^{\omega t}}_{\text{unstable mode}}
+ \underbrace{\frac{1}{2}\left(\theta_0-\frac{\dot\theta_0}{\omega}\right) e^{-\omega t}}_{\text{stable mode}}
$$
>The first term grows without bound as $t \to +\infty$ because $e^{\omega t} \to +\infty$; it is therefore the unstable mode.  

 >The second term decays to zero as $t \to +\infty$ because $e^{-\omega t} \to 0$; it is therefore the stable mode.



so a small forward lean naturally grows rather than decays unless the controller actively generates a stabilizing torque. extract unstable mode:

$$
A= \frac{1}{2}(\theta_0+\frac{\dot\theta_0}{\omega})
$$

so the degree of danger: $\theta_0+\frac{\dot\theta_0}{\omega}$.


## 2. divergent part of the current state

> the motion of an inverted pendulum is the sum of a divergent mode and a decaying mode. tt is the divergent mode that causes the robot to fall, so we need to find a way to extract it. we use the derivative property of $e^t$ to eliminate variables and derive the equation of the divergent mode.

known inverted pendulum equation $\ddot{\theta}=\omega^2\theta$

$$
\ddot{\theta}-\omega^2\theta=0
$$
solve this second-order linear differential equation with constant coefficients, and we done

$$
\begin{cases}
\theta(t)=A e^{\omega t}+B e^{-\omega t} \\
\frac{\dot \theta(t)}{\omega}=A e^{\omega t}-B e^{-\omega t}
\end{cases}
$$

add them up

$$
\theta(t)+\frac{\dot \theta(t)}{\omega}=2A e^{\omega t} \\
$$



## 3. horizontal position dynamics

$$
x_{\mathrm{CoM}} \approx h\theta \\
\dot x_{\mathrm{CoM}} \approx h\dot\theta \\
\ddot x_{\mathrm{CoM}} \approx h\ddot\theta \\
$$

original dynamics $\ddot{\theta}=\omega^2\theta$ multiply both sides by h:

$$
h \ddot{\theta}=\omega^2 h \theta \\
\ddot{x}=\omega^2 x \\
x(t)=A  e^{\omega t}+B e^{-\omega t} \\
\dot x(t)=A \omega e^{\omega t}+B \omega e^{-\omega t} \\
$$
so the degree of position danger is $x+\frac{\dot x}{\omega}$

$$
\begin{aligned}
x+\frac{\dot x}{\omega} &= A e^{\omega t}+B e^{-\omega t}+
                          \frac{A \omega e^{\omega t}-B \omega e^{-\omega t}}{\omega} \\
&= A \omega e^{\omega t}+B \omega e^{-\omega t}+A \omega e^{\omega t}-B \omega e^{-\omega t} \\
&= 2A e^{\omega t}
\end{aligned}
$$

this quantity purely represents the divergent part of the current state. for convenience, we define$x_{cp}$

$$
x_{cp}=x+\frac{\dot x}{\omega}=2A e^{\omega t}
$$

## 4. capture point

suppose the robot's CoM is currently at $x$. if we instantaneously move the support point to some position $p$ such that the robot comes exactly to a stop, then this $p$ is the new capture point  

the position of the CoM relative to the new capture point is $x-p$, not the original $x$. therefore, the new inverted pendulum equation becomes: 
$$
\ddot x = \omega^2(x-p)
$$

The condition for the robot to come exactly to a stop is $x \to p$ and $\dot x \to 0$

$$
\begin{cases}
2A e^{\omega t}=(x-p)+\frac{\dot x}{\omega} \\
2A e^{\omega t}=0 \\
\end{cases} \\
p=x+\frac{\dot x}{\omega} \\
\boxed{p=x_{cp}} \\
p_{cp}=p_{CoM}+\frac{\dot p_{CoM}}{\omega}
$$

therefore, in the ideal case, the foot should land at the capture point

## 5. plugging in real robot data

$$
x_{cp}=x_{CoM}+\frac{\dot x_{CoM}}{\omega} \\
\begin{cases}
x_{cp}=x_{CoM}+\frac{\dot x_{CoM}}{\omega} \\
y_{cp}=y_{CoM}+\frac{\dot y_{CoM}}{\omega} \\
\end{cases}\\
\omega=\sqrt{\frac{g}{h}}=\sqrt{\frac{9.81}{0.671}}
\approx 3.82\ \mathrm{rad/s}\\
\begin{cases}
x_{cp}=x_{CoM}+\frac{\dot x_{CoM}}{3.82} \\
y_{cp}=y_{CoM}+\frac{\dot y_{CoM}}{3.82} \\
\end{cases}\\
$$


## 6. capture point with two possible support legs

at the current control cycle, we have:
$$
p_{CoM}=
\begin{bmatrix}
  x_{CoM}  \\
  y_{CoM} 
\end{bmatrix}
\qquad
\dot p_{CoM}=
\begin{bmatrix}
  \dot x_{CoM}  \\
  \dot y_{CoM} 
\end{bmatrix}
\qquad
p_L=
\begin{bmatrix}
  x_{L}  \\
  y_{L}
\end{bmatrix}
\qquad
p_R=
\begin{bmatrix}
  x_{R}  \\
  y_{R}
\end{bmatrix}
$$




the two possible support-leg cases are therefore:

1. left leg is the support leg, right foot is the swing foot
2. right leg is the support leg, left foot is the swing foot


### 6.1. capture point relative in the horizontal plane

$$
p_{cp}=p_{CoM}+\frac{\dot p_{CoM}}{\omega}
$$

for two-dimensional motion, apply the same equation independently to the two horizontal directions:

$$
x_{cp}=x_{CoM}+\frac{\dot x_{CoM}}{\omega} \\
y_{cp}=y_{CoM}+\frac{\dot y_{CoM}}{\omega} \\
$$


### 6.2. left leg as the support leg

$$
p_{support}=p_L \\
p_{cp}^{(L)}=p_{CoM}+\frac{\dot p_{CoM}}{\omega} \\
x_{cp}^{(L)}=x_{CoM}+\frac{\dot x_{CoM}}{\omega} \\
y_{cp}^{(L)}=y_{CoM}+\frac{\dot y_{CoM}}{\omega} \\
$$

therefore

$$
p_{R,target}=p_{cp}^{(L)}
$$

Similarly, when the right leg is the support leg and the left foot is the swing foot

$$
p_{cp}^{(R)}
=
p_{CoM}
+
\frac{\dot p_{CoM}}{\omega}.
$$


## 7. 300 ms prediction


$$
T_{delay}\approx0.30\ \mathrm{s}.
$$

therefore, using the current Capture Point directly would cause the foot to target an outdated state.

assuming the current inverted-pendulum dynamics remain approximately valid during this short prediction horizon, we predict the CoM state forward by $T_{delay}$.

for the horizontal position:

$$
x(t+T)
=
A e^{\omega(t+T)}
+
B e^{-\omega(t+T)}
$$

and

$$
\dot{x}(t+T)
=
A\omega e^{\omega(t+T)}
-
B\omega e^{-\omega(t+T)}.
$$

The predicted Capture Point is then

$$
x_{cp,pred}
=
x(t+T)
+
\frac{\dot{x}(t+T)}{\omega}
$$

and similarly for the lateral direction:

$$
y_{cp,pred}
=
y(t+T)
+
\frac{\dot{y}(t+T)}{\omega}
$$

Therefore, the foot target used by the controller is based on the predicted Capture Point rather than the instantaneous Capture Point:

$$
\boxed{
p_{target}=p_{cp}(t+T_{delay})
}
\qquad
T_{delay}\approx0.30\ \mathrm{s}.
$$



## 8. implementation

