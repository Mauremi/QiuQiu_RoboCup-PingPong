Assumptions: paddle end velocity is $v_\mathrm p$ (constant), ball is coming at the direction $\theta$ (from the middle line of the table to the trajectory of ball) and at the velocity of $v$, the velocity of the ball after hit is $v'$, the direction is $\theta'$, paddle will hit the ball at $(100, y_0)$, and the target is at $(2740, 720)$.

Purpose: to calculate the hitting angle $\varphi$, which makes the ball hit the target after reflection.

In this special contest, the speed of the ball coming is not so slow to ignore, considering the maximum height of the serve device. And if we simply make the end motor to hit the ball directly TOWARDS the target, the ball might offset track. Thus, we use the reflection model. The velocity loss rate in the normal direction is assumed to be $e$.

Consider the ball trajectory after hit. Its motion direction is towards the target. Then, $$\tan\theta'=\frac{720-y_0}{2740-100}$$The exit angle and the incident angle are $$\theta_\mathrm r=\theta'-\varphi, \quad \theta_\mathrm i=\varphi-\theta$$Do resolution. $$u_\mathrm n'=v'\cos(\theta'-\varphi)-v_\mathrm p, \quad v_\mathrm n=v\cos(\varphi-\theta)+v_\mathrm p,\quad v_\mathrm t'=v'\sin(\theta'-\varphi), \quad v_\mathrm t=v\sin(\varphi-\theta)$$In the normal direction, $$v_\mathrm n'=e\cdot v_\mathrm n,$$and in the tangent direction, $$v_\mathrm t'=\frac57v_\mathrm t$$because the rolling condition is like that in [[2025PingPongCampus/QiuQiu_RoboCup-PingPong/special/Description of Ping Pong Velocity|Description of Ping Pong Velocity]]. Based on the two equations, we take $\varphi$ and $v'$ as unknown, and solve out that$$\frac57v\cdot\frac{\sin(\varphi-\theta)}{\tan(\theta'-\varphi)}=ev\cos(\varphi-\theta)+(1+e)v_\mathrm p$$which is unsolvable!

Consequently, I recommend we use deep learning, inputting $\theta, v, \theta'$, computer deciding the value of $\varphi$ (in our program is `goA`).


Moreover, if we want the real $e$ value, we can use the formula$$e=\dfrac{\frac57v\cdot\frac{\sin(\varphi-\theta)}{\tan(\theta'-\varphi)}-v_p}{v\cos(\varphi-\theta)+v_p}$$