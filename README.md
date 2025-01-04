# 3D Rigidbody Physics

<p>This repository will cover the basics of kinematics, energy and dynamics.

## Useful formulas

$$ ax^2 + bx + c $$
$$ a \in (-\infty, 0) U (0, \infty) $$
$$ b^2 - 4ac \ge 0 $$

$$ x = \frac{-b ± \sqrt{b^2 - 4ac}}{2a} $$


## Vectors

$$
\vec{A} = \begin{bmatrix}
rsin(ϕ)cos(θ) \\
rsin(ϕ)sin(θ) \\
rcos(ϕ)
\end{bmatrix}
$$

Where $r$ is the magnitude of $\vec{A}$
<p></p>
Magnitude:

$$ ||\vec{A}|| = \sqrt{\vec{A}_x^2 + \vec{A}_y^2 + \vec{A}_z^2 } $$
<p></p>

Angle between vectors $\vec{a}$ and $\vec{b}$:

$$ cos(θ) = \frac{\sum \vec{a}_n\vec{b}_n}{||\vec{a}|| * ||\vec{b}||} $$
$$ θ = cos^{-1}\frac{\sum \vec{a}_n\vec{b}_n}{||\vec{a}|| * ||\vec{b}||} $$
<p></p>

## Derivatives / Differentiation
<p>By first principle:</p>

$$f'(x) = \lim_{h \to 0} \frac{1}{h}(f(x+h) - f(x)) = \frac{df}{dx} = \frac{d}{dx}$$

<p>Product rule:</p>

$$\frac{d}{dx}[f(x)g(x)] = f'(x)g(x) + f(x)g'(x)$$

<p>Quotient rule:</p>

$$\frac{d}{dx}\frac{f(x)}{g(x)} = \frac{f'(x)g(x) - f(x)g'(x)}{g(x)^2}$$

<p>Chain rule:</p>

$$\frac{d}{dx}f(g(x)) = f'(g(x))g'(x)$$


## Common derivatives


$\frac{d}{dx}[c] = 0$
<p></p>

$\frac{d}{dx}[ax^b] = abx^{b-1}$
<p></p>

$\frac{d}{dx}[ax^2 + bx + c] = 2ax + b$
<p></p>

$\frac{d}{dx}[\sqrt{a}] = \frac{a'}{2\sqrt{a}}$
<p></p>

$\frac{d}{dx}[ac^x] = ac^xln(c)$
<p></p>

$\frac{d}{dx}[ln(x)] = \frac{1}{x}$
<p></p>

$\frac{d}{dx}[sin(x)] = cos(x)$
<p></p>

$\frac{d}{dx}[cos(x)] = -sin(x)$
<p></p>

$\frac{d}{dx}[tan(x)] = sec^2(x)$

| Translation | Angular | 
|----------|----------|
| $v = \frac{dx}{dt}$| $ω = \frac{dθ}{dt}$
| $a = \frac{dv}{dt}$ | $α = \frac{dω}{dt}$


## 2D Kinematics

<p></p>

| Translation | Angular | 
|----------|----------|
| $Δx = v_0t$| $Δθ = ω_0t$
| $Δx = v_0t + \frac{1}{2}at^2$ | $Δθ = ω_0t + \frac{1}{2}αt^2$
| $Δx = v_1t - \frac{1}{2}at^2$ | $Δθ = ω_1t - \frac{1}{2}αt^2$
| $v_1^2 = v_0^2 + 2aΔx$ | $ω_1^2 = ω_0^2 + 2αΔθ$

### Projectile example

$$\vec{v}_x = \vec{v}cos(θ)$$
$$\vec{v}_y = \vec{v}sin(θ)$$
$$g = -9.8m/s^2$$
$$Δy = v_yt + \frac{1}{2}gt^2, Δx = v_xt$$

## Forces

$\vec{F}_{net} = \sum \vec{F}_n = m\vec{a}$
<p></p>

$\vec{F}_{gravity} = mgsin(θ)$
<p></p>

$\vec{F}_{s} = -kx$
<p></p>

$\vec{T} = mg - m\vec{a}$
<p></p>


$\vec{f}_{friction} = μ_sN$ (static friction)
<p></p>

$\vec{f}_{friction} = μ_kN$ (kinetic friction)
<p></p>

$g = 9.8m/s^2$
<p></p>

### Inclined plane example at angle $θ$ with external force $X$ at angle $ϕ$ in 2D

$$\vec{F}_{nety} = \sum \vec{F}_y = \vec{N} - mgcos(θ) -Xsin(ϕ) = 0$$
$$\vec{N} = mgcos(θ) + Xsin(ϕ)$$

$$\vec{F}_{netx} = \sum \vec{F}_x = \vec{X}cos(ϕ) - mgsin(θ) -\vec{f} = m\vec{a}$$
$$m\vec{a} = Xcos(ϕ) - mgsin(θ) - μ_kN$$
$$m\vec{a} = Xcos(ϕ) - mgsin(θ) - μ_k(mgcos(θ) + Xsin(ϕ))$$
$$m\vec{a} = Xcos(ϕ) - mgsin(θ) - μ_kmgcos(θ) - μ_kXsin(ϕ)$$
$$\vec{a} = \frac{Xcos(ϕ)}{m} - gsin(θ) - μ_kgcos(θ) - \frac{μ_kXsin(ϕ)}{m}$$
$$\vec{a} = \frac{Xcos(ϕ) - μ_kXsin(ϕ)}{m} - g(sin(θ) - μ_kcos(θ))$$

### Inclined plane example at angle $θ$ with external force $F_{ext}$ at angle $ϕ$ in 3D
Coordinate system:
<p></p>

$x$ is along the plane (downhill)
<p></p>

$y$ is perpendicular to $x$
<p></p>

$z$ is perpendicular to the plane (normal)
<p></p>

$$ \vec{F}_{gx} = mgsin(θ)_xcos(θ)_y $$
<p></p>

$$ \vec{F}_{gy} = mgsin(θ)_y $$
<p></p>

$$ \vec{F}_{gz} = mgcos(θ)_xcos(θ)_y $$
<p></p>

$$
\vec{F}_{\text{net},x} = \sum \vec{F}_x = \vec{F}_{gx} + \vec{F}_{\text{ext}} \cos(\phi_1) - \vec{f}_x = m a_x
$$

$$
\vec{F}_{\text{net},y} = \sum \vec{F}_y = \vec{F}_{gy} + \vec{F}_{\text{ext}} \cos(\phi_2) - \vec{f}_y = m a_y
$$

$$
\vec{F}_{\text{net},z} = \sum \vec{F}_z = \vec{F}_{gz} + \vec{F}_{\text{ext}} \cos(\phi_3) - \vec{N} = 0
$$

$$
\vec{N} = \vec{F}_{gz} + \vec{F}_{\text{ext},z}
$$

$$ \vec{f}_x = μN, \vec{f}_y =μN $$
<p></p>

$$a_x = \frac{\vec{F}_{gx} + \vec{F}_{ext,x} -\vec{f}_x}{m}, a_y = \frac{\vec{F}_{gy} + \vec{F}_{ext,y} -\vec{f}_y}{m}$$

## Torque and inertia

$τ_{net} = \sum τ_n =  Iα$

## Energy

<p></p>

$U_g = mgh$
<p></p>

$U_s = \int F_s dx = \int -kx dx = \frac{1}{2}kx^2 + C = \frac{1}{2}kx^2$
<p></p>

$K = \frac{1}{2}mv^2$