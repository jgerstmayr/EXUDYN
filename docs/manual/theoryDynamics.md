# Dynamics: Mechanical principles

Following kinematics, dynamics comes into play, which involves the study of the forces and torques that cause the motion, bridging the gap between the motion observed and the reasons behind it.
In this section, we shortly recap mechanical principles, which are used throughout for the simulation of multibody systems.

## Newton's basic principles

In the study of multibody systems, Newton's basic principles lay the foundational framework. At the heart of these principles are three laws that govern the motion of bodies:

 **1. Newton's first law (law of inertia)**: A body remains at rest, or in uniform motion in a straight line, unless acted upon by a force $\fv$, expressed as:

$$
  \fv = 0 \implies \frac{\dd \vv}{\dd t} = 0
$$
where $\fv$ is the net force applied to the body, and $\frac{d\vv}{dt}$ is the acceleration of the body.

**2. Newton's second law (law of motion)**: The sum of the forces F acting on a point mass equals the change in momentum of this mass,

$$
  \fv= \frac{\dd\,\Jm}{\dd t} = \dot \Jm
$$
where $\Jm$ is the (linear, translational) momentum of the mass $m$. Assuming a constant mass of the body (which is not true for some cases such as rockets), we find that the acceleration $\av$ of an object is directly proportional to the net force $\fv$ acting upon it and inversely proportional to its mass, given by the equation:

$$
  \fv= m \av
$$ (eq-theory-newton-fma)

where $m$ is the mass of the object.

 **3. Newton's third law (action and reaction)**: For every action, there is an equal and opposite reaction. This principle is fundamental in analyzing the interactions within multibody systems and can be summarized as:

$$
  \fv_{12} = -\fv_{21}
$$
in which we observe $\fv_{12}$ as the force exerted by body 1 on body 2, and $\fv_{21}$ as the force exerted by body 2 on body 1.

These principles are pivotal in understanding and modeling the dynamics of multibody systems, providing the basis for further exploration and analysis in this field.
Newton's principles represent linear (translational) motion, but may be transformed to rotations by using angular momentum, as well. This leads to Euler's equations, see the description of the `ObjectRigidBody`.

## The Lagrange-d'Alembert principle

For a constant mass $m$, it follows from Newton's second law that

$$
  \sum \fv  = m \av \, ,
$$ (eq-newton-dalembert)

{eq}`eq-newton-dalembert` can also be written in the form

$$
  \sum \fv - m \av = \Null
$$
The vector $(- m \av )$ is referred to as the inertia force, since it can apparently be balanced to zero in the sense of a generalized equilibrium. This equilibrium is called dynamic equilibrium, in analogy to statics.

In **D'Alembert's Principle**, the inertia force $\fv_i = -m \av$ is introduced, and the sum of the forces is written as

$$
  \fv_e + \fv_c + \fv_i = \mathbf{0}
$$
where applied forces $\fv_e$ and constraint forces $\fv_c$ are distinguished.

A key property of the constraint forces is that the virtual work done by constraint forces always vanishes,

$$
  \fv_c \cdot \delta \rv = 0 \, .
$$ (eq-virt-arb-zwangskraefte)

where here $\delta \rv$ is the virtual displacement associated with the constraint force.

## Generalized Principle of Virtual Work

With the generalized principle of virtual work, it follows

$$
  \left(\fv_e + \fv_i \right) \cdot \delta \rv = 0 \quad \text{or} \quad
$$
Thus, we obtain the **Lagrange-d'Alembert** principle as

$$
  \delta W_e + \delta W_T = 0 \quad \text{or} \quad
$$
with the virtual work $W_e$ of applied forces and $W_i$ of inertia forces,

$$
  \delta W_e = \fv_e \cdot \delta \rv \quad \mathrm{and} \quad
  \delta W_T = \fv_i \cdot \delta \rv \, .
$$
For a system of $n$ mass points, it follows

$$
  \sum_{i=0}^{n-1} \left(\fv_{e,i} \cdot \delta \rv_i -  m_i \av_i \cdot \delta \rv_i \right) = 0
$$
It is ultimately characterized by the fact that, in comparison to D'Alembert's principle, constraint forces do not appear.

Note that this principle is used in Exudyn at many places to derive equations for rigid and flexible bodies, as well as for connectors, such as spring-dampers.

## Virtual displacements

The virtual displacement $\delta \rv_j$ or the variation of any quantity can be written in terms of variation of the underlying coordinates, see {ref}`sec-theory-generalized-coordinates`,

$$
  \delta \rv_j = \sum_{i=1}^n \frac{\partial \rv_j}{\partial  q_i} \delta q_i \, .
$$ (eq-theory-virtual-displacement)

## Generalized Forces

To derive the Lagrangian equations, we first consider the virtual work done by $N$ forces on $N$ mass points

$$
  \delta W = \sum_{j=1}^N \fv_j \cdot \delta \rv_j \, .
$$
Here, $\fv_j$ represents the force on, and $\delta \rv_j$ represents the displacement of, the $j$th mass point.

Using {eq}`eq-theory-virtual-displacement`, the virtual work can be expressed as

$$
  \delta W = \sum_{j=1}^N \fv_j \cdot \sum_{i=1}^n \frac{\partial \rv_j}{\partial  q_i} \delta q_i =
  \sum_{i=1}^n \left(\sum_{j=1}^N \fv_j \cdot \frac{\partial \rv_j}{\partial  q_i} \right) \delta q_i \, .
$$
In the process of translating the virtual work of forces or moments, we identify what are called generalized forces $Q_i$,

$$
  Q_i = \sum_{j=1}^N \fv_j \cdot \frac{\partial \rv_j}{\partial  q_i} \, .
$$ (eq-theory-generalized-forces)

Consequently, the virtual work can also be written as,

$$
  \delta W = \sum_{i=1}^n Q_i \, \delta q_i \, .
$$
## Lagrange's Equations of Motion

We define the kinetic energy as $T$, which for $N$ mass points is given by

$$
  T = \frac{1}{2} \sum_{j=1}^N  m_j \vv_j^2
$$
Using the kinetic energy $T$, the following **Lagrangian equations** can be written,

$$
  \frac{\mathrm{d}}{\mathrm{d}t} \frac{\partial T}{\partial \dot q_i} - \frac{\partial T}{\partial q_i} = Q_i, \quad i=1,2,\ldots,n \, .
$$
These are valid under the assumption of minimal coordinates with holonomic constraints, resulting in the fact that all virtual displacements $\delta q_i$ are independent of each other.

For forces that possess a potential $V$, i.e. the variation of the potential reads

$$
  \delta V = \sum_{i=1}^n \frac{\partial V}{\partial q_i}\delta q_i = - \sum_{i=1}^n Q_i^{(V)} \delta q_i \, ,
$$
and assuming that $Q_i$ now only includes forces **without a potential**, the Lagrange equations can be rewritten as follows,

$$
  \frac{\mathrm{d}}{\mathrm{d}t} \frac{\partial T}{\partial \dot q_i} - \frac{\partial T}{\partial q_i} = Q_i + Q_i^{(V)}=
  Q_i - \frac{\partial V}{\partial q_i}, \quad i=1,2,\ldots,n \, .
$$
With the **definition** $L=T-V$ and the relationship $\frac{\partial V}{\partial \dot q_i} = 0$, a common form of Lagrange's equations is obtained:

$$
  \frac{\mathrm{d}}{\mathrm{d}t} \frac{\partial L}{\partial \dot q_i} - \frac{\partial L}{\partial q_i} = Q_i, \quad i=1,2,\ldots,n \, .
$$
This equation is also used sometimes to derive equations of motion for rigid or flexible bodies in Exudyn.

## Multibody formulations: redundant and minimal coordinates

In multibody system dynamics, we primarily distinguish between two formulations:

- **redundant coordinates**
- **minimal coordinates**

Formulations based on minimal coordinates appeared naturally and early, in principle every model in dynamics,
such as a 1D motion of a mass point according to Newton's laws, see {eq}`eq-theory-newton-fma`, Euler's equations, or a single DOF mass-spring-damper model.
The **advantages of a minimal-coordinates** formulation are:

- coordinates represent the degrees of freedom
- direct access to relevant coordinates
- direct application of forces to the coordinates
- application of any class of solvers, as long as equations are non-stiff

The **disadvantages** are related to the **additional efforts to derive the equations of motion**, which has to be done for every different system,
and the general restriction to open-loop systems, making it difficult to be applied to closed-loop systems (but not always impossible).
There are many modifications, which allow closed-loop systems, e.g., via augmented Lagrangians or similar approaches.

 A **redundant-coordinates** formulation has the following **advantages**:

- being **extremely versatile** and allowing practically any combination of (closed-loop) constraints
- easy extension to flexible bodies, sliding constraints, etc.
- multibody systems can be created easily by **combining a set of bodies and constraints** ($\ra$ user friendly)

**Disadvantages** are clearly the **resulting index 3** (position-level) constraints,
for which only very few implicit solvers (time integration) exist, and it requires always matrix factorization during solving.
A further problem is that this formulation may potentially lead to redundant constraints, which need to be treated specially with solvers,
and that the degrees of freedom are less obvious and that relevant **(e.g., joint) coordinates are not directly accessible** on the equations level.

 It is left to the user, which formulation to chose when working with multibody systems.
In Exudyn, there is the option to create tree-like rigid body systems
using the `ObjectKinematicTree`, which allows to use minimal coordinates for open-loop systems. In general, Exudyn is a redundant
multibody dynamics formulation for reasons of simpler assembly of equations of motion.

 As an example, we investigate the equations of motion for a **mathematical pendulum** with mass $m$, distance $L$ between support and mass,
under gravity $g$.

 In the minimal coordinates formulation, see {ref}`fig-theory-formulations-pendulum`, we define the angle $\varphi$, being zero in the horizontal configuration,

$$
  m L \ddot \varphi = m g \cos \varphi
$$
leading to one $2^\mathrm{nd}$ order ordinary differential equation (ODE2), using the minimal coordinate $\varphi$.

(fig-theory-formulations-pendulum)=
```{figure} /docs/figures/pendulum.png
:width: 200

Mathematical pendulum with minimal coordinate $\varphi$.
```

 With redundant coordinates, see {ref}`fig-theory-formulations-pendulumconstraint`, we may introduce two Cartesian coordinates ($x$, $y$), to define the location of the mass in
the plane, leading to two differential equations for the mass point,

$$
  m \ddot x = f_x, \quad
  m \ddot y = f_y
$$ (eq-theory-pendulum-redundantcoords)

together with a constraint equation

$$
	c(x,y) = \sqrt{x^2 + y^2} - L = 0
$$ (eq-theory-pendulum-redundantconstraint)

(fig-theory-formulations-pendulumconstraint)=
```{figure} /docs/figures/pendulumConstraint.png
:width: 200

Mathematical pendulum with redundant coordinates ($x$, $y$).
```

 Note that {eq}`eq-theory-pendulum-redundantcoords` and {eq}`eq-theory-pendulum-redundantconstraint` include 3 equations for only two unknowns ($x$, $y$).
Therefore, we need to introduce an additional unknown Lagrange multiplier $\lambda$ for the redundant multibody formulation,
which represents a force in direction of the pendulum's string, exactly such that the length $L$ is preserved.
The factor $\lambda$ has to be projected with the constraint derivatives (Jacobian) onto the coordinates ($x$, $y$), giving the forces

$$
  f_x = -\frac{\partial c}{\partial x} \lambda, \quad
  f_y = m g - \frac{\partial c}{\partial y} \lambda \, .
$$
Therefore, we can rewrite {eq}`eq-theory-pendulum-redundantcoords` as

$$
  m \ddot x + \frac{\partial c}{\partial x} \lambda = 0, \quad
  m \ddot y + \frac{\partial c}{\partial y} \lambda = m g
$$ (eq-theory-pendulum-redundantfinal)

with the algebraic constraint, written in a computationally more efficient form,

$$
  c(x,y) = x^2 + y^2 - L^2 = 0
$$ (eq-theory-pendulum-redundantconstraintfinal)

The equations of motion are formed by both {eq}`eq-theory-pendulum-redundantfinal` and {eq}`eq-theory-pendulum-redundantconstraintfinal`,
which are **differential-algebraic equations (DAEs) of index 3** and therefore much harder to solve than ordinary differential equations
in case of minimal coordinates.

 In Exudyn, we therefore use either the generalized-$\alpha$ solver, or an implicit trapezoidal rule in combination with an index 2
reduction, see the section on solvers for more details.
