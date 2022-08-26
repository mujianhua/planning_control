# Planning
TODO: reference_line_provider

## reference
* [apollo](https://github.com/ApolloAuto/apollo)
* Li B, Ouyang Y, Li L, et al. Autonomous driving on curvy roads without reliance on Frenet frame: A cartesian-based trajectory planning method[J]. IEEE Transactions on Intelligent Transportation Systems, 2022. ( [IEEE Xplore](https://ieeexplore.ieee.org/abstract/document/9703250))


## Planner

### CARTESIAN

* vehicle kinematics formula
$$
\frac{d}{d t}\left[\begin{array}{c}
x(t) \\
y(t) \\
\theta(t) \\
v(t) \\
\phi(t) \\
a(t)
\end{array}\right]=\left[\begin{array}{c}
v(t) \cdot \cos \theta(t) \\
v(t) \cdot \sin \theta(t) \\
v(t) \cdot \tan \phi(t) / L_{W} \\
a(t) \\
\omega(t) \\
j e r k(t)
\end{array}\right]
$$

### LATTICE

## Reference Line


