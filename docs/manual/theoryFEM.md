# Finite elements

(sec-integrationpoints)=
## Integration points

For several tasks, especially for finite elements and contact, different integration rules are used, which are summarized here.
The interval of all integration rules is $\in [-1,1]$, thus giving a total sum for integration weights of 2.
The points $\xi_{ip}$ and weights $w_{ip}$ for Gauss rules read:

The following table collects some typical **input parameters** for nodes, objects and markers:

| **type/order** | **point 0** | **point 1** | **point 2** | **point 3** |
|---|---|---|---|---|
| Gauss 1 | 0 |  |  |  |
| Gauss 3 | $-\sqrt{1 / 3}$ | $\sqrt{1 / 3}$ |  |  |
| Gauss 5 | $-\sqrt{3 / 5}$ | 0 | $\sqrt{3 / 5}$ |  |
| Gauss 7 | $-\sqrt{3 / 7 + \sqrt{120} / 35}$ | $-\sqrt{3 / 7 - \sqrt{120} / 35}$ | $\sqrt{3 / 7 - \sqrt{120} / 35}$ | $\sqrt{3 / 7 + \sqrt{120} / 35}$ |
|  |  |  |  |  |
| **type/order** | **weight 0** | **weight 1** | **weight 2** | **weight 3** |
| Gauss 1 | 2 |  |  |  |
| Gauss 3 | 1 | 1 |  |  |
| Gauss 5 | $5 / 9$ | $8 / 9$ | $5 / 9$ |  |
| Gauss 7 | $1 / 2 - 5 / (3 \sqrt{120})$ | $1 / 2 + 5 / (3*\sqrt{120})$ | $1 / 2 + 5 / (3*\sqrt{120})$ | $1 / 2 - 5 / (3*\sqrt{120})$ |

The points $\xi_{ip}$ and weights $w_{ip}$ for Lobatto rules read:

| **type/order** | **point 0** | **point 1** | **point 2** | **point 3** |
|---|---|---|---|---|
| Lobatto 1 | -1 | 1 |  |  |
| Lobatto 3 | -1 | 0 | 1 |  |
| Lobatto 5 | -1 | $-\sqrt{1/5}$ | $\sqrt{1/5}$ | 1 |
|  |  |  |  |  |
| **type/order** | **weight 0** | **weight 1** | **weight 2** | **weight 3** |
| Lobatto 1 | 1 | 1 |  |  |
| Lobatto 3 | $1/3$ | $4/3$ | $1/3$ |  |
| Lobatto 5 | $1/6$ | $5/6$ | $5/6$ | $1/6$ |

Further integration rules can be found in the C++ code of Exudyn, see file `BasicLinalg.h`.
