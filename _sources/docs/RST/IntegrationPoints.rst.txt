.. _sec-integrationpoints:


Integration Points
==================

For several tasks, especially for finite elements and contact, different integration rules are used, which are summarized here.
The interval of all integration rules is \ :math:`\in [-1,1]`\ , thus giving a total sum for integration weights of 2.
The points \ :math:`\xi_{ip}`\  and weights \ :math:`w_{ip}`\  for Gauss rules read:

The following table collects some typical \ **input parameters**\  for nodes, objects and markers: 

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | \ **type/order**\ 
     - |  \ **point 0**\ 
     - |  \ **point 1**\ 
     - |  \ **point 2**\ 
     - |  \ **point 3**\  
   * - | Gauss 1
     - |  0
     - | 
     - | 
     - | 
   * - | Gauss 3
     - |  \ :math:`-\sqrt{1 / 3}`\ 
     - |  \ :math:`\sqrt{1 / 3}`\ 
     - | 
     - | 
   * - | Gauss 5
     - |  \ :math:`-\sqrt{3 / 5}`\ 
     - |  0
     - |  \ :math:`\sqrt{3 / 5}`\ 
     - | 
   * - | Gauss 7
     - |  \ :math:`-\sqrt{3 / 7 + \sqrt{120} / 35}`\ 
     - |  \ :math:`-\sqrt{3 / 7 - \sqrt{120} / 35}`\ 
     - |  \ :math:`\sqrt{3 / 7 - \sqrt{120} / 35}`\ 
     - |  \ :math:`\sqrt{3 / 7 + \sqrt{120} / 35}`\  
   * - | 
     - | 
     - | 
     - | 
     - | 
   * - | \ **type/order**\ 
     - |  \ **weight 0**\ 
     - |  \ **weight 1**\ 
     - |  \ **weight 2**\ 
     - |  \ **weight 3**\  
   * - | Gauss 1
     - |  2
     - | 
     - | 
     - | 
   * - | Gauss 3
     - |  1
     - |  1
     - | 
     - | 
   * - | Gauss 5
     - |  \ :math:`5 / 9`\ 
     - |  \ :math:`8 / 9`\ 
     - |  \ :math:`5 / 9`\ 
     - |  
   * - | Gauss 7
     - |  \ :math:`1 / 2 - 5 / (3 \sqrt{120})`\ 
     - |  \ :math:`1 / 2 + 5 / (3*\sqrt{120})`\ 
     - |  \ :math:`1 / 2 + 5 / (3*\sqrt{120})`\ 
     - |  \ :math:`1 / 2 - 5 / (3*\sqrt{120})`\  


The points \ :math:`\xi_{ip}`\  and weights \ :math:`w_{ip}`\  for Lobatto rules read: 

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | \ **type/order**\ 
     - |  \ **point 0**\ 
     - |  \ **point 1**\ 
     - |  \ **point 2**\ 
     - |  \ **point 3**\ 
   * - | Lobatto 1
     - |  -1
     - |  1
     - | 
     - | 
   * - | Lobatto 3
     - |  -1
     - |  0
     - |  1
     - | 
   * - | Lobatto 5
     - |  -1
     - |  \ :math:`-\sqrt{1/5}`\ 
     - |  \ :math:`\sqrt{1/5}`\ 
     - |  1
   * - | 
     - | 
     - | 
     - | 
     - | 
   * - | \ **type/order**\ 
     - |  \ **weight 0**\ 
     - |  \ **weight 1**\ 
     - |  \ **weight 2**\ 
     - |  \ **weight 3**\  
   * - | Lobatto 1
     - |  1
     - |  1
     - | 
     - | 
   * - | Lobatto 3
     - |  \ :math:`1/3`\ 
     - |  \ :math:`4/3`\ 
     - |  \ :math:`1/3`\ 
     - | 
   * - | Lobatto 5
     - |  \ :math:`1/6`\ 
     - |  \ :math:`5/6`\ 
     - |  \ :math:`5/6`\ 
     - |  \ :math:`1/6`\ 

Further integration rules can be found in the C++ code of Exudyn, see file \ ``BasicLinalg.h``\ .




\ **For further information on this topic read**\ : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_
