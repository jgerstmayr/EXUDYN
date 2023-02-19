|Documentation GithubIO| |PyPI version exudyn| |PyPI pyversions| |PyPI download month|

.. |PyPI version exudyn| image:: https://badge.fury.io/py/exudyn.svg
   :target: https://pypi.python.org/pypi/exudyn/

.. |PyPI pyversions| image:: https://img.shields.io/pypi/pyversions/exudyn.svg
   :target: https://pypi.python.org/pypi/exudyn/

.. |PyPI download month| image:: https://img.shields.io/pypi/dm/exudyn.svg
   :target: https://pypi.python.org/pypi/exudyn/



.. |Documentation GithubIO| image:: https://img.shields.io/website-up-down-green-red/https/jgerstmayr.github.io/EXUDYN.svg
   :target: https://jgerstmayr.github.io/EXUDYN/

%%SECTIONLEVEL[0][Exudyn]

******
Exudyn
******


**A flexible multibody dynamics systems simulation code with Python and C++**

Exudyn version = 1.5.103.dev1 (Fitzgerald)

+  **University of Innsbruck**, Department of Mechatronics, Innsbruck, Austria

Exudyn 1.5 is out! It includes now Python 3.7/8 - 3.10 wheels for MacOS (since 1.5.11.dev1 also showing tkinter dialogs!), linux and windows. See theDoc.pdf chapter **Issues and Bugs** for changes!

If you like using Exudyn, please add a *star* on github and follow us on 
`Twitter @RExudyn <https://twitter.com/RExudyn>`_ !

+  **NOTE**: for pure installation, use **pip install exudyn** (see further description below)
+  *free, open source* and with plenty of *documentation* and *examples*
+  **pre-built** for Python 3.6 - 3.10 under **Windows** and **Linux**, Python 3.8 - 3.10 under **MacOS** available; build wheels yourself, see `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ )
+  Exudyn can be linked to any other Python package, but we explicitly mention: `NGsolve <https://github.com/NGSolve/ngsolve>`_, `OpenAI <https://github.com/openai>`_, `OpenAI gym <https://github.com/openai/gym>`_, `Robotics Toolbox (Peter Corke) <https://github.com/petercorke/robotics-toolbox-python>`_, `Pybind11 <https://github.com/pybind/pybind11>`_

.. |pic1| image:: docs/demo/screenshots/pistonEngine.gif
   :width: 200

.. |pic2| image:: docs/demo/screenshots/hydraulic2arm.gif
   :width: 200

.. |pic3| image:: docs/demo/screenshots/particles2M.gif
   :width: 120

.. |pic4| image:: docs/demo/screenshots/shaftGear.png
   :width: 160

.. |pic5| image:: docs/demo/screenshots/rotor_runup_plot3.png
   :width: 190

.. |pic6| image:: docs/theDoc/figures/DrawSystemGraphExample.png
   :width: 240
   
|pic1| |pic2| |pic3| |pic4| |pic5| |pic6|

A paper on Exudyn has been presented at the `6th Joint International Conference on Multibody System Dynamics <http://imsdacmd2020.iitd.ac.in>`_ and submitted to the proceedings: J. Gerstmayr, Exudyn - A C++ based Python package for flexible multibody systems, Proceedings of The 6th Joint International Conference on Multibody System Dynamics and the 10th Asian Conference on Multibody System Dynamics 2020, New Delhi, India, 2022. `PDF <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/publications/GerstmayrIMSD2022.pdf>`_

This README document is a small part of the complete documentation found as PDF document in docs/theDoc/theDoc.pdf.
It is auto-generated from .tex files (sorry for some conversion errors!). 
Due to limitations for complex formulas and tables in .rst files, details of the reference manual and many other parts of the documentation are only available in theDoc.pdf, see the `github page of Exudyn <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ !

For license, see LICENSE.txt in the root github folder on github!

Changes can be tracked in the 

In addition to the tutorial in the documentation, many ( **100+** ) examples can be found under main/pythonDev/Examples and main/pythonDev/TestModels .

Tutorial videos can be found in the `youtube channel of Exudyn <https://www.youtube.com/playlist?list=PLZduTa9mdcmOh5KVUqatD9GzVg_jtl6fx>`_ !

Enjoy the Python library for multibody dynamics modeling, simulation, creating large scale systems, parameterized systems, component mode synthesis, optimization, ...




\ **FOR FURTHER INFORMATION see** `Exudyn Github pages <https://jgerstmayr.github.io/EXUDYN>`_ and for details (incl. equations) see `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ !!!

