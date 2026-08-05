.. _sec-utf8:


Character encoding: UTF-8
=========================

Character encoding is a major issue in computer systems, as different languages need a huge amount of different characters,
see the amusing blog of Joel Spolsky:

`The Absolute Minimum Every Software Developer Absolutely, Positively Must Know About Unicode ... <https://www.joelonsoftware.com/2003/10/08/the-absolute-minimum-every-software-developer-absolutely-positively-must-know-about-unicode-and-character-sets-no-excuses/>`_

More about encoding can be found in `Wikipedia:UTF-8 <https://en.wikipedia.org/wiki/UTF-8>`_. UTF-8 encoding tables can be found within the wikipedia article and a comparison with the first 256 characters of unicode is provided at `UTF-8 char table <https://www.utf8-chartable.de/>`_.


For short, Exudyn uses UTF-8 character encoding in texts / strings drawn in OpenGL renderer window.
However, the set of available UTF-8 characters in Exudyn is restricted to a very small set of characters (as compared to available characters in UTF-8).
For an example of available UTF-8 characters, see \ ``examples/solutionViewerTest.py``\ .


Greek characters include all lower case characters (including variations) and only upper case characters, which are different from latin characters: \ :math:`\alpha, \beta, \gamma, ... \sigma, \varphi, \varepsilon; \Gamma, \Delta, \Theta, \Lambda, \Xi, \Pi, \Sigma, \Phi, \Psi, {\mathbf{O}}ega`\ .



