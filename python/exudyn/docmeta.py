#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Toolchain metadata of documented functions and classes, kept out of the docstrings:
#           @docmeta(author='...', date='...', status='...', public=False). The documentation
#           generator reads the decorator statically; at runtime it only attaches the values
#           as __docmeta__ and returns the function or class unchanged. An unknown keyword
#           raises TypeError instead of vanishing.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-15 (created; revision plan steps 36/37)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#public API of this module; kept complete by tools/checkAll.py (revision plan step 107c)
__all__ = [
    'docmeta',
    ]

def docmeta(*, author=None, date=None, status=None, public=True):
    """Attach documentation metadata; public=False keeps a function or class out of the
    generated reference documentation."""
    def Attach(item):
        item.__docmeta__ = {'author': author, 'date': date, 'status': status, 'public': public}
        return item
    return Attach
