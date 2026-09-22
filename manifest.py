# MicroPython manifest for cameraif: deliberately empty.
#
# cameraif is a native module with no Python half. Its whole surface is the
# C module, and there is nothing to freeze. This file exists so the
# repository can be pointed at by a manifest include without a special case.

# MicroPython 1.29: the manifest names its own C module (workspace retool, piece 1).
c_module(".")  # this directory holds the micropython.cmake / micropython.mk for the C half
