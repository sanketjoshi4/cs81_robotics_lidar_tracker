#!/usr/bin/env python


def show_pos(np_pos):
    """ A printable version of 1d numpy array for pose """
    return show(float(np_pos[0][0])), show(float(np_pos[1][0]))


def show(x, n=2):
    """
    This formats a number for better readability in the log, so that they are vertically aligned
        show (1.0, n=3)         = " 001.000"       # Padded on both sides
        show (23.1238, n=3)     = " 023.124"       # Rounded an padded
        show (-1.2, n=3)        = "-001.200"       # Aligns negative numbers with positive
        show (1111.0, n=3)      = " 111.000"       # Capped to 3 on both sides of decimal
    """
    if x is None:
        return ""

    neg = x < 0
    x = round(abs(x), n)
    xstr = str(x)
    frac = '.' in xstr
    splits = xstr.split(".")
    pad0 = "".join(["0" for _ in range(n)])
    intpart = (pad0 + splits[0])[-n:]
    fracpart = pad0 if not frac else (splits[1] + pad0)[:n]
    return ("-" if neg else " ") + intpart + "." + fracpart
