# testbit module contains various bit test an manipulation routines
# HJK Jan 2016
# test value of a bit in integer and return 1 (true) or 0 (false)


def testbit2int(int_type, offset):
    mask = 1 << offset
    if (int_type & mask):
        return 1
    else:
        return 0


def testBit(int_type, offset):
    mask = 1 << offset
    return(int_type & mask)

# setBit() returns an integer with the bit at 'offset' set to 1.
def setBit(int_type, offset):
    mask = 1 << offset
    return(int_type | mask)

# clearBit() returns an integer with the bit at 'offset' cleared.
def clearBit(int_type, offset):
    mask = ~(1 << offset)
    return(int_type & mask)

# toggleBit() returns an integer with the bit at 'offset' inverted, 0 -> 1 and 1 -> 0.
def toggleBit(int_type, offset):
    mask = 1 << offset
    return(int_type ^ mask)
