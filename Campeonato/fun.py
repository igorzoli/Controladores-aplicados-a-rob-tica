import math 

def norm (norm):
    if norm > math.pi:
        while norm > math.pi:
            norm = norm - (math.pi * 2)
    elif norm < -math.pi:
        while norm < -math.pi:
            norm = norm + (math.pi * 2)
    return norm