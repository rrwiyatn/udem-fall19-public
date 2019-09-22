def vel2wheel(v, omega, wheel_dist, wheel_rad):
    
    gain = 1
    trim = 0
    
    # Maximal speed
    if v > 0.5:
        v = 0.5
    elif v < -0.5:
        v = -0.5
    
    
##### Fill the code here:
    left_rate = (v + (wheel_dist/2.) * omega) / (wheel_rad * 2. * 3.141592653589793)
    right_rate = (v - (wheel_dist/2.) * omega) / (wheel_rad * 2. * 3.141592653589793)
####


    
    
    return left_rate, right_rate