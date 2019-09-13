from math import sin, cos, sqrt, exp, atan2, acos, asin, pi
from random import uniform
import ephem

V     = 0.002278
S     = 0.639837
D     = -0.000982
a0    = -103.264 * pi / 180.0
E     =  253.174 * pi / 180.0
eps   =    1.053 * pi / 180.0
COPx  = 986.444
COPy  = 539.240
alpha = 1.570796
flat  = 0.000000

# Calculate the affine coefficients
# Inputs are (COPx, COPy, alpha, flat)

dilation = sqrt(1-flat)
K = COPx*sin(alpha) + COPy*cos(alpha)
L = COPy*sin(alpha) - COPx*cos(alpha)
c = cos(alpha)*cos(alpha)*dilation + sin(alpha)*sin(alpha)/dilation
d = sin(alpha)*cos(alpha)*dilation - sin(alpha)*cos(alpha)/dilation;
e = -(K*cos(alpha)*dilation*dilation - COPy*dilation + L*sin(alpha))/dilation
f = sin(alpha)*cos(alpha)*dilation - sin(alpha)*cos(alpha)/ dilation
g = sin(alpha)*sin(alpha)*dilation + cos(alpha)*cos(alpha)/dilation
h = -(K*sin(alpha)*dilation*dilation - COPx*dilation - L*cos(alpha))/dilation

# For given coordinates (x, y), calculate azimuth and elevation
def PixelToAzEl ( px, py ):
    # Apply the affine transformation to the input coordinates
    # PURPOSE: Correct elliptical image distortion

    pxt = g*px + f*py + h
    pyt = d*px + c*py + e
    # print( "pxt: %f pyt: %f" % (pxt,pyt))

    # Now apply the Borovicka calibration equations
    # NOTE 1: The equation for "b" employs atan2(x,y), which is equivalent to (in Excel) atan2(y,x) = atan2(X,Y)
    # NOTE 2: The equation set yields azimuth measured from cardinal SOUTH

    x = pxt - COPx
    y = pyt - COPy

    # print( "x: %f y: %f" % (x,y))
    r = sqrt( x*x + y*y )
    u = V*r + S*(exp(D*r) - 1)
    b = a0 - E + atan2(x, y)

    # print( "b: %f" % b)
    angle = b
    z = u
    if eps != 0.0:
        z = acos(cos(u)*cos(eps)-sin(u)*sin(eps)*cos(b))
        sinAngle = sin(b)*sin(u)/sin(z)
        cosAngle = (cos(u)-cos(eps)*cos(z))/(sin(eps)*sin(z))
        angle = atan2(sinAngle,cosAngle)

    elev = pi/2 - z
    azim = angle + E # Measured from cardinal SOUTH.

    return (azim,elev)

# Convert azimuth and elevation to pixel coordinates
# Azimuth in radians measured from cardinal SOUTH
def AzElToPixel ( azim, elev ):
    # First crude guess
    angle = pi/2.0 + azim + a0

    r = (pi/2-elev)/V

    # print("azim: %f a0: %f angle: %s" % (azim,a0,angle))

    px0 =   r*cos(angle) + COPx
    py0 =  -r*sin(angle) + COPy

    print( "px0: %f py0: %f" % (px0,py0) )

    px = px0
    py = py0

    for i in range(15):
        azt,elt = PixelToAzEl( px, py )

        print("Az: %f El: %f" % (azt,elt))
        angle = pi/2.0 + azt + a0

        r = (pi/2-elt)/V

        pxt =  r*cos(angle) + COPx
        pyt = -r*sin(angle) + COPy

        dpx = pxt - px0
        dpy = pyt - py0

        px = px - dpx
        py = py - dpy

        dsq = dpx*dpx + dpy*dpy

        # print( "px: %f py: %f dsq: %f" % (px,py,dsq) )
        if dsq < 1.0e-6:
            break

    return (px,py,sqrt(dsq))

# Convert azimuth and elevation to degrees;
def RadToDeg ( azim,  elev ):
    # Subtract PI tp express azimuth conventionally, i.e., measured from cardinal North
    azim = azim - pi

    # Convert radians to degrees
    elev = elev * 180.0 / pi
    azim = azim * 180.0 / pi

    # Compute azimuth on the interval [0, 360)
    while azim < 0.0:
        azim += 360.0
    while azim >= 360.0:
        azim -= 360.0;

    return (azim,elev)

# Convert ezimuth and elevation to radians
def DegToRad( azim, elev ):
    # Add 180 tp express azimuth measured from cardinal Soutn
    azim = azim + 180.0

    # Convert degrees to radians
    elev = elev * pi / 180.0
    azim = azim * pi / 180.0

    return (azim,elev)


# Call the necessary functions; print results
def FinalPixelToAzEl ( px, py ):
    (azim,elev) = PixelToAzEl(px, py)
    (azim,elev) = RadToDeg(azim, elev)

    print("elev = ", elev )
    print("azim = ", azim )

    return (azim,elev)

def Test():
    # px = 986.444
    # py = 439.240

    # print( "px: %f py: %f" % (px,py) )

    # (azim,elev) = PixelToAzEl(px, py)
    # (azim,elev) = RadToDeg(azim, elev)

    # print( "Az: %f El: %f" % (azim,elev))

    # azim,elev = DegToRad(azim,elev)
    # px,py,dist = AzElToPixel( azim,elev )

    # print( "px: %f py: %f dist: %f" % (px,py,dist) )

    # maxDist = 0.0
    # maxAzim = 0.0
    # maxElev = 0.0

    # for i in range(10000):
    #     azim = uniform(0.0,360.0)
    #     elev = uniform(0.0,90.0)

    #     azim2,elev2 = DegToRad(azim,elev)
    #     px,py,dist = AzElToPixel( azim2,elev2 )

    #     if dist > maxDist:
    #         maxDist = dist
    #         maxAzim = azim
    #         maxElev = elev

    # print("Max dist: %f at azim: %f elev: %f" % (maxDist,maxAzim,maxElev))

    vega = ephem.star("Vega")
    #vega.compute("2019/9/5")

    abq = ephem.Observer()
    abq.lon = "-106.5428"
    abq.lat = "35.1497"
    abq.elevation = 1690.0
    abq.date = "2019/9/9 09:00:00"

    print("Date: %s" % abq.date)

    vega.compute(abq)
    alt = float(vega.alt)*180.0/pi
    az  = float(vega.az)*180.0/pi

    print("Alt: %f Az: %f" % (alt,az))

    azim,elev = DegToRad(az,alt)
    px,py,dist = AzElToPixel( azim,elev )

    print( "px: %f py: %f dist: %f" % (px,py,dist) )

    # azim = 0.0
    # elev = 0.0

    # azim,elev = DegToRad(azim,elev)
    # px,py,dist = AzElToPixel( azim,elev )

    # print( "px: %f py: %f dist: %f" % (px,py,dist) )

    # azim,elev = PixelToAzEl(px, py)
    # azim,elev = RadToDeg(azim, elev)

    # print("Az: %f El: %f" % (azim,elev))




    
