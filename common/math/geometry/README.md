# Geometry Package    {#page}

This package provides features for geometric calculation.
Before reading the code in geometry package, please understand these words.

## Cartesian Coordinate System
Generally, it is also called a `rectangular coordinate system` or `orthogonal coordinate system`.
In geometry package, we treat 3-dimension cartesian coordinate system with x,y,z axis.

## Frenet Coordinate System
The frenet coordinate system is a coordinate system that is defined along a curve.
In the geometry package, a curve consisted of 3 3rd order polynomials about each x,y, and z axis in a cartesian coordinate system.
We define the shape of the curve by changing `s` value of the 3rd order polynomials `as^3+bs^2+cs+d` in range `s = [0,1]`.
In this coordinate system, `s` value is normalized in range `s = [0,1]`, s value does not consider the length when the curve is stretched.

If the coefficient `a,b = 0`, the shape of the curve is a line segment.
In this case, the position of the `s = 0` is the start point, and the position of the `s = 1` is the end position.

If the coefficient `a,b,c = 0`, the shape of the curve is a point.
In this case, only `s = 0` case is valid, all of the other cases are invalid.
If you find a case that the shape is a point but s is not equal to 0, please notify to developers.

Some of functions in the geometry package have `denormalize_s` argument with bool type.
If the `denormalize_s = false`, the return value `s` of the function is normalized and in range `s = [0,1]`.
If the `denormalize_s = true`, the return value `s` is denormalized and it returns the value `s` times the length of the curve.

\warning
__A catmull-rom spline curve consists of multiple hermite curves. Each hermite curve has a different length.__
__So, the catmull-rom spline curve can not normalize its length and frenet coordinate of catmull-rom spline curve is always denormalized.__
