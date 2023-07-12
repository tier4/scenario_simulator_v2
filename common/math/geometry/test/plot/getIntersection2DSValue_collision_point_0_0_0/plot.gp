# Setting color
set linetype 1 lc rgb "blue" lw 1
set linetype 2 lc rgb "orange" lw 1
set linetype cycle 2

set terminal pngcairo

set output "getIntersection2DSValue_collision_point_0_0_0.png"
set term png size 1920,1080
set size square

set multiplot layout 1,2
    set title "Line segment \"line\" and point in cartesian coordinate."
    set title font "Latin-Modern,15"

    set xrange [-1.2:1.2]
    set xtic 0.2
    set xlabel "x axis in cartesian coordinate (m)"
    set xlabel font "Latin-Modern,15"
    
    set yrange [-1.2:1.2]
    set ytic 0.2
    set ylabel "y axis in cartesian coordinate (m)"
    set ylabel font "Latin-Modern,15"

    set grid linewidth 1

    set key below reverse Left width -15 height 1
    set key box
    set key font "Latin-Modern,13"

    plot \
        "data.dat" index 0 \
            with lines linewidth 3 title "Line string from (x,y,z) = (0,-1,0) to (x,y,z) = (0,1,0)", \
        "data.dat" index 1 \
            with points ps 2 pointtype 7 title "Point (x,y,z) = (0,0,0)"

    set title "Line segment \"line\" and point in normalized frenet coordinate along \"line\"."
    set xrange [-1.2:1.2]
    set xtic 0.2
    set xlabel "t axis in normalized frenet coordinate"
    set xlabel font "Latin-Modern,15"
    
    set xrange [-1.2:1.2]
    set ytic 0.2
    set ylabel "s axis in normalized frenet coordinate"
    set ylabel font "Latin-Modern,15"

    plot \
        "data.dat" index 2 \
            with lines linewidth 3 title "Line string from (x,y,z) = (0,-1,0) to (x,y,z) = (0,1,0)", \
        "data.dat" index 3 \
            with points ps 2 pointtype 7 title "Point (x,y,z) = (0,0,0) in "

unset multiplot