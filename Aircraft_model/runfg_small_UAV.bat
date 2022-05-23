C:
cd C:\Program Files\FlightGear 2020.3

SET FG_ROOT=C:\Program Files\FlightGear 2020.3\data
.\\bin\fgfs --aircraft=c172p --fdm=null --native-fdm=socket,in,30,localhost,5502,udp --fog-fastest --disable-clouds --start-date-lat=2004:06:01:09:00:00 --disable-sound --in-air --enable-freeze --airport=KSFO --runway=10L --altitude=1 --heading=0 --offset-distance=0 --offset-azimuth=0 --enable-terrasync --prop:/sim/rendering/shaders/quality-level=0
