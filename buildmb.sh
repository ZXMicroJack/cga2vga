

if [ "$1" == "clean" ]; then
docker run -ti --rm -v`pwd`:/work --name lpthdpico  picobuild5 bash -c "cd /work/mandelbrot; make clean"
else
docker run -ti --rm -v`pwd`:/work --name lpthdpico  picobuild5 bash -c "cd /work/mandelbrot; cmake .; make"
fi
mp=/media/`whoami`/RPI-RP2

if [ "$1" == "run" ]; then
	echo q > /dev/ttyACM0
	while [ ! -e ${mp} ]; do
		sleep 1
	done
	cp ./mandelbrot/mandelbrot.uf2 ${mp}
fi
