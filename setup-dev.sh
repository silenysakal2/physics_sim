build()
{
	g++ -o main src/main.cpp src/physics.cpp -lSDL3 -O1 -ffast-math
}

release()
{
	g++ -o main src/main.cpp src/physics.cpp -lSDL3 -O3 -ffast-math
}

run()
{
	./main
}

bnr()
{
	if build; then
		run
	fi
}
