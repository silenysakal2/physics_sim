build()
{
	g++ -o main src/main.cpp src/physics.cpp -lSDL3 -O1
}

release()
{
	g++ -o main src/main.cpp src/physics.cpp -lSDL3 -O3
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
