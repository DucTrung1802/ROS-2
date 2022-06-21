all:
	g++ -fPIC -shared myclass.cc -o myclass.so
	g++ class_user.cc -ldl -o class_user.out
	./class_user.out
