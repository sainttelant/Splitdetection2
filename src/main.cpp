#include "adas_em.h"
#include <thread>
#include <iostream>
#include <string>

using namespace std;

int main()
{	
	AdasEM::adas app;
	app.ecal_init();
	app.createAbdObjRectPub();
	app.subTopic();
	
	std::thread t1(&AdasEM::adas::capture1Thrd, &app);
	//sleep(2);
	std::thread t2(&AdasEM::adas::process1Thrd,&app);
	t1.join();
	t2.join();

	return 0;
}
