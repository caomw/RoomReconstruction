#include "launcher.h"
	 

/**Funcion principal*/
int main (int argc, char** argv){


	// Objets
	pcl::PCDReader reader;
	
		

	// Load file
	system("clear");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ); 
	if (argc<1){
		std::cout << "No se ha introdujo nombre del archivo *.pcd para cargar" << std::endl;
		std::cout << "$./info_nube <NOMBRE DEL ARCHIVO PCD>" <<std::endl;
		return (-1);
	}
	reader.read (argv[1], *cloud);
	
	launcher launcher_main(cloud);
			
	return (1);
}
