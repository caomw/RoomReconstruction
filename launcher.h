#ifndef LAUNCHER_H
#define LAUNCHER_H

#include "viewer_rt.h"
#include "room_inside_detect.h"
	

	class launcher
		{
		
		public:

			launcher (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);	
			~launcher ();	

		private:
	
			//Objects
			room_inside_detect detector;
			viewer_rt viewer;
			
			
			//Functions
			void display_data ();
			void change_parameters();
		
};	
	#include "launcher.hpp"
#endif

