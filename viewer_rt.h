#ifndef VIEWER_RT_H
#define VIEWER_RT_H

#include <string> 
#include <iostream>
#include <stdio.h>
#include <sys/wait.h> 
#include <sys/sem.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/common_headers.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>

 
	class viewer_rt
		{
		public:

			viewer_rt ();	
			~viewer_rt ();
			void delete_all (int window);
			void delete_last (int window);
			void load_xyz (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud, int window, bool diff_in);
			void load_xyz_rbg (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, int window);
			void load_mesh(pcl::PolygonMesh polymesh, int window);
			void turn_rotation ();
			void set_diff ();
			void get_process_vector (std::vector <int> *vector_process_out, std::vector <const char *> *vector_name_out);
			
			
		
		private:
		
			#define WRITE_PCD 1
			#define REQUEST_LOAD_CLOUD 2
			#define VIEWER 4


			#define ACT_GIRO 1
			#define SET_DIFF 2
			#define LOAD_XYZ_CLOUD 10
			#define DELETE_LAST 30
			#define DELETE_ALL 40
			#define LOAD_XYZRBG_CLOUD 50
			#define LOAD_MESH 60

			
			
			
			int i;
			bool diff;
			
			//variable to control processes
			int pid;
			std::vector <int> vector_pid;
			std::vector <const char *> name_process;
			int status;
			
			//Semeforos variables
			int semid, key, num_semaphore;
			struct sembuf operation;
                     
	
			//Variables for pipe
			int pipe_flags[2],pipe_cloud[2];
			int act;
			bool run,turn_on;
	
	
			//Variables for strings
			char cloud_name[50];
	
			//Variables for the viewer
			int v1,v2,v3,v4;
			int obj1, obj2, obj3;
			float turn_view;
			
		};

	#include "viewer_rt.hpp"
#endif
