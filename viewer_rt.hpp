#ifndef VIEWER_RT_HPP
#define VIEWER_RT_HPP

// CONSTRUCTOR
	
	viewer_rt::viewer_rt (){
		//Inicialization
			i = 0;
			
			key = 10;
			num_semaphore = 6;
			
			act = 0;
			run = true;
			turn_on = false;
			turn_view = 0;
			
			v1 = 0; 
			v2 = 0; 
			v3 = 0; 
			v4 = 0;
			obj1 = 0; 
			obj2 = 0; 
			obj3 = 0;
			
		
		
			//Make semaphore
			if((semid = semget(key,num_semaphore,IPC_CREAT|0600)) == -1)
				std::cout << "[WIEWER_RT] ERROR CREATING SEMAPHORE" << std::endl;
			
			operation.sem_flg = 0;
			
			
			
			
			//Initialze semaphore
			semctl(semid,WRITE_PCD,SETVAL,1);
			semctl(semid,REQUEST_LOAD_CLOUD,SETVAL,0);
			semctl(semid,5,SETVAL,0);
	
	
	
	
			//Make and set pipes
			if (pipe(pipe_flags) == -1)
				std::cout << "[WIEWER_RT] ERROR CREATING THE PRINCIPAL PIPE" << std::endl;

			if (pipe(pipe_cloud) == -1)
				std::cout << "[WIEWER_RT] ERROR CREATING THE SECOND PIPE" << std::endl;

		
			
			
		//Fork the execcution
			
			pid = fork();
			vector_pid.push_back(pid);
			name_process.push_back("VIEWER_RT");
			
			
			if (pid == -1)      
				std::cout << "[VIEWER_RT] ERROR MAKING FORK" << std::endl;
			
			
			
			
			
			//Child execution (paralel to the principal proccess)
			else if (pid == 0) 
			{
			
		
				pcl::visualization::PCLVisualizer viewer ("vista 3D");
				viewer.setSize(1360,768);
				
				
				
				
				//Configure viewer
				viewer.initCameraParameters ();
				
				/*
				//( xmin , ymin, xmax , ymax, viewport )//
				//two windows
				//viewer.createViewPort(0.0, 0.5, 1, 1, v1);
				//viewer.createViewPort(0.0, 0.0, 1, 0.5, v2);
		
		
				//three windows (A)
				//viewer.createViewPort(0.0, 0.66, 1, 1, v1);
				//viewer.createViewPort(0.0, 0.33, 1, 0.66, v2);
				//viewer.createViewPort(0.0, 0.0, 1, 0.33, v3);
		
				//three windows (B)
				//viewer.createViewPort(0, 0.5, 1, 1, v1);				
				//viewer.createViewPort(0, 0, 0.5, 0.5, v2);
				//viewer.createViewPort(0.5, 0, 1, 0.5, v3);
				//viewer.setBackgroundColor (255, 255, 255);
				//viewer.createViewPort(0, 0, 1, 1, v1);
		
		
				//four windows
				//viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v1);
				//viewer.createViewPort(0.5, 0.5, 1, 1, v2);
				//viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v3);
				//viewer.createViewPort(0.5, 0.0, 1.0, 0.5, v4);
				*/
				
				
				viewer.setCameraPosition( 10 , 10, 10 , 10 , 10 , 10  );
				viewer.addCoordinateSystem (1); 
		
		

	
				//Principal loop (child execution), no ends until end of program
				while (run){ 
			
					//Refresh the viewer display
					viewer.spinOnce (100);
					boost::this_thread::sleep (boost::posix_time::microseconds (10000));
					turn_view=turn_view + 0.25;
					if (turn_on) {viewer.setCameraPosition( 20* cos(turn_view*(3.1416/180)) , 20* sin(turn_view*(3.1416/180)) ,10 , 20* cos(turn_view*(3.1416/180)) , 20 * sin(turn_view*(3.1416/180)) , 20 );}

			
			
			
					//Check if it has received signal 
					if (semctl(semid,5,GETVAL,0) > 0){
					
				
						//Reset the switch possibilities
						operation.sem_num = 5;
						operation.sem_op = -1;
						semop(semid,&operation,1);
				
						
						
						
						//Read instruction from pipe
						read(pipe_flags[0],&act,1);	
						
				
				
				
						//SWITCH POSSIBILITIES
			
						//Load simple cloud 
						if (act >= LOAD_XYZ_CLOUD && act < LOAD_XYZ_CLOUD + 10 && diff == false){ 
														
							pcl::PCDReader reader;	
							boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud (new pcl::PointCloud<pcl::PointXYZ> );
							reader.read ("temp.pcd", *cloud);				

					
							if (act-LOAD_XYZ_CLOUD == 1){
								obj1++;
								sprintf(cloud_name,"object_v1 %d",obj1);
								viewer.addPointCloud<pcl::PointXYZ> (cloud, cloud_name,v1);		
					
								}
				
							else if (act-LOAD_XYZ_CLOUD == 2){
								obj2++;
								sprintf(cloud_name,"object_v2 %d",obj2);
								viewer.addPointCloud<pcl::PointXYZ> (cloud, cloud_name,v2);
					
								}
				
							else if (act-LOAD_XYZ_CLOUD == 3){
								obj3++;
								sprintf(cloud_name,"object_v3 %d",obj3);
								viewer.addPointCloud<pcl::PointXYZ> (cloud, cloud_name,v3);
							}
						
						
					
				
				
						//Reset semaphore and load variables
						operation.sem_num = REQUEST_LOAD_CLOUD;
						operation.sem_op = 1;
						semop(semid,&operation,1);
						act = 0;
						}
						
						
						
						
						//Load simple cloud (diff)
						else if (act >= LOAD_XYZ_CLOUD && act < LOAD_XYZ_CLOUD + 10 && diff ==  true){ 
							
							pcl::PCDReader reader;								
							boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud (new pcl::PointCloud<pcl::PointXYZ> );
							reader.read ("temp.pcd", *cloud);				
							
							//temp
							double r,g,b;
							r = 0;
							b = 0;
							g = 0;
							
							diff = false;
					
							if (act-LOAD_XYZ_CLOUD == 1){
								obj1++;
								sprintf(cloud_name,"object_v1 %d",obj1);
								
								if (obj1 == 1) r = 255;
								if (obj1 == 2) b = 255;
								if (obj1 == 3) g = 255;
								if (obj1 > 3) {
									r = 255 - 100 * obj1;
									b = 255 - 100 * obj1;
									g = 255 - 100 * obj1;
								}
								pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, r, g, b);
								viewer.addPointCloud<pcl::PointXYZ> (cloud,single_color, cloud_name,v1);		
					
								}
				
							else if (act-LOAD_XYZ_CLOUD == 2){
								obj2++;
								sprintf(cloud_name,"object_v2 %d",obj2);
								if (obj2 == 1) r = 255;
								if (obj2 == 2) b = 255;
								if (obj2 == 3) g = 255;
								if (obj2 > 3) {
									r = 255 - 100 * obj2;
									b = 255 - 100 * obj2;
									g = 255 - 100 * obj2;
								}
								
								pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, r, g, b);
								viewer.addPointCloud<pcl::PointXYZ> (cloud,single_color, cloud_name,v2);
					
								}
				
							else if (act-LOAD_XYZ_CLOUD == 3){
								obj3++;
								sprintf(cloud_name,"object_v3 %d",obj3);
								if (obj3 == 1) r = 255;
								if (obj3 == 2) b = 255;
								if (obj3 == 3) g = 255;
								if (obj3 > 3) {
									r = 255 - 100 * obj3;
									b = 255 - 100 * obj3;
									g = 255 - 100 * obj3;
								}
								
								pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, r, g, b);
								viewer.addPointCloud<pcl::PointXYZ> (cloud,single_color, cloud_name,v3);
							}
						
						
					
				
				
						//reset semaphore and load variables
						operation.sem_num = REQUEST_LOAD_CLOUD;
						operation.sem_op = 1;
						semop(semid,&operation,1);
						act = 0;
						}
						
						
						
						//Load RBG cloud
						if (act >= LOAD_XYZRBG_CLOUD && act < LOAD_XYZRBG_CLOUD + 10){ 
														
							pcl::PCDReader reader;	
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRBG (new pcl::PointCloud<pcl::PointXYZRGB>);
							reader.read ("temp.pcd", *cloudRBG);				

					
							if (act-LOAD_XYZRBG_CLOUD == 1){
								obj1++;
								sprintf(cloud_name,"object_v1 %d",obj1);
								viewer.addPointCloud<pcl::PointXYZRGB> (cloudRBG, cloud_name,v1);		
					
								}
				
							else if (act-LOAD_XYZRBG_CLOUD == 2){
								obj2++;
								sprintf(cloud_name,"object_v2 %d",obj2);
								viewer.addPointCloud<pcl::PointXYZRGB> (cloudRBG, cloud_name,v2);
					
								}
				
							else if (act-LOAD_XYZRBG_CLOUD == 3){
								obj3++;
								sprintf(cloud_name,"object_v3 %d",obj3);
								viewer.addPointCloud<pcl::PointXYZRGB> (cloudRBG, cloud_name,v3);
							}
						
						
					
				
				
						//Reset semaphore and load variables
						operation.sem_num = REQUEST_LOAD_CLOUD;
						operation.sem_op = 1;
						semop(semid,&operation,1);
						act = 0;
						}
						
						
						
						//Load mesh
						if (act >= LOAD_MESH && act < LOAD_MESH + 10){ 
														
								
							pcl::PolygonMesh polymesh;
							pcl::io::loadPolygonFile ("temp.vtk", polymesh);
	


					
					
							if (act-LOAD_MESH == 1){
								obj1++;
								sprintf(cloud_name,"object_v1 %d",obj1);
								viewer.addPolygonMesh (polymesh, cloud_name,v1);		
					
								}
				
							else if (act-LOAD_MESH == 2){
								obj2++;
								sprintf(cloud_name,"object_v2 %d",obj2);
								viewer.addPolygonMesh (polymesh, cloud_name,v2);
					
								}
				
							else if (act-LOAD_MESH == 3){
								obj3++;
								sprintf(cloud_name,"object_v3 %d",obj3);
								viewer.addPolygonMesh (polymesh, cloud_name,v3);
							}
						
						
					
				
				
						//Reset semaphore and load variables
						operation.sem_num = REQUEST_LOAD_CLOUD;
						operation.sem_op = 1;
						semop(semid,&operation,1);
						act = 0;
						}
						
						
						
						
						//Delete last cloud added
						else if (act >= DELETE_LAST && act < DELETE_LAST + 10){ 
					
						
							if (act-DELETE_LAST == 1){
							
								sprintf(cloud_name,"object_v1 %d",obj1);
								if (viewer.removePointCloud(cloud_name,v1)){
									obj1--;
								}
							}
					
					
					
							else if (act-DELETE_LAST == 2){
							
								sprintf(cloud_name,"object_v2 %d",obj2);
								if (viewer.removePointCloud(cloud_name,v2)){
									obj2--;			
								}
							}	
					
					
							else if (act-DELETE_LAST == 3){
							
								sprintf(cloud_name,"object_v3 %d",obj3);
								if (viewer.removePointCloud(cloud_name,v3)){
									obj3--;
								
								}
							}
						
							//Reset semaphore and load variables
							act=0;
							operation.sem_num = REQUEST_LOAD_CLOUD;
							operation.sem_op = 1;
							semop(semid,&operation,1);				
						
						}
			
				
				
				
						//Delete all clouds
						else if (act >= DELETE_ALL && act < DELETE_ALL + 10 ){ 
					
							if (act-DELETE_ALL == 1){
								if (viewer.removeAllPointClouds(v1)){
									obj1=0;
								}		
							}
					
					
					
							else if (act-DELETE_ALL == 2){
								if (viewer.removeAllPointClouds(v2)){
									obj2=0;
								}	
							}
						
							
					
							else if (act-DELETE_ALL == 3){
								if (viewer.removeAllPointClouds(v3)){
									obj3=0;
								}						
							}
						
						
						
							else  {
								if (viewer.removeAllPointClouds(v1)){
									obj1=0;
								}
								
								if (viewer.removeAllPointClouds(v2)){
									obj2=0;
								
								}
						
								if (viewer.removeAllPointClouds(v3)){
									obj3=0;
									
								}
							}	
					
							//Reset semaphore and load variables
							act=0;
							operation.sem_num = REQUEST_LOAD_CLOUD;
							operation.sem_op = 1;
							semop(semid,&operation,1);	
					
						}
					
						
						
						
						//Activate/deactivate turn 
						else if (act == ACT_GIRO){
				
							if (turn_on) {
								turn_on = false;
							}
					
							else {
								turn_on = true;
							}
	
					
							//Reset semaphore and load variables
							operation.sem_num = REQUEST_LOAD_CLOUD;
							operation.sem_op = 1;
							semop(semid,&operation,1);
							act = 0;
						}
	
						
						
						
						//Activate diff cloud	
						else if (act == SET_DIFF){
				
							diff = true;
	
					
							//Reset semaphore and load variables
							operation.sem_num = REQUEST_LOAD_CLOUD;
							operation.sem_op = 1;
							semop(semid,&operation,1);
							act = 0;
						}	
	
	
				}//END OF SWITCH POSSIBILITIES
				
			}// END OF PRINCIPAL LOOP
			
		}// END OF CHILD PROCCESS
			
			
			///temporal
			
			//RGB cloud
			/*
			else if (pid > 0){
				std::cout << "Prueba de carga de RGB" << std::endl;
			
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

				uint8_t r(255), g(15), b(15);
				for (float z(-1.0); z <= 1.0; z += 0.05){
					for (float angle(0.0); angle <= 360.0; angle += 5.0){
				

						pcl::PointXYZRGB point;
						point.x = 0.5 * cosf (pcl::deg2rad(angle));
						point.y = sinf (pcl::deg2rad(angle));
						point.z = z;
						uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
						point.rgb = *reinterpret_cast<float*>(&rgb);
						point_cloud_ptr->points.push_back (point);
					}
				
					if (z < 0.0){
						r -= 12;
						g += 12;
					}
				
					else{
						g -= 12;
						b += 12;
					}
				}
 
				point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
				point_cloud_ptr->height = 1;
				
		
						
				load_xyz_rbg (point_cloud_ptr, 1);
			
				getchar();
			}
			*/
			
			
			//MESH
			/*
			else if (pid > 0){
				std::cout << "Prueba de carga de mallado" << std::endl;
				pcl::PolygonMesh malla;
				pcl::io::loadPolygonFileVTK  ("mesh.vtk", malla);
				load_mesh(malla, 1);
				getchar();
			}			
			*/	
			
			///temporal
			
		
		
	}


// DESTRUCTOR
	viewer_rt::~viewer_rt (){	

		//Kill all executed proccess
		for (int i = 0; i < vector_pid.size(); i++){ 
			kill(vector_pid[i], SIGKILL); 
		}	
	
		//Kill the created semaphore
		semctl(semid,0,IPC_RMID,0);
		
		
		//close pipes
		close(pipe_flags[1]);
		close(pipe_flags[0]);
		close(pipe_cloud[1]);
		close(pipe_cloud[0]);	
	}




// PUBLIC FUCTIONS


	void viewer_rt::delete_all (int window){
		
		if (window > 4) window = 4;
		
		pid = fork();
		vector_pid.push_back(pid);
		name_process.push_back("DELETE_ALL");
   
		if (pid == -1)
			std::cout << "[VIEWER_RT] ERROR MAKING FORK" << std::endl;
			
    

		//Child proccess
		else if (pid == 0) {
			
			if (vector_pid.size () >1)
				waitpid(vector_pid[vector_pid.size ()-1],&status,0);
			
			
			//Set pipe message
			int request=DELETE_ALL + window;	
	

			//Request resources of temp file
			operation.sem_num = WRITE_PCD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
			
			
			//Solicitation issued
			write(pipe_flags[1], &request, 1);
			
			
			//Instruction sent, request for the resorces of viewer
			operation.sem_num = 5;
			operation.sem_op = 1;
			semop(semid,&operation,1);
	
	
			//Wait for the load
			operation.sem_num = REQUEST_LOAD_CLOUD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
	
	
			//Freeing resources
			operation.sem_num = WRITE_PCD;
			operation.sem_op = 1;
			semop(semid,&operation,1);
    
			
			//End execution
			exit(EXIT_SUCCESS);
		}	
		
		return;
		
	}
	
	
		
	void viewer_rt::delete_last (int window){
		
		if (window > 4) window = 4;
		
		pid = fork();
		vector_pid.push_back(pid);
		name_process.push_back("DELETE_LAST");
	   
		if (pid == -1)
			std::cout << "[VIEWER_RT] ERROR MAKING FORK" << std::endl;
		
    
		//Child proccess	
		else if (pid == 0) {
			
			if (vector_pid.size () >1){
				waitpid(vector_pid[vector_pid.size ()-1],&status,0);
			}
	
	
			//Set pipe message
			int request=DELETE_LAST + window;	




			//Request resources of temp file
			operation.sem_num = WRITE_PCD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
			
			
			
			
			//Solicitation issued
			write(pipe_flags[1], &request, 1);
			
			
			
			
			//Instruction sent, request for the resorces of viewer
			operation.sem_num = 5;
			operation.sem_op = 1;
			semop(semid,&operation,1);
	
	
	
	
			//Wait for the load
			operation.sem_num = REQUEST_LOAD_CLOUD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
			
			
			
			
			//Freeing resources
			operation.sem_num = WRITE_PCD;
			operation.sem_op = 1;
			semop(semid,&operation,1);
    
			
			
			
			//End execution
			exit(EXIT_SUCCESS);	
		}	
		
		return;
	
	}
	
	
		
	void viewer_rt::load_xyz (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud,int window, bool diff_in){
		
		//Check if the cloud is empty
		if (cloud->empty ())
			return;
		
		
		
		if (window > 4) window=4; 
		if (diff_in) set_diff ();

		//Fork the program
		pid = fork();
		vector_pid.push_back(pid);
		name_process.push_back("LOAD_XYZ");
	   
		if (pid == -1)
			std::cout << "[VIEWER_RT] ERROR MAKING FORK" << std::endl;
		
    
    
		//Child process
		else if (pid == 0) {
			
			if (vector_pid.size () >1)
				waitpid(vector_pid[vector_pid.size ()-1],&status,0);
			
			
			
			//Set pipe message
			int request=LOAD_XYZ_CLOUD+window;
			
			
			
	
			//Objects
			pcl::PCDWriter writer;



	
			//Request resources
			operation.sem_num = WRITE_PCD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
			
			
			
			
			//Solicitation issued
			write(pipe_flags[1], &request, 1);
			writer.write<pcl::PointXYZ> ("temp.pcd", *cloud, false);
	
			
			
			
			//Request for the resorces of viewer
			operation.sem_num = 5;
			operation.sem_op = 1;
			semop(semid,&operation,1);




			//Wait for the load	
			operation.sem_num = REQUEST_LOAD_CLOUD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
			
			
			
			
			//Freeing resources
			operation.sem_num = WRITE_PCD;
			operation.sem_op = 1;
			semop(semid,&operation,1);
	
	
	
	
			//End execution
			exit(EXIT_SUCCESS);
		}

		return;
		
	}
	
	
	
	void viewer_rt::load_xyz_rbg (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud, int window){
		//Check if the cloud is empty
		if (cloud->empty ())
			return;
		
		
		
		if (window > 4) window=4; 

		//Fork the program
		pid = fork();
		vector_pid.push_back(pid);
		name_process.push_back("LOAD_XYZRBG");
	   
		if (pid == -1)
			std::cout << "[VIEWER_RT] ERROR MAKING FORK" << std::endl;
		
    
    
		//Child process
		else if (pid == 0) {
			
			if (vector_pid.size () >1)
				waitpid(vector_pid[vector_pid.size ()-1],&status,0);
			
			
			
			//Set pipe message
			int request=LOAD_XYZRBG_CLOUD+window;
			
			
			
	
			//Objects
			pcl::PCDWriter writer;



	
			//Request resources
			operation.sem_num = WRITE_PCD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
			
			
			
			
			//Solicitation issued
			write(pipe_flags[1], &request, 1);
			writer.write<pcl::PointXYZRGB> ("temp.pcd", *cloud, false);
	
			
			
			
			//Request for the resorces of viewer
			operation.sem_num = 5;
			operation.sem_op = 1;
			semop(semid,&operation,1);




			//Wait for the load	
			operation.sem_num = REQUEST_LOAD_CLOUD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
			
			
			
			
			//Freeing resources
			operation.sem_num = WRITE_PCD;
			operation.sem_op = 1;
			semop(semid,&operation,1);
	
	
	
	
			//End execution
			exit(EXIT_SUCCESS);
		}

		return;
	}
	
	
	
	void viewer_rt::load_mesh(pcl::PolygonMesh polymesh, int window){
				//Check if the cloud is empty
	
		if (window > 4) window=4; 

		//Fork the program
		pid = fork();
		vector_pid.push_back(pid);
		name_process.push_back("LOAD_MESH");
	   
		if (pid == -1)
			std::cout << "[VIEWER_RT] ERROR MAKING FORK" << std::endl;
		
    
    
		//Child process
		else if (pid == 0) {
			
			if (vector_pid.size () >1)
				waitpid(vector_pid[vector_pid.size ()-1],&status,0);
			
			
			
			//Set pipe message
			int request=LOAD_MESH+window;
			
			
		


	
			//Request resources
			operation.sem_num = WRITE_PCD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
			
			
			
			
			//Solicitation issued
			write(pipe_flags[1], &request, 1);
			pcl::io::saveVTKFile ("temp.vtk", polymesh);
	
			
			
			
			//Request for the resorces of viewer
			operation.sem_num = 5;
			operation.sem_op = 1;
			semop(semid,&operation,1);




			//Wait for the load	
			operation.sem_num = REQUEST_LOAD_CLOUD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
			
			
			
			
			//Freeing resources
			operation.sem_num = WRITE_PCD;
			operation.sem_op = 1;
			semop(semid,&operation,1);
	
	
	
	
			//End execution
			exit(EXIT_SUCCESS);
		}

		return;
	}
	
	
	
	void viewer_rt::turn_rotation (){
		
		pid = fork();
		vector_pid.push_back(pid);
		name_process.push_back("TURN_ROTATION");
	   
		if (pid == -1)
			std::cout << "[VIEWER_RT] ERROR MAKING FORK" << std::endl;
		
    
    
    
		//Child process
		else if (pid == 0) {
		
			
			if (vector_pid.size () >1)
				waitpid(vector_pid[vector_pid.size ()-1],&status,0);
			
	
			//Set pipe message
			int request=ACT_GIRO;	
	
	
	
	
			//Request resources
			operation.sem_num = WRITE_PCD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
			
			
			
			
			//Solicitation issued
			write(pipe_flags[1], &request, 1);
	
	
	
			
			//Request for the resorces of viewer
			operation.sem_num = 5;
			operation.sem_op = 1;
			semop(semid,&operation,1);
	
	
	
	
			//Wait for the load	
			operation.sem_num = REQUEST_LOAD_CLOUD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
		
		
		
			
			//Freeing resources
			operation.sem_num = WRITE_PCD;
			operation.sem_op = 1;
			semop(semid,&operation,1);
    
    
    
    
			//End execution
			exit(EXIT_SUCCESS);
		}	
	
		return;

	}


	
	void viewer_rt::set_diff (){
		
		pid = fork();
		vector_pid.push_back(pid);
		name_process.push_back("SET_DIFF");
	   
		if (pid == -1)
			std::cout << "[VIEWER_RT] ERROR MAKING FORK" << std::endl;
		
    
    
    
		//Child process
		else if (pid == 0) {
		
			
			if (vector_pid.size () >1)
				waitpid(vector_pid[vector_pid.size ()-1],&status,0);
			
	
	
	
			//Set pipe message
			int request=SET_DIFF;	
	
	
			//Request resources
			operation.sem_num = WRITE_PCD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
			
			
			//Solicitation issued
			write(pipe_flags[1], &request, 1);
	
			
			//Request for the resorces of viewer
			operation.sem_num = 5;
			operation.sem_op = 1;
			semop(semid,&operation,1);
	
	
			//Wait for the load	
			operation.sem_num = REQUEST_LOAD_CLOUD;
			operation.sem_op = -1;
			semop(semid,&operation,1);
		
			
			//Freeing resources
			operation.sem_num = WRITE_PCD;
			operation.sem_op = 1;
			semop(semid,&operation,1);
    
    
			//End execution
			exit(EXIT_SUCCESS);
		}	
	
		return;

	}	



	void viewer_rt::get_process_vector (std::vector <int> *vector_process_out, std::vector <const char *> *vector_name_out){
		*vector_process_out = vector_pid;
		*vector_name_out = name_process;
		return;
	}
#endif
	
			
			
			
			
			
			
			
