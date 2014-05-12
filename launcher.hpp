#ifndef LAUCHER_HPP
#define LAUCHER_HPP

// CONSTRUCTOR (PRINCIPAL MENU)
	
	 launcher::launcher (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
		
		
		// Set the display process and configure viewer
		//viewer.turn_rotation();
		detector.display_process(&viewer);
		
		
		// Set the cloud
		detector.set_cloud(cloud_in);


		

		
			
		bool run (true);
		
		char archivo [50];

   
		while (run==true){
				

			system("clear");
			std::cout 	<< "........................MENU...............................\n\n"
						<< "On/Off rotation.........................................[1]\n"
						<< "Change configuration parameters.........................[2]\n"
						<< "Display data from the process of reconstruction.........[3]\n\n"
						<< "Show original cloud in the viewer.......................[4]\n"
						<< "Show filtered cloud in the viewer.......................[5]\n"
						<< "Show proyected walls in the viewer......................[6]\n"
						<< "Show reconstructed intersections in the viewer..........[7]\n"
						<< "Show reconstruction points in the viewer................[8]\n"
						<< "Show polygons mesh in the viewer........................[9]\n"
						<< "Show polygons mesh (triangles) in the viewer............[10]\n"
						<< "Show point mesh in the viewer...........................[11]\n\n"					            
						<< "Clear viewer............................................[12]\n"
						<< "Delete last added cloud.................................[13]\n\n"
						<< "Save filtered cloud.....................................[14]\n"
						<< "Save reconstructed intersections cloud..................[15]\n"
						<< "Save reconstruction points cloud........................[16]\n"
						<< "Save polygonmesh room...................................[17]\n"
						<< "save pointmesh room.....................................[18]\n"
						<< "Save polygonmesh room in 'STL' format...................[19]\n"
						<< "Save polygonmesh (triangles) room in 'STL' format.......[20]\n\n"
						<< "Exit program............................................[0]\n"
						<< "\n";
		
			std::cin >> archivo;
			getchar();
		
			//Exit program
			if (!strcmp(archivo,"0")){ 
				system("clear");
				run=false;
			}
		
		
			//On/Off rotation
			else if (!strcmp(archivo,"1")){
				viewer.turn_rotation();
			}
		
		
			//Change configuration parameters
			else if (!strcmp(archivo,"2")){
				change_parameters();
			} 
		
		
			//Display data from the process of reconstruction
			else if (!strcmp(archivo,"3")){
				display_data ();
				getchar();
			} 
		
		
			//Show original cloud in the viewer
			else if (!strcmp(archivo,"4")){
				
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_display (new pcl::PointCloud<pcl::PointXYZ>);
				detector.get_cloud(cloud_to_display);
				viewer.load_xyz(cloud_to_display,1,true);

			} 
			

			//Show filtered cloud in the viewer
			else if (!strcmp(archivo,"5")){
				
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_display (new pcl::PointCloud<pcl::PointXYZ>);
				detector.get_cloud_fitered(cloud_to_display);
				viewer.load_xyz(cloud_to_display,1,true);
			} 
			
			
			//Show proyected walls in the viewer
			else if (!strcmp(archivo,"6")){
				
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_display (new pcl::PointCloud<pcl::PointXYZ>);
				detector.get_proyected_walls(cloud_to_display);
				viewer.load_xyz(cloud_to_display,1,true);
			} 
			
			
			//Show reconstructed intersections in the viewer
			else if (!strcmp(archivo,"7")){
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_display (new pcl::PointCloud<pcl::PointXYZ>);
				detector.get_reconstructed_room(0.01,cloud_to_display);
				viewer.load_xyz(cloud_to_display,1,true);
				
			} 
			
			
			//Show reconstruction points in the viewer
			else if (!strcmp(archivo,"8")){
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_display (new pcl::PointCloud<pcl::PointXYZ>);
				std::vector <point> important_points;
				detector.get_important_points(&important_points);
				cloud_to_display = detector.vectorpoint_2_pointcloud(important_points);
				viewer.load_xyz(cloud_to_display,1,true);
				
			}
			
			
			//Show polygons mesh in the viewer
			else if (!strcmp(archivo,"9")){
				
				pcl::PolygonMesh poygonmesh_room;
				detector.get_poygonmesh_room(&poygonmesh_room);
				
				
			
				viewer.load_mesh(poygonmesh_room, 1);
				
				/*
				for (size_t i=0; i < polygon_planes.size(); i++){
					//viewer.load_mesh(detector.polygon_2_polygonmesh(polygon_planes[i]), 1);
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_display (new pcl::PointCloud<pcl::PointXYZ>);
					for (size_t j=0; j <  polygon_planes[i].points_index.size(); j++)
						cloud_to_display->push_back(important_points[polygon_planes[i].points_index[j]].p);
					viewer.delete_all(1);
					viewer.load_xyz(cloud_to_display,1,false);
					detector.get_cloud_fitered(cloud_to_display);
					viewer.load_xyz(cloud_to_display,1,true);
					getchar();	
				}
				*/		
				
				
			}
			
			
			//Show polygons mesh (triangles) in the viewer
			else if (!strcmp(archivo,"10")){
				
				pcl::PolygonMesh poygonmesh_triangles_room;
				detector.get_poygonmesh_triangles_room(&poygonmesh_triangles_room);
				
				
			
				viewer.load_mesh(poygonmesh_triangles_room, 1);
			}
			
			//Show point mesh in the viewer 
			else if (!strcmp(archivo,"11")){
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_display (new pcl::PointCloud<pcl::PointXYZ>);
				detector.get_pointmesh_room(0.05, cloud_to_display);
				viewer.load_xyz(cloud_to_display,1,true);
				
			}
			
			
			//Clear viewer
			else if (!strcmp(archivo,"12")){
				viewer.delete_all(1);
			}
			
			
			//Delete last added cloud
			else if (!strcmp(archivo,"13")){
				viewer.delete_last(1);
			}
			
				
			//Save filtered cloud	
			else if (!strcmp(archivo,"14")){
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_save (new pcl::PointCloud<pcl::PointXYZ>);
				detector.get_cloud_fitered(cloud_to_save);
				if (!cloud_to_save->empty())
					pcl::io::savePCDFileASCII ("cloud_filtered.pcd", *cloud_to_save);
				
			}	
			
			
			//Save reconstructed intersections cloud	
			else if (!strcmp(archivo,"15")){
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_save (new pcl::PointCloud<pcl::PointXYZ>);
				detector.get_reconstructed_room(0.01,cloud_to_save);
				if (!cloud_to_save->empty())
					pcl::io::savePCDFileASCII ("reconstructed_intersections_cloud.pcd", *cloud_to_save);
			}	
			
			
			//Save reconstruction points cloud	
			else if (!strcmp(archivo,"16")){
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_save (new pcl::PointCloud<pcl::PointXYZ>);
				std::vector <point> important_points;
				detector.get_important_points(&important_points);
				cloud_to_save = detector.vectorpoint_2_pointcloud(important_points);
				if (!cloud_to_save->empty())
					pcl::io::savePCDFileASCII ("reconstruction_points_cloud.pcd", *cloud_to_save);
			}
			
			
			
			//Save polygonmesh room
			else if (!strcmp(archivo,"17")){
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_save (new pcl::PointCloud<pcl::PointXYZ>);
				detector.get_pointmesh_room(0.05, cloud_to_save);
				if (!cloud_to_save->empty())
					pcl::io::savePCDFileASCII ("pointmesh_room.pcd", *cloud_to_save);
			}
			
			
						
			//Save pointmesh room	
			else if (!strcmp(archivo,"18")){
				pcl::PolygonMesh poygonmesh_room;
				detector.get_poygonmesh_room(&poygonmesh_room);
				if (!poygonmesh_room.polygons.empty())
					pcl::io::saveVTKFile ("polygonmesh_room.vtk", poygonmesh_room);
			}
			
			
			
			//Save polygonmesh room in 'STL' format
			else if (!strcmp(archivo,"19")){
				pcl::PolygonMesh poygonmesh_room;
				detector.get_poygonmesh_room(&poygonmesh_room);
				if (!poygonmesh_room.polygons.empty())
					pcl::io::savePolygonFileSTL  ("STL_room.stl", poygonmesh_room);
						
			}
			
			
			//Save polygonmesh room (triangles) 'STL' format
			else if (!strcmp(archivo,"20")){
				pcl::PolygonMesh poygonmesh_triangles_room;
				detector.get_poygonmesh_triangles_room(&poygonmesh_triangles_room);
				if (!poygonmesh_triangles_room.polygons.empty())
					pcl::io::savePolygonFileSTL  ("STL_room(triangles).stl", poygonmesh_triangles_room);
						
			}
			
	}//End while	
}


// DESTRUCTOR
	
	 launcher::~launcher (){	
		// kill all the procces
		std::vector <int> vector_pid;	
		std::vector <const char *> name_process;
		viewer.get_process_vector (&vector_pid, &name_process);
	
		for (int i = 0; i < vector_pid.size(); i++){ 
			kill(vector_pid[i], SIGKILL); 
		}	
		
		std::cout << "TERMINATE EXECUTION" << std::endl;
	}



// OTHERS MENUS
	
	void launcher::change_parameters(){
	
	bool run(true);	
	
	while (run==true){	
		system("clear");
		std::cout << "..................PARAMETERS........................\n"
				  << "KEY FUNCTIONS\n"
				  << "	Density filter:\n"
				  << "	Change grid....................................[1]\n\n"
				 
				  << "	Find wall coeficientes:\n"
				  << "	Change perc_of_search..........................[2]\n"
				  << "	Change Iterations..............................[3]\n"
				  << "	Change radio_coef..............................[4]\n"
				  << "	Change lim_segmentos...........................[5]\n"
				  << "	Change p_admissible_coef.......................[6]\n\n"
				 
				  << "	Proyect planes type B:\n"
				  << "	Change perceptil_n.............................[7]\n"
				  << "	Change min_proyect.............................[8]\n"
				  << "	Change int_min.................................[9]\n"
				  << "	Change tol_seg.................................[10]\n"
				  << "	Change size_min................................[11]\n\n"
				  
				  << "SECUNDARY FUNCTIONS:\n"
				  << "	Get intersection coefficients:\n"
				  << "	Change avoid_ecual_planes......................[12]\n\n"
				  
				  << "	Get intersection with planes:\n"                       
				  << "	Change angle_diff..............................[13]\n\n"
				  
				  << "	Conect two points:\n"
				  << "	Change direction_adm...........................[14]\n\n"
				  
				  << "OTHERS\n"
				  << "	Set default variables..........................[15]\n"
				  << "	Change reference_vector........................[16]\n"
				  << "	Set visualization time.........................[17]\n"
				  << "	Set process visualization......................[18]\n\n"
				  
				  << "Go back........................................[0]\n\n";

		char archivo [50];        
		std::cin >> archivo;        
     

     
		if (!strcmp(archivo,"0")){ 
				run = false;
		} 
     
		//Change grid
		else if (!strcmp(archivo,"1")){
			float grid;
			detector.get_density_filter_grid(&grid);
			
			std::cout << "grid -> old: " << grid << "	new: ";
			std::cin >> grid;
			
			detector.set_density_filter_grid(grid);
			
		}
		
		//Change perc_of_search
		else if (!strcmp(archivo,"2")){
			float perc_of_search;
			detector.get_find_wall_coeficientes_perc_of_search(&perc_of_search);
		
			std::cout << "perc_of_search -> old: " << perc_of_search << "	new: ";
			std::cin >> perc_of_search;
			
			detector.set_find_wall_coeficientes_perc_of_search(perc_of_search);
		}
		
		//Change Iterations
		else if (!strcmp(archivo,"3")){
			int Iterations;
			
			detector.get_find_wall_coeficientes_Iterations(&Iterations);
			
			std::cout << "Iterations -> old: " << Iterations << "	new: ";
			std::cin >> Iterations;	
			
			
			detector.set_find_wall_coeficientes_Iterations(Iterations);
		}
		
		//Change radio_coef
		else if (!strcmp(archivo,"4")){
			float radio_coef;

			detector.get_find_wall_coeficientes_radio_coef(&radio_coef);

			std::cout << "radio_coef -> old: " << radio_coef << "	new: ";
			std::cin >> radio_coef;	
			
			detector.set_find_wall_coeficientes_radio_coef(radio_coef);
		}
		
		//Change lim_segmentos
		else if (!strcmp(archivo,"5")){
			
			float lim_segmentos;
			
			detector.get_find_wall_coeficientes_lim_segmentos(&lim_segmentos);
		
			std::cout << "lim_segmentos -> old: " << lim_segmentos << "	new: ";
			std::cin >> lim_segmentos;	
			
			detector.set_find_wall_coeficientes_lim_segmentos(lim_segmentos);
			
		}
		
		//Change p_admissible_coef
		else if (!strcmp(archivo,"6")){
			
			float p_admissible_coef;
			detector.get_find_wall_coeficientes_p_admissible_coef(&p_admissible_coef);
					
			std::cout << "p_admissible_coef -> old: " << p_admissible_coef << "	new: ";
			std::cin >> p_admissible_coef;	
			
			detector.set_find_wall_coeficientes_p_admissible_coef(p_admissible_coef);
			
			
		}
		
		//Change perceptil_n
		else if (!strcmp(archivo,"7")){
			
			float perceptil_n;
			
			detector.get_proyect_planes_type_B_perceptil_n(&perceptil_n);
							
			std::cout << "perceptil_n -> old: " << perceptil_n << "	new: ";
			std::cin >> perceptil_n;
			
			detector.set_proyect_planes_type_B_perceptil_n(perceptil_n);	
			
		}
		
		//Change min_proyect
		else if (!strcmp(archivo,"8")){
			float min_proyect;
			
			detector.get_proyect_planes_type_B_min_proyect(&min_proyect);
						
			std::cout << "min_proyect -> old: " << min_proyect << "	new: ";
			std::cin >> min_proyect;	
			
			detector.set_proyect_planes_type_B_min_proyect(min_proyect);
			
		}
		
		//Change int_min
		else if (!strcmp(archivo,"9")){
			float int_min;
			
			detector.get_proyect_planes_type_B_int_min(&int_min);
							
			std::cout << "int_min -> old: " << int_min << "	new: ";
			std::cin >> int_min;	
			
			detector.set_proyect_planes_type_B_int_min(int_min);	
			
		}
		
		//Change tol_seg
		else if (!strcmp(archivo,"10")){
			float tol_seg;
			
			detector.get_proyect_planes_type_B_tol_seg(&tol_seg);
									
			std::cout << "tol_seg -> old: " << tol_seg << "	new: ";
			std::cin >> tol_seg;	
			
			detector.set_proyect_planes_type_B_tol_seg(tol_seg);
		}
		
		//Change size_min
		else if (!strcmp(archivo,"11")){
			
			float size_min;
			
			detector.get_proyect_planes_type_B_size_min(&size_min);
											
			std::cout << "size_min -> old: " << size_min << "	new: ";
			std::cin >> size_min;	
			
			detector.set_proyect_planes_type_B_size_min(size_min);	
		}
		
		//Change avoid_ecual_planes
		else if (!strcmp(archivo,"12")){
			float avoid_ecual_planes;
			detector.get_intersection_coefficients_avoid_ecual_planes(&avoid_ecual_planes);
														
			std::cout << "avoid_ecual_planes -> old: " << avoid_ecual_planes << "	new: ";
			std::cin >> avoid_ecual_planes;	
			
			detector.set_intersection_coefficients_avoid_ecual_planes(avoid_ecual_planes);
		}
		
		//Change angle_diff
		else if (!strcmp(archivo,"13")){
			float angle_diff;
			detector.get_intersection_with_planes_angle_diff(&angle_diff);
																	
			std::cout << "angle_diff -> old: " << angle_diff << "	new: ";
			std::cin >> angle_diff;	
			
			detector.set_intersection_with_planes_angle_diff(angle_diff);
		}
		
		//Change direction_adm
		else if (!strcmp(archivo,"14")){
			float direction_adm;
			detector.get_conect_two_points_direction_adm(&direction_adm);
																				
			std::cout << "direction_adm -> old: " << direction_adm << "	new: ";
			std::cin >> direction_adm;	
			
			detector.set_conect_two_points_direction_adm(direction_adm);
		}
		
		//Set default variables
		else if (!strcmp(archivo,"15")){
			detector.set_default_variables();
		}
		
		//Change reference_vector
		else if (!strcmp(archivo,"16")){
			pcl::PointXYZ reference;
			detector.get_reference_vector(&reference);
		
			std::cout << "Vector reference: " << reference << std::endl;
			std::cout << "New vector reference: "  << std::endl;
			std::cout << "x: ";
			std::cin  >> reference.x; 
			std::cout << "y: ";
			std::cin  >> reference.y;
			std::cout << "z: ";
			std::cin  >> reference.z;
		
			detector.set_reference_vector(reference);
		
		}
		
		//Set visualization time
		else if (!strcmp(archivo,"17")){
			int time_in;
			std::cout << "New visualization time: ";
			std::cin >> time_in;
			detector.set_visualization_time(time_in);
		}
		
		//Set process visualization
		else if (!strcmp(archivo,"18")){
			bool P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
			detector.get_process_visualization(&P1, &P2, &P3, &P4, &P5, &P6, &P7, &P8, &P9, &P10, &P11);
			
				
			int process;
				
			std::cout << "Process 1 (density_filter): " << P1 << " -> ";
			std::cin >> process;
				
			if ( process  > 0) P1 = true;
			else P1 = false; 

				
			std::cout << "Process 2 (find_wall_coeficients ): " << P2 << " -> ";
			std::cin >> process;
				
			if ( process  > 0) P2 = true;
			else P2 = false;
				
				
			std::cout << "Process 3 (identify_type_of_plane): " << P3 << " -> ";
			std::cin >> process;
				
			if ( process  > 0) P3 = true;
			else P3 = false;
				
				
			std::cout << "Process 4 (get_intersections_for_planes_type_A): " << P4 << " -> ";
			std::cin >> process;
				
			if ( process  > 0) P4 = true;
			else P4 = false;
				
				
			std::cout << "Process 5 (proyect_planes_type_B): " << P5 << " -> ";
			std::cin >> process;
				
			if ( process  > 0) P5 = true;
			else P5 = false;
				
				
			std::cout << "Process 6 (delete_incorrect_planes): " << P6 << " -> ";
			std::cin >> process;
				
			if ( process  > 0) P6 = true;
			else P6 = false;
				
				
			std::cout << "Process 7 (insert_corners_points): " << P7 << " -> ";
			std::cin >> process;
				
			if ( process  > 0) P7 = true;
			else P7 = false;
				
				
			std::cout << "Process 8 (make_polygons_in_planes_type_A): " << P8 << " -> ";
			std::cin >> process;
				
			if ( process  > 0) P8 = true;
			else P8 = false;
				
				
			std::cout << "Process 9 (make_polygons_in_planes_type_B): " << P9 << " -> ";
			std::cin >> process;
				
			if ( process  > 0) P9 = true;
			else P9 = false;
				
				
			std::cout << "Process 10 (find_polygons): " << P10 << " -> ";
			std::cin >> process;
				
			if ( process  > 0) P10 = true;
			else P10 = false;
				
				
			std::cout << "Process 11 (complete_segments): " << P11 << " -> ";
			std::cin >> process;
				
			if ( process  > 0) P11 = true;
			else P11 = false;
				
				
			detector.set_process_visualization(P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11);
			
		}
		
		
    

	}
	
	return;
}


	
	void launcher::display_data (){
		system("clear");
		std::cout << "........................DISPLAY DATA..........................\n\n"
				  << "Wall and intersection coefficients.........................[1]\n"
				  << "Points information.........................................[2]\n"
				  << "Points from plane i information............................[3]\n"
				  << "Polygons planes............................................[4]\n"
				  << "Execution of parallel proccess.............................[5]\n"
				  << "Processes times............................................[6]\n\n"
				  
				  << "Go back....................................................[0]\n"
				  << "\n";
     
		char archivo [50];        
		std::cin >> archivo;        
     
     
		//Wall and intersection coefficients
		if (!strcmp(archivo,"1")){ 
			
			system("clear");
			std::vector <plane>  wall_coef;
			std::vector <line> coef_line_out;
				
			detector.get_wall_coefficients(&wall_coef);
			detector.get_intersection_coefficients(&coef_line_out);
			for (size_t i = 0; i < wall_coef.size(); i++ ){
			
				if (wall_coef[i].type_A_L == true) std::cout << "Plane type A lower" << std::endl;
				if (wall_coef[i].type_A_U == true) std::cout << "Plane type A upper" << std::endl;
				if (wall_coef[i].type_B == true) std::cout << "Plane type B" << std::endl;
			
				std::cout   << "\t a: " << wall_coef[i].a 
							<< "\t b: " << wall_coef[i].b 
							<< "\t c: " << wall_coef[i].c 
							<< "\t d: " << wall_coef[i].d 
							<< "\t points: " << wall_coef[i].n_points 
							<< std::endl
							<< "\n\t\tIntersections:"
							<< std::endl; 
					   
				for (size_t j=0; j < wall_coef[i].index_of_plane_intersection.size(); j++){
					std::cout 
					   
							<< "\t\t a: " << wall_coef[wall_coef[i].index_of_plane_intersection[j]].a 
							<< "\t\t b: " << wall_coef[wall_coef[i].index_of_plane_intersection[j]].b 
							<< "\t\t c: " << wall_coef[wall_coef[i].index_of_plane_intersection[j]].c 
							<< "\t\t d: " << wall_coef[wall_coef[i].index_of_plane_intersection[j]].d 
							<< "\t\t points: " << wall_coef[i].points_in_intersection[j]
							<< std::endl
							<< "\t\t\t (Vx,Vy,Vz)  ->  (" << coef_line_out[wall_coef[i].index_of_line_intersection[j]].vx << " , " 
														  << coef_line_out[wall_coef[i].index_of_line_intersection[j]].vy << " , " 
														  << coef_line_out[wall_coef[i].index_of_line_intersection[j]].vz << ")\n"
							<< "\t\t\t  (x,y,z)    ->  (" << coef_line_out[wall_coef[i].index_of_line_intersection[j]].x << " , " 
														  << coef_line_out[wall_coef[i].index_of_line_intersection[j]].y << " , " 
														  << coef_line_out[wall_coef[i].index_of_line_intersection[j]].z << ")"
							<< std::endl;	    
				}
		
		}
		
			getchar();
		}
	    
	    
	    //Points information
	    else if (!strcmp(archivo,"2")){
			
			system("clear");
			std::vector <point> important_points;
			detector.get_important_points(&important_points);
			
			std::ofstream file;
			file.open ("Points data.txt");
		
			for (size_t i = 0; i < important_points.size(); i++ ){
				std::cout << "\nPoint " << i << ": " <<important_points[i].p << std::endl;
				file << "\nPoint " << i << ": " <<important_points[i].p << std::endl;
			
				std::cout << "\tBelongs to:" << std::endl;
				file << "\tBelongs to:" << std::endl;
			
				for (size_t j = 0; j < important_points[i].index_of_plane_belonging.size(); j++){
					std::cout << "\t\tPlane index: "<< important_points[i].index_of_plane_belonging[j];
					file << "\t\tPlane index: "<< important_points[i].index_of_plane_belonging[j];
				
					std::cout << "\t\tAngle: " << important_points[i].angle_swept[j] << std::endl;
					file << "\t\tAngle: " << important_points[i].angle_swept[j] << std::endl;
				}
			
				std::cout << "\tConect to: " << std::endl;
				file <<  "\tConect to: " << std::endl;
			
				for (size_t j = 0; j < important_points[i].index_of_point_conected.size(); j++){
					std::cout << "\t\t"<<important_points[i].index_of_point_conected[j] << " -> " << important_points[important_points[i].index_of_point_conected[j]].p << std::endl;
					file <<  "\t\t"<<important_points[i].index_of_point_conected[j] << " -> " << important_points[important_points[i].index_of_point_conected[j]].p << std::endl;
				}
				
			}
			
			file.close();
			getchar();
		}
	    
	    
	    //Points from plane i information
	    else if (!strcmp(archivo,"3")){ 
			
			system("clear");
			size_t plane_index;
			std::cout << "Index of plane: " ;
			std::cin >> plane_index;
		
			
			std::vector <point> important_points;
			detector.get_important_points(&important_points);
			std::ofstream file;
			file.open ("Points data from plane i.txt");
		
		
			for (size_t i=0; i < important_points.size(); i++){
			
				bool ok(false);
				for (size_t j=0; j < important_points[i].index_of_plane_belonging.size(); j++)
					if (important_points[i].index_of_plane_belonging[j] == plane_index) ok = true;
			
				if (ok){	
				
					std::cout << "\nPoint " << i << ": " <<important_points[i].p << std::endl;
					file << "\nPoint " << i << ": " <<important_points[i].p << std::endl;
			
					std::cout << "\tBelongs to:" << std::endl;
					file << "\tBelongs to:" << std::endl;
			
					for (size_t j = 0; j < important_points[i].index_of_plane_belonging.size(); j++){
						if (important_points[i].index_of_plane_belonging[j] == plane_index){
							std::cout << "\t\tPlane index: "<< important_points[i].index_of_plane_belonging[j];
							file << "\t\tPlane index: "<< important_points[i].index_of_plane_belonging[j];
				
							std::cout << "\t\tAngle: " << important_points[i].angle_swept[j] << std::endl;
							file << "\t\tAngle: " << important_points[i].angle_swept[j] << std::endl;
						}
					}
			
					std::cout << "\tConect to: " << std::endl;
					file <<  "\tConect to: " << std::endl;
			
					for (size_t j = 0; j < important_points[i].index_of_point_conected.size(); j++){
						std::cout << "\t\t"<<important_points[i].index_of_point_conected[j] << " -> " << important_points[important_points[i].index_of_point_conected[j]].p << std::endl;
						file <<  "\t\t"<<important_points[i].index_of_point_conected[j] << " -> " << important_points[important_points[i].index_of_point_conected[j]].p << std::endl;
					}
				}	
			
					
			}
			
			
			file.close();
			getchar();
		}
	
	    
	    //Polygons planes
		else if (!strcmp(archivo,"4")){ 
			
			system("clear");
			std::vector <polygon_vertices> polygon_planes;
			detector.get_polygons_planes(&polygon_planes);
			std::vector <point> important_points;
			detector.get_important_points(&important_points);
			
			
			for (size_t i=0; i < polygon_planes.size(); i++){
				std::cout << "\nPolygon: " << i << " belongs to the plane: " << polygon_planes[i].plane_index << "\nCenter -> " << polygon_planes[i].center << std::endl;
				for (size_t j=0; j < polygon_planes[i].points_index.size(); j++)
					std::cout << polygon_planes[i].points_index[j] << " -> " << important_points[polygon_planes[i].points_index[j]].p << std::endl;
			}
		
			getchar();
		
		}
	
	
		//Execution of parallel proccess
		else if (!strcmp(archivo,"5")){ 
			
			system("clear");
			std::vector <int> vector_pid;	
			std::vector <const char *> name_process;
			viewer.get_process_vector (&vector_pid, &name_process);
	
			std::cout << std::endl;
			std::cout << "Number of executed procces: " <<   vector_pid.size() << std::endl;
			for (int i = 0; i < vector_pid.size();i++) 
				std::cout <<  "PROCESS: " << name_process[i] << "\t PID: " << vector_pid[i] << endl; 
		
			getchar();
	
		}
	
	
		//Processes times
		else if (!strcmp(archivo,"6")){
			
			system("clear");
			std::vector<double> time_out_vector;
			detector.get_process_time_vector(&time_out_vector);
			
			for (size_t i=0; i < time_out_vector.size()-1; i++)
				std::cout << "For process " << i << " " << time_out_vector[i] << std::endl;
			
			std::cout << "Total time: " << time_out_vector[time_out_vector.size()-1] << std::endl;
			getchar();
		} 
	
	}



#endif
	
			
			
			
			
			
			
			
