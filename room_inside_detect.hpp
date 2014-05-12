#ifndef ROOM_INSIDE_DETECT_HPP
#define ROOM_INSIDE_DETECT_HPP


///PUBLIC (GENERAL FUCTIONS) 

	// CONSTRUCTOR
	 room_inside_detect::room_inside_detect(){

		process_done = true;
		
		//Display process
		display_density_filter = false;
		display_find_wall_coeficients = false;
		display_find_intersections_coeficients = false;
		display_identify_type_of_plane = false;
		display_proyect_planes_type_B = false;
		display_delete_incorrect_planes = false;
		display_insert_corners_points = false;
		display_make_polygons_in_planes_type_A = false;
		display_make_polygons_in_planes_type_B = false;
		display_find_polygons = false;
		display_complete_segments = false;
		display_process_time = false;
		wait_for_enter = false;
		wait_time = 0;
		
		
			
		
		
		//GENERAL
		sensor.x = 0;
		sensor.y = 0;
		sensor.z = 0;
		
		reference.x = 1;
		reference.y = 1;
		reference.z = 0;
		
		
		
		//PRIVATE KEY FUNCTIONS
						

			//density_filter variables
			grid = -1;
		
		
			// find_wall_coeficients
			perc_of_search = -1;
			Iterations = -1;
			radio_coef = -1;
			lim_segmentos = -1;
			p_admissible_coef = -1;	

			
			//proyect_planes_type_B
			perceptil_n = -1;
			min_proyect = -1;
			int_min = -1;
			tol_seg = -1;
			size_min = -1;
		
		
			
		//PRIVATE SECONDARY FUNCTIONS
			
			
			//get_intersection_planes
			avoid_ecual_planes = -1;
		
		
			//intersection_with_planes
			angle_diff = -1;
		
		
			//conect_two_points
			direction_adm = -1;
				
	
		}


	// DESTRUCTOR
	room_inside_detect::~room_inside_detect (){	


	}


	// SET_CLOUD /////////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::set_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
		
		if (!cloud_in->empty()){
			cloud_original = *cloud_in;
			process_done = false;
		}
		

		return;
	}
			
	
	// DISPLAY_PROCESS ///////////////////////////////////////////////////////////////////////////////////////////////////	
	void room_inside_detect::display_process(viewer_rt *viewer_main){
		display = true;
		viewer = viewer_main;
		return;	
	}	
	
	
	// EXTRACT_PLANE /////////////////////////////////////////////////////////////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZ>::Ptr room_inside_detect::extract_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float tolerancia,bool invert, plane plane_in){
		
		
		// CLouds
		pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud (new pcl::PointCloud<pcl::PointXYZ> );
		
		
		// Indices
		std::vector<int> indices;
		std::vector<int> invert_indices;
		
		
		// Coeficientes
		Eigen::Vector4f model_coefficients (plane_in.a , plane_in.b , plane_in.c , plane_in.d) ;
		
	
	
		// extract plane
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr Sample_Consensus_Model_Plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
		Sample_Consensus_Model_Plane->selectWithinDistance (model_coefficients , tolerancia, indices);
	
		if (invert){
			for (size_t i=0; i < cloud->points.size (); i++){
				invert_indices.push_back(i);
				for (size_t j=0; j < indices.size (); j++){
					if (indices[j]==i){
						invert_indices.pop_back();
					}
				}			
			} 
		
			pcl::copyPointCloud<pcl::PointXYZ>(*cloud , invert_indices, *extracted_cloud);
		}
	
		else {
			pcl::copyPointCloud<pcl::PointXYZ>(*cloud , indices, *extracted_cloud);
		}
	
		
		return(extracted_cloud);
	}
		
	
	// VECTORPOINT_2_POINTCLOUD ///////////////////////////////////////////////////////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZ>::Ptr room_inside_detect::vectorpoint_2_pointcloud(std::vector <point> points_in){
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr points_out (new pcl::PointCloud<pcl::PointXYZ> );
		
		for (size_t i=0; i < points_in.size(); i++){
			points_out->push_back(points_in[i].p);
		}	
		return (points_out);	
	}
	
	




///PUBLIC (GET KEY DATA FUNCTIONS) 
	
	//GET_CLOUD_ORIGINAL //////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){
		
		*cloud_out = cloud_original;
		
		return;
	}
	
	
	// GET_CLOUD_FILTERED ////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_cloud_fitered (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){
		
		if (cloud_filtered.empty() && !cloud_original.empty())
			density_filter();
		
		*cloud_out = cloud_filtered;
		return;
	}
	
	
	// GET_WALL_COEFFICIENTS //////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_wall_coefficients (std::vector <plane> *coef_out) {
		
		if (!process_done) process_cloud();
		
		*coef_out = wall_coef;
		
		return;
	}
	
	
	// GET_INTERSECTION_COEFFICIENTS ///////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_intersection_coefficients (std::vector <line> *coef_line_out){
		
		if (!process_done) process_cloud();
		
		*coef_line_out = line_coef;
		
		return;
	}
	
	
	//GET_IMPORTANT_POINTS ///////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_important_points(std::vector <point> *important_points_out){
		
		if (!process_done) process_cloud();
		
		*important_points_out = important_points;
		return;
	}
	
	
	//GET_POLYGONS_PLANES ///////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_polygons_planes(std::vector <polygon_vertices> *polygon_planes_out){
		
		if (!process_done) process_cloud();
		
		*polygon_planes_out = polygon_planes;
		return;
	}
	
	
	//GET_PROYECTED_WALLS //////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_proyected_walls (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){
		
		if (!process_done) process_cloud();
		
		*cloud_out = cloud_proyected;
	
		return;
	}
	
	
	// GET_PROCESS_TIME_VECTOR //////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_process_time_vector(std::vector<double> *time_out_vector){
		
		if (!process_done) process_cloud();
		
		*time_out_vector = vector_process_time;
		
		return;
	}
	
	
	

///PUBLIC (GET SECONDARY DATA FUNCTIONS) 
	
	// GET_RECONSTRUCTION_ROOM //////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_reconstructed_room (float interval, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){
		
		if (!process_done) process_cloud();
		if (cloud_reconstructed.empty()) complete_segments (interval);
		
		
		*cloud_out = cloud_reconstructed;
	
		return;
	}
	
	
	// GET_POLYGONMESH_ROOM /////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_poygonmesh_room(pcl::PolygonMesh *poygonmesh_room_out ){
		if (!process_done) process_cloud();
		if (poygonmesh_room.polygons.empty()) make_polygonmesh();
		
		*poygonmesh_room_out = poygonmesh_room;
		
	
		return;
	}
	
	
	// GET_POLYGONMESH_TRIANGLES_ROOM /////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_poygonmesh_triangles_room(pcl::PolygonMesh *poygonmesh_triangles_room_out){
		if (!process_done) process_cloud();
		if (poygonmesh_triangles_room.polygons.empty()) make_polygonmesh_triangles();
		
		
		*poygonmesh_triangles_room_out = poygonmesh_triangles_room;
	
		return;
	}
	
	
	// GET_POINTMESH_ROOM ///////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_pointmesh_room(float interval, pcl::PointCloud<pcl::PointXYZ>::Ptr pointmesh_room_out){
		if (!process_done) process_cloud();
		if (pointmesh_room.empty()) make_pointmesh(interval);
		
		
		*pointmesh_room_out = pointmesh_room;
	
		return;
	}
		
	
///PUBLIC (SET-GET VARIABLES FUNCTIONS) 
	
	//Key functions
	void room_inside_detect::get_density_filter_grid(float *grid_out){
		*grid_out = grid;
		return;
	}
	void room_inside_detect::set_density_filter_grid(float grid_in){
		grid = grid_in;
		return;
	} 
			
	void room_inside_detect::get_find_wall_coeficientes_perc_of_search(float *perc_of_search_out){
		*perc_of_search_out = perc_of_search;
		return;
	}
	void room_inside_detect::set_find_wall_coeficientes_perc_of_search(float perc_of_search_in){
		perc_of_search = perc_of_search_in;
		return;
	}
			
	void room_inside_detect::get_find_wall_coeficientes_Iterations(int *Iterations_out){
		*Iterations_out = Iterations;
		return;
	}
	void room_inside_detect::set_find_wall_coeficientes_Iterations(int Iterations_in){
		Iterations = Iterations_in;
		return;
	}
		
	void room_inside_detect::get_find_wall_coeficientes_radio_coef(float *radio_coef_out){
		*radio_coef_out = radio_coef;
		return;
	}
	void room_inside_detect::set_find_wall_coeficientes_radio_coef(float radio_coef_in){
		radio_coef = radio_coef_in;
		return;
	}
			
	void room_inside_detect::get_find_wall_coeficientes_lim_segmentos(float *lim_segmentos_out){
		*lim_segmentos_out = lim_segmentos;
		return;
	}
	void room_inside_detect::set_find_wall_coeficientes_lim_segmentos(float lim_segmentos_in){
		lim_segmentos = lim_segmentos_in;
		return;
	}
			
	void room_inside_detect::get_find_wall_coeficientes_p_admissible_coef(float *p_admissible_coef_out){
		*p_admissible_coef_out = p_admissible_coef;
		return;
	}	
	void room_inside_detect::set_find_wall_coeficientes_p_admissible_coef(float p_admissible_coef_in){
		p_admissible_coef = p_admissible_coef_in;
		return;
	}		
	
	void room_inside_detect::get_proyect_planes_type_B_perceptil_n(float *perceptil_n_out){
		*perceptil_n_out = perceptil_n;
		return;
	}
	void room_inside_detect::set_proyect_planes_type_B_perceptil_n(float perceptil_n_in){
		perceptil_n = perceptil_n_in;
		return;
	}
		
	void room_inside_detect::get_proyect_planes_type_B_min_proyect(float *min_proyect_out){
		*min_proyect_out = min_proyect;
		return;
	}
	void room_inside_detect::set_proyect_planes_type_B_min_proyect(float min_proyect_in){
		min_proyect = min_proyect_in;
		return;
	}
			
	void room_inside_detect::get_proyect_planes_type_B_int_min(float *int_min_out){
		*int_min_out = int_min;
		return;
	}
	void room_inside_detect::set_proyect_planes_type_B_int_min(float int_min_in){
		int_min = int_min_in;
		return;
	}
			
	void room_inside_detect::get_proyect_planes_type_B_tol_seg(float *tol_seg_out){
		*tol_seg_out = tol_seg;
		return;
	}
	void room_inside_detect::set_proyect_planes_type_B_tol_seg(float tol_seg_in){
		tol_seg = tol_seg_in;
		return;
	}
			
	void room_inside_detect::get_proyect_planes_type_B_size_min(float *size_min_out){
		*size_min_out = size_min;
		return;
	}
	void room_inside_detect::set_proyect_planes_type_B_size_min(float size_min_in){
		size_min = size_min_in;
		return;
	}
	
	
	//Secundary funnctions
	void room_inside_detect::get_intersection_coefficients_avoid_ecual_planes(float *avoid_ecual_planes_out){
		*avoid_ecual_planes_out = avoid_ecual_planes;
		return;
	}
	void room_inside_detect::set_intersection_coefficients_avoid_ecual_planes(float avoid_ecual_planes_in){
		avoid_ecual_planes = avoid_ecual_planes_in;
		return;
	}
			
	void room_inside_detect::get_intersection_with_planes_angle_diff(float *angle_diff_out){
		*angle_diff_out = angle_diff;
		return;
	}
	void room_inside_detect::set_intersection_with_planes_angle_diff(float angle_diff_in){
		angle_diff = angle_diff_in;
		return;
	}
			
	void room_inside_detect::get_conect_two_points_direction_adm(float *direction_adm_out){
		*direction_adm_out = direction_adm;
		return;
	}
	void room_inside_detect::set_conect_two_points_direction_adm(float direction_adm_in){
		direction_adm = direction_adm_in;
		return;
	}
	
	
	//Others
	void room_inside_detect::set_default_variables(){
	
	//GENERAL
		sensor.x = 0;
		sensor.y = 0;
		sensor.z = 0;
		
		reference.x = 1;
		reference.y = 1;
		reference.z = 0;
		
		
		
	//PRIVATE KEY FUNCTIONS
		
		//density_filter variables
		grid = -1;
		
		
		// find_wall_coeficients
		perc_of_search = -1;
		Iterations = -1;
		radio_coef = -1;
		lim_segmentos = -1;
		p_admissible_coef = -1;
		
		
		//proyect_planes_type_B
		perceptil_n = -1;
		min_proyect = -1;
		int_min = -1;
		tol_seg = -1;
		size_min = -1;
		
		
		
	//PRIVATE SECONDARY FUNCTIONS
		
		//get_intersection_planes
		avoid_ecual_planes = -1;
		
		
		//intersection_with_planes
		angle_diff = -1;
		
		
		//conect_two_points
		direction_adm = -1;
		
		cloud_filtered.clear();
		cloud_reconstructed.clear();
		wall_coef.clear();
		line_coef.clear();
		important_points.clear();
		
	}
	
	void room_inside_detect::get_reference_vector(pcl::PointXYZ *reference_out){
		*reference_out = reference;
		return;
	}
	
	void room_inside_detect::set_reference_vector(pcl::PointXYZ reference_in){
		reference = reference_in;
		return;
	}
	
	void room_inside_detect::get_process_visualization(bool *P1, bool *P2, bool *P3, bool *P4, bool *P5, bool *P6, bool *P7, bool *P8, bool *P9, bool *P10, bool *P11){ 
	
	
		*P1 = display_density_filter;
	
		*P2 = display_find_wall_coeficients;
	
		*P3 = display_find_intersections_coeficients;
	
		*P4 = display_identify_type_of_plane;
	
		*P5 = display_proyect_planes_type_B;
	
		*P6 = display_delete_incorrect_planes;
	
		*P7 = display_insert_corners_points;
	
		*P8 = display_make_polygons_in_planes_type_A; 
	
		*P9 = display_make_polygons_in_planes_type_B;
	
		*P10 = display_find_polygons;

		*P11 = display_complete_segments;
		
		return; 
	
	}
	
	void room_inside_detect::set_process_visualization(bool P1, bool P2, bool P3, bool P4, bool P5, bool P6, bool P7, bool P8, bool P9, bool P10, bool P11){ 
	
	
		display_density_filter = P1;
	
		display_find_wall_coeficients = P2;
	
		display_find_intersections_coeficients = P3;
	
		display_identify_type_of_plane = P4;
	
		display_proyect_planes_type_B = P5;
	
		display_delete_incorrect_planes = P6;
	
		display_insert_corners_points = P7;
	
		display_make_polygons_in_planes_type_A = P8; 
	
		display_make_polygons_in_planes_type_B = P9;
	
		display_find_polygons = P10;

		display_complete_segments = P11;
	
		return;
	}
	
	void room_inside_detect::set_visualization_time(int time_in){
		
		wait_time = time_in;
		
		if (wait_time > 0) wait_for_enter = false;
		else wait_for_enter = true;
		
		return;
	}
	
	
	
	
	
///PRIVATE (SECONDARY FUNCTIONS)
	
	// RESET_DATA ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::reset_data(){
		
			cloud_filtered.clear();
			cloud_proyected.clear();
			cloud_reconstructed.clear();
			pointmesh_room.clear();
			
			
			pcl::PolygonMesh poygonmesh_room_new;
			poygonmesh_room = poygonmesh_room_new;
			poygonmesh_triangles_room = poygonmesh_room_new;
		
		
			//Coeficients
			wall_coef.clear();
			line_coef.clear();
			important_points.clear();
			polygon_planes.clear();
			vector_process_time.clear();
			
		return;
	}
	
	
	// CHECK_PLANE_SYSTEM ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool room_inside_detect::check_plane_system(size_t index_A, size_t index_B, size_t index_C){
		
		plane plane_A, plane_B,plane_C;
		
		plane_A = wall_coef[index_A];
		plane_B = wall_coef[index_B];
		plane_C = wall_coef[index_C];
		
		
		pcl::PointXYZ v1,v2,v3;
		v1.x = plane_A.a;
		v1.y = plane_A.b;
		v1.z = plane_A.c;
		
		v2.x = plane_B.a;
		v2.y = plane_B.b;
		v2.z = plane_B.c;
		
		v3.x = plane_C.a;
		v3.y = plane_C.b;
		v3.z = plane_C.c;
		
		bool ok(true);
		if (angle_diff < 0) angle_diff = 0.5235; // 30��
		if (angle_between_two_vetors(v1, v2) < angle_diff) ok = false;
		if (angle_between_two_vetors(v1, v3) < angle_diff) ok = false;
		if (angle_between_two_vetors(v2, v3) < angle_diff) ok = false;
		
		return (ok);
	}
	
	
	// INTERSECTION_WITH_PLANES ////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::intersection_with_planes(size_t index_A, size_t index_B, size_t index_C){
		
		plane plane_A, plane_B,plane_C;
		
		plane_A = wall_coef[index_A];
		plane_B = wall_coef[index_B];
		plane_C = wall_coef[index_C];
		
		
		bool ok;
		ok = check_plane_system(index_A, index_B, index_C);
		
		
		
		if (ok){
			
			Eigen::Matrix3f system_matrix; 
			Eigen::Vector3f system_vector, system_solution; 
			Eigen::LDLT<Eigen::Matrix3f> solver; 
			pcl::PointXYZ point_out;

		
		
			//calculate the equation of the intersection	
			system_matrix (0,0) = plane_A.a; 
			system_matrix (0,1) = plane_A.b;
			system_matrix (0,2) = plane_A.c;
			system_matrix (1,0) = plane_B.a; 
			system_matrix (1,1) = plane_B.b; 
			system_matrix (1,2) = plane_B.c;
			system_matrix (2,0) = plane_C.a; 
			system_matrix (2,1) = plane_C.b;
			system_matrix (2,2) = plane_C.c;  
		
		
			system_vector (0) = -plane_A.d;
			system_vector (1) = -plane_B.d;
			system_vector (2) = -plane_C.d;
			
		 
					
			//solution of the system	
			system_solution = system_matrix.inverse() * system_vector;
		
			//Store the value and return
			point_out.x = system_solution(0);
			point_out.y = system_solution(1);
			point_out.z = system_solution(2);
			
			point point_added;
			
			point_added.p = point_out;
			
			point_added.index_of_plane_belonging.push_back(index_A);
			point_added.angle_swept.push_back(get_sweep_angle(point_out, plane_A));
			
			point_added.index_of_plane_belonging.push_back(index_B);
			point_added.angle_swept.push_back(get_sweep_angle(point_out, plane_B));
			
			point_added.index_of_plane_belonging.push_back(index_C);
			point_added.angle_swept.push_back(get_sweep_angle(point_out, plane_C));
			
			important_points.push_back(point_added);
			
			//show in viewer
			if (display && display_insert_corners_points){
				pcl::PointCloud<pcl::PointXYZ>::Ptr viewer_c (new pcl::PointCloud<pcl::PointXYZ>);
				viewer_c->push_back(point_out);
				viewer->load_xyz(viewer_c,2,false);
			}

		}
		

		
		

		
		return;
			
	}
		
	
	// EXTRACT_POINT_VECTORPOINT /////////////////////////////////////////////////////////////////////////////
	std::vector <point> room_inside_detect::extract_point_vectorpoint(std::vector <point> points_in, size_t index){
		
		std::vector <point> points_out;
		
		for (size_t i=0; i < points_in.size(); i++){
			if (i != index){
				
				points_out.push_back(points_in[i]);
				
				std::vector <size_t> new_index_of_point_conected;
				for (size_t j=0; j < points_out[points_out.size()-1].index_of_point_conected.size(); j++){
					
					
					if (points_out[points_out.size()-1].index_of_point_conected[j] > index) new_index_of_point_conected.push_back(points_in[i].index_of_point_conected[j]-1);
					else if (points_out[points_out.size()-1].index_of_point_conected[j] < index) new_index_of_point_conected.push_back(points_in[i].index_of_point_conected[j]);	
				}
				
				points_out[points_out.size()-1].index_of_point_conected = new_index_of_point_conected;

				
			}
		}
		
	
		return (points_out);
	}
	
	
	// VECTOR_ORGANIZE_DESCENDING /////////////////////////////////////////////////////////////////////////////////////////////////////
	std::vector<size_t> room_inside_detect::vector_organize_descending(std::vector<size_t> vector_in){
		
		std::vector<size_t> vector_out = vector_in;
		size_t aux;
		
		for (size_t i=0; i < vector_out.size(); i++){
			for (size_t j=i+1; j < vector_out.size(); j++){
				if (vector_out[i] < vector_out[j]){
					aux = vector_out[i];
					vector_out[i] = vector_out[j];
					vector_out[j] = aux;
				}
				
			}
		}

		return (vector_out);
	}
	
	
	// FIND_FAREST_POINTS ////////////////////////////////////////////////////////////////////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZ>::Ptr room_inside_detect::find_farest_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
		/*
		std::cout << cloud_in->size() << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr farest_points (new pcl::PointCloud<pcl::PointXYZ>);
		
		
		
		if (cloud_in->points.size() > 1){ 	
			
		
			pcl::PointXYZ point;
			point.x = 0;
			point.y = 0;
			point.z = 0;
			
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud (cloud_in);	
			
			std::vector<int> point_indices;
			std::vector<float> point_distances;
			kdtree.nearestKSearch (point, cloud_in->size(), point_indices, point_distances);
		
			farest_points->push_back(point);
			farest_points->push_back(point);
		}
		
		
		else{
			pcl::PointXYZ point;
			point.x = 0;
			point.y = 0;
			point.z = 0;
			farest_points->push_back(point);
			farest_points->push_back(point);
		}
		std::cout << farest_points->size() << std::endl;
		return (farest_points);
		
		
		*/
			
		pcl::PointCloud<pcl::PointXYZ>::Ptr farest_points (new pcl::PointCloud<pcl::PointXYZ>);
		
		int rmax(0),tmax(0);
		float module_max(0);
				
		for (int r=0; r < cloud_in->size () ; r++){
		
			for (int t=0; t < cloud_in->size () ; t++){
						
				float module,x,y,z;
						
				x=cloud_in->points[r].x - cloud_in->points[t].x;
				y=cloud_in->points[r].y - cloud_in->points[t].y;
				z=cloud_in->points[r].z - cloud_in->points[t].z;
				module = x*x + y*y + z*z;
						
				if ( module > module_max ) {
					module_max = module;
					tmax=t;
					rmax=r;
				}
			}
		}
		
		farest_points->push_back(cloud_in->points[tmax]);
		farest_points->push_back(cloud_in->points[rmax]);
		
		return (farest_points);
	}
	
	
	// DIVIDE_LINE ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZ>::Ptr room_inside_detect::divide_line(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float interval){
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
		
		int end;
		double dd,
				x,
				y,
				z,
				module, 
				d;
	
		x = cloud_in->points[1].x - cloud_in->points[0].x;
		y = cloud_in->points[1].y - cloud_in->points[0].y;
		z = cloud_in->points[1].z - cloud_in->points[0].z;
				

		module = x*x + y*y + z*z;
	
		// distance
		d = sqrt(module);
	
		// interval
		end = d / interval;
		dd= 1.0 / end;
		
		
		pcl::PointXYZ point;
		
				
		// fills with the specefic interval
		for (int k=0; k < end ;k++){
					
			point.x = cloud_in->points[0].x + (cloud_in->points[1].x - cloud_in->points[0].x)*k*dd;
			point.y = cloud_in->points[0].y + (cloud_in->points[1].y - cloud_in->points[0].y)*k*dd;
			point.z = cloud_in->points[0].z + (cloud_in->points[1].z - cloud_in->points[0].z)*k*dd;
				
			cloud_out->push_back(point);		
		}
		
		
		return (cloud_out);
	}
	
	
	// EXTRACT_LINE /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::extract_line_index(size_t index){
		
		//extract index
		std::vector<line> line_coef_aux;
		for (size_t i=0; i < line_coef.size(); i++)
			if (i != index)
				line_coef_aux.push_back(line_coef[i]);
		line_coef = line_coef_aux;
		
		
		//change plane reference index	
		for (size_t i=0; i < wall_coef.size(); i++){
			std::vector<size_t> index_of_line_intersection_aux;
			for (size_t j=0; j < wall_coef[i].index_of_line_intersection.size(); j++){
				
				if (wall_coef[i].index_of_line_intersection[j] < index)
					index_of_line_intersection_aux.push_back(wall_coef[i].index_of_line_intersection[j]);
				
				else if (wall_coef[i].index_of_line_intersection[j] > index)
					index_of_line_intersection_aux.push_back(wall_coef[i].index_of_line_intersection[j]-1);
			}
			
			wall_coef[i].index_of_line_intersection = index_of_line_intersection_aux;
		}
		
	}
	
	
	// EXTRACT_WALL /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::extract_wall_index(size_t index){

		//extract index
		std::vector <plane> wall_coef_aux;
		for (size_t i = 0; i < wall_coef.size(); i++)
			if (index != i)
				wall_coef_aux.push_back(wall_coef[i]);
		
		wall_coef = wall_coef_aux;
				
				
		//change plane references and extract incorrect intersection lines
		for (size_t i=0; i < wall_coef.size(); i++){			
			std::vector<size_t> index_of_plane_intersection_aux;		
			for (size_t j=0; j < wall_coef[i].index_of_plane_intersection.size(); j++){
				if (wall_coef[i].index_of_plane_intersection[j] < index)
					index_of_plane_intersection_aux.push_back(wall_coef[i].index_of_plane_intersection[j]);
					
				else if (wall_coef[i].index_of_plane_intersection[j] > index)
					index_of_plane_intersection_aux.push_back(wall_coef[i].index_of_plane_intersection[j]-1);
					
				else if (wall_coef[i].index_of_plane_intersection[j] == index)
					extract_line_index(wall_coef[i].index_of_line_intersection[j]);
				}
			wall_coef[i].index_of_plane_intersection = index_of_plane_intersection_aux;
		}
		
		
		//find points tha belong to this plane
		std::vector<size_t> points_to_delete;
		for (size_t i=0; i < important_points.size(); i++)
			for (size_t j=0; j < important_points[i].index_of_plane_belonging.size(); j++)
				if (important_points[i].index_of_plane_belonging[j] == index)
					points_to_delete.push_back(i);
		
		points_to_delete = vector_organize_descending(points_to_delete);
		
		
		for (size_t i=0; i < points_to_delete.size(); i++)
			important_points = extract_point_vectorpoint(important_points, points_to_delete[i]);
		
		
		//change points references and extract points
		for (size_t i=0; i < important_points.size(); i++){
			std::vector<size_t> index_of_plane_belonging_aux;
			for (size_t j=0; j < important_points[i].index_of_plane_belonging.size(); j++){
				if (important_points[i].index_of_plane_belonging[j] < index)
					index_of_plane_belonging_aux.push_back(important_points[i].index_of_plane_belonging[j]);
				
				else if (important_points[i].index_of_plane_belonging[j] > index)
					index_of_plane_belonging_aux.push_back(important_points[i].index_of_plane_belonging[j]-1);
			}
			important_points[i].index_of_plane_belonging = index_of_plane_belonging_aux;
		}
		
		
		return;
		
	}
	
	
	// ANGLE_BETWEEN_TWO_VECTORS ////////////////////////////////////////////////////////////////////////////////////////////////////
	float room_inside_detect::angle_between_two_vetors(pcl::PointXYZ v1, pcl::PointXYZ v2){
		
		double module_A,module_B,escalar_AB;
		double A,B,C;
		double cos_alfa,angle_cos;
		float angle;
	
		
		module_A = v1.x * v1.x;
		module_A += v1.y * v1.y;
		module_A += v1.z * v1.z;
		module_A = sqrt(module_A);              
			
		
		module_B = v2.x * v2.x;
		module_B += v2.y * v2.y;
		module_B += v2.z * v2.z;
		module_B = sqrt(module_B);
		
		
		A =  v1.y * v2.z; 
		A -= v2.y * v1.z;
		
		B =  v1.z * v2.x; 
		B -= v2.z * v1.x;
		
		C =  v1.x * v2.y; 
		C -= v2.x * v1.y;
		 			
		
		escalar_AB  = v1.x * v2.x;
		escalar_AB += v1.y * v2.y;
		escalar_AB += v1.z * v2.z;
			

		
		
		cos_alfa = module_A * module_B;
		cos_alfa = escalar_AB / cos_alfa;
		
		


		
		angle = acos(cos_alfa);
		
		
	
		return (angle);
	}
	
	
	// FULL_ANGLE_BETWEEN_TWO_VECTORS //////////////////////////////////////////////////////////////////////////////////////
	float room_inside_detect::full_angle_between_two_vetors(pcl::PointXYZ v1, pcl::PointXYZ v2, size_t plane_index){
		
		double module_A,module_B,module_AB,escalar_AB;
		double A,B,C;
		double cos_alfa,angle_cos, sin_alfa, angle_sin;
		float angle;
		float sin_sig;	
		
		
		module_A = v1.x * v1.x;
		module_A += v1.y * v1.y;
		module_A += v1.z * v1.z;
		module_A = sqrt(module_A);              
			
		
		module_B = v2.x * v2.x;
		module_B += v2.y * v2.y;
		module_B += v2.z * v2.z;
		module_B = sqrt(module_B);
		
		
		module_AB  = A*A;
		module_AB += B*B; 
		module_AB += C*C;
		module_AB = sqrt(module_AB);
		
		
		
		A =  v1.y * v2.z; 
		A -= v2.y * v1.z;
		
		B =  v1.z * v2.x; 
		B -= v2.z * v1.x;
		
		C =  v1.x * v2.y; 
		C -= v2.x * v1.y;
		 			
		
		escalar_AB  = v1.x * v2.x;
		escalar_AB += v1.y * v2.y;
		escalar_AB += v1.z * v2.z;
			
		
		
		sin_sig  = A * wall_coef[plane_index].a;
		sin_sig -= B * wall_coef[plane_index].b;
		sin_sig += C * wall_coef[plane_index].c;
		
		
		
		cos_alfa = module_A * module_B;
		cos_alfa = escalar_AB / cos_alfa;
		
		
		sin_alfa = module_A * module_B;
		sin_alfa = module_AB / sin_alfa;


		
		angle = acos(cos_alfa);
		
		
		if  (sin_sig < 0)
			angle = (3.141618 * 2) - angle;
		
	
		return (angle);
	}
	
	
	// GET_SWEEP_ANGLE ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	float room_inside_detect::get_sweep_angle(pcl::PointXYZ p1, plane p){
		pcl::PointXYZ v1,v2;
		
		v1.x = p1.x - p.sensor.x;
		v1.y = p1.y - p.sensor.y;
		v1.z = p1.z - p.sensor.z;
		
		v2.x = p.reference.x - p.sensor.x;
		v2.y = p.reference.y - p.sensor.y;
		v2.z = p.reference.z - p.sensor.z;
		
		
		double module_A,module_B,module_AB,escalar_AB;
		double A,B,C;
		double sin_alfa,cos_alfa,angle_cos,angle_sin;
		float angle;
		float sin_sig;
		
		module_A = v1.x * v1.x;
		module_A += v1.y * v1.y;
		module_A += v1.z * v1.z;
		module_A = sqrt(module_A);              
			
		
		module_B = v2.x * v2.x;
		module_B += v2.y * v2.y;
		module_B += v2.z * v2.z;
		module_B = sqrt(module_B);
		
		
		A =  v1.y * v2.z; 
		A -= v2.y * v1.z;
		
		B =  v1.z * v2.x; 
		B -= v2.z * v1.x;
		
		C =  v1.x * v2.y; 
		C -= v2.x * v1.y;
		 			
		module_AB  = A*A;
		module_AB += B*B; 
		module_AB += C*C;
		module_AB = sqrt(module_AB);
		
		
		escalar_AB  = v1.x * v2.x;
		escalar_AB += v1.y * v2.y;
		escalar_AB += v1.z * v2.z;
			
		
		sin_alfa = module_A * module_B;
		sin_alfa = module_AB / sin_alfa;
		
		
		cos_alfa = module_A * module_B;
		cos_alfa = escalar_AB / cos_alfa;
		
		
		sin_sig  = A * p.a;
		sin_sig -= B * p.b;
		sin_sig += C * p.c;
		

		
		angle = acos(cos_alfa);
		
		
		if  (sin_sig < 0)
			angle = (3.141618 * 2) - angle;
		
		
		
		return(angle);
	}
	 
	
	// SIMPLIFY STRAIGHT LINE ////////////////////////////////////////////////////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZ>::Ptr room_inside_detect::simplify_straight_line (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
		
		int jmax,
			imax;
		
		double x,
				y,
				z,
				module, 
				module_max=0;
			
	
	
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > points (new pcl::PointCloud<pcl::PointXYZ> );
	
	
		for (int i=0; i < cloud_in->size () ; i++){
		
			for (int j=0; j < cloud_in->size () ; j++){
			
				x=cloud_in->points[i].x - cloud_in->points[j].x;
				y=cloud_in->points[i].y - cloud_in->points[j].y;
				z=cloud_in->points[i].z - cloud_in->points[j].z;
				module = x*x + y*y + z*z;
				if ( module > module_max ) {
					module_max = module;
					jmax=j;
					imax=i;
					}
				}
			}
		
		points->push_back(cloud_in->points[imax]);
		points->push_back(cloud_in->points[jmax]);
		
		return (points);
	}
	
	
	// SEARCH_BIGGER_ANGLE_IN_CONEXIONS ////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool room_inside_detect::search_bigger_angle_in_conexion(size_t index_in, std::vector<size_t> index_points_in, std::vector<float> angles_points_in, size_t *index_out, float *angle_out){
		
		bool pass_throw_cero(false);
		float angle_min; 
		
		*angle_out = angles_points_in[index_in];
		*index_out = index_in;
		
		
		
		for (size_t i=0; i < important_points[index_in].index_of_point_conected.size(); i++){
			for (size_t j=0; j < index_points_in.size(); j++)
				if (important_points[index_points_in[index_in]].index_of_point_conected[i] == index_points_in[j]){ //en esta linea esta el problema			
					if (angles_points_in[j] > *angle_out && angles_points_in[j] - *angle_out < 3.1416){
						*angle_out = angles_points_in[j];
						*index_out = j;
						}
					
					if (angles_points_in[j] - angles_points_in[index_in] > 3.1416 || angles_points_in[j] - angles_points_in[index_in] < -3.1416)
						pass_throw_cero = true;
				}
				}	
		
	
			
		return(pass_throw_cero);
		
	}
	
	
	// SEARCH_LOWER_ANGLE_FROM ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::search_lower_angle_from(float starting_angle, std::vector<float> angles_points_in, float *angle_out, float *max_angle, size_t *index_out){
		
		*max_angle = 0;
		
		for (size_t i=0; i < angles_points_in.size(); i++)
			if (*max_angle < angles_points_in[i]){
				*max_angle = angles_points_in[i];
				*index_out = i;
			}
		
		*angle_out = *max_angle;
		for (size_t i=0; i < angles_points_in.size(); i++)
			if (angles_points_in[i] < *angle_out && angles_points_in[i] > starting_angle){
				*angle_out = angles_points_in[i];
				*index_out = i;
			}
		
		
		return;
	}
	
	
	// CONCECT_TWO_POINTS /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool room_inside_detect::conect_two_points(size_t index_p1, size_t index_p2){
	
		//check the number of connections
		if (important_points[index_p1].index_of_point_conected.size() >= 3 && important_points[index_p2].index_of_point_conected.size() >= 3)
			return (false);
		
		//check planes on common
		size_t planes_on_common(0);
		for (size_t i=0; i < important_points[index_p1].index_of_plane_belonging.size(); i++)
			for (size_t j=0; j < important_points[index_p2].index_of_plane_belonging.size(); j++)
				if (important_points[index_p1].index_of_plane_belonging[i] == important_points[index_p2].index_of_plane_belonging[j])
					planes_on_common++;
		
		if (planes_on_common < 1 || planes_on_common > 2)
			return (false);
		
		
		//check if the point is allready conected
		for (size_t i=0; i < important_points[index_p1].index_of_point_conected.size(); i++)
			if (important_points[index_p1].index_of_point_conected[i] == index_p1)
				return(false);

		for (size_t i=0; i < important_points[index_p2].index_of_point_conected.size(); i++)
			if (important_points[index_p2].index_of_point_conected[i] == index_p1)
				return(false);
		
		
		//check the direcction of the conexion
		pcl::PointCloud<pcl::PointXYZ> directions;
		if (direction_adm < 0 ) direction_adm = 0.52; // 30��
		for (size_t i=0; i < important_points[index_p1].index_of_point_conected.size(); i++){
			pcl::PointXYZ dir;
			dir.x = important_points[important_points[index_p1].index_of_point_conected[i]].p.x - important_points[index_p1].p.x; 
			dir.y = important_points[important_points[index_p1].index_of_point_conected[i]].p.y - important_points[index_p1].p.y; 
			dir.z = important_points[important_points[index_p1].index_of_point_conected[i]].p.z - important_points[index_p1].p.z; 
			
			for (size_t j=0; j < directions.size(); j++)
				if (angle_between_two_vetors(dir, directions.points[j]) < direction_adm)
					return(false);
			
			directions.push_back(dir);
		}
		
		
		
		
		//check the maximun of plane combinations
		bool cond4(true);
					
		std::vector<size_t> conexions_planes_A;
		std::vector<size_t> conexions_planes_B;
		std::vector<size_t> principal_planes;
					
					
		for (size_t j=0; j < important_points[index_p1].index_of_plane_belonging.size(); j++)	
			principal_planes.push_back(important_points[index_p1].index_of_plane_belonging[j]);					
					
		for (size_t j=0; j < important_points[index_p2].index_of_plane_belonging.size(); j++)	
			principal_planes.push_back(important_points[index_p2].index_of_plane_belonging[j]);					
		
											
		for (size_t j=0; j < important_points[index_p1].index_of_point_conected.size(); j++)
			for (size_t k=0; k < important_points[important_points[index_p1].index_of_point_conected[j]].index_of_plane_belonging.size(); k++)				 
				conexions_planes_A.push_back(important_points[important_points[index_p1].index_of_point_conected[j]].index_of_plane_belonging[k]);
			
					
		for (size_t j=0; j < important_points[index_p2].index_of_point_conected.size(); j++)
			for (size_t k=0; k < important_points[important_points[index_p2].index_of_point_conected[j]].index_of_plane_belonging.size(); k++)
				conexions_planes_B.push_back(important_points[important_points[index_p2].index_of_point_conected[j]].index_of_plane_belonging[k]);
			
						
					
		std::vector <size_t> count_A;
		std::vector <size_t> count_B;
			
		for (size_t j=0; j < wall_coef.size(); j++){
						
			count_A.push_back(0);
			count_B.push_back(0);
						
			for (size_t k=0; k < principal_planes.size(); k++){
				if (j == principal_planes[k]){
					count_A[j]++;
					count_B[j]++;
				}		
			}				
		}
							
					
		for (size_t j=0; j < wall_coef.size(); j++){
						
			for (size_t k=0; k < conexions_planes_A.size(); k++){
				if (j == conexions_planes_A[k]){
					count_A[j]++;
				}		
			}
					
		}
					
				
		for (size_t j=0; j < wall_coef.size(); j++){
						
			for (size_t k=0; k < conexions_planes_B.size(); k++){
				if (j == conexions_planes_B[k]){
					count_B[j]++;
				}		
			}
					
		}
					
					
		for (size_t j=0; j < count_A.size(); j++){
			size_t max_planes(3);
				if (wall_coef[j].special) max_planes++;
							
				//if (count_A[j] > max_planes) return(false);
		}
					
					
		for (size_t j=0; j < count_B.size(); j++){
			size_t max_planes(3);
			if (wall_coef[j].special) max_planes++;
						
			//if (count_B[j] > max_planes) return(false);
		}
		
		
		
		
		//store de new conexion
		important_points[index_p1].index_of_point_conected.push_back(index_p2);
		important_points[index_p2].index_of_point_conected.push_back(index_p1);
		
	
		return(true);
		
	}
	
	
	// CHECK_POLYGON ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool room_inside_detect::check_polygon(std::vector<size_t> index_points_in, size_t plane_index){
	}
	
	
	// ANGLE_SWEPT_FROM_CENTER_POINT ///////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::vector <float> room_inside_detect::angle_swept_from_center_point(std::vector<size_t> index_points, size_t plane_index){
		
		std::vector <float> angles;
		
		if (index_points.size() == 0)
			return (angles);
		
		
		
		
		//get center
		pcl::PointXYZ center;
		for (size_t i=0; i < index_points.size(); i++){
			center.x += important_points[index_points[i]].p.x;
			center.y += important_points[index_points[i]].p.y;
			center.z += important_points[index_points[i]].p.z;
		}
			
		center.x /= index_points.size();
		center.y /= index_points.size();
		center.z /= index_points.size();
		
		
		//get direction of the vectors
		pcl::PointCloud<pcl::PointXYZ> directions;
		
		for (size_t i=0; i < index_points.size(); i++){
			pcl::PointXYZ dir;
			dir.x =  important_points[index_points[i]].p.x - center.x;
			dir.y =  important_points[index_points[i]].p.y - center.y;
			dir.z =  important_points[index_points[i]].p.z - center.z;
			directions.push_back(dir);
		}
		
		//calculate angles the reference will be the first
		for (size_t i=0; i < directions.size(); i++)
			angles.push_back(full_angle_between_two_vetors(directions.points[i], directions.points[0],plane_index));
				
		
		
		return(angles);
		
	}
	
	
	// GET_PLANES_INTERSECTION ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	line room_inside_detect::get_planes_intersection(size_t index_plane_1,size_t index_plane_2){
		
		line inter;
		
		// For avoid planes equal
		double A,B,C,D,F;
			
		A = wall_coef[index_plane_1].a - wall_coef[index_plane_2].a;
		B = wall_coef[index_plane_1].b - wall_coef[index_plane_2].b;
		C = wall_coef[index_plane_1].c - wall_coef[index_plane_2].c;
		D = wall_coef[index_plane_1].d - wall_coef[index_plane_2].d;
		F = abs(A*100) + abs (B*100) + abs (C*100) + abs(D*100);
		
		
		if (avoid_ecual_planes < 0) avoid_ecual_planes = 30;
		
		if (F > avoid_ecual_planes){
		
			// Objects for calculate the coefficients of the intersections 
			Eigen::Matrix2f system_matrix; 
			Eigen::Vector2f system_vector, system_solution; 
			Eigen::LDLT<Eigen::Matrix2f> solver; 
		
		
			wall_coef[index_plane_1].index_of_plane_intersection.push_back(index_plane_2);
			wall_coef[index_plane_1].points_in_intersection.push_back(0);
			wall_coef[index_plane_2].index_of_plane_intersection.push_back(index_plane_1);
			wall_coef[index_plane_2].points_in_intersection.push_back(0);

					
					
			//calculate the equation of the intersection	
			system_matrix (0,0) = wall_coef[index_plane_1].b; 
			system_matrix (0,1) = wall_coef[index_plane_1].c; 
			system_matrix (1,0) = wall_coef[index_plane_2].b; 
			system_matrix (1,1) = wall_coef[index_plane_2].c; 
			system_vector (0) = - wall_coef[index_plane_1].d; 
			system_vector (1) = - wall_coef[index_plane_2].d; 
					
	
			//solution of the system	
			system_solution = system_matrix.inverse() * system_vector;
					
					
			
			double module;
					
			
			inter.x=0; 
			inter.y = system_solution (0);
			inter.z = system_solution (1);
			
			

			//calculate the direction of the line intersection
			inter.vx  = (wall_coef[index_plane_2].b * wall_coef[index_plane_1].c); 
			inter.vx -= (wall_coef[index_plane_2].c * wall_coef[index_plane_1].b);
		 		
			inter.vy  = (wall_coef[index_plane_2].c * wall_coef[index_plane_1].a);  
			inter.vy -= (wall_coef[index_plane_2].a * wall_coef[index_plane_1].c);
	
			inter.vz  = (wall_coef[index_plane_2].a * wall_coef[index_plane_1].b);     
			inter.vz -= (wall_coef[index_plane_2].b * wall_coef[index_plane_1].a);
					
						
			module  = inter.vx * inter.vx;
			module += inter.vy * inter.vy;
			module += inter.vz * inter.vz;
			module = sqrt (module);
			
			inter.vx = inter.vx / module;
			inter.vy = inter.vy / module;
			inter.vz = inter.vz / module;	

			inter.index_of_plane_a = index_plane_1;
			inter.index_of_plane_b = index_plane_2;
			
			//Store the line coefficients
			wall_coef[index_plane_1].index_of_line_intersection.push_back(line_coef.size()-1);
			wall_coef[index_plane_2].index_of_line_intersection.push_back(line_coef.size()-1);
		
		}		
		
		
		return(inter);
	}
	
	
	//GET_ANGLE_LASER_SWEEP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	float room_inside_detect::get_angle_laser_sweep(pcl::PointXYZ p1, pcl::PointXYZ reference){
		double module_A,module_B,module_AB,escalar_AB;
		double A,B,C;
		double sin_alfa,cos_alfa,angle_cos,angle_sin;
		float angle;
		float sin_sig;
		
		module_A = p1.x * p1.x;
		module_A += p1.y * p1.y;
		module_A += p1.z * p1.z;
		module_A = sqrt(module_A);              
			
		
		module_B = reference.x * reference.x;
		module_B += reference.y * reference.y;
		module_B += reference.z * reference.z;
		module_B = sqrt(module_B);
		
		
		A =  p1.y * reference.z; 
		A -= reference.y * p1.z;
		
		B =  p1.z * reference.x; 
		B -= reference.z * p1.x;
		
		C =  p1.x * reference.y; 
		C -= reference.x * p1.y;
		 			
		module_AB  = A*A;
		module_AB += B*B; 
		module_AB += C*C;
		module_AB = sqrt(module_AB);
		
		
		escalar_AB  = p1.x * reference.x;
		escalar_AB += p1.y * reference.y;
		escalar_AB += p1.z * reference.z;
			
		
		sin_alfa = module_A * module_B;
		sin_alfa = module_AB / sin_alfa;
		
		
		cos_alfa = module_A * module_B;
		cos_alfa = escalar_AB / cos_alfa;
		
		
		sin_sig = C * 1;
		

		
		angle = acos(cos_alfa);
		
		
		if  (sin_sig < 0)
			angle = (3.141618 * 2) - angle;
		
		
		
		return(angle);

		
		
	}
	
	
	// FIND_POLYGON_CONEXIONS ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::find_polygons_conexions(size_t point_index,size_t plane_index, size_t max_iterations){
		polygon_vertices polygon;
		
		polygon.points_index.push_back(point_index);
		
		
		bool point_found(true);
		size_t count(0);
		
		
		while(point_found && count < max_iterations){
			
			count++;
			point_found = false;
			std::vector <size_t> temp_store;
			
			for (size_t i=0; i < polygon.points_index.size(); i++)	
				for (size_t j=0; j < important_points[polygon.points_index[i]].index_of_point_conected.size(); j++)
					for (size_t k=0; k < important_points[important_points[polygon.points_index[i]].index_of_point_conected[j]].index_of_plane_belonging.size(); k++)
						if (important_points[important_points[polygon.points_index[i]].index_of_point_conected[j]].index_of_plane_belonging[k] == plane_index)
							temp_store.push_back(important_points[polygon.points_index[i]].index_of_point_conected[j]);
		
			for (size_t i=0; i < temp_store.size(); i++){
				bool store(true);
				for (size_t j=0; j < polygon.points_index.size(); j++)
						if (temp_store[i] == polygon.points_index[j])
							store = false;
				
				if(store){
					polygon.points_index.push_back(temp_store[i]);
					point_found = true;
				}
			
			}
			
			
			
		
		}
		
		
		pcl::PointXYZ center;
		center.x = 0;
		center.y = 0;
		center.z = 0;
		
		
		for (size_t i=0; i < polygon.points_index.size(); i++){
			center.x += important_points[polygon.points_index[i]].p.x;
			center.y += important_points[polygon.points_index[i]].p.y;
			center.z += important_points[polygon.points_index[i]].p.z;
		}
		
		
		center.x = center.x / polygon.points_index.size();
		center.y = center.y / polygon.points_index.size();
		center.z = center.z / polygon.points_index.size();
			
		
		
		if (polygon.points_index.size() > 2){
		
			//Organize points
			std::vector<size_t> disorganized_list;
			std::vector<size_t> organized_list;
			
			
			disorganized_list = polygon.points_index;
			organized_list.push_back(polygon.points_index[0]);
			disorganized_list = extract_i_data_from_list(0, disorganized_list);
			
			while (disorganized_list.size() > 0){
				bool found(false);
				for (size_t i=0; i < important_points[organized_list[organized_list.size()-1]].index_of_point_conected.size(); i++)
					for (size_t j=0; j < disorganized_list.size(); j++)
						if (disorganized_list[j] == important_points[organized_list[organized_list.size()-1]].index_of_point_conected[i] && !found){
							found = true;
							organized_list.push_back(disorganized_list[j]);
							disorganized_list = extract_i_data_from_list(j, disorganized_list);
						}
			}
						
					
			polygon.points_index = organized_list;
		
		
		
		
			polygon.plane_index = plane_index;
			polygon.center = center;
			polygon_planes.push_back(polygon);
		}
		
		return;
		
	}
	
	
	// EXTRACT_I_DATA_FROM_LIST ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::vector<size_t> room_inside_detect::extract_i_data_from_list(size_t i, std::vector<size_t> list){
		 std::vector<size_t> list_out;
		 
		 for (size_t j=0; j < list.size(); j++)
			if (j != i)
				list_out.push_back(list[j]);
		
		return(list_out);
		
	}
	
	
	// OBTAINING_PLANE_COEFICIENTS ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	plane room_inside_detect::obtaining_plane_coeficients(size_t p1, size_t p2, size_t p3, size_t p4){
		
		plane plane_out;
		
		//get vectors
		pcl::PointXYZ v1,v2;
		
		v1.x = important_points[p1].p.x - important_points[p2].p.x; 
		v1.y = important_points[p1].p.y - important_points[p2].p.y;
		v1.z = important_points[p1].p.z - important_points[p2].p.z; 
		 
		v2.x = important_points[p1].p.x - important_points[p3].p.x;
		v2.y = important_points[p1].p.y - important_points[p3].p.y;
		v2.z = important_points[p1].p.z - important_points[p3].p.z;
		
		
		
		double A,B,C,D;
		
		A =  v1.y * v2.z; 
		A -= v2.y * v1.z;
		
		B =  v1.z * v2.x; 
		B -= v2.z * v1.x;
		
		C =  v1.x * v2.y; 
		C -= v2.x * v1.y;
		
		D = important_points[p4].p.x*A;
		D = important_points[p4].p.y*B;
		D = important_points[p4].p.z*C;
		D = -D;
		
		plane_out.a = A; 
		plane_out.b = B; 
		plane_out.c = C;
		plane_out.d = D;
		 
		
		return(plane_out);
		
	}
	
	
	// MAKE_POINTMESH_TRIANGLE //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZ>::Ptr room_inside_detect::make_pointmesh_triangle(pcl::PointXYZ base1, pcl::PointXYZ base2, pcl::PointXYZ top,float interval){
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr side1 (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr side2 (new pcl::PointCloud<pcl::PointXYZ>);
		
		/*
		side1->push_back(base1);
		side1->push_back(top);		
		side2->push_back(base2);		
		side2->push_back(top);		
		
		
		if (squaredEuclideanDistance (base1,top) < squaredEuclideanDistance (base2, top)){
			side1 = divide_line(side1, interval);
			float interval2;
			interval2 = squaredEuclideanDistance (base2,top) / side1->size();
			side2 = divide_line(side2, interval2);
		}
		
		else{
			side2 = divide_line(side2, interval);
			float interval2;
			interval2 = squaredEuclideanDistance (base1,top) / side2->size();
			side1 = divide_line(side1, interval2);
		}
		
		
		size_t size_min;
		if (side1->size() < side2->size()) size_min = side1->size();
		else size_min = side2->size();
		
		
		for	(size_t i=0; i < size_min; i++){
			pcl::PointCloud<pcl::PointXYZ>::Ptr aux (new pcl::PointCloud<pcl::PointXYZ>);
			aux->push_back(side1->points[i]);
			aux->push_back(side2->points[i]);
			*cloud_out += *divide_line(aux, interval);
		}
		
		*/
		
		
		side1->push_back(base1);
		side1->push_back(base2);		
			
		side1 = divide_line(side1, interval);
		

		for	(size_t i=0; i < side1->size(); i++){
			pcl::PointCloud<pcl::PointXYZ>::Ptr aux (new pcl::PointCloud<pcl::PointXYZ>);
			aux->push_back(side1->points[i]);
			aux->push_back(top);
			*cloud_out += *divide_line(aux, interval);
		}
		


		
		return(cloud_out);
	}
	
	
///PRIVATE (KEY FUNCTIONS)

	// PROCESS_CLOUD /////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::process_cloud(){
	
		if (!cloud_original.empty()){
			reset_data();
			
			all_process_time.tic();
		
			sub_process_time.tic();
			density_filter ();
			vector_process_time.push_back(sub_process_time.toc());
			
		
			sub_process_time.tic();
			find_wall_coeficients ();
			vector_process_time.push_back(sub_process_time.toc());

			
			sub_process_time.tic();
			identify_type_of_plane();
			vector_process_time.push_back(sub_process_time.toc());
		
			sub_process_time.tic();
			get_intersections_for_planes_type_A();
			vector_process_time.push_back(sub_process_time.toc());
			
			sub_process_time.tic();
			proyect_planes_type_B();
			vector_process_time.push_back(sub_process_time.toc());
		
			sub_process_time.tic();
			delete_incorrect_planes();
			vector_process_time.push_back(sub_process_time.toc());
		
			sub_process_time.tic();
			insert_corners_points();
			vector_process_time.push_back(sub_process_time.toc());
		
			sub_process_time.tic();
			make_polygons_in_planes_type_A();
			vector_process_time.push_back(sub_process_time.toc());
		
			sub_process_time.tic();
			make_polygons_in_planes_type_B();
			vector_process_time.push_back(sub_process_time.toc());
		
			sub_process_time.tic();
			find_polygons();
			vector_process_time.push_back(sub_process_time.toc());
		
			vector_process_time.push_back(all_process_time.toc());
			
			process_done = true;
		}
		
	
	return;
	}

	
	// DENSITY_FILTER ////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::density_filter (){
			
			
			// Objets & variables
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux (new pcl::PointCloud<pcl::PointXYZ>);
			*cloud_aux = cloud_original;
			if (grid < 0) grid = 0.1;

			// Filter
			voxelgrid_filter.setInputCloud (cloud_aux);
			voxelgrid_filter.setLeafSize (grid,grid,grid);
			voxelgrid_filter.filter (cloud_filtered);
			
			//Display visualization
			*cloud_aux = cloud_filtered; 
			if (display && display_density_filter) viewer->delete_all(1);
			if (display && display_density_filter) viewer->load_xyz(cloud_aux,1,false);
			
			
			
			if (display && display_density_filter) std::cout << "Points before filter: " << cloud_original.size() << std::endl;
			if (display && display_density_filter) std::cout << "Points after filter: " << cloud_filtered.size() << std::endl;
			if (display && display_density_filter) std::cout << "Press enter to continue...." << std::endl;
			if (display && display_density_filter && wait_for_enter) getchar();
			if (display && display_density_filter && !wait_for_enter) sleep(wait_time);
			

				
		return;		
	}
	
	
	// FIND_WALL_COEFICIENTES /////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::find_wall_coeficients (){

		
		
		//declaracion de variables y objetos
		std::vector <double> coef; 
		
			
		//coeficientes
		pcl::ModelCoefficients::Ptr coeficientes (new pcl::ModelCoefficients ());
	
	
		//indices
		pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
				
		
		//nubes
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_left (new pcl::PointCloud<pcl::PointXYZ> );
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > found_plane (new pcl::PointCloud<pcl::PointXYZ> );
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > found_plane_filtered (new pcl::PointCloud<pcl::PointXYZ> );


		
	
		//inicializacion
		*cloud_left = cloud_filtered;
		int original_size = cloud_left->size ();
		
		if (display && display_find_wall_coeficients) viewer->delete_all(1);
		
		
	
		// Rutina principal
		if (perc_of_search < 0) perc_of_search = 0.15;
		if (Iterations < 0) Iterations = 1000; 
		if (radio_coef < 0) radio_coef = 0.05;
	
		while (cloud_left->size () >= perc_of_search * original_size){
	
	
			// Localizacion del plano 
			SAC_Segmentation.setOptimizeCoefficients (true);
			SAC_Segmentation.setModelType (pcl::SACMODEL_PLANE);
			SAC_Segmentation.setMethodType (pcl::SAC_RANSAC);
			SAC_Segmentation.setMaxIterations (Iterations); 
			SAC_Segmentation.setDistanceThreshold (radio_coef);      
			SAC_Segmentation.setInputCloud (cloud_left);
			SAC_Segmentation.segment (*indices, *coeficientes);
		
		
			// Extraer plano econtrado
			Extract_Indices.setInputCloud (cloud_left);
			Extract_Indices.setIndices (indices);
			Extract_Indices.setNegative (false);
			Extract_Indices.filter (*found_plane);
		
		
			// Extraer el plano de la nube
			Extract_Indices.setInputCloud (cloud_left);
			Extract_Indices.setIndices (indices);
			Extract_Indices.setNegative (true);
			Extract_Indices.filter (*cloud_left);
		
		
			// Filtrar posibles planos erroneos		
			if (lim_segmentos < 0) lim_segmentos = 0.5;
		
			Concave_Hull.setInputCloud (found_plane);
			Concave_Hull.setDimension(2);
			Concave_Hull.setAlpha (lim_segmentos);
			Concave_Hull.reconstruct(*found_plane_filtered);
		
		
		
		
		
			// Almacenar datos
			if (p_admissible_coef < 0) p_admissible_coef = 0.25;
			if (found_plane_filtered->points.size () <= p_admissible_coef * found_plane->points.size ()){
				
				if (display && display_find_wall_coeficients) viewer->load_xyz(found_plane,1,true);
				
				//Get sensor proyected
				pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_proyected (new pcl::PointCloud<pcl::PointXYZ> );
				sensor_proyected->push_back(sensor);
				sensor_proyected->push_back(reference);
				
				
				Project_Inliers.setModelType (pcl::SACMODEL_PLANE);
				Project_Inliers.setModelCoefficients (coeficientes);
				
				Project_Inliers.setInputCloud (sensor_proyected);
				Project_Inliers.filter (*sensor_proyected);
				
				
				plane temp_plane_data;
				temp_plane_data.a = coeficientes->values[0];
				temp_plane_data.b = coeficientes->values[1];
				temp_plane_data.c = coeficientes->values[2];
				temp_plane_data.d = coeficientes->values[3];
				temp_plane_data.n_points = found_plane->points.size (); 
				temp_plane_data.special = false;
				temp_plane_data.layer = 0;
				temp_plane_data.sensor = sensor_proyected->points[0];
				temp_plane_data.reference = sensor_proyected->points[1];
				
				wall_coef.push_back (temp_plane_data);
			}	
		}
		
		
		if (display && display_find_wall_coeficients) std::cout << "Wall coeficients found" << std::endl;
		if (display && display_find_wall_coeficients) std::cout << "Press enter to continue...." << std::endl;
		if (display && display_find_wall_coeficients && wait_for_enter) getchar();
		if (display && display_find_wall_coeficients && !wait_for_enter) sleep(wait_time);
		return;
		
	}
	

	// IDENTIFY_TYPE_OF_PLANE ///////////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::identify_type_of_plane(){
		for (size_t i=0; i < wall_coef.size(); i++){
			
			float pro_v, pro_h;
			wall_coef[i].type_B = false;
			wall_coef[i].type_A_U = false;
			wall_coef[i].type_A_L = false;
			
			
			
			pro_v = wall_coef[i].c;
			
			pro_h = wall_coef[i].a * wall_coef[i].a; 
			pro_h += wall_coef[i].b * wall_coef[i].b; 
			pro_h = sqrt(pro_h);
			
			
			if (pro_h > pro_v)
				wall_coef[i].type_B = true;

			
			else {
				if (wall_coef[i].d / wall_coef[i].c < 0 )
					wall_coef[i].type_A_U = true;
				else 
					wall_coef[i].type_A_L = true;
				
			} 
		}
		
		if (display && display_identify_type_of_plane) std::cout << "Planes identified" << std::endl;
		if (display && display_identify_type_of_plane) std::cout << "Press enter to continue...." << std::endl;
		if (display && display_identify_type_of_plane && wait_for_enter) getchar();
		if (display && display_identify_type_of_plane && !wait_for_enter) sleep(wait_time);
		
		
		return;
	}
	
	
	// GET_INTERSECTIONS_FOR_PLANES_TYPE_A //////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::get_intersections_for_planes_type_A(){
		
		std::vector<size_t> index_type_A_U;
		std::vector<size_t> index_type_A_L;
		
		//get the intersections for planes type A
		for (size_t i=0; i < wall_coef.size(); i++){
			if (wall_coef[i].type_A_U)
				index_type_A_U.push_back(i);
			
			else if (wall_coef[i].type_A_L)
				index_type_A_L.push_back(i);
		}
		
		
		//for now only works this combination, the common one
		if (index_type_A_U.size() > 1) return;
		if (index_type_A_L.size() > 1) return;
		
		
		//get the intersection for each
		for (size_t i=0; i < wall_coef.size(); i++)
			if (wall_coef[i].type_B){
				for (size_t j=0; j < index_type_A_U.size(); j++)
					line_coef.push_back(get_planes_intersection(i, index_type_A_U[j]));
				for (size_t j=0; j < index_type_A_L.size(); j++)
					line_coef.push_back(get_planes_intersection(i, index_type_A_L[j]));
			}
			
		
		if (display && display_find_intersections_coeficients) std::cout << "Intersections between planes obtained" << std::endl;
		if (display && display_find_intersections_coeficients) std::cout << "Press enter to continue...." << std::endl;
		if (display && display_find_intersections_coeficients && wait_for_enter) getchar();
		if (display && display_find_intersections_coeficients && !wait_for_enter) sleep(wait_time);
		
		
		return;
	}


	// PROTECT_PLANES_TYPE_B ///////////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::proyect_planes_type_B(){
		
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud (new pcl::PointCloud<pcl::PointXYZ> );
		*cloud = cloud_filtered;
		
		if (display && display_proyect_planes_type_B) viewer->delete_all(1);
		
		
		for (size_t i = 0; i < line_coef.size(); i++ ){

			
				size_t plane_in_work;
				// extracting the planes
				pcl::PointCloud<pcl::PointXYZ>::Ptr  aux1_cloud (new pcl::PointCloud<pcl::PointXYZ> );
				pcl::PointCloud<pcl::PointXYZ>::Ptr  aux2_cloud (new pcl::PointCloud<pcl::PointXYZ> );

				pcl::PointCloud<pcl::PointXYZ>::Ptr  aux11_cloud (new pcl::PointCloud<pcl::PointXYZ> );

				pcl::PointCloud<pcl::PointXYZ>::Ptr  proyected_plane (new pcl::PointCloud<pcl::PointXYZ> );
						
				
				
				//extract the planes
				if (wall_coef[line_coef[i].index_of_plane_a].type_B){
					plane_in_work = line_coef[i].index_of_plane_a;
					aux1_cloud = extract_plane(cloud, grid / 2 ,false, wall_coef[line_coef[i].index_of_plane_a]);
					aux2_cloud = extract_plane(cloud, grid / 2 ,false, wall_coef[line_coef[i].index_of_plane_b]);
				}
				
					
				else {
					plane_in_work = line_coef[i].index_of_plane_b;
					aux1_cloud = extract_plane(cloud, grid / 2 ,false, wall_coef[line_coef[i].index_of_plane_b]);
					aux2_cloud = extract_plane(cloud, grid / 2 ,false, wall_coef[line_coef[i].index_of_plane_a]);
				}
					
				
				
				//extract the the rest of the planes	
				for (size_t j =0; j < wall_coef.size(); j++ )
					if (j != plane_in_work)
						aux1_cloud = extract_plane(aux1_cloud, grid / 2 ,true, wall_coef[j]);
				
				
				
				aux11_cloud = aux1_cloud;

				

				
				
				// display in viewer
				if (display && display_proyect_planes_type_B) viewer->delete_all(1);
				if (display && display_proyect_planes_type_B) viewer->load_xyz(aux1_cloud,1,true);
				if (display && display_proyect_planes_type_B) viewer->load_xyz(aux2_cloud,1,true);
				
				
				// Load coefficients 
				pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
				
				coefficients->values.resize (6);
				coefficients->values[0] = line_coef[i].x;
				coefficients->values[1] = line_coef[i].y;
				coefficients->values[2] = line_coef[i].z;
				coefficients->values[3] = line_coef[i].vx;
				coefficients->values[4] = line_coef[i].vy;
				coefficients->values[5] = line_coef[i].vz;
				
				
				
				// Projecting all points of the plane found 1 on the line intersection	 						
				Project_Inliers.setModelType (pcl::SACMODEL_LINE);
				Project_Inliers.setModelCoefficients (coefficients);
				Project_Inliers.setInputCloud (aux1_cloud);
				Project_Inliers.filter (*proyected_plane);
				
				//if (display && display_proyect_planes_type_B) viewer->load_xyz(proyected_plane,1,true);
				
				
				//make the cells	
				pcl::PointCloud<pcl::PointXYZ>::Ptr cells (new pcl::PointCloud<pcl::PointXYZ>);
				
				aux1_cloud = find_farest_points(proyected_plane);
				cells = divide_line(aux1_cloud,grid);
				

				
				// Filter out noise in cloud to eliminate both projections			
				std::vector<size_t> proyections;
				
				
				pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
				kdtree.setInputCloud (proyected_plane);				
				
				for (size_t j=0; j< cells->size(); j++){
					std::vector< int > k_indices;
					std::vector< float > k_sqr_distances;
				
			
					kdtree.radiusSearch(cells->points[j],grid,k_indices,k_sqr_distances);
					proyections.push_back(k_indices.size());
					
					pcl::PointXYZ point;
					float x(0),y(0),z(0);
					int n_points(0);
					
					for (size_t k = 0; k < k_indices.size(); k++){
						
						x += proyected_plane->points[k_indices[k]].x;
						y += proyected_plane->points[k_indices[k]].y;
						z += proyected_plane->points[k_indices[k]].z;
						n_points++;
					}
					
					point.x = x / n_points;
					point.y = y / n_points;
					point.z = z / n_points;
					
					line_coef[i].cells.push_back(point);

		
				}
				
				
				//gets perceptil
				std::vector <size_t> organized_vector;
				organized_vector = vector_organize_descending(proyections);
				
				
				float perceptil;
				if (perceptil_n < 0) perceptil_n = 0.5;
				if (min_proyect < 0) min_proyect = 5;
				
				perceptil = organized_vector[organized_vector.size()*perceptil_n];
				if (min_proyect > perceptil) perceptil = min_proyect;
				
				if (display && display_proyect_planes_type_B) std::cout << "For line: " << i << " n: " << perceptil_n << " Perceptil: " << perceptil << std::endl;
				
				
				//filter de points 
				for (size_t j = 0; j < proyections.size(); j++){
					if (proyections[j] > perceptil)
						line_coef[i].proyections.push_back(true);
					else 
						line_coef[i].proyections.push_back(false);
				}
				
				
				
				//get proyection
				pcl::PointCloud<pcl::PointXYZ>::Ptr aux_proyected (new pcl::PointCloud<pcl::PointXYZ>);
				
				
				for (size_t j = 0; j < line_coef[i].cells.size(); j++)
					if (line_coef[i].proyections[j]) aux_proyected->push_back(line_coef[i].cells.points[j]);
							
				
				cloud_proyected += *aux_proyected;
				
				
				
				// display in viewer				
				//if (display && display_proyect_planes_type_B) viewer->load_xyz(aux_proyected,1,true);

				
				
				//get segments
				pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_segmets (new pcl::search::KdTree<pcl::PointXYZ>);
				
				if (tol_seg < 0) tol_seg = 1.0;
				if (size_min < 0) size_min = 0.04;
					
				tree_segmets->setInputCloud (aux_proyected);	
				std::vector<pcl::PointIndices> indices_ext;		
				Euclidean_Cluster_Extraction.setClusterTolerance (tol_seg);
				Euclidean_Cluster_Extraction.setMinClusterSize (size_min * aux_proyected->points.size ());
				Euclidean_Cluster_Extraction.setMaxClusterSize (aux_proyected->points.size ());
				Euclidean_Cluster_Extraction.setSearchMethod (tree_segmets);
				Euclidean_Cluster_Extraction.setInputCloud (aux_proyected);
				Euclidean_Cluster_Extraction.extract (indices_ext); 
				
				if (indices_ext.size() > 1) wall_coef[plane_in_work].special = true;
				
				// Extract segments found
				for (std::vector<pcl::PointIndices>::const_iterator it = indices_ext.begin (); it != indices_ext.end (); ++it){

					pcl::PointCloud<pcl::PointXYZ>::Ptr  segment_cloud (new pcl::PointCloud<pcl::PointXYZ> );								
					for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
						segment_cloud->points.push_back(aux_proyected->points[*pit]);
						}
        
					segment_cloud->width = segment_cloud->points.size ();
					segment_cloud->height = 1;
					segment_cloud->is_dense = true;
					
					
					// display in viewer				
					if (display && display_proyect_planes_type_B) viewer->load_xyz(segment_cloud,1,false);	
					
					
					// Reduce the segment
					segment_cloud = simplify_straight_line (segment_cloud);
					
	
						
					// store the segments
					point pt_0;
					point pt_1;
					
					pt_0.p = segment_cloud->points[0];
					pt_0.index_of_plane_belonging.push_back(line_coef[i].index_of_plane_a);
					pt_0.angle_swept.push_back(get_sweep_angle(pt_0.p,wall_coef[line_coef[i].index_of_plane_a]));
					pt_0.index_of_plane_belonging.push_back(line_coef[i].index_of_plane_b);
					pt_0.angle_swept.push_back(get_sweep_angle(pt_0.p,wall_coef[line_coef[i].index_of_plane_b]));
					pt_0.index_of_point_conected.push_back(important_points.size()+1);
					
					
					pt_1.p = segment_cloud->points[1];
					pt_1.index_of_plane_belonging.push_back(line_coef[i].index_of_plane_a);
					pt_1.angle_swept.push_back(get_sweep_angle(pt_1.p,wall_coef[line_coef[i].index_of_plane_a]));
					pt_1.index_of_plane_belonging.push_back(line_coef[i].index_of_plane_b);
					pt_1.angle_swept.push_back(get_sweep_angle(pt_1.p,wall_coef[line_coef[i].index_of_plane_b]));
					pt_1.index_of_point_conected.push_back(important_points.size());
					
					important_points.push_back(pt_0);
					important_points.push_back(pt_1);
				
				}
				
				if (display && display_proyect_planes_type_B && !wait_for_enter) sleep(wait_time);
				
				
		}
			
			
		
		if (display && display_proyect_planes_type_B) std::cout << "Proyected cloud created" << std::endl;
		if (display && display_proyect_planes_type_B) std::cout << "Press enter to continue...." << std::endl;
		if (display && display_proyect_planes_type_B && wait_for_enter) getchar();
		if (display && display_proyect_planes_type_B && !wait_for_enter) sleep(wait_time);
		
		return;	
		
		
		
	}
	
	
	// DETELE_INCORRECT_PROYECTIONS ///////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::delete_incorrect_planes(){
		
		std::vector<size_t> planes_to_eliminate;
		
		for (size_t i=0; i < wall_coef.size(); i++){
			
			
			//get index belonging to the plane i
			std::vector <segment> segm_d;
			for (size_t j=0; j < important_points.size(); j++){
			
			
				for (size_t k=0; k < important_points[j].index_of_plane_belonging.size(); k++){
					if (important_points[j].index_of_plane_belonging[k] == i){
						segment temp_segm;
						temp_segm.initial = false;
						temp_segm.index.push_back(j);
						temp_segm.angle_swept.push_back(important_points[j].angle_swept[k]);
						
						for (size_t l=0; l < important_points[j].index_of_point_conected.size(); l++){
							
							size_t target_index;
							target_index = important_points[j].index_of_point_conected[l];
							
							for (size_t m=0; m < important_points[target_index].index_of_plane_belonging.size(); m++){
								if (important_points[target_index].index_of_plane_belonging[m] == i){
								
									temp_segm.index.push_back(target_index);
									temp_segm.angle_swept.push_back(important_points[target_index].angle_swept[m]);
								}
							}
						}
						
						temp_segm.center.x = 0;
						temp_segm.center.y = 0;
						temp_segm.center.z = 0;
						
						size_t npoints(0);
						
						for (size_t l=0; l < temp_segm.index.size(); l++){
							temp_segm.center.x += important_points[temp_segm.index[l]].p.x;
							temp_segm.center.y += important_points[temp_segm.index[l]].p.y;
							temp_segm.center.z += important_points[temp_segm.index[l]].p.z;
							npoints++;
						} 
						
						temp_segm.center.x = temp_segm.center.x / npoints;
						temp_segm.center.y = temp_segm.center.y / npoints;
						temp_segm.center.z = temp_segm.center.z / npoints;
						
						segm_d.push_back(temp_segm);
					}
				}		
							
			}

			
			//eliminate clooned segmens
			std::vector <segment> segm;
			for (size_t j=0; j < segm_d.size(); j++){
				if (segm_d[j].angle_swept.front() < segm_d[j].angle_swept.back())
					segm.push_back(segm_d[j]);
			}
			
			
			//find initial segment
			for (size_t j=0; j < segm.size(); j++){
				
				if (segm[j].angle_swept.back() - segm[j].angle_swept.front() > 3.141592)
					segm[j].initial = true;
			}
			
			
			//detect incorrect wall proyections and store planes to eliminate
			for (size_t j=0; j < segm.size(); j++){
				if (!segm[j].initial)
					for (size_t k=0; k < segm.size(); k++)
						if (!segm[k].initial){
							bool cond1,cond2,cond3;
							cond1 = j != k;
							cond2 = segm[j].angle_swept.front() > segm[k].angle_swept.front();
							cond3 = segm[k].angle_swept.back() > segm[j].angle_swept.front();
							
							if (cond1 && cond2 && cond3){
								
								if (squaredEuclideanDistance (segm[j].center,wall_coef[i].sensor) > squaredEuclideanDistance (segm[k].center,wall_coef[i].sensor)){
									
									for (size_t l=0; l < segm[k].index.size(); l++){
										for (size_t m=0; m < important_points[segm[k].index[l]].index_of_plane_belonging.size(); m++){
											if (important_points[segm[k].index[l]].index_of_plane_belonging[m] != i){
												bool store(true);
												for (size_t n=0; n < planes_to_eliminate.size(); n++)
													if (planes_to_eliminate[n] == important_points[segm[k].index[l]].index_of_plane_belonging[m])
														store = false;
												if (store) 
													planes_to_eliminate.push_back(important_points[segm[k].index[l]].index_of_plane_belonging[m]);
											}
										}
									}
								}
								
								else {
									
									for (size_t l=0; l < segm[j].index.size(); l++){
										for (size_t m=0; m < important_points[segm[j].index[l]].index_of_plane_belonging.size(); m++){
											if (important_points[segm[j].index[l]].index_of_plane_belonging[m] != i){
												bool store(true);
												for (size_t n=0; n < planes_to_eliminate.size(); n++)
													if (planes_to_eliminate[n] == important_points[segm[j].index[l]].index_of_plane_belonging[m])
														store = false;
												if (store) 
													planes_to_eliminate.push_back(important_points[segm[j].index[l]].index_of_plane_belonging[m]);
											}
										}
									}
								}
							}
						}
					}
			
			
			//the segments that pass throw 0�� are not checked, I have no example cloud to improve this!!!!!	
			
		}
		
		//display info
		if (display && display_delete_incorrect_planes)
			for (size_t j=0; j < planes_to_eliminate.size(); j++)
				std::cout << "The plane " << planes_to_eliminate[j] << " has been eliminated" << std::endl;
		
		
		//extract incorrect planes and ponints 
		for (size_t j=0; j < planes_to_eliminate.size(); j++)
			extract_wall_index(planes_to_eliminate[j]);
			
		
		
		
		if (display && display_delete_incorrect_planes) std::cout << "Incorrect planes deleted " << std::endl;
		if (display && display_delete_incorrect_planes) std::cout << "Press enter to continue...." << std::endl;
		if (display && display_delete_incorrect_planes && wait_for_enter) getchar();
		if (display && display_delete_incorrect_planes && !wait_for_enter) sleep(wait_time);
		
		return;
	}
	
	
	// INSERT_CORNERS_POINTS ///////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::insert_corners_points(){
		std::vector<std::vector <size_t> > ec_data;
		
		for (size_t i=0; i < wall_coef.size(); i++){
			
			std::vector<size_t> ec_data_aux;
			ec_data_aux.push_back(i);
			ec_data_aux.push_back(i);
			ec_data_aux.push_back(i);
			for (size_t j=0; j < wall_coef[i].index_of_plane_intersection.size(); j++){
				
				ec_data_aux[1] = wall_coef[i].index_of_plane_intersection[j];
				
				
				for (size_t k=j+1; k < wall_coef[i].index_of_plane_intersection.size(); k++){
					ec_data_aux[2] = wall_coef[i].index_of_plane_intersection[k];
					ec_data.push_back(ec_data_aux);
				}
						
			}
		}
		
		//ordered planes
		for (size_t i=0; i < ec_data.size(); i++)
			ec_data[i] = vector_organize_descending(ec_data[i]); 
		
		
		//extract duplicated
		std::vector<std::vector <size_t> > ec_data2;
		for (size_t i=0; i < ec_data.size(); i++){

			bool ok(true);
			for (size_t j=0; j < ec_data2.size(); j++){
				bool cond1, cond2, cond3;
				cond1 = ec_data[i][0] == ec_data2[j][0]; 
				cond2 = ec_data[i][1] == ec_data2[j][1]; 
				cond3 = ec_data[i][2] == ec_data2[j][2]; 
				
				if (cond1  && cond2 && cond3) ok = false;
			}
			
			if (ok) ec_data2.push_back(ec_data[i]);
		}
				
	
	
		//extract no intersections planes
		std::vector<std::vector <size_t> > ec_data3;
		for (size_t i=0; i < ec_data2.size(); i++)
			if (check_plane_system(ec_data2[i][0], ec_data2[i][1], ec_data2[i][2]))
				ec_data3.push_back(ec_data2[i]);
		
		
				
		//count the number of points for intersection
		std::vector<std::vector <size_t> > plane_count;
		for (size_t i=0; i < wall_coef.size(); i++){
			for (size_t j=i+1; j < wall_coef.size(); j++){
				std::vector<size_t> count_ij;
				count_ij.push_back(i);
				count_ij.push_back(j);
				count_ij.push_back(0);
			
				for (size_t k=0; k < ec_data3.size(); k++){
						if (ec_data3[k][0] == j && ec_data3[k][1] == i) count_ij[2]++;
						else if (ec_data3[k][1] == j && ec_data3[k][2] == i) count_ij[2]++;
						else if (ec_data3[k][0] == j && ec_data3[k][2] == i) count_ij[2]++;
				}
				plane_count.push_back(count_ij);
			}
		}
		
		
		//Eliminate false corners
		/*
		for (size_t i=0; i < plane_count.size(); i++){
			if (plane_count[i][2] > 2){
				std::vector<std::vector <size_t> > for_extract;
				for (size_t k=0; k < ec_data3.size(); k++){
					bool save(false);
					if (ec_data3[k][0] == plane_count[i][1]  && ec_data3[k][1] == plane_count[i][0] ) save = true;
					else if (ec_data3[k][1] == plane_count[i][1]  && ec_data3[k][2] == plane_count[i][0] ) save = true;
					else if (ec_data3[k][0] == plane_count[i][1]  && ec_data3[k][2] == plane_count[i][0] ) save = true;
					
					if (save) for_extract.push_back(ec_data3[k]);		
				}
				
				std::vector<size_t> n_intersections;
				for (size_t k=0; k < for_extract.size(); k++){
					n_intersections.push_back(0);
					for (size_t l=0; l < wall_coef.size(); l++)
						if (l == for_extract[k][0] || l == for_extract[k][1] || l == for_extract[k][2]) 
							for (size_t m=0; m < wall_coef[l].index_of_plane_intersection.size(); m++)
								if (wall_coef[l].index_of_plane_intersection[m] == for_extract[k][0] 
								   || wall_coef[l].index_of_plane_intersection[m] == for_extract[k][1]
								   || wall_coef[l].index_of_plane_intersection[m] == for_extract[k][2]) 
									n_intersections[n_intersections.size()-1]++;
				}
				
				//find min
				size_t min(999),index_min;
				for (size_t k=0; k < n_intersections.size(); k++)
					if (min >  n_intersections[k]){
						min = n_intersections[k];
						index_min = k;
					}
				
				//extract min
				std::vector<std::vector <size_t> > ec_data4;
				for (size_t k=0; k < ec_data3.size(); k++)
					if (ec_data3[k][0] != for_extract[index_min][0] || ec_data3[k][1] != for_extract[index_min][1] || ec_data3[k][2] != for_extract[index_min][2]) 
					    ec_data4.push_back(ec_data3[k]);

						
				ec_data3 = ec_data4;
			
			}
		}
			*/	
		
		
		
		//calculate intersection points
		if (display && display_insert_corners_points) viewer->delete_all(1);
		for (size_t i=0; i < ec_data3.size(); i++)
			intersection_with_planes(ec_data3[i][0], ec_data3[i][1], ec_data3[i][2]);
		
		
		if (display && display_insert_corners_points) std::cout << "Corner points added " << std::endl;
		if (display && display_insert_corners_points) std::cout << "Press enter to continue...." << std::endl;
		if (display && display_insert_corners_points && wait_for_enter) getchar();
		if (display && display_insert_corners_points && !wait_for_enter) sleep(wait_time);
		
		return;
	}
	
	
	// MAKE POLYGONS IN PLANES TYPE A ////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::make_polygons_in_planes_type_A(){
		
		if (display && display_make_polygons_in_planes_type_A) viewer->delete_all(1);
		
		std::vector<size_t> index_type_A_U;
		std::vector<size_t> index_type_A_L;
		
		
		//for the proyection plane  
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
				
		coefficients->values.resize (4);
		coefficients->values[0] = 0;
		coefficients->values[1] = 0;
		coefficients->values[2] = 1;
		coefficients->values[3] = 0;
		
		
		//get the wall type A
		for (size_t i=0; i < wall_coef.size(); i++){
			if (wall_coef[i].type_A_U)
				index_type_A_U.push_back(i);
			
			else if (wall_coef[i].type_A_L)
				index_type_A_L.push_back(i);
		}
		
		
		
		
		
		// get the points tha belong to planes type A_U
		std::vector<size_t> index_points_U;
		
		for (size_t i=0; i < index_type_A_U.size(); i++)
			for (size_t j=0; j < important_points.size(); j++)
				for (size_t k=0; k < important_points[j].index_of_plane_belonging.size(); k++)
					if (important_points[j].index_of_plane_belonging[k] == index_type_A_U[i])
							index_points_U.push_back(j);
		
		
		// get the points tha belong to planes type A_L
		std::vector<size_t> index_points_L;
		
		for (size_t i=0; i < index_type_A_L.size(); i++)
			for (size_t j=0; j < important_points.size(); j++)
				for (size_t k=0; k < important_points[j].index_of_plane_belonging.size(); k++)
					if (important_points[j].index_of_plane_belonging[k] == index_type_A_L[i])
							index_points_L.push_back(j);
		
		
		
		
		// extract points U
		pcl::PointCloud<pcl::PointXYZ>::Ptr  points_U (new pcl::PointCloud<pcl::PointXYZ> );
		
		for (size_t i=0; i < index_points_U.size(); i++)
				points_U->push_back(important_points[index_points_U[i]].p);
		
		if (display && display_make_polygons_in_planes_type_A) viewer->load_xyz(points_U,1,false);
		
		
		
		
		// extract points L
		pcl::PointCloud<pcl::PointXYZ>::Ptr  points_L (new pcl::PointCloud<pcl::PointXYZ> );
		
		for (size_t i=0; i < index_points_L.size(); i++)
				points_L->push_back(important_points[index_points_L[i]].p);
		
		if (display && display_make_polygons_in_planes_type_A) viewer->load_xyz(points_L,1,false);
		
		
		
				
		//Proyect all points on plane z = 0						
		Project_Inliers.setModelType (pcl::SACMODEL_PLANE);
		Project_Inliers.setModelCoefficients (coefficients);
		Project_Inliers.setInputCloud (points_U);
		Project_Inliers.filter (*points_U);
		Project_Inliers.setInputCloud (points_L);
		Project_Inliers.filter (*points_L);
		
		if (display && display_make_polygons_in_planes_type_A) viewer->load_xyz(points_U,1,true);
		if (display && display_make_polygons_in_planes_type_A) viewer->load_xyz(points_L,1,true);
		
		
		
		// reconstruct using the laser angle
		std::vector<float> angles_U;

		for (size_t i=0; i < points_U->size(); i++)
			angles_U.push_back(get_angle_laser_sweep(points_U->points[i], points_U->points[points_U->points.size() - 1]));
			
		

		
		// reconstruct using the laser angle
		std::vector<float> angles_L;
		for (size_t i=0; i < points_L->size(); i++)
			angles_L.push_back(get_angle_laser_sweep(points_L->points[i], points_L->points[points_L->points.size()-1]));
			
	
		
		
		//Organize angles and indices for U plane
		for (size_t i=0; i < angles_U.size(); i++)
			for (size_t j=i+1; j < angles_U.size(); j++)
				if (angles_U[i] > angles_U[j]){
					float aux;
					size_t index_aux;
					
					aux = angles_U[i];
					index_aux = index_points_U[i];
					
					angles_U[i] = angles_U[j];
					index_points_U[i] = index_points_U[j];
					
					angles_U[j] = aux;
					index_points_U[j] = index_aux;
				}
				
			
		//Organize angles and indices for L plane
		for (size_t i=0; i < angles_L.size(); i++)
			for (size_t j=i+1; j < angles_L.size(); j++)
				if (angles_L[i] > angles_L[j]){
					float aux;
					size_t index_aux;
					
					aux = angles_L[i];
					index_aux = index_points_L[i];
					
					angles_L[i] = angles_L[j];
					index_points_L[i] = index_points_L[j];
					
					angles_L[j] = aux;
					index_points_L[j] = index_aux;
				}
				
		

		//Make conexions for U plane
		if (index_points_U.size() > 2) 
			conect_two_points(index_points_U[0], index_points_U[index_points_U.size()-1]);
		
		for (size_t i=0; i < index_points_U.size()-1; i++)
			conect_two_points(index_points_U[i], index_points_U[i+1]);
		
		
		
		
		//Make conexions for L plane
		if (index_points_L.size() > 2) 
			conect_two_points(index_points_L[0], index_points_L[index_points_L.size()-1]);
		
		for (size_t i=0; i < index_points_L.size()-1; i++)
			conect_two_points(index_points_L[i], index_points_L[i+1]);
		
		
		

		if (display && display_make_polygons_in_planes_type_A) std::cout << "Poligons made in plane type A" << std::endl;
		if (display && display_make_polygons_in_planes_type_A) std::cout << "Press enter to continue...." << std::endl;
		if (display && display_make_polygons_in_planes_type_A && wait_for_enter) getchar();
		if (display && display_make_polygons_in_planes_type_A && !wait_for_enter) sleep(wait_time);
		
		return;	
	}


	// MAKE POLYGONS IN PLANES TYPE B /////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::make_polygons_in_planes_type_B(){
			
		
		std::vector<size_t> index_type_A_U;
		std::vector<size_t> index_type_A_L;
		
		
		//for the proyection plane  
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
				
		coefficients->values.resize (4);
		coefficients->values[0] = 0;
		coefficients->values[1] = 0;
		coefficients->values[2] = 1;
		coefficients->values[3] = 0;
		
		
		//get the wall type A
		for (size_t i=0; i < wall_coef.size(); i++){
			if (wall_coef[i].type_A_U)
				index_type_A_U.push_back(i);
			
			else if (wall_coef[i].type_A_L)
				index_type_A_L.push_back(i);
		}
		
		
		
		
		
		// get the points tha belong to planes type A_U
		std::vector<size_t> index_points_U;
		
		for (size_t i=0; i < index_type_A_U.size(); i++)
			for (size_t j=0; j < important_points.size(); j++)
				for (size_t k=0; k < important_points[j].index_of_plane_belonging.size(); k++)
					if (important_points[j].index_of_plane_belonging[k] == index_type_A_U[i])
							index_points_U.push_back(j);
		
		
		// get the points tha belong to planes type A_L
		std::vector<size_t> index_points_L;
		
		for (size_t i=0; i < index_type_A_L.size(); i++)
			for (size_t j=0; j < important_points.size(); j++)
				for (size_t k=0; k < important_points[j].index_of_plane_belonging.size(); k++)
					if (important_points[j].index_of_plane_belonging[k] == index_type_A_L[i])
							index_points_L.push_back(j);
		
		
		
		
		// extract points U
		pcl::PointCloud<pcl::PointXYZ>::Ptr  points_U (new pcl::PointCloud<pcl::PointXYZ> );
		
		for (size_t i=0; i < index_points_U.size(); i++)
				points_U->push_back(important_points[index_points_U[i]].p);
		

		
		
		
		
		// extract points L
		pcl::PointCloud<pcl::PointXYZ>::Ptr  points_L (new pcl::PointCloud<pcl::PointXYZ> );
		
		for (size_t i=0; i < index_points_L.size(); i++)
				points_L->push_back(important_points[index_points_L[i]].p);
		

		
		
		
				
		//Proyect all points on plane z = 0						
		Project_Inliers.setModelType (pcl::SACMODEL_PLANE);
		Project_Inliers.setModelCoefficients (coefficients);
		Project_Inliers.setInputCloud (points_U);
		Project_Inliers.filter (*points_U);
		Project_Inliers.setInputCloud (points_L);
		Project_Inliers.filter (*points_L);
		
		
		
		
		// reconstruct using the laser angle
		std::vector<float> angles_U;
		for (size_t i=0; i < points_U->size(); i++)
			angles_U.push_back(get_angle_laser_sweep(points_U->points[i], points_U->points[points_U->size() - 1]));
			
		
		
		// reconstruct using the laser angle
		std::vector<float> angles_L;
		for (size_t i=0; i < points_L->size(); i++)
			angles_L.push_back(get_angle_laser_sweep(points_L->points[i], points_L->points[points_L->size() - 1]));
			
		
		
		//organize vectors
		
		
		for (size_t i=0; i < angles_L.size(); i++)
			for (size_t j=i; j < angles_L.size(); j++)
				if (angles_L[i] < angles_L[j]){
					float aux;
					aux = angles_L[i];
					angles_L[i] = angles_L[j];
					angles_L[j] = aux;
					
					size_t index_aux;
					index_aux = index_points_L[i];
					index_points_L[i] = index_points_L[j];
					index_points_L[j] = index_aux; 
				}
		
		
		for (size_t i=0; i < angles_U.size(); i++)
			for (size_t j=i; j < angles_U.size(); j++)
				if (angles_U[i] < angles_U[j]){
					float aux;
					aux = angles_U[i];
					angles_U[i] = angles_U[j];
					angles_U[j] = aux;
					
					size_t index_aux;
					index_aux = index_points_U[i];
					index_points_U[i] = index_points_U[j];
					index_points_U[j] = index_aux; 
				}
						
		
		
		//search posible conexions
		std::vector<size_t> posible_points_L;
		std::vector<size_t> posible_points_U;
	
		
		for (size_t i=0; i < index_points_L.size(); i++){
			std::vector <bool> planes_verification;
			for (size_t k=0; k < important_points[index_points_L[i]].index_of_point_conected.size(); k++)
				for (size_t j=0; j < important_points[index_points_L[i]].index_of_plane_belonging.size(); j++){
					bool belong(false);
					for (size_t l=0; l < important_points[important_points[index_points_L[i]].index_of_point_conected[k]].index_of_plane_belonging.size(); l++)
						if (important_points[index_points_L[i]].index_of_plane_belonging[j] == important_points[important_points[index_points_L[i]].index_of_point_conected[k]].index_of_plane_belonging[l])
							belong=true;	
					
					
					planes_verification.push_back(belong);
				
				}	
					
			
			
			
			bool all_true(true);
			for (size_t k=0; k < planes_verification.size(); k++)
				if (!planes_verification[k]) all_true = false;
			
			if (!all_true) 
				posible_points_L.push_back(index_points_L[i]);
			
		}	
		
			
		
		
		for (size_t i=0; i < index_points_U.size(); i++){
			std::vector <bool> planes_verification;
			for (size_t k=0; k < important_points[index_points_U[i]].index_of_point_conected.size(); k++)
				for (size_t j=0; j < important_points[index_points_U[i]].index_of_plane_belonging.size(); j++){
					bool belong(false);
					for (size_t l=0; l < important_points[important_points[index_points_U[i]].index_of_point_conected[k]].index_of_plane_belonging.size(); l++)
						if (important_points[index_points_U[i]].index_of_plane_belonging[j] == important_points[important_points[index_points_U[i]].index_of_point_conected[k]].index_of_plane_belonging[l])
							belong=true;	
					
					
					planes_verification.push_back(belong);
				
				}	
					
			
			
			
			bool all_true(true);
			for (size_t k=0; k < planes_verification.size(); k++)
				if (!planes_verification[k]) all_true = false;
			
			if (!all_true) 
				posible_points_U.push_back(index_points_U[i]);
			
		}	

		
		
		
		if (posible_points_L.size() == posible_points_U.size())
			for (size_t i=0; i < posible_points_U.size(); i++)
				conect_two_points(posible_points_L[i], posible_points_U[i]);
		
		
		
		//find new planes
		if (posible_points_L.size() == posible_points_U.size())
			for (size_t i=0; i < posible_points_U.size(); i++)
				for (size_t j=0; j < important_points[posible_points_U[i]].index_of_point_conected.size(); j++)
					for (size_t k=0; k < posible_points_U.size(); k++)
						if (posible_points_U[k] == important_points[posible_points_U[i]].index_of_point_conected[j]){
						
								plane new_plane;
								new_plane = obtaining_plane_coeficients(posible_points_U[i], posible_points_L[i], posible_points_U[k], posible_points_L[k]);
								
								new_plane.n_points = 0;
								new_plane.type_A_L = false;
								new_plane.type_A_U = false;
								new_plane.type_B = true;
								
								bool ok_c1;
								bool ok_c2;
								bool ok_c3;
								bool ok_c4;
								
								ok_c1 = important_points[posible_points_U[i]].index_of_plane_belonging.size() < 3;
								ok_c2 = important_points[posible_points_L[i]].index_of_plane_belonging.size() < 3;
								ok_c3 = important_points[posible_points_U[k]].index_of_plane_belonging.size() < 3;
								ok_c4 = important_points[posible_points_L[k]].index_of_plane_belonging.size() < 3;
								
								if (ok_c1 && ok_c2 && ok_c3 && ok_c4){
									wall_coef.push_back(new_plane);
									
									std::vector<size_t> index_type_A_U;
									std::vector<size_t> index_type_A_L;
		
									//get the intersections for planes type A
									for (size_t l=0; l < wall_coef.size(); l++){
										if (wall_coef[l].type_A_U)
											index_type_A_U.push_back(l);
			
										else if (wall_coef[l].type_A_L)
											index_type_A_L.push_back(l);
										}
		
									
									for (size_t l=0; l < index_type_A_U.size(); l++)
										line_coef.push_back(get_planes_intersection(wall_coef.size()-1, index_type_A_U[l]));
									for (size_t l=0; l < index_type_A_L.size(); l++)
										line_coef.push_back(get_planes_intersection(wall_coef.size()-1, index_type_A_L[l]));
									
									
									
									
									
									important_points[posible_points_U[i]].index_of_plane_belonging.push_back(wall_coef.size()-1);
									important_points[posible_points_L[i]].index_of_plane_belonging.push_back(wall_coef.size()-1);
									important_points[posible_points_U[k]].index_of_plane_belonging.push_back(wall_coef.size()-1);
									important_points[posible_points_L[k]].index_of_plane_belonging.push_back(wall_coef.size()-1);
									important_points[posible_points_U[i]].angle_swept.push_back(0);
									important_points[posible_points_L[i]].angle_swept.push_back(0);
									important_points[posible_points_U[k]].angle_swept.push_back(0);
									important_points[posible_points_L[k]].angle_swept.push_back(0);
								}
							}
							

					
					
					
		
		

		if (display && display_make_polygons_in_planes_type_B) std::cout << "Poligons made in planes type B" << std::endl;
		if (display && display_make_polygons_in_planes_type_B) std::cout << "Press enter to continue...." << std::endl;
		if (display && display_make_polygons_in_planes_type_B && wait_for_enter) getchar();
		if (display && display_make_polygons_in_planes_type_B && !wait_for_enter) sleep(wait_time);
		
		
		return;	
	}
	
	
	// FIND_POLYGONS ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::find_polygons(){
		
		for (size_t i=0; i < wall_coef.size(); i++){
			std::vector <size_t> index_points;
			for (size_t j=0; j < important_points.size(); j++)
				for (size_t k=0; k < important_points[j].index_of_plane_belonging.size(); k++)
					if (important_points[j].index_of_plane_belonging[k] == i)
						index_points.push_back(j);
		
		
			for (size_t j=0; j < index_points.size(); j++){
				bool ok(true);
				for (size_t k=0; k < polygon_planes.size(); k++)
					if (polygon_planes[k].plane_index == i)
						for (size_t l=0; l < polygon_planes[k].points_index.size(); l++)
							if (polygon_planes[k].points_index[l] == index_points[j])
								ok = false;
				
				
				if (ok)
					find_polygons_conexions(index_points[j], i, index_points.size());
							
			
			}
				
		
		}
		
		
		
		if (display && display_find_polygons)
			for (size_t i=0; i < polygon_planes.size(); i++){
				std::cout << "Polygon: " << i << " belongs to the plane: " << polygon_planes[i].plane_index << std::endl;
				for (size_t j=0; j < polygon_planes[i].points_index.size(); j++)
					std::cout << polygon_planes[i].points_index[j] << std::endl;
			}
		
		
		if (display && display_find_polygons) std::cout << "Poligons found" << std::endl;
		if (display && display_find_polygons) std::cout << "Press enter to continue...." << std::endl;
		if (display && display_find_polygons && wait_for_enter) getchar();
		if (display && display_find_polygons && !wait_for_enter) sleep(wait_time);
		
		
		return;	
	
	}

		
	// COMPLETE_SEGMENTS //////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::complete_segments (float interval){
		
		
		int end;
		double dd,
				x,
				y,
				z,
				module, 
				d;
	

		pcl::PointXYZ point;
		
		
		
		for (size_t i = 0; i <important_points.size() ; i++){
			for (size_t j = 0; j < important_points[i].index_of_point_conected.size(); j++){
				
				// module
				x = important_points[i].p.x - important_points[important_points[i].index_of_point_conected[j]].p.x;
				y = important_points[i].p.y - important_points[important_points[i].index_of_point_conected[j]].p.y;
				z = important_points[i].p.z - important_points[important_points[i].index_of_point_conected[j]].p.z;
				

				module = x*x + y*y + z*z;
	
				// distance
				d = sqrt(module);
	
				// interval
				end = d / interval;
				dd= 1.0 / end;
				
				
				// fills with the specefic interval
				for (int k=0; k < end ;k++){
					
					point.x = important_points[i].p.x + (important_points[important_points[i].index_of_point_conected[j]].p.x - important_points[i].p.x)*k*dd;
					point.y = important_points[i].p.y + (important_points[important_points[i].index_of_point_conected[j]].p.y - important_points[i].p.y)*k*dd;
					point.z = important_points[i].p.z + (important_points[important_points[i].index_of_point_conected[j]].p.z - important_points[i].p.z)*k*dd;
				
					cloud_reconstructed.push_back(point);	
				}	
			}
		}
		
		
		if (display && display_complete_segments) viewer->delete_all(1);
		if (display && display_complete_segments){
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux (new pcl::PointCloud<pcl::PointXYZ>);
			*cloud_aux = cloud_reconstructed;
			viewer->load_xyz(cloud_aux,1,false);
		}
		if (display && display_complete_segments) std::cout << "Segments completed " << std::endl;
		if (display && display_complete_segments) std::cout << "Press enter to continue...." << std::endl;
		if (display && display_complete_segments && wait_for_enter) getchar();
		if (display && display_complete_segments && !wait_for_enter) sleep(wait_time);
		
		return;
	}	
	
	
	// MAKE_POLYGONMESH ///////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::make_polygonmesh() {
		
		pcl::PCLPointCloud2 cloud_out;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux (new pcl::PointCloud<pcl::PointXYZ> );
		

		
		for (size_t i=0; i <  important_points.size(); i++)
			cloud_aux->push_back(important_points[i].p);
		
		
		for (size_t i=0; i <  polygon_planes.size(); i++)
			if (polygon_planes[i].points_index.size() > 2){
				pcl::Vertices  vertices;
				
				for (size_t j=0; j <  polygon_planes[i].points_index.size(); j++)
					vertices.vertices.push_back(polygon_planes[i].points_index[j]);
					
				
				poygonmesh_room.polygons.push_back(vertices);
			}
					
		
		
		
		pcl::toPCLPointCloud2(*cloud_aux, cloud_out);
		
		poygonmesh_room.cloud = cloud_out;
		
		return;
	}		
	
	
	// MAKE_POLYGONMESH_TRIAGLES //////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::make_polygonmesh_triangles(){		
		pcl::PCLPointCloud2 cloud_out;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux (new pcl::PointCloud<pcl::PointXYZ> );
		

		
		for (size_t i=0; i <  important_points.size(); i++)
			cloud_aux->push_back(important_points[i].p);
		
		for (size_t i=0; i <  polygon_planes.size(); i++)
			cloud_aux->push_back(polygon_planes[i].center);
	
		
		
		for (size_t i=0; i <  polygon_planes.size(); i++)
			if (polygon_planes[i].points_index.size() > 2){
				pcl::Vertices  vertices1;
				vertices1.vertices.push_back(polygon_planes[i].points_index[0]);
				vertices1.vertices.push_back(polygon_planes[i].points_index[polygon_planes[i].points_index.size() -1]);
				vertices1.vertices.push_back(important_points.size() + i);
				poygonmesh_triangles_room.polygons.push_back(vertices1);
				
				for (size_t j=0; j <  polygon_planes[i].points_index.size()-1; j++){
					pcl::Vertices  vertices2;
					vertices2.vertices.push_back(polygon_planes[i].points_index[j]);
					vertices2.vertices.push_back(polygon_planes[i].points_index[j+1]);
					vertices2.vertices.push_back(important_points.size() + i);
					poygonmesh_triangles_room.polygons.push_back(vertices2);
				}
			}
					
		
		
		
		pcl::toPCLPointCloud2(*cloud_aux, cloud_out);
		
		poygonmesh_triangles_room.cloud = cloud_out;
		
		return;
	}
	
	
	// MAKE_POINTMESH ////////////////////////////////////////////////////////////////////////////////////////////////
	void room_inside_detect::make_pointmesh(float interval){
		for (size_t i=0; i <  polygon_planes.size(); i++){
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mesh (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mesh2 (new pcl::PointCloud<pcl::PointXYZ>);
			if (polygon_planes[i].points_index.size() > 2){
				*cloud_mesh += *make_pointmesh_triangle(important_points[polygon_planes[i].points_index[polygon_planes[i].points_index.size() -1]].p, important_points[polygon_planes[i].points_index[0]].p, polygon_planes[i].center, interval);
				
				for (size_t j=0; j <  polygon_planes[i].points_index.size()-1; j++)
					*cloud_mesh += *make_pointmesh_triangle(important_points[polygon_planes[i].points_index[j]].p, important_points[polygon_planes[i].points_index[j+1]].p, polygon_planes[i].center, interval);	
			
				
				voxelgrid_filter.setInputCloud (cloud_mesh);
				voxelgrid_filter.setLeafSize (interval,interval,interval);
				voxelgrid_filter.filter (*cloud_mesh2);
				pointmesh_room += *cloud_mesh2;
				
			}
		}
		
		return;
	}
	
#endif
