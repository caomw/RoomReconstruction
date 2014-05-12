#ifndef ROOM_INSIDE_DETECT_H
#define ROOM_INSIDE_DETECT_H
	
	#include "definitions.h"
	#include "viewer_rt.h"
	
	#include <string> 
	#include <iostream>
	#include <stdio.h>
	#include <math.h>  

	#include <Eigen/Geometry>
	#include <Eigen/Cholesky>
	#include <Eigen/LU>

	#include <sys/wait.h> 
	#include <sys/sem.h>

	#include <boost/thread/thread.hpp>

	#include <pcl/surface/concave_hull.h>
	#include <pcl/surface/convex_hull.h>

	#include <pcl/point_cloud.h>
	#include <pcl/point_types.h>

	#include <pcl/impl/point_types.hpp>
	
	#include <pcl/common/common_headers.h> 

	#include <pcl/console/parse.h>
	#include <pcl/console/time.h>

	#include <pcl/kdtree/kdtree_flann.h>
	#include <pcl/kdtree/kdtree.h>

	#include <pcl/search/kdtree.h>

	#include <pcl/features/normal_3d.h>
	#include <pcl/features/integral_image_normal.h>

	#include <pcl/sample_consensus/method_types.h>
	#include <pcl/sample_consensus/model_types.h>
	#include <pcl/sample_consensus/sac_model_plane.h>

	#include <pcl/impl/point_types.hpp>

	#include <pcl/io/io.h>
	#include <pcl/io/pcd_io.h>

	#include <pcl/ModelCoefficients.h>

	#include <pcl/filters/voxel_grid.h>
	#include <pcl/filters/extract_indices.h>
	#include <pcl/filters/statistical_outlier_removal.h>
	#include <pcl/filters/conditional_removal.h>
	#include <pcl/filters/extract_indices.h>
	#include <pcl/filters/voxel_grid.h>
	#include <pcl/filters/project_inliers.h>
	#include <pcl/filters/radius_outlier_removal.h>
	#include <pcl/filters/passthrough.h>
	#include <pcl/filters/statistical_outlier_removal.h>
	
	#include <pcl/segmentation/sac_segmentation.h>
	#include <pcl/segmentation/extract_clusters.h>
	
	

	class room_inside_detect
		{
		
		public:
			
		///PUBLIC GENERAL FUCTIONS 		
			
			room_inside_detect ();				
			~room_inside_detect ();
			
			void set_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);			
			void display_process(viewer_rt *viewer_main); 		
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr extract_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float tolerancia,bool invert, plane plane_in);
			pcl::PointCloud<pcl::PointXYZ>::Ptr vectorpoint_2_pointcloud(std::vector <point> points_in);


		
		///PUBLIC GET KEY DATA FUNCTIONS 
			//Data
			void get_wall_coefficients (std::vector <plane> *coef_out);	
			void get_intersection_coefficients (std::vector <line> *coef_line_out);
			void get_important_points(std::vector <point> *important_points_out);
			void get_polygons_planes(std::vector <polygon_vertices> *polygon_planes_out);
			void get_process_time_vector(std::vector<double> *time_out_vector);
		
			
			//Clouds
			void get_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out); 
			void get_cloud_fitered (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
			void get_proyected_walls (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
			
		
		
		
		///PUBLIC GET SECONDARY DATA FUNCTIONS 
			void get_reconstructed_room (float interval, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
			void get_poygonmesh_room(pcl::PolygonMesh *poygonmesh_room_out);
			void get_poygonmesh_triangles_room(pcl::PolygonMesh *poygonmesh_triangles_room_out);
			void get_pointmesh_room(float interval, pcl::PointCloud<pcl::PointXYZ>::Ptr pointmesh_room_out);
			



		///PUBLIC SET-GET VARIABLES FUNCTIONS
			//Key functions
			void get_density_filter_grid(float *grid_out);
			void set_density_filter_grid(float grid_in);
			
			void get_find_wall_coeficientes_perc_of_search(float *perc_of_search_out);
			void set_find_wall_coeficientes_perc_of_search(float perc_of_search_in);
			
			void get_find_wall_coeficientes_Iterations(int *Iterations_out);
			void set_find_wall_coeficientes_Iterations(int Iterations_in);
			
			void get_find_wall_coeficientes_radio_coef(float *radio_coef_out);
			void set_find_wall_coeficientes_radio_coef(float radio_coef_in);
			
			void get_find_wall_coeficientes_lim_segmentos(float *lim_segmentos_out);
			void set_find_wall_coeficientes_lim_segmentos(float lim_segmentos_in);
			
			void get_find_wall_coeficientes_p_admissible_coef(float *p_admissible_coef_out);
			void set_find_wall_coeficientes_p_admissible_coef(float p_admissible_coef_in);	
			
			void get_proyect_planes_type_B_perceptil_n(float *perceptil_n_out);
			void set_proyect_planes_type_B_perceptil_n(float perceptil_n_in);
			
			void get_proyect_planes_type_B_min_proyect(float *min_proyect_out);
			void set_proyect_planes_type_B_min_proyect(float min_proyect_in);
			
			void get_proyect_planes_type_B_int_min(float *int_min_out);
			void set_proyect_planes_type_B_int_min(float int_min_in);
			
			void get_proyect_planes_type_B_tol_seg(float *tol_seg_out);
			void set_proyect_planes_type_B_tol_seg(float tol_seg_in);
			
			void get_proyect_planes_type_B_size_min(float *size_min_out);
			void set_proyect_planes_type_B_size_min(float size_min_in);
			
			//Secundary functions
			void get_intersection_coefficients_avoid_ecual_planes(float *avoid_ecual_planes_out);
			void set_intersection_coefficients_avoid_ecual_planes(float avoid_ecual_planes_in);
			
			void get_intersection_with_planes_angle_diff(float *angle_diff_out);
			void set_intersection_with_planes_angle_diff(float angle_diff_in);
			
			void get_conect_two_points_direction_adm(float *direction_adm_out);
			void set_conect_two_points_direction_adm(float direction_adm_in);
				
				
			//Others
			void set_default_variables();
			
			void set_reference_vector(pcl::PointXYZ reference);
			void get_reference_vector(pcl::PointXYZ *reference);
			
			void get_process_visualization(bool *P1, bool *P2, bool *P3, bool *P4, bool *P5, bool *P6, bool *P7, bool *P8, bool *P9, bool *P10, bool *P11);
			void set_process_visualization(bool P1, bool P2, bool P3, bool P4, bool P5, bool P6, bool P7, bool P8, bool P9, bool P10, bool P11);
			
			void set_visualization_time(int time_in);
			
			
			
		private:
			
		///OBJECTS, VECTORS AND STRUCTURES
			
			// Clouds
			pcl::PointCloud<pcl::PointXYZ> cloud_original;
			pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
			pcl::PointCloud<pcl::PointXYZ> cloud_proyected;
			pcl::PointCloud<pcl::PointXYZ> cloud_reconstructed;
			pcl::PointCloud<pcl::PointXYZ> pointmesh_room;
		
					
		
			//Coeficients and data
			std::vector <plane> wall_coef;
			std::vector <line> line_coef;
			std::vector <point> important_points;
			std::vector <polygon_vertices> polygon_planes;
			pcl::PolygonMesh poygonmesh_room;
			pcl::PolygonMesh poygonmesh_triangles_room;
			std::vector<double> vector_process_time;
			

			
			// Internal objects
			pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_filter;
			pcl::SACSegmentation<pcl::PointXYZ> SAC_Segmentation;
			pcl::ExtractIndices<pcl::PointXYZ> Extract_Indices;
			pcl::ConcaveHull <pcl::PointXYZ> Concave_Hull;
			pcl::ProjectInliers<pcl::PointXYZ> Project_Inliers;
			pcl::RadiusOutlierRemoval<pcl::PointXYZ> Radius_Outlier_Removal;
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> Euclidean_Cluster_Extraction;
			pcl::console::TicToc process_time;
			pcl::console::TicToc sub_process_time;
			pcl::console::TicToc all_process_time;
			
			
			
			// External objects
			viewer_rt *viewer;
			
			
		///CONFIGURATION VARIABLES
				
			//GENERAL
			bool wait_for_enter;
			int wait_time;
			bool process_done;
			bool display;
			bool display_process_time;
			size_t layer_in_progress;
			pcl::PointXYZ sensor;
			pcl::PointXYZ reference;
			
			
			
		
			//PRIVATE KEY FUNCTIONS	
			
				//density_filter variables
				bool default_DF;
				bool display_density_filter;
				float grid;
			
				
				//find_wall_coeficients ()
				bool default_FWC;
				bool display_find_wall_coeficients;
				float perc_of_search;
				int Iterations;
				float radio_coef;
				float lim_segmentos;
				float p_admissible_coef;
			
			
				//identify_type_of_plane()
				bool display_identify_type_of_plane;	
			
			
				//get_intersections_for_planes_type_A()
				bool display_find_intersections_coeficients;
				
			
				//proyect_planes_type_B()
				bool display_proyect_planes_type_B;
				float perceptil_n;
				float min_proyect;
				float int_min;
				float tol_seg;
				float size_min;
			
			
				//delete_incorrect_planes()
				bool display_delete_incorrect_planes;
			
			
				//insert_corners_points()
				bool display_insert_corners_points;
			
			
				//make_polygons_in_planes_type_A
				bool display_make_polygons_in_planes_type_A;
			
			
				//make_polygons_in_planes_type_B
				bool display_make_polygons_in_planes_type_B;
			
			
				//find_polygons
				bool display_find_polygons;
			
			
				//complete_segments
				bool display_complete_segments;
			
			
			
			
			//PRIVATE SECONDARY FUNCTIONS
			
				//get_intersection_planes
				float avoid_ecual_planes;
			
	
				//intersection_with_planes
				float angle_diff;
	
	
				//conect_two_points
				float direction_adm;
	
			
	
			
	
			
		///PRIVATE KEY FUNCTIONS
			
			void process_cloud();
			void density_filter ();
			void find_wall_coeficients ();	
			void identify_type_of_plane();
			void get_intersections_for_planes_type_A();
			void proyect_planes_type_B();		
			void delete_incorrect_planes(); 
			void insert_corners_points();
			void make_polygons_in_planes_type_A();
			void make_polygons_in_planes_type_B();
			void find_polygons();
			void complete_segments (float interval);
			void make_polygonmesh();
			void make_polygonmesh_triangles();
			void make_pointmesh(float interval);
			
			
			
			
				
		
		///PRIVATE SECONDARY FUNCTIONS
		
			void reset_data();
			void extract_wall_index(size_t index);
			void extract_line_index(size_t index);
			void intersection_with_planes(size_t index_A, size_t index_B, size_t index_C);	
			void search_lower_angle_from(float starting_angle, std::vector<float> angles_points_in, float *angle_out, float *max_angle, size_t *index_out);
			void find_polygons_conexions(size_t point_index,size_t plane_index, size_t max_iterations);
			
			
			bool search_bigger_angle_in_conexion(size_t index_in, std::vector<size_t> index_points_in, std::vector<float> angles_points_in ,size_t *index_out, float *angle_out);
			bool check_plane_system(size_t index_A, size_t index_B, size_t index_C);		
			bool conect_two_points(size_t index_p1, size_t index_p2);
			bool check_polygon(std::vector<size_t> index_points_in, size_t plane_index);
			
			float get_angle_laser_sweep(pcl::PointXYZ p1, pcl::PointXYZ reference);
			float distance(pcl::PointXYZ p1, pcl::PointXYZ p2);
			float get_sweep_angle(pcl::PointXYZ p1, plane p);
			float angle_between_two_vetors(pcl::PointXYZ v1, pcl::PointXYZ v2);
			float full_angle_between_two_vetors(pcl::PointXYZ v1, pcl::PointXYZ v2, size_t plane_index);
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr simplify_straight_line (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
			pcl::PointCloud<pcl::PointXYZ>::Ptr find_farest_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
			pcl::PointCloud<pcl::PointXYZ>::Ptr divide_line(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,float interval);
			pcl::PointCloud<pcl::PointXYZ>::Ptr make_pointmesh_triangle(pcl::PointXYZ base1, pcl::PointXYZ base2, pcl::PointXYZ top, float interval);
			
			
			std::vector<point> extract_point_vectorpoint(std::vector <point> points_in, size_t index);
			
			std::vector<size_t> vector_organize_descending(std::vector<size_t> vector_in);
			std::vector<size_t> extract_i_data_from_list(size_t i, std::vector<size_t> list);
			
			std::vector <float> angle_swept_from_center_point(std::vector<size_t> index_points, size_t plane_index);
			
			line get_planes_intersection(size_t index_plane_1,size_t index_plane_2);
			
			plane obtaining_plane_coeficients(size_t p1, size_t p2, size_t p3, size_t p4);
	};
	
	
	#include "room_inside_detect.hpp"
	
#endif
