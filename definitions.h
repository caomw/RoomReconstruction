struct line{
	float vx;
	float vy;
	float vz;
	float x;
	float y;
	float z;
	size_t layer;
	std::vector <bool> proyections;
	pcl::PointCloud<pcl::PointXYZ> cells;
	size_t index_of_plane_a;
	size_t index_of_plane_b;
};

struct plane{
	float a;
	float b;
	float c;
	float d;
	pcl::PointXYZ sensor;
	pcl::PointXYZ center;
	pcl::PointXYZ reference;
	size_t layer;
	bool special;
	bool type_A_L;
	bool type_A_U;
	bool type_B;
	size_t n_points;
	std::vector <size_t> index_of_plane_intersection;
	std::vector <size_t> points_in_intersection;
	std::vector <size_t> index_of_line_intersection;
};



struct point{
	pcl::PointXYZ p;
	std::vector <float> angle_swept;
	std::vector <float> angle_swept_center;
	std::vector <size_t> index_of_plane_belonging;
	std::vector <size_t> index_of_point_conected;
	size_t layer;
};

struct segment{
	pcl::PointXYZ center;
	std::vector <size_t> index;
	std::vector <float> angle_swept;
	bool initial;
};

struct polygon_vertices{
	pcl::PointXYZ center;
	std::vector <size_t> points_index;
	size_t plane_index;
};


