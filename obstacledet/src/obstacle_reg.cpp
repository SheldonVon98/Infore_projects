#include "obstacle_reg.h"
#include <obstacle_det/Obstacles.h>
#include <obstacle_det/CircleObstacle.h>


struct ComplementaryFilter{
	float current;
	float previous;
	void update(float value){
		current = 0.2 * value + 0.8 * previous;
		previous = current;
	};
};


ObstacleReg::ObstacleReg(){
    cloudSub = nh.subscribe<PcROS>("/obstacle_cloud_merge", 1, &ObstacleReg::cloudCallback, this);
	obstacles_pub = nh.advertise<obstacle_det::Obstacles>("obstacles_circles", 10);

}

ObstacleReg::~ObstacleReg(){
}

void ObstacleReg::cloudCallback(const PcROS_Ptr& cloud_msg){


    // Process the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    /* Creating the KdTree from input point cloud*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*cloud_msg, *input_cloud);

	if(input_cloud->empty()) return;


    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(600);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract(cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    // Vector of cluster pointclouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;
	// Radius of cluster
    std::vector<float> cluster_radius;

    // Cluster centroids
    std::vector<pcl::PointXYZ> clusterCentroids;

    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
			new pcl::PointCloud<pcl::PointXYZ>);
		float x = 0.0;
		float y = 0.0;
		int numPts = 0;
		for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

			cloud_cluster->points.push_back(input_cloud->points[*pit]);
			x += input_cloud->points[*pit].x;
			y += input_cloud->points[*pit].y;
			numPts++;
		}

		pcl::PointXYZ centroid;
		centroid.x = x / numPts;
		centroid.y = y / numPts;
		centroid.z = 0.0;

		
	    std::vector<float> cluster2center;

		for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

			cloud_cluster->points.push_back(input_cloud->points[*pit]);
			x += input_cloud->points[*pit].x;
			y += input_cloud->points[*pit].y;
			numPts++;

			#if PCL_VERSION_COMPARE(>, 1, 7, 2)
				cluster2center.push_back(pcl::geometry::distance(input_cloud->points[*pit], centroid));
			#else
				cluster2center.push_back(pcl::geometry::distance(input_cloud->points[*pit].getVector3fMap(), 
																centroid.getVector3fMap()));
			#endif
		}

		cluster_radius.push_back(*std::max_element(cluster2center.begin(), cluster2center.end()));

		cluster_vec.push_back(cloud_cluster);

		// Get the centroid of the cluster
		clusterCentroids.push_back(centroid);
    }


	obstacle_det::ObstaclesPtr obstacles_msg(new obstacle_det::Obstacles);
    obstacles_msg->header.stamp = ros::Time();
    obstacles_msg->header.frame_id = "map";
    for (int i = 0; i < clusterCentroids.size(); i++) {
		obstacle_det::CircleObstacle circle;
		circle.center.x = clusterCentroids.at(i).x;
		circle.center.y = clusterCentroids.at(i).y;
		circle.velocity.x = 0.0;
		circle.velocity.y = 0.0;
		
		circle.true_radius = cluster_radius[i];// 0.2;
		// circle.radius = circle.true_radius > 0.8 ? circle.true_radius: 0.8;
		circle.radius = circle.true_radius * 2.5;
		obstacles_msg->circles.push_back(circle);

    }
    obstacles_pub.publish(obstacles_msg);
}


int main (int argc, char** argv) {
	ros::init (argc, argv, "obstacle_reg");
	ObstacleReg *obstacleReg = new ObstacleReg();
	ros::spin();
}