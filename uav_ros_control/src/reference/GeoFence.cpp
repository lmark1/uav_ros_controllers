#include <uav_ros_control/reference/GeoFence.h>
#include <typeinfo>

uav_reference::GeoFence::GeoFence(ros::NodeHandle& nh, std::string filename) :
	_global_to_local(nh)
{

	YAML::Node config = YAML::LoadFile(filename);
	YAML::Node constraints_list = config["gps_constraints"];

	ROS_INFO_STREAM("Loading GPS constraint points from file:\n" << filename);
	ROS_INFO_STREAM("Loaded GPS points:\n" << constraints_list);

	ros::Duration(1).sleep();
	ros::spinOnce();

	ROS_INFO_STREAM("Constraint points converted to local frame:");
	int i = 0;
	for (YAML::const_iterator ti = constraints_list.begin(); ti != constraints_list.end(); ++ti)
	{
		const YAML::Node& constraint = *ti;
		double lat = constraint["lat"].as<double>();
		double lon = constraint["lon"].as<double>();
		double alt = 0;
		Eigen::Vector3d temp_vector = _global_to_local.toLocal(lat, lon, alt);
		geometry_msgs::Vector3 vertex;
		vertex.x = temp_vector.x();
		vertex.y = temp_vector.y();
		vertex.z = temp_vector.z();
		_vertices.push_back(vertex);
		std::cout << "X: " << vertex.x << ", Y: " << vertex.y << std::endl;
	}
	_vertices.push_back(_vertices[0]);


	// Define Publisher
	_pub =
		nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("geofence_out", 1);

	// Define Subscriber
	_sub = 
		nh.subscribe("geofence_in", 1, &uav_reference::GeoFence::referenceCb, this);

}

uav_reference::GeoFence::~GeoFence()
{	
}

void uav_reference::GeoFence::referenceCb(
	const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& msg)
{
	geometry_msgs::Vector3 current_position = msg->transforms.front().translation;

	if (checkInside(current_position, 4))
	{
		geometry_msgs::Twist empty_twist;
		// std::cout << "inside" << std::endl;
		_pub.publish(*msg);
		_last_valid_position = *msg;
		_last_valid_position.velocities.front() = geometry_msgs::Twist();
		_last_valid_position.accelerations.front() = empty_twist;
	}
	else
	{
		// std::cout << "outside" << std::endl;
		_pub.publish(_last_valid_position);
	}

}


// checkInside(): winding number test for a point in a polygon
//      Input:   current = current position,
//               verices[] = vertex points of a polygon vertices[n+1] with vertices[n]=vertices[0]
//      Return:  true if current position is inside of the polygon
bool uav_reference::GeoFence::checkInside( 
	geometry_msgs::Vector3 current, 
	int n)
{
	int wn = 0;  // the winding number counter

    // std::cout << "Current: " << current << std::endl;
	// loop through all edges of the polygon
    for (int i=0; i<n; i++) {   // edge from V[i] to  V[i+1]
    	// std::cout << "Vertex: " << _vertices[i] << std::endl;

        if (_vertices[i].y <= current.y) {          // start y <= current.y
            if (_vertices[i+1].y  > current.y)      // an upward crossing
                 if (isLeft( _vertices[i], _vertices[i+1], current) > 0)  // current left of  edge
                     ++wn;            // have  a valid up intersect
        }
        else {                        // start y > current.y (no test needed)
            if (_vertices[i+1].y  <= current.y)     // a downward crossing
                 if (isLeft( _vertices[i], _vertices[i+1], current) < 0)  // current right of  edge
                     --wn;            // have  a valid down intersect
        }
    }
    return wn > 0;
}

// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2  on the line
//            <0 for P2  right of the line
//    See: Algorithm 1 "Area of Triangles and Polygons"
 int uav_reference::GeoFence::isLeft( 
 	geometry_msgs::Vector3 P0,
 	geometry_msgs::Vector3 P1,
 	geometry_msgs::Vector3 P2 )
{

    return ( (P1.x - P0.x) * (P2.y - P0.y)
            - (P2.x -  P0.x) * (P1.y - P0.y) );
}

void uav_reference::runDefault(
	uav_reference::GeoFence& geoFenceObj, ros::NodeHandle& nh)
{
	ros::spin();
}