#include <ros/ros.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class PredictedObjectsVisualizer {
public:
    PredictedObjectsVisualizer() {
        // Subscribe to the detected objects topic
        sub = nh.subscribe("/prediction/objects", 10, &PredictedObjectsVisualizer::PredictCallback, this);
        
        // Publisher for marker array
        pub = nh.advertise<visualization_msgs::MarkerArray>("/prediction/objects_marker", 10);
    }

    // Callback function to process detected objects
    void PredictCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& data) {
        visualization_msgs::MarkerArray marker_array;

        // Remove previous markers by sending DELETE action
        for (size_t i = 0; i < previous_marker_ids.size(); ++i) {
            visualization_msgs::Marker delete_marker;
            delete_marker.header = data->header;
            delete_marker.ns = "predicted_objects";
            delete_marker.id = previous_marker_ids[i];
            delete_marker.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(delete_marker);
        }
        previous_marker_ids.clear();

        // Iterate through the detected objects and create markers for them
        for (size_t i = 0; i < data->objects.size(); ++i) {
            const auto& obj = data->objects[i];
            visualization_msgs::Marker marker;
            marker.header = data->header;
            marker.ns = "predicted_objects";
            marker.id = i;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            // Set marker position (start of the arrow)
            marker.pose.position.x = obj.pose.position.x;
            marker.pose.position.y = obj.pose.position.y;
            marker.pose.position.z = obj.pose.position.z;

            // Set marker orientation (arrow direction)
            marker.pose.orientation.x = obj.pose.orientation.x;
            marker.pose.orientation.y = obj.pose.orientation.y;
            marker.pose.orientation.z = obj.pose.orientation.z;
            marker.pose.orientation.w = obj.pose.orientation.w;

            // Set marker scale (size of the arrow)
            marker.scale.x = 1.0;  // Length of the arrow
            marker.scale.y = 0.5;  // Width of the arrow shaft
            marker.scale.z = 0.5;  // Height of the arrowhead

            // Set marker color
            marker.color.r = 0.0;
            marker.color.g = 1.0;  // Green color
            marker.color.b = 0.0;
            marker.color.a = 1.0;  // Fully opaque

            // Add marker to the array
            marker_array.markers.push_back(marker);

            // Save marker ID for potential removal in the next callback
            previous_marker_ids.push_back(marker.id);
        }

        // Publish the markers
        pub.publish(marker_array);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    std::vector<int> previous_marker_ids;  // Stores IDs of the markers to be removed in the next cycle
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cbmp_visual_node");
    PredictedObjectsVisualizer visualizer;
    ros::spin();
    return 0;
}
