#include "control_p2/math/pure_pursuit.hpp"

Pursuit_Algorithm::Pursuit_Algorithm(float missionSpeed){
    this->missionSpeed = missionSpeed;
}

lart_msgs::msg::DynamicsCMD Pursuit_Algorithm::calculate_control(lart_msgs::msg::PathSpline path, geometry_msgs::msg::PoseStamped current_pose,
             float current_speed, float current_steering){
    //Declare variable to return
    lart_msgs::msg::DynamicsCMD control_output;
    
    // Transform the path to the car's coordinate system
    vector<array<float, 2>> path_points = transform_path(path);

    // Calculate lookahead point based on the speed with a min and max distances
    float look_ahead_distance = clamp(speed_to_lookahead(current_speed), MIN_LOOKAHEAD, MAX_LOOKAHEAD);
    
    // Define the target point
    optional<array<float, 2>> closest_point = get_closest_point(path_points, look_ahead_distance);

    // If no closest point is found, return the current steering angle
    if (!closest_point.has_value())
    {
        control_output.steering_angle = getAvgAngle();
        control_output.rpm = 0;
        
        return control_output;
    }

    // Set the target point        
    this->target_point = path.poses[this->closest_point_index];

    // If the closest point is in front of the car then consider the desired angle to be 0
    if ((*closest_point)[0] == 0)
    {
        // Keep previous angles to calculate the average
        keepAvgAngle(0.0f);

        control_output.steering_angle = getAvgAngle();
        control_output.rpm = MS_TO_RPM(calculate_desiredSpeed(path));
        return control_output;
    }
    // Calculate angle between the closest point and (0,0) (because the point is returned relative to (0,0)) instead of the rear!!
    float alpha = atan2((*closest_point)[1], (*closest_point)[0]);

    // Calculate steering angle (pure pursuit algorithm)
    float steering_angle = atan2(2 * WHEELBASE_M * sin(alpha), look_ahead_distance);

    // Keep previous angles to calculate the average
    keepAvgAngle(steering_angle);

    // Set the control output
    control_output.steering_angle = getAvgAngle();
    control_output.rpm = MS_TO_RPM(calculate_desiredSpeed(path));

    return control_output;
}

vector<array<float, 2>> Pursuit_Algorithm::transform_path(lart_msgs::msg::PathSpline path){
    vector<array<float, 2>> path_points;

    // Define the transformation
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(DEFAULT_IMU_TO_REAR_AXLE, 0.0, 0.0));
    transform.setRotation(tf2::Quaternion(0, 0, 0, 1));

    for (long unsigned int i = 0; i < path.poses.size(); i++)
    {
        // Create an array with X and Y position of the path, shifting the X value to the rear of the car
        geometry_msgs::msg::PoseStamped input_pose = path.poses[i];
        tf2::Transform input_transform;
        tf2::fromMsg(input_pose.pose, input_transform);

        // Apply the transformation
        tf2::Transform transformed_transform = transform * input_transform;
        geometry_msgs::msg::Pose transformed_pose;
        transformed_pose.position.x = transformed_transform.getOrigin().x();
        transformed_pose.position.y = transformed_transform.getOrigin().y();
        transformed_pose.position.z = transformed_transform.getOrigin().z();
        transformed_pose.orientation.x = transformed_transform.getRotation().x();
        transformed_pose.orientation.y = transformed_transform.getRotation().y();
        transformed_pose.orientation.z = transformed_transform.getRotation().z();
        transformed_pose.orientation.w = transformed_transform.getRotation().w();

        // Extract the transformed X and Y positions
        std::array<float, 2> point = {
            static_cast<float>(transformed_pose.position.x),
            static_cast<float>(transformed_pose.position.y)
        };

        path_points.push_back(point);
    }
    return path_points;
}

float Pursuit_Algorithm::speed_to_lookahead(float speed){
    float look_ahead_distance = 3.6f + 0.5f * speed;
    return look_ahead_distance;
}

optional<array<float, 2>> Pursuit_Algorithm::get_closest_point(vector<array<float, 2>> path_points, float look_ahead_distance)
{
    if(path_points.size() > MIN_TARGET_INDEX)
    {
        this->closest_point_index = fastRound((look_ahead_distance)/SPACE_BETWEEN_POINTS) - 1;
        return path_points[this->closest_point_index];
    }
    this->closest_point_index = -1;
    return nullopt;
}

int Pursuit_Algorithm::fastRound(float x) {
    return static_cast<int>(x + 0.5f);
}

float Pursuit_Algorithm::calculate_desiredSpeed(lart_msgs::msg::PathSpline path){
    if(closest_point_index > -1){
        float curvature = abs(path.curvature[closest_point_index]);
        
        if(curvature < 0.0001){
            curvature = 0.0001; // Avoid division by zero
        }

        float velocity = sqrt(this->vehicle.get_grip_coefficient() * LART_GRAVITY * (1/curvature));
        
        return velocity;
    }
    return 0.0;
}

void Pursuit_Algorithm::keepAvgAngle(float steering_angle){
    int slot = cycles % SIZE_AVG_ARRAY;
    avg_angle[slot] = steering_angle;
    cycles++;
}

float Pursuit_Algorithm::getAvgAngle(){
    float sum = 0;
    int interval = SIZE_AVG_ARRAY;

    //intialize the interval to the number of cycles if it is less than the size of the array
    if(cycles < SIZE_AVG_ARRAY){
        if (cycles == 0){
            return 0.0;
        }
        interval = cycles;
    }

    for(int i = 0; i < interval; i++){
        sum += avg_angle[i];
    }
    return sum / interval;
}

geometry_msgs::msg::PoseStamped Pursuit_Algorithm::get_target_point()
{
    return this->target_point;
}
