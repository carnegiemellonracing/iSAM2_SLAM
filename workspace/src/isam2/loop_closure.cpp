/**
 * @file loop_closure.cpp
 * @brief Performs loop closure
 */

bool loop_closure(vector<Point2> &orange_cones) {
    if (orangeCones.size() == 0)
    {
        orangeNotSeen++;
        if (orangeNotSeen >= 25)
        {
            // std::cout <<"orange not seen flag true" << std::endl;
            orangeNotSeenFlag = true;
        }
    }
    else
    {
        orangeNotSeen = 0;
    }


    if (orangeCones.size() == 2)
    { // what if there's more than 2?
      // std::cout <<"Added two cones to orange cones"<<endl;
        bool left = false;
        bool right = false;
        vector<Point2> orangeLoopReferenceCones(2);
        for (uint i = 0; i < orangeCones.size(); i++)
        {
            if (left == false && orangeCones[i].y() < 0)
            {
                // add the left cone here
                left = true;
                orangeLoopReferenceCones[0] = orangeCones[i];
            }
            if (right == false && orangeCones[i].y() > 0)
            {
                // add the right cone here
                right = true;
                orangeLoopReferenceCones[1] = orangeCones[i];
            }
        }
        orangeCones = orangeLoopReferenceCones;
    }

    if (orangeNotSeenFlag == true && orangeCones.size() >= 2)
    {
       // std::cout<<"found loop closure" << std::endl;
       Point2 diff = Point2(init_odom.x() - global_odom.x(),
                       init_odom.y() - global_odom.y());
       float dist_from_start = norm2(diff);

       bool near_start = (dist_from_start <= (float)10);

       float bearing_diff = abs(init_odom.theta() - global_odom.theta());
       bool facing_start = bearing_diff <= (float)(M_PI / 2);

       RCLCPP_INFO(this->get_logger(), "BEARING DIFF: %lf | DIST FROM START: %lf",
                               bearing_diff, dist_from_start);
       if (near_start && facing_start)
       {
           RCLCPP_INFO(this->get_logger(), "LOOP CLOSURE\n\n");
           loopClosure = true;
       }
    }
}
