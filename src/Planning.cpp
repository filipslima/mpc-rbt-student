#include "Planning.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>
  
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        // add code here
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        // Service for path
        // add code here
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
        "/plan_path",
        std::bind(&PlanningNode::planPath, this, _1, _2)
        );

        // Publisher for path
        // add code here
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        RCLCPP_INFO(get_logger(), "Planning node started TESTSTSTSTSD.");

        // Connect to map server
        // add code here
        while (!map_client_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_WARN(get_logger(), "Waiting for /map_server/map service...");
        }

        // Request map
        // add code here
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        map_client_->async_send_request(
        request,
        std::bind(&PlanningNode::mapCallback, this, _1)
        );

        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    // add code here

    // ********
    // * Help *
    // ********
    auto response = future.get();
    if (response) {
        int occupied_before = 0;
        for (auto v : response->map.data) {
            if (v == -1 || v >= 50) occupied_before++;
        }

        map_ = response->map;

        RCLCPP_INFO(
            get_logger(),
            "Map received: width=%u, height=%u, resolution=%.3f",
            map_.info.width,
            map_.info.height,
            map_.info.resolution
        );

        dilateMap();

        int occupied_after = 0;
        for (auto v : map_.data) {
            if (v == -1 || v >= 50) occupied_after++;
        }

        RCLCPP_INFO(get_logger(), "Occupied before: %d, after dilation: %d",
                    occupied_before, occupied_after);
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to receive map.");
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    aStar(request->start, request->goal);
    smoothPath();

    path_pub_->publish(path_);
    */
    if (map_.data.empty()) {
        RCLCPP_WARN(get_logger(), "Map not available yet. Returning empty path.");

        path_ = nav_msgs::msg::Path();
        path_.header.frame_id = "map";
        path_.header.stamp = this->now();

        response->plan = path_;
        return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Planning request received: start=(%.2f, %.2f), goal=(%.2f, %.2f)",
        request->start.pose.position.x,
        request->start.pose.position.y,
        request->goal.pose.position.x,
        request->goal.pose.position.y
    );

    aStar(request->start, request->goal);
    RCLCPP_INFO(get_logger(), "Path size after A*: %zu", path_.poses.size());
    
    smoothPath();

    path_.header.frame_id = "map";
    path_.header.stamp = this->now();

    response->plan = path_;
    path_pub_->publish(path_);

    RCLCPP_INFO(get_logger(), "Published planned path with %zu poses.", path_.poses.size());
}

void PlanningNode::dilateMap() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    ... processing ...
    map_ = dilatedMap;
    */
    if (map_.data.empty()) {
        return;
    }

    nav_msgs::msg::OccupancyGrid dilatedMap = map_;

    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const int radius = 10;

    RCLCPP_INFO(get_logger(), "Dilating map with radius = %d", radius);

    auto idx = [width](int x, int y) {
        return y * width + x;
    };

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int current = map_.data[idx(x, y)];

            if (current == -1 || current >= 50) {
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        const int nx = x + dx;
                        const int ny = y + dy;

                        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                            continue;
                        }

                        if (dx * dx + dy * dy <= radius * radius) {
                            dilatedMap.data[idx(nx, ny)] = 100;
                        }
                    }
                }
            }
        }
    }

    map_ = dilatedMap;
    RCLCPP_INFO(get_logger(), "Map dilated.");
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    Cell cStart(...x-map..., ...y-map...);
    Cell cGoal(...x-map..., ...y-map...);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    openList.push_back(std::make_shared<Cell>(cStart));

    while(!openList.empty() && rclcpp::ok()) {
        ...
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    */

    path_ = nav_msgs::msg::Path();
    path_.header.frame_id = "map";
    path_.header.stamp = this->now();

    if (map_.data.empty()) {
        RCLCPP_ERROR(get_logger(), "Map is empty.");
        return;
    }

    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const double resolution = map_.info.resolution;
    const double origin_x = map_.info.origin.position.x;
    const double origin_y = map_.info.origin.position.y;

    auto getIndex = [width](int x, int y) {
        return y * width + x;
    };

    auto isInsideMap = [width, height](int x, int y) {
        return x >= 0 && y >= 0 && x < width && y < height;
    };

    auto isFree = [&](int x, int y) {
        if (!isInsideMap(x, y)) {
            return false;
        }

        int value = map_.data[getIndex(x, y)];

        // -1 = unknown, >=50 = obstacle
        if (value == -1) {
            return false;
        }

        return value < 50;
    };

    auto worldToMap = [&](double wx, double wy, int &mx, int &my) {
        mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
        my = static_cast<int>(std::floor((wy - origin_y) / resolution));
        return isInsideMap(mx, my);
    };

    auto mapToWorld = [&](int mx, int my, double &wx, double &wy) {
        wx = origin_x + (mx + 0.5) * resolution;
        wy = origin_y + (my + 0.5) * resolution;
    };

    int start_x, start_y, goal_x, goal_y;

    if (!worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
        RCLCPP_ERROR(get_logger(), "Start is outside map.");
        return;
    }

    if (!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        RCLCPP_ERROR(get_logger(), "Goal is outside map.");
        return;
    }

    RCLCPP_INFO(get_logger(), "Start map coords: (%d, %d)", start_x, start_y);
    RCLCPP_INFO(get_logger(), "Goal map coords: (%d, %d)", goal_x, goal_y);

    if (isInsideMap(start_x, start_y)) {
    RCLCPP_INFO(get_logger(), "Start cell value: %d", map_.data[getIndex(start_x, start_y)]);
    }
    if (isInsideMap(goal_x, goal_y)) {
        RCLCPP_INFO(get_logger(), "Goal cell value: %d", map_.data[getIndex(goal_x, goal_y)]);
    }

    if (!isFree(start_x, start_y)) {
        RCLCPP_ERROR(get_logger(), "Start is in obstacle or unknown cell.");
        return;
    }

    if (!isFree(goal_x, goal_y)) {
        RCLCPP_ERROR(get_logger(), "Goal is in obstacle or unknown cell.");
        return;
    }

    if (start_x == goal_x && start_y == goal_y) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = start.pose.position.x;
        pose.pose.position.y = start.pose.position.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path_.poses.push_back(pose);
        return;
    }

    std::vector<double> gScore(width * height, std::numeric_limits<double>::infinity());
    std::vector<double> fScore(width * height, std::numeric_limits<double>::infinity());
    std::vector<bool> closedList(width * height, false);
    std::vector<int> parentX(width * height, -1);
    std::vector<int> parentY(width * height, -1);
    std::vector<std::pair<int, int>> openList;

    auto heuristic = [&](int x, int y) {
        return std::hypot(static_cast<double>(goal_x - x), static_cast<double>(goal_y - y));
    };

    int start_idx = getIndex(start_x, start_y);
    gScore[start_idx] = 0.0;
    fScore[start_idx] = heuristic(start_x, start_y);
    parentX[start_idx] = start_x;
    parentY[start_idx] = start_y;
    openList.push_back({start_x, start_y});

    const std::vector<std::pair<int, int>> directions = {
        {-1,  0}, {1,  0}, {0, -1}, {0,  1},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };

    bool found = false;

    while (!openList.empty() && rclcpp::ok()) {
        auto best_it = openList.begin();
        double best_f = fScore[getIndex(best_it->first, best_it->second)];

        for (auto it = openList.begin(); it != openList.end(); ++it) {
            double current_f = fScore[getIndex(it->first, it->second)];
            if (current_f < best_f) {
                best_f = current_f;
                best_it = it;
            }
        }

        const int x = best_it->first;
        const int y = best_it->second;
        openList.erase(best_it);

        const int current_idx = getIndex(x, y);

        if (closedList[current_idx]) {
            continue;
        }

        closedList[current_idx] = true;

        if (x == goal_x && y == goal_y) {
            found = true;
            break;
        }

        for (const auto &[dx, dy] : directions) {
            const int nx = x + dx;
            const int ny = y + dy;

            if (!isInsideMap(nx, ny) || !isFree(nx, ny)) {
                continue;
            }

            const int neighbor_idx = getIndex(nx, ny);

            if (closedList[neighbor_idx]) {
                continue;
            }

            if (dx != 0 && dy != 0) {
                if (!isFree(x + dx, y) || !isFree(x, y + dy)) {
                    continue;
                }
            }

            const double step_cost = (dx == 0 || dy == 0) ? 1.0 : std::sqrt(2.0);
            const double tentative_g = gScore[current_idx] + step_cost;
            const double tentative_f = tentative_g + heuristic(nx, ny);

            if (tentative_f < fScore[neighbor_idx]) {
                gScore[neighbor_idx] = tentative_g;
                fScore[neighbor_idx] = tentative_f;
                parentX[neighbor_idx] = x;
                parentY[neighbor_idx] = y;
                openList.push_back({nx, ny});
            }
        }
    }

    if (!found) {
        RCLCPP_ERROR(get_logger(), "Unable to plan path.");
        return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> reconstructed_path;

    int x = goal_x;
    int y = goal_y;

    while (!(x == start_x && y == start_y)) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();

        mapToWorld(x, y, pose.pose.position.x, pose.pose.position.y);
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        reconstructed_path.push_back(pose);

        const int idx = getIndex(x, y);
        const int px = parentX[idx];
        const int py = parentY[idx];

        if (px < 0 || py < 0) {
            RCLCPP_ERROR(get_logger(), "Path reconstruction failed.");
            path_.poses.clear();
            return;
        }

        x = px;
        y = py;
    }

    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "map";
    start_pose.header.stamp = this->now();
    mapToWorld(start_x, start_y, start_pose.pose.position.x, start_pose.pose.position.y);
    start_pose.pose.position.z = 0.0;
    start_pose.pose.orientation.x = 0.0;
    start_pose.pose.orientation.y = 0.0;
    start_pose.pose.orientation.z = 0.0;
    start_pose.pose.orientation.w = 1.0;
    reconstructed_path.push_back(start_pose);

    std::reverse(reconstructed_path.begin(), reconstructed_path.end());
    path_.poses = reconstructed_path;

    RCLCPP_INFO(get_logger(), "A* path created with %zu poses.", path_.poses.size());
}

void PlanningNode::smoothPath() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    ... processing ...
    path_.poses = newPath;
    */
   if (path_.poses.size() < 3) {
        return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;

    const int iterations = 50;
    const double alpha = 0.1;
    const double beta = 0.3;

    for (int iter = 0; iter < iterations; ++iter) {
        for (size_t i = 1; i + 1 < newPath.size(); ++i) {
            const double orig_x = path_.poses[i].pose.position.x;
            const double orig_y = path_.poses[i].pose.position.y;

            const double prev_x = newPath[i - 1].pose.position.x;
            const double prev_y = newPath[i - 1].pose.position.y;

            const double next_x = newPath[i + 1].pose.position.x;
            const double next_y = newPath[i + 1].pose.position.y;

            double &x = newPath[i].pose.position.x;
            double &y = newPath[i].pose.position.y;

            x += alpha * (orig_x - x) + beta * (prev_x + next_x - 2.0 * x);
            y += alpha * (orig_y - y) + beta * (prev_y + next_y - 2.0 * y);
        }
    }

    path_.poses = newPath;
}

Cell::Cell(int c, int r) {
    (void)c;
    (void)r;
}
 