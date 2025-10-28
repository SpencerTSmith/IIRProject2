# TO RUN
```bash
TODO: Update this once we finish
roslaunch project1_turtlebot project1.launch
```

# TODO:
- Document the following
    - The robot code, as well as why we used our particular architecture, and how the code embodies that architecture (1.5 to 2 pages in length, roughly 80 characters per line, 50 lines per page)
    - Update the launch file documentation if needed
- Implement the following
    - Task Planning - Determines order of tasks (coordinate point pairs to visit) to minimize distance traveled. We can keep this simple and view distances as just straight lines with no obstacles between 'tasks'
    - Path Planning - Calculate turn angle and straight-line distance necessary to move between points, in the order determined by task planner. Can either plan entire path from the start, or as we go from point to point (Document which we choose).
    - Navigation - Follow the path from plan and avoid collisions along the way
    - Execution Monitoring -
        - As long as we are getting closer to desired point (distance decreasing) do nothing
        - Getting within 1 foot of desired point is success, report that
            - If there are still points to visit, move to next
            - If last point in list, report overall success and prompt user for more tasks
        - If progress is stopped and we are not in either of the previous states, we report failure to reach the desired point
            - If the point we failed to reach is the destination of the pair, we simply move onto the next point in the task plan.
            - If the point is the start point in a task, remove that task from the plan.

# COMPLETE:
- Document the following
    - (see TODO: May need to update this if we end up changing it) The world and launch files, translating the xml into English (1.5 to 2 pages in length, roughly 80 characters per line, 50 lines per page)
- Gazebo map world file
- (see TODO: We might launch new things) Launch file
- .pgm and .yaml map files (might need to change origin)
