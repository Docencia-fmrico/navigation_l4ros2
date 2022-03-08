# Práctica de navegación

[![GitHub Action
Status](https://github.com/Docencia-fmrico/navigation/workflows/main/badge.svg)](https://github.com/Docencia-fmrico/navigation)


**Entrega:** Miércoles 2/3 

En la moqueta verde del laboratorio se limitará con unas paredes, y se pondrán obstáculos (cajas) dentro el viernes 25/2. No habrá cambios en el escenario desde este momento. El miércoles, al inicio de la clase se proporcionarán un conjunto de waypoints en un fichero de parámetros como este:

```
patrolling_node:
  ros__parameters:
    waypoints: ["wp1", "wp2"]
    wp1: [1.0, 1.0]
    wp2: [-1.0, -1,0]
```

El robot debe ir en orden la coordenada (x, y) de cada uno de ellos, emitiendo un sonido cuando lo considera alcanzado. Se cronometrará el tiempo que tarda en hacerlo.

La velocidad lineal no podrá ser nunca superior a 0.4. Se descalificará a quien incumpla esta regla.

Habrá dos rondas:

- Ronda 1: Habrá 4 waypoints, y ninguno en la posición de un obstáculo.
- Ronda 2: Habrá 3-7 waypoints, alguno de ellos en la posición de un obstáculo. En este caso, se podrá ir al siguiente en cuanto se detecte este caso.




# Behavior Tree diagram 
![scheme](./bt.png)


The behavior tree is designed to perform the task of going to a specific number of waypoints located in the map using Nav2 for ROS 2. For this purpose we separated it into three main actions, the "GetNextWp" that gets the next waypoint coordinates from the params file and verifies that it can be reached, the "Move" action that passes those points to the Nav2 program and waits for the navigation result, and the "SoundFeedback" that publishes a different message on the sound topic depending on the navigation outcome. We also tried to implement a "LedFeedback" similar to the sound one but we ran into some linking problems and finally decided not to add it.

# Testing of different controllers and planners
## DWBLocalPlanner: the default

with this config in the file `tiago_nav_params.yaml`:
     
     controller_server:
       .......
       # DWB parameters
        FollowPath:
          plugin: "dwb_core::DWBLocalPlanner"
          debug_trajectory_details: True
          min_vel_x: 0.0
          min_vel_y: 0.0
          max_vel_x: 0.3
          max_vel_y: 0.0
          max_vel_theta: 0.5
          min_speed_xy: 0.0
          max_speed_xy: 0.5
          min_speed_theta: 0.0
          ........
          ........
          
We got the best results, reaching the goal safely and fastly.

## Nav2 regulated pure pursuit controller

with this config in the file `kobuki_nav_pursuit_params.yaml`:
    
    
    controller_server:
    .....
      FollowPath:
        plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        desired_linear_vel: 0.4
        max_linear_accel: 1.5
        max_linear_decel: 1.5
        lookahead_dist: 0.6
        min_lookahead_dist: 0.3
        max_lookahead_dist: 0.9
        lookahead_time: 1.5
        rotate_to_heading_angular_vel: 1.8

The navigation works but it takes to much time to plan a path, although it navigates directly if no new object appears.

## Rotation Shim controller

with this config in the file `kobuki_nav_rotation_shim_params.yaml`:

    controller_server:
    .....
      FollowPath:
        plugin: "nav2_rotation_shim_controller::RotationShimController"
        primary_controller: "dwb_core::DWBLocalPlanner"
        angular_dist_threshold: 0.785
        forward_sampling_distance: 0.5
        rotate_to_heading_angular_vel: 1.8
        max_angular_accel: 3.2
        simulate_ahead_time: 1.0
      # DWB parameters
      FollowPath:
        plugin: "dwb_core::DWBLocalPlanner"
        ......
        .....
        
The robot starts spinning and fails many times to start a route to the goal.
