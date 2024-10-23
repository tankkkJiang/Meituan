docker exec race_scene_sdk_container mv /evaluator/config/scene.config /evaluator/config/scene_old.config
docker exec race_car_sdk_container mv /car_log/config.json /car_log/config_old.json
docker exec race_drone_sdk_container  mv /drone/drone.json /drone/drone_old.json
docker exec race_user_sdk_container  mv /config/config.json /config/config_old.json 
docker cp config/car/config.json race_car_sdk_container:car_log/config.json
docker cp config/drone/drone.json race_drone_sdk_container:drone/drone.json
docker cp config/scene/scene.config race_scene_sdk_container:evaluator/config/scene.config
docker cp config/user/config.json race_user_sdk_container:config/config.json
docker commit race_user_sdk_container uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:user_new
docker commit race_car_sdk_container uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:cars_new
docker commit race_drone_sdk_container uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:drones_new
docker commit race_scene_sdk_container uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:scene_new
./stop_race.sh
./start_race_new.sh